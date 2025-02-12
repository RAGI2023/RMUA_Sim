#!/usr/bin/env python3

import rospy
import cv2
import torch
import torch.backends.cudnn as cudnn
import numpy as np
from cv_bridge import CvBridge
from pathlib import Path
import os
import sys
from rostopic import get_topic_type

from sensor_msgs.msg import Image, CompressedImage
from detection_msgs.msg import BoundingBox, BoundingBoxes
import message_filters

# add yolov5 submodule to path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0] / "yolov5"
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative path

# import from yolov5 submodules
from models.common import DetectMultiBackend
from utils.general import (
    check_img_size,
    check_requirements,
    non_max_suppression,
    scale_coords
)
from utils.plots import Annotator, colors
from utils.torch_utils import select_device
from utils.augmentations import letterbox


@torch.no_grad()
class Yolov5Detector:
    def __init__(self):
        self.conf_thres = rospy.get_param("~confidence_threshold")
        self.iou_thres = rospy.get_param("~iou_threshold")
        self.agnostic_nms = rospy.get_param("~agnostic_nms")
        self.max_det = rospy.get_param("~maximum_detections")
        self.classes = rospy.get_param("~classes", None)
        self.line_thickness = rospy.get_param("~line_thickness")
        self.view_image = rospy.get_param("~view_image")
        # Initialize weights 
        weights = rospy.get_param("~weights")
        # Initialize model
        self.device = select_device(str(rospy.get_param("~device","")))
        self.model = DetectMultiBackend(weights, device=self.device, dnn=rospy.get_param("~dnn"), data=rospy.get_param("~data"))
        self.stride, self.names, self.pt, self.jit, self.onnx, self.engine = (
            self.model.stride,
            self.model.names,
            self.model.pt,
            self.model.jit,
            self.model.onnx,
            self.model.engine,
        )

        # Setting inference size
        self.img_size = [rospy.get_param("~inference_size_w", 640), rospy.get_param("~inference_size_h",480)]
        self.img_size = check_img_size(self.img_size, s=self.stride)

        # Half
        self.half = rospy.get_param("~half", False)
        self.half &= (
            self.pt or self.jit or self.onnx or self.engine
        ) and self.device.type != "cpu"  # FP16 supported on limited backends with CUDA
        if self.pt or self.jit:
            self.model.model.half() if self.half else self.model.model.float()
        bs = 1  # batch_size
        cudnn.benchmark = True  # set True to speed up constant image size inference
        self.model.warmup()  # warmup        
        
        # Initialize subscriber to Image/CompressedImage topic
        input_image_type, input_image_topic_right, _ = get_topic_type(rospy.get_param("~input_image_topic_right"), blocking = True)
        _, input_image_topic_left, _ = get_topic_type(rospy.get_param("~input_image_topic_left"), blocking = True)
        self.compressed_input = input_image_type == "sensor_msgs/CompressedImage"

        if self.compressed_input:
            self.image_sub = rospy.Subscriber(
                input_image_topic_right, CompressedImage, self.callback, queue_size=1
            )
        else:
            # self.image_sub = rospy.Subscriber(
            #     input_image_topic_right, Image, self.callback, queue_size=1
            # )
            self.suber_right = message_filters.Subscriber(input_image_topic_right, Image)
            self.suber_left = message_filters.Subscriber(input_image_topic_left, Image)
            # self.sync_handler_ptr = message_filters.ExactTimeSynchronizer([self.suber_left, self.suber_right], 1)
            self.sync_handler_ptr = message_filters.ApproximateTimeSynchronizer([self.suber_left, self.suber_right], 1, slop=0.01)
            self.sync_handler_ptr.registerCallback(self.callback)

        # Initialize prediction publisher
        self.pred_pub = rospy.Publisher(
            rospy.get_param("~output_topic"), BoundingBoxes, queue_size=10
        )
        # Initialize image publisher
        self.publish_image = rospy.get_param("~publish_image")
        if self.publish_image:
            self.image_pub = rospy.Publisher(
                rospy.get_param("~output_image_topic"), Image, queue_size=10
            )
        
        # Initialize CV_Bridge
        self.bridge = CvBridge()

    def callback(self, data_left, data_right):
        """adapted from yolov5/detect.py"""
        # print(data.header)
        if self.compressed_input:
            # im_left = self.bridge.compressed_imgmsg_to_cv2(data_left, desired_encoding="bgr8")
            return
        else:
            im_left = self.bridge.imgmsg_to_cv2(data_left, desired_encoding="bgr8")
            im_right = self.bridge.imgmsg_to_cv2(data_right, desired_encoding="bgr8")
        
        # print(im_left.shape)

        im_left, im0_left = self.preprocess(im_left)
        im_right, im0_right = self.preprocess(im_right)

        # print(im.shape)
        # print(img0.shape)
        # print(img.shape)
        # 左右堆叠为一个批次
        im_left = im_left.squeeze(0)  # 去掉第一维（批次维度）
        im_right = im_right.squeeze(0)  # 去掉第一维（批次维度）
        im_batch = torch.stack([torch.from_numpy(im_left), torch.from_numpy(im_right)], dim=0).to(self.device)
        # Run inference
        # im_left = torch.from_numpy(im_left).to(self.device) 
        # im_left = im_left.half() if self.half else im_left.float()
        im_batch = im_batch.half() if self.half else im_batch.float()
        # im_left /= 255
        im_batch /= 255
        # if len(im_left.shape) == 3:
        #     im_left = im_left[None]
        #     im_right = im_right[None]
        
        # im_batch = torch.stack([torch.from_numpy(im_left), torch.from_numpy(im_right)], dim=0).to(self.device)
        # im_batch = im_batch.half() if self.half else im_batch.float()
        # im_batch /= 255
        if len(im_batch.shape) == 3:
            im_batch = im_batch[None]

        # print(im_batch.shape)
        # print(im_batch.shape)
        # pred = self.model(im_left, augment=False, visualize=Fals
        pred = self.model(im_batch, augment=False, visualize=False)
        pred = non_max_suppression(
            pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det
        )

        ### To-do move pred to CPU and fill BoundingBox messages
        
        # Process predictions 
        det_left = pred[0].cpu().numpy()

        bounding_boxes_left = BoundingBoxes()
        bounding_boxes_left.header = data_left.header
        bounding_boxes_left.image_header = data_left.header
        
        annotator_left = Annotator(im0_left, line_width=self.line_thickness, example=str(self.names))
        # print("im_left shape: ", im_left.shape)
        # print("im0_left shape: ", im0_right.shape)
        if len(det_left):
            # Rescale boxes from img_size to im0 size
            # print("det_left shape: ", det_left.shape)
            det_left[:, :4] = scale_coords(im_left.shape[1:], det_left[:, :4], im0_left.shape[:2]).round()

            # Write results
            for *xyxy, conf, cls in reversed(det_left):
                bounding_box_left = BoundingBox()
                c = int(cls)
                # Fill in bounding box message
                bounding_box_left.Class = self.names[c]
                bounding_box_left.probability = conf 
                bounding_box_left.xmin = int(xyxy[0])
                bounding_box_left.ymin = int(xyxy[1])
                bounding_box_left.xmax = int(xyxy[2])
                bounding_box_left.ymax = int(xyxy[3])

                bounding_boxes_left.bounding_boxes.append(bounding_box_left)

                # Annotate the image
                if self.publish_image or self.view_image:  # Add bbox to image
                      # integer class
                    label = f"{self.names[c]} {conf:.2f}"
                    annotator_left.box_label(xyxy, label, color=colors(c, True))       
        
        det_right = pred[1].cpu().numpy()
        # print("det_right shape: ", det_right.shape)
        bounding_boxes_right = BoundingBoxes()
        bounding_boxes_right.header = data_left.header
        bounding_boxes_right.image_header = data_left.header
        annotator_right = Annotator(im0_right, line_width=self.line_thickness, example=str(self.names))
        if len(det_right):
            # Rescale boxes from img_size to im0 size
            det_right[:, :4] = scale_coords(im_right.shape[1:], det_right[:, :4], im0_right.shape[:2]).round()

            # Write results
            for *xyxy, conf, cls in reversed(det_right):
                bounding_box_right = BoundingBox()
                c = int(cls)
                # Fill in bounding box message
                bounding_box_right.Class = self.names[c]
                bounding_box_right.probability = conf 
                bounding_box_right.xmin = int(xyxy[0])
                bounding_box_right.ymin = int(xyxy[1])
                bounding_box_right.xmax = int(xyxy[2])
                bounding_box_right.ymax = int(xyxy[3])

                bounding_boxes_right.bounding_boxes.append(bounding_box_right)

                # Annotate the image
                if self.publish_image or self.view_image:  # Add bbox to image
                      # integer class
                    label = f"{self.names[c]} {conf:.2f}"
                    annotator_right.box_label(xyxy, label, color=colors(c, True)) 
                ### POPULATE THE DETECTION MESSAGE HERE

            # Stream results
            # im0_left = annotator_left.result()

        # Publish prediction
        # self.pred_pub.publish(bounding_boxes_left)

        # Publish & visualize images
        if self.view_image:
            combined_image = cv2.hconcat([im0_left, im0_right])
            cv2.imshow("Image", combined_image)
            cv2.waitKey(1)  # 1 millisecond
        if self.publish_image:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(im0_left, "bgr8"))
        

    def preprocess(self, img):
        """
        Adapted from yolov5/utils/datasets.py LoadStreams class
        """
        img0 = img.copy()
        img = np.array([letterbox(img, self.img_size, stride=self.stride, auto=self.pt)[0]])
        # Convert
        img = img[..., ::-1].transpose((0, 3, 1, 2))  # BGR to RGB, BHWC to BCHW
        img = np.ascontiguousarray(img)

        return img, img0 


if __name__ == "__main__":

    check_requirements(exclude=("tensorboard", "thop"))
    
    rospy.init_node("yolov5", anonymous=True)
    detector = Yolov5Detector()
    
    rospy.spin()
