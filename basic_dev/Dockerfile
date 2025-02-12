FROM ros:noetic-ros-core-focal
ADD src /basic_dev/src/
ADD setup.bash /

RUN chmod +x /setup.bash

USER root

RUN apt update && apt install -y python3-catkin-tools ros-noetic-geographic-msgs \
 ros-noetic-tf2-sensor-msgs ros-noetic-tf2-geometry-msgs ros-noetic-image-transport \
 net-tools python3-pip \
 libopencv-dev python3-opencv



ENV ROS_DISTRO noetic

WORKDIR /basic_dev/src/yolov5_ros/src/yolov5
RUN pip3 install --no-cache-dir -r requirements.txt -i https://mirrors.aliyun.com/pypi/simple

WORKDIR /basic_dev/
# RUN . /opt/ros/${ROS_DISTRO}/setup.sh && catkin_make --only-pkg-with-deps airsim_ros && . devel/setup.sh && catkin_make --only-pkg-with-deps basic_dev

ENTRYPOINT [ "/setup.bash" ]