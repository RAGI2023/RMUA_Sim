FROM ros:noetic-ros-core-focal
USER root

RUN sed -i 's|http://archive.ubuntu.com|http://mirrors.aliyun.com|g' /etc/apt/sources.list && \
    apt update && apt install -y python3-catkin-tools ros-noetic-geographic-msgs \
    ros-noetic-cv-bridge ros-noetic-roslaunch \
    ros-noetic-tf2-sensor-msgs ros-noetic-tf2-geometry-msgs ros-noetic-image-transport \
    net-tools python3-pip \
    libopencv-dev python3-opencv \
    && apt clean && rm -rf /var/lib/apt/lists/*

ADD src/yolov5_ros/src/yolov5/requirements.txt /
RUN pip3 install --no-cache-dir -r requirements.txt -i https://mirrors.aliyun.com/pypi/simple

ENV ROS_DISTRO=noetic

WORKDIR /basic_dev/src/yolov5_ros/src/yolov5

WORKDIR /basic_dev/
ADD src /basic_dev/src/
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && catkin_make --only-pkg-with-deps airsim_ros  \
&& . devel/setup.sh \
&& catkin_make --only-pkg-with-deps basic_dev controller_test detection_msgs imu_gps_odometry yolov5_ros

ADD src/yolov5_ros/src/yolov5/best.pt /basic_dev/src/yolov5_ros/src/yolov5/
ADD setup.bash /
RUN chmod +x /setup.bash
ENTRYPOINT [ "/setup.bash" ]