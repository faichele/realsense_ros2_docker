# FROM arm64v8/ros:humble
FROM nvcr.io/nvidia/l4t-cuda:11.4.19-devel as ros-humble-builder

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends -o Dpkg::Options::="--force-confdef" tzdata software-properties-common && \
    rm -rf /var/lib/apt/lists/*

RUN sed -r -i 's/^deb(.*)$/deb\1 contrib/g' /etc/apt/sources.list

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends -o Dpkg::Options::="--force-confdef" \
    dirmngr \
    gnupg2 \
    ca-certificates \
    python3-pip \
    python3-setuptools \
    python3-rosdep python3-rosinstall-generator python3-vcstools python3-rosinstall build-essential \
    python3-flake8-docstrings \
    python3-pytest-cov \
    git \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install vcstool

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

# setup sources.list
RUN apt-get update && apt-get -q -y --no-install-recommends -o Dpkg::Options::="--force-confdef" install curl apt-transport-https gnupg2 lsb-release && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN echo "deb http://packages.ros.org/ros2/ubuntu focal main" > /etc/apt/sources.list.d/ros2-latest.list

RUN mkdir -p /etc/apt/keyrings && curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | tee /etc/apt/keyrings/librealsense.pgp

RUN echo "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" > /etc/apt/sources.list.d/realsense2.list

RUN apt-get update && apt-get -q -y --no-install-recommends -o Dpkg::Options::="--force-confdef" install librealsense2 librealsense2-dev librealsense2-utils

RUN python3 -m pip install -U \
   flake8-blind-except \
   flake8-builtins \
   flake8-class-newline \
   flake8-comprehensions \
   flake8-deprecated \
   flake8-import-order \
   flake8-quotes \
   "pytest>=5.3" \
   pytest-repeat \
   pytest-rerunfailures

RUN mkdir -p /root/ros2_humble_ws/src
WORKDIR /root/ros2_humble_ws

RUN vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src

RUN apt-get upgrade -y
RUN rm -rf /etc/ros/rosdep && rosdep init && \
    rosdep update
    # rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"

RUN apt-get update && apt-get -y -q --no-install-recommends install libacl1-dev libssl-dev libtinyxml2-dev libxaw7-dev libasio-dev libeigen3-dev python3.8-dev libpython3.8-dev python3-colcon-*

RUN bash -c "colcon build --merge-install --install-base /opt/ros/${ROS_DISTRO} --packages-up-to rosidl_cmake"

RUN apt-get update && apt-get -y -q --no-install-recommends install qt5-default python3-sip-dev python3-sip python3-lark pyqt5-dev python3-pyqt5 libopencv-dev libbullet-dev

RUN bash -c "colcon build --merge-install --install-base /opt/ros/${ROS_DISTRO}"

# RUN apt-get update && apt-get install -y \
#     ros-${ROS_DISTRO}-realsense2-camera && \
#     rm -rf /var/lib/apt/lists/*

RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
RUN apt-add-repository "deb https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main" && apt-get update && apt-get install -q -y --no-install-recommends cmake libboost-python-dev

RUN mkdir -p /root/realsense_ws/src
COPY ./src/diagnostics_ros2/ /root/realsense_ws/src/diagnostics_ros2/
COPY ./src/vision_opencv_ros2/ /root/realsense_ws/src/vision_opencv_ros2/
COPY ./src/realsense_ros2/ /root/realsense_ws/src/realsense_ros2/
COPY ./src/realsense_splitter/ /root/realsense_ws/src/realsense_splitter/
COPY ./src/nvblox_ros_common/ /root/realsense_ws/src/nvblox_ros_common/
COPY ./src/isaac_ros_launch_utils/ /root/realsense_ws/src/isaac_ros_launch_utils/

RUN rm -f /root/realsense_ws/src/realsense_splitter/COLCON_IGNORE

WORKDIR /root/realsense_ws

# RUN curl -sSf https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/arm64/3bf863cc.pub | tee /etc/apt/keyrings/cudatoolkit.pub
# RUN apt-key del A4B469963BF863CC && \
#     apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/arm64/3bf863cc.pub
# RUN echo "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/arm64/ /" > /etc/apt/sources.list.d/cudatoolkit.list

# RUN apt-get update && apt-get -y install cuda-toolkit-12 && apt-get --purge -y remove nsight-compute && rm -rf /var/cache/apt/archives/*

RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --merge-install --install-base /opt/ros/${ROS_DISTRO}"

RUN apt-get update && apt-get -y -q --no-install-recommends install python3-netifaces

COPY ./scripts/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

RUN /usr/sbin/groupadd -g 1000 ros_group
RUN /usr/sbin/useradd -m -u 1000 -g 1000 ros_user

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
