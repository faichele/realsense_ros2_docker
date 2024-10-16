FROM arm64v8/ros:foxy as builder

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    ca-certificates \
    python3-pip \
    python3-setuptools \
    python3-rosdep python3-rosinstall-generator python3-vcstools python3-rosinstall build-essential \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros1-latest.list

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

RUN mkdir -p /etc/ros/rosdep/sources.list.d
COPY $PWD/config_files/20-default.list /etc/ros/rosdep/sources.list.d/20-default.list

RUN apt-get update && apt-get -y install curl gnupg2 lsb-release && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN echo "deb http://packages.ros.org/ros2/ubuntu bionic main" > /etc/apt/sources.list.d/ros2-latest.list

# RUN apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key

COPY $PWD/config_files/all_apt_keys.gpg /workspace/all_apt_keys.gpg
RUN apt-key add /workspace/all_apt_keys.gpg

# Ubuntu 18 is bionic
RUN echo "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" > /etc/apt/sources.list.d/realsense2.list

RUN apt-get update && apt-get install -y \
  librealsense2-utils librealsense2-dev \
  build-essential \
  cmake \
  git \
  libbullet-dev \
  python3-pip \
  python3-flake8 \
  python3-pytest-cov \
  python3-vcstools \
  wget && \
  apt-get install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev libcunit1-dev && \
  rm -rf /var/lib/apt/lists/*

RUN pip3 install -U colcon-common-extensions \
    argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest \
  vcstool \
  lark-parser

RUN mkdir -p /workspace/ros_foxy_ws/src
WORKDIR /workspace/ros_foxy_ws

RUN wget https://raw.githubusercontent.com/ros2/ros2/foxy/ros2.repos && \
    vcs import src < ros2.repos

RUN apt-get update && rosdep update && \
    rosdep install --from-paths src --ignore-src --rosdistro foxy -y --skip-keys "console_bridge fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers"

RUN apt-get update && apt-get -y install libeigen3-dev libogre-1.12-dev libxaw7-dev libx11-dev python3-numpy libfreetype6-dev liblog4cxx-dev

COPY $PWD/src/realsense_ros2 /workspace/ros_foxy_ws/src/realsense_ros2

ENV MAKEFLAGS="-j2 -l1"
RUN colcon build --symlink-install --executor parallel --packages-skip rviz_rendering rviz_ogre_vendor rviz_assimp_vendor rviz_rendering_tests --packages-end rmw_connext_shared_cpp --cmake-args -DBUILD_TESTING=0

COPY $PWD/src/vision_opencv_ros2 /workspace/ros_foxy_ws/src/vision_opencv_ros2

RUN apt-get update && apt-get install -y \
  libboost-python-dev \
  libopencv-dev && \
  rm -rf /var/lib/apt/lists/*

ENV MAKEFLAGS="-j4 -l1"
RUN colcon build --symlink-install --executor parallel --packages-skip rviz_rendering rviz_ogre_vendor rviz_assimp_vendor rviz_rendering_tests --packages-end nav_msgs --cmake-args -DBUILD_TESTING=0

RUN apt-get update && apt-get -y install build-essential cmake git pkg-config libgtk-3-dev \
libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev \
gfortran openexr libatlas-base-dev python3-dev python3-numpy \
libtbb2 libtbb-dev libdc1394-22-dev libopenexr-dev \
libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev && \
rm -rf /var/lib/apt/lists/*

COPY $PWD/src/opencv_3.4 /workspace/opencv_3.4
WORKDIR /workspace/opencv_3.4
RUN mkdir build && \
    cd build && \
    cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D INSTALL_C_EXAMPLES=OFF \
    -D INSTALL_PYTHON_EXAMPLES=OFF \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D BUILD_EXAMPLES=OFF .. && \
    make -j4 -l1 install

WORKDIR /workspace/ros_foxy_ws

COPY $PWD/src/image_common_ros2 /workspace/ros_foxy_ws/src/image_common_ros2
COPY $PWD/src/common_interfaces_ros2/diagnostic_msgs /workspace/ros_foxy_ws/src/diagnostic_msgs

RUN rm -rf /workspace/ros_foxy_ws/src/diagnostic_msgs
RUN colcon build --symlink-install --executor parallel --packages-select diagnostic_msgs --cmake-args -DBUILD_TESTING=0


# RUN colcon build --symlink-install --executor parallel --packages-skip rviz_rendering rviz_ogre_vendor rviz_assimp_vendor rviz_rendering_tests demo_nodes_cpp demo_nodes_cpp_native turtlesim --packages-end realsense2_camera --cmake-args -DBUILD_TESTING=0

RUN mkdir -p /workspace/ros_distro/foxy && cp -ar /opt/ros/foxy /workspace/ros_distro/foxy

FROM arm64v8/ros:foxy as release
# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    rm --f /etc/localtime && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    python3-empy libboost-thread1.71 libboost-system1.71 libboost-filesystem1.71 libboost-program-options1.71 libboost-regex1.71 libboost-chrono1.71 python3-rospkg python3-catkin-pkg python3-netifaces\
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros2-latest.list

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS2_DISTRO foxy

COPY $PWD/scripts/ros2_entrypoint.sh /ros_entrypoint.sh
RUN rm -rf /opt/ros && mkdir -p /opt/ros/${ROS2_DISTRO} 
COPY --from=builder /workspace/ros_distro/${ROS2_DISTRO}/* /opt/ros/${ROS2_DISTRO}
RUN ls /opt/ros -lR

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
