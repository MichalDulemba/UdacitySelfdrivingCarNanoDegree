# Docker image for CarND Capstone - GPU
FROM nvidia/cuda:8.0-cudnn6-devel-ubuntu16.04

# ros kinetic for ubuntu xenial

# ros-core
# install packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# bootstrap rosdep
RUN rosdep init \
    && rosdep update

# install ros packages
ENV ROS_DISTRO kinetic
RUN apt-get update && apt-get install -y \
    ros-kinetic-ros-core=1.3.1-0* \
    && rm -rf /var/lib/apt/lists/*

# ros-base
# install ros packages
RUN apt-get update && apt-get install -y \
    ros-kinetic-ros-base=1.3.1-0* \
    && rm -rf /var/lib/apt/lists/*

# ros-robot
# install ros packages
RUN apt-get update && apt-get install -y \
    ros-kinetic-robot=1.3.1-0* \
&& rm -rf /var/lib/apt/lists/*

# Install Dataspeed DBW https://goo.gl/KFSYi1 from binary
# adding Dataspeed server to apt
RUN sh -c 'echo "deb [ arch=amd64 ] http://packages.dataspeedinc.com/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-dataspeed-public.list'
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys FF6D3CDA
RUN apt-get update

# setup rosdep
RUN sh -c 'echo "yaml http://packages.dataspeedinc.com/ros/ros-public-'$ROS_DISTRO'.yaml '$ROS_DISTRO'" > /etc/ros/rosdep/sources.list.d/30-dataspeed-public-'$ROS_DISTRO'.list'
RUN rosdep update
RUN apt-get install -y ros-$ROS_DISTRO-dbw-mkz
# RUN apt-get upgrade -y
# end installing Dataspeed DBW

# install python packages
RUN apt-get install -y python-pip
COPY requirements-gpu.txt ./requirements.txt
RUN pip install -r requirements.txt

# install required ros dependencies
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-pcl-ros \
    ros-$ROS_DISTRO-image-proc

# socket io
RUN apt-get install -y netbase

# development: everyone needs editors and git
RUN apt-get install -y vim nano git

# udacity capstone
RUN mkdir /udacity
VOLUME ["/udacity"]
VOLUME ["/root/.ros/log/"]
WORKDIR /udacity/

# setup entrypoint
COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
