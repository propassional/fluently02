FROM nvidia/cuda:11.8.0-cudnn8-devel-ubuntu22.04
LABEL maintainer="RoccoFelici<rocco.felici@supsi.ch>"

ENV ROS_DISTRO="humble"
ENV ROS_DOMAIN_ID=5
ENV DEBIAN_FRONTEND=noninteractive

# basic libraries
RUN apt-get update -q && apt-get install -yq git nano libtiff-dev curl gnupg2 lsb-release python3-pip vim

# python libraries
RUN python3 -m pip install rasa==3.6.20
RUN pip install setuptools==58.2.0 nltk spacy quantulum3 numpy==1.26.00 

# ROS2 
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update && apt-get -yq install ros-${ROS_DISTRO}-desktop ros-dev-tools

# working directory 
COPY ros2_ws /home/ros2_ws
COPY entrypoint.sh /entrypoint.sh

COPY fastrtps-profile.xml /home/
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/home/fastrtps-profile.xml

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "source /home/ros2_ws/install/setup.bash" >> ~/.bashrc

WORKDIR /home/ros2_ws
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]