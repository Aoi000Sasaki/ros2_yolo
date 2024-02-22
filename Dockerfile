ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}
ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo git vim \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-desktop python3-colcon-common-extensions
RUN rm -rf /etc/apt/apt.conf.d/docker-clean
ENV SHELL /bin/bash

# for yolov8
RUN apt install --no-install-recommends -y python3-pip git zip curl htop libgl1 libglib2.0-0 libpython3-dev gnupg g++ libusb-1.0-0
RUN python3 -m pip install --upgrade pip wheel
RUN pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
RUN pip install ultralytics

USER $USERNAME
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source /home/user/ws/install/local_setup.bash" >> ~/.bashrc
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
RUN echo "export _colcon_cd_root=/home/user/ws" >> ~/.bashrc
RUN echo "alias cb='colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --event-handlers console_direct+'" >> ~/.bashrc

CMD ["/bin/bash"]
