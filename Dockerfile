FROM ros:humble-ros-base@sha256:7bea3d9aa2483d3ca34c8e30d921b79273b0913bd7dc64bebf51d082b5d107e4

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-rviz2 \
        ros-humble-message-filters \
        ros-humble-pcl-ros \
        libeigen3-dev \
        python3-colcon-common-extensions \
        python3-vcstool \
        git \
        sudo \
    && rm -rf /var/lib/apt/lists/*

ARG UID=1000
ARG GID=1000
RUN groupadd --gid ${GID} dev \
    && useradd --uid ${UID} --gid ${GID} --create-home --shell /bin/bash dev \
    && echo "dev ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/dev \
    && chmod 0440 /etc/sudoers.d/dev

RUN echo "source /opt/ros/humble/setup.bash" >> /home/dev/.bashrc \
    && echo "[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash" >> /home/dev/.bashrc

USER dev
WORKDIR /ws
