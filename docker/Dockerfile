FROM ubuntu:24.04

ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_APT_SOURCE_VERSION=1.2.0
ARG USERNAME=ascend

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV ROS_DISTRO=jazzy

# Install non-ROS tools 
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        locales \
        sudo \
        vim \
        nano \
        git \
        openssh-client \
        bash-completion \
    && locale-gen en_US.UTF-8 \
    && update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# Configure the ROS 2 apt repository


RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    software-properties-common \
    && add-apt-repository universe \
    && curl -fsSL -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb" \
    && dpkg -i /tmp/ros2-apt-source.deb \
    && rm -f /tmp/ros2-apt-source.deb \
    && rm -rf /var/lib/apt/lists/*


# Install ROS 2 Jazzy

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-base \
        ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*


# # Create non-root user
# RUN useradd --create-home --shell /bin/bash "${USERNAME}" \ 
#     && usermod --append --groups sudo "${USERNAME}" \ 
#     && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}" \ 
#     && chmod 0440 "/etc/sudoers.d/${USERNAME}" \ 
#     && echo "source /opt/ros/jazzy/setup.bash" >> "/home/${USERNAME}/.bashrc" \ 
#     && chown "${USERNAME}:${USERNAME}" "/home/${USERNAME}/.bashrc"


RUN groupmod --new-name ${USERNAME} ubuntu \
    && usermod --login ${USERNAME} ubuntu \
    && usermod --home /home/${USERNAME} --move-home ${USERNAME} \
    && usermod --shell /bin/bash ${USERNAME} \
    && usermod --append --groups sudo ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" \
        > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && echo "source /opt/ros/jazzy/setup.bash" \
        >> /home/${USERNAME}/.bashrc
        
WORKDIR /workspaces/ros2-px4

USER ${USERNAME}

CMD ["bash"]