FROM ubuntu:jammy

# Setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# Install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# Setup keys
RUN set -eux; \
    key='C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654'; \
    export GNUPGHOME="$(mktemp -d)"; \
    gpg --batch --keyserver keyserver.ubuntu.com --recv-keys "$key"; \
    gpg --batch --export "$key" > /usr/share/keyrings/ros2-latest-archive-keyring.gpg; \
    gpgconf --kill all; \
    rm -rf "$GNUPGHOME"

# Setup sources.list
RUN echo "deb [ signed-by=/usr/share/keyrings/ros2-latest-archive-keyring.gpg ] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2-latest.list

# Setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO humble

# Install ROS 2 packages and development tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-core \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    && rosdep init \
    && rosdep update \
    && rm -rf /var/lib/apt/lists/*

# Copy your ROS package and install Python dependencies
WORKDIR /inverseKin
COPY . /inverseKin/
RUN pip install --no-cache-dir -r requirements.txt

# Build your ROS package using colcon
RUN . /opt/ros/humble/setup.sh && \
    colcon build



# Run your ROS node
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && colcon build && source install/setup.bash && ros2 run inverseKin server"]
