FROM ros:jazzy

# Install essentials
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-venv \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    ros-jazzy-slam-toolbox \
    ros-jazzy-robot-localization \
    ros-jazzy-nav2-bringup \
    ros-jazzy-pcl-ros \
    ros-jazzy-pcl-conversions \
    ros-jazzy-sensor-msgs \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-tools \
    ros-jazzy-rviz2 \
    ros-jazzy-rqt \
    ros-jazzy-rqt-common-plugins \
    ros-jazzy-demo-nodes-cpp \
    ros-jazzy-demo-nodes-py \
    ros-jazzy-imu-tools \
    ros-jazzy-rviz-imu-plugin \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-interfaces \
    ros-jazzy-gz-sensors-vendor \
    ros-jazzy-gz-rendering-vendor \
    git vim nano wget curl tmux htop \
    && rm -rf /var/lib/apt/lists/* 
    
# Initialize rosdep
RUN rosdep init || true && rosdep update

# Create workspace
RUN mkdir -p /ros2_ws/src
# Create and setup venv
RUN python3 -m venv /ros2_ws/.venv --system-site-packages
# Auto activate venv on shell start
  
# Auto source on shell start
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash 2>/dev/null || true" >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=0" >> /root/.bashrc && \
    echo "export RCUTILS_COLORIZED_OUTPUT=1" >> /root/.bashrc && \
    echo "source /ros2_ws/.venv/bin/activate" >> /root/.bashrc  \
    echo "export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/" >> /root/.bashrc && \
    echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/opt/ros/jazzy/opt/gz_sensors_vendor/lib/" >> /root/.bashrc

# Terminal colors
ENV TERM=xterm-256color
ENV FORCE_COLOR=1
RUN echo 'export PS1="(.venv) \[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]# "' >> /ros2_ws/.venv/bin/activate
# Colored bash prompt
RUN echo 'export PS1="\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]# "' >> /root/.bashrc

WORKDIR /ros2_ws
CMD ["bash"]