FROM ros:humble-ros-base

WORKDIR /ros2_serial

RUN git clone https://github.com/yxSakana/ros2_serial src --depth=1 && \
    apt-get update && rosdep install --from-paths src -y --ignore-src && \
    colcon build --symlink-install && \
    rm -Rf /var/lib/apt/lists/* && \
    sed --in-place --expression \
        '$isource "/ros2_serial/install/setup.bash"' \
        /ros_entrypoint.sh && \
    chsh -c /bin/bash && \
    echo 'source "/ros2_serial/install/setup.bash"' >> ~/.bashrc
