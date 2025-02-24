# syntax = docker/dockerfile:1.3

ARG ROS_DISTRO=humble

######################### Tutorial Image  #################################################

FROM moveit/moveit2:${ROS_DISTRO}-source as tutorial_image


# Copy sources from docker context
# TODO: combine COPYs in one layer somehow
COPY iscoin_description src/moveit2_tutorials/iscoin_description
COPY iscoin_driver src/moveit2_tutorials/iscoin_driver
COPY iscoin_simulation_gz src/moveit2_tutorials/iscoin_simulation_gz

# Fetch required upstream sources for building
RUN vcs import src < src/moveit2_tutorials/iscoin_driver/.github/iscoin.humble.repos

#Build the tutorials and set up the entrypoint/bashrc
RUN --mount=type=cache,target=/root/.ccache/ \
    # Enable ccache
    . "/opt/ros/${ROS_DISTRO}/setup.sh" &&\
    . "install/setup.sh" &&\
    sudo apt update && rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y && \
    sudo apt install -y ros-${ROS_DISTRO}-rmw-cyclonedds-cpp && \
    colcon build \
            --cmake-args -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=OFF -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
            --ament-cmake-args -DCMAKE_BUILD_TYPE=Release \
            --event-handlers desktop_notification- status- && \
    ccache -s && \
    rm -rf /var/lib/apt/lists/* && \
    # Update /ros_entrypoint.sh to source our new workspace
    sed -i "s#/opt/ros/\$ROS_DISTRO/setup.bash#$ROS_UNDERLAY/install/setup.bash#g" /ros_entrypoint.sh && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /root/ws_moveit/install/setup.bash" >> /root/.bashrc