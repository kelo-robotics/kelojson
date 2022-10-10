FROM ros:melodic

ARG SSH_KEY

SHELL [ "/bin/bash", "-c" ]

# Install dependencies
RUN apt-get -qq update > /dev/null && \
    apt-get -yqq install sudo \
                         git \
                         ssh \
                         python-catkin-tools \
                         ros-$ROS_DISTRO-catkin \
                         ros-$ROS_DISTRO-tf \
                         ros-$ROS_DISTRO-tf2-geometry-msgs \
                         libyaml-cpp-dev > /dev/null && \
    apt-get clean > /dev/null

WORKDIR /workspace/catkin_ws/src

# Use SSH deploy keys to clone dependencies and then remove the ssh config
RUN mkdir /root/.ssh/ && \
    echo "$SSH_KEY" > /root/.ssh/id_rsa && \
    chmod -R 600 /root/.ssh/ && \
    touch /root/.ssh/known_hosts && \
    ssh-keyscan -T 60 git.locomotec.com >> /root/.ssh/known_hosts && \
    git clone git@git.locomotec.com:kelo/common/geometry_common.git && \
    git clone git@git.locomotec.com:kelo/common/yaml_common.git && \
    rm -rf /root/.ssh/

# Copy the kelojson source code to the docker container
WORKDIR /workspace/catkin_ws/src/kelojson
ADD . /workspace/catkin_ws/src/kelojson/

# Compile the ROS catkin workspace
RUN cd /workspace/catkin_ws && \
    /ros_entrypoint.sh catkin build --no-status

# Run unit tests
RUN source /workspace/catkin_ws/devel/setup.bash && \
    cd /workspace/catkin_ws/src/kelojson && \
    /ros_entrypoint.sh catkin build --this --no-status --catkin-make-args run_tests -- && \
    rosrun kelojson kelojson_test
