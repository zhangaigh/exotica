FROM ros:kinetic-ros-core

RUN mkdir -p catkin_ws/src && cd catkin_ws/src && catkin_init_workspace && git clone --recursive https://github.com/ipab-slmc/exotica.git && rosdep update
RUN apt-get update && apt-get install -y python-catkin-tools
RUN rosdep install --from-paths src --ignore-src -r -y
RUN catkin build -s