FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
 && apt-get install -y -qq \
    python3-pip \
    python3-kconfiglib \
    python3-jinja2 \
    python3-jsonschema \
    python3-future \
    gcc-arm-none-eabi \
    rsync \
    dirmngr \
    build-essential \
    lsb-release \
    wget \
    curl \
    gnupg2 \
    qtbase5-dev \
    cmake  \
    git \
    && apt-get clean

RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update && \
    apt-get install -y -qq gz-garden \
    && apt-get clean

RUN pip3 install vcstool pyros-genmsg

RUN mkdir -p /home/vehicle_gateway/src && cd /home/vehicle_gateway && git clone https://github.com/osrf/vehicle_gateway src/vehicle_gateway --depth 1 && \
    vcs import src < src/vehicle_gateway/dependencies.repos

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $(lsb_release -cs)) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null & \
    wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

RUN apt-get update && apt-get upgrade -q -y && \
    apt -y install python3-rosdep python3-colcon-common-extensions \
     $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/gz\|sdf/d' | tr '\n' ' ') \
    && apt-get clean

RUN rosdep init && \
  rosdep update && \
  cd /home/vehicle_gateway && \
  rosdep install --from-paths ./ -i -y --rosdistro humble --ignore-src --skip-keys="gz-transport12 gz-common5 gz-math7 gz-msgs9 gz-gui7 gz-cmake3 gz-sim7 zenohc gz-transport7 gz-plugin2"

RUN cd /home/vehicle_gateway && . /opt/ros/humble/local_setup.sh && colcon build --merge-install --event-handlers console_direct+ --cmake-args -DBUILD_TESTING=0

RUN apt-get install -y -q ros-humble-ros2launch
COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
