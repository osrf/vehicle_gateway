FROM dustynv/ros:humble-desktop-l4t-r35.3.1

# Or use the following image for the full desktop version (includes rviz, demos, tutorials, etc.):
# FROM dustynv/ros:humble-ros-base-l4t-r32.7.1

ENV VEHICLE_GATEWAY_WS /root/vg
WORKDIR ${VEHICLE_GATEWAY_WS}

# Install general dependencies
RUN \
	apt-get update && \
	apt-get install -y curl && \
	curl https://sh.rustup.rs -sSf | sh -s -- -y && \
	apt-get install -y \
		python3-jinja2 \
		python3-jsonschema \
		gcc-arm-none-eabi \
		libfuse2 \
		python3-pip \
		git \
		python3-vcstool \
		python3-future \
		rsync \
		&& \
	pip3 install \
		pyros-genmsg \
		kconfiglib \
		&& \
	apt-get clean

# Get source code for the vehicle gateway workspace
RUN \
	mkdir -p ${VEHICLE_GATEWAY_WS}/src && \
	cd ${VEHICLE_GATEWAY_WS}/src && \
	git clone https://github.com/osrf/vehicle_gateway && \
	cd ${VEHICLE_GATEWAY_WS} && \
	vcs import src < src/vehicle_gateway/dependencies.repos

# Install dependencies for the vehicle gateway workspace
RUN \
	cd ${VEHICLE_GATEWAY_WS} && \
	. /opt/ros/humble/install/setup.bash && \
	rosdep update && \
	rosdep install \
		--from-paths src \
		--ignore-src \
		-y \
		--skip-keys="gz-transport12 gz-common5 gz-math7 gz-msgs9 gz-gui7 gz-cmake3 gz-sim7 zenohc xacro sdformat_urdf image_transport_plugins control_toolbox gps_msgs aruco_opencv_msgs aruco_opencv joy rviz2 rqt_topic rqt_plot rqt_image_view"

# Build workspace
#
# See which packages will be built:
#   $ colcon graph --packages-up-to vehicle_gateway_python vehicle_gateway_models vehicle_gateway_multi
#     > px4_msgs                +    *.
#     > vehicle_gateway          +  **.
#     > vehicle_gateway_models    +
#     > zenohc                     +*
#     > vehicle_gateway_multi       +
#     > vehicle_gateway_px4          +*
#     > vehicle_gateway_python        +
RUN \
	. /ros_entrypoint.sh && \
	. "${HOME}/.cargo/env" && \
	colcon build \
		--event-handlers console_direct+ \
		--packages-up-to \
			vehicle_gateway_python \
			vehicle_gateway_models \
			vehicle_gateway_multi
