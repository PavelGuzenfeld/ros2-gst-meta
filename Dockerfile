# =============================================================================
# Stage 1: Build and run unit tests
# =============================================================================
FROM ros:jazzy AS builder

ENV DEBIAN_FRONTEND=noninteractive

# Install GStreamer dev packages and build tools
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        pkg-config \
        libgstreamer1.0-dev \
        libgstreamer-plugins-base1.0-dev \
        gstreamer1.0-tools \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /src
COPY . .

# Build with cmake (NOT colcon -- we need direct control over the build)
RUN bash -c '\
    source /opt/ros/jazzy/setup.bash && \
    cmake -B build -S . \
        -DCMAKE_BUILD_TYPE=Release \
        -DROS2_GST_META_BUILD_TESTS=ON \
    && cmake --build build --parallel "$(nproc)"'

# Run unit tests
RUN bash -c '\
    source /opt/ros/jazzy/setup.bash && \
    cd build && ctest --output-on-failure --timeout 60'

# Detect the GStreamer plugin directory (works on both x86_64 and aarch64)
RUN GST_PLUGIN_DIR=$(pkg-config --variable=pluginsdir gstreamer-1.0) && \
    echo "$GST_PLUGIN_DIR" > /tmp/gst_plugin_dir

# =============================================================================
# Stage 2: E2E integration test runner
# =============================================================================
FROM ros:jazzy AS test

ENV DEBIAN_FRONTEND=noninteractive

# Install GStreamer runtime + tools and ROS 2 demo/sensor message packages
RUN apt-get update && apt-get install -y --no-install-recommends \
        gstreamer1.0-tools \
        gstreamer1.0-plugins-base \
        gstreamer1.0-plugins-good \
        libgstreamer1.0-0 \
        ros-jazzy-rclcpp \
        ros-jazzy-sensor-msgs \
        ros-jazzy-std-msgs \
    && rm -rf /var/lib/apt/lists/*

# Detect the GStreamer plugin directory at runtime for this arch
RUN mkdir -p $(pkg-config --variable=pluginsdir gstreamer-1.0 2>/dev/null \
    || echo /usr/lib/$(dpkg-architecture -qDEB_HOST_MULTIARCH)/gstreamer-1.0)

# Copy the built plugin shared library to the correct arch-specific dir
# Use a shell to resolve the path dynamically
COPY --from=builder /src/build/libgstros2gstmeta.so /tmp/libgstros2gstmeta.so
RUN GST_PLUGIN_DIR=$(pkg-config --variable=pluginsdir gstreamer-1.0 2>/dev/null \
    || echo /usr/lib/$(dpkg-architecture -qDEB_HOST_MULTIARCH)/gstreamer-1.0) && \
    cp /tmp/libgstros2gstmeta.so "$GST_PLUGIN_DIR/" && \
    rm /tmp/libgstros2gstmeta.so

# Copy the E2E test script
COPY test/e2e_test.sh /opt/e2e_test.sh
RUN chmod +x /opt/e2e_test.sh

ENTRYPOINT ["/opt/e2e_test.sh"]
