# syntax=docker/dockerfile:1
ARG FROM_IMAGE=dustynv/ros:iron-pytorch-l4t-r35.3.1
ARG UNDERLAY_WS=/opt/acfr/underlay_ws
ARG OVERLAY_WS=/opt/acfr/overlay_ws

# multi-stage for building
FROM $FROM_IMAGE AS base

# config dependencies install
ARG DEBIAN_FRONTEND=noninteractive
RUN rosdep update
ENV PYTHONUNBUFFERED 1

# install CI dependencies
ARG RTI_NC_LICENSE_ACCEPTED=yes
# Stop docker from deleting the cached deb packages
RUN rm -f /etc/apt/apt.conf.d/docker-clean; echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/03keep-cache
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y \
    ccache \
    dh-make \
    fakeroot \
    lcov \
    lld \
    python3-bloom \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-pip \
    python3-vcstool \
    python3-wheel \
    openssh-client \
    iputils-ping \
    && pip3 install wheel setuptools numpy scipy Jetson.GPIO schedule \
    && pip3 install \
    git+https://github.com/ruffsl/colcon-cache.git \
    git+https://github.com/ruffsl/colcon-clean.git \
    && colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml \
    && colcon mixin update default \
    && colcon metadata update \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p ~/.ssh
RUN echo '\
    Host *\n\
    StrictHostKeyChecking no' >> ~/.ssh/config

# multi-stage for caching
FROM base AS cacher

# copy overlay source
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS/src
COPY ./ ./stack/
# If there is an overlay.repos file, clone the repos to build in the overlay alongside our packages
RUN --mount=type=ssh if [ -f stack/overlay.repos ]; then vcs import --recursive ./ < stack/overlay.repos; fi
# Run the overlay_post.sh script if it exists to tweak the downloaded repos if needed
RUN --mount=type=ssh if [ -f stack/overlay_post.sh ]; then . stack/overlay_post.sh; fi

# clone underlay source
ARG UNDERLAY_WS
WORKDIR $UNDERLAY_WS/src
# If there is an underlay.repos file, clone the repos to build in the underlay underneath our packages
RUN --mount=type=ssh if [ -f ${OVERLAY_WS}/src/stack/underlay.repos ]; then vcs import --recursive ./ < ${OVERLAY_WS}/src/stack/underlay.repos; fi
# Run the underlay_post.sh script if it exists to tweak the downloaded repos if needed
RUN --mount=type=ssh if [ -f ${OVERLAY_WS}/src/stack/underlay_post.sh ]; then . ${OVERLAY_WS}/src/stack/underlay_post.sh; fi

# copy manifests for caching
WORKDIR /opt/acfr
RUN find . -name "src" -type d \
    -mindepth 1 -maxdepth 2 -printf '%P\n' \
    | xargs -I % mkdir -p /tmp/opt/acfr/% && \
    find . -name "package.xml" \
    | xargs cp --parents -t /tmp/opt/acfr && \
    find . -name "COLCON_IGNORE" \
    | xargs cp --parents -t /tmp/opt/acfr || true

# multi-stage for building
FROM base AS builder

SHELL ["/bin/bash", "-c"]

# install underlay dependencies
ARG UNDERLAY_WS
ENV UNDERLAY_WS $UNDERLAY_WS
WORKDIR $UNDERLAY_WS
RUN find src -mindepth 1 -maxdepth 1 -type d -print0 | xargs -0 -I "{}" touch "{}"/COLCON_IGNORE
COPY --from=cacher /tmp/$UNDERLAY_WS ./
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    source /ros_entrypoint.sh && \
    apt-get update && rosdep install -q -y \
    --from-paths src \
    --ignore-src \
    -r -i --rosdistro iron \
    --skip-keys=libopencv-dev \
    --skip-keys=libopencv-imgproc-dev \
    && rm -rf /var/lib/apt/lists/*

# build underlay source
COPY --from=cacher $UNDERLAY_WS ./
ARG UNDERLAY_MIXINS="release ccache build-testing-off"
ARG CCACHE_DIR="$UNDERLAY_WS/.ccache"
RUN source /ros_entrypoint.sh && \
    colcon cache lock && \
    colcon build \
    --symlink-install \
    --mixin $UNDERLAY_MIXINS \
    --event-handlers console_direct+

# install overlay dependencies
ARG OVERLAY_WS
ENV OVERLAY_WS $OVERLAY_WS
WORKDIR $OVERLAY_WS
COPY --from=cacher /tmp/$OVERLAY_WS ./
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    source $UNDERLAY_WS/install/setup.bash && \
    apt-get update && rosdep install -q -y \
    --from-paths src \
    --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# multi-stage for testing
FROM builder AS tester

# build overlay source
COPY --from=cacher $OVERLAY_WS ./
ARG OVERLAY_MIXINS="release ccache"
ARG CCACHE_DIR="$OVERLAY_WS/.ccache"
RUN source $UNDERLAY_WS/install/setup.bash && \
    colcon cache lock && \
    colcon build \
    --symlink-install \
    --mixin $OVERLAY_MIXINS

RUN echo 'echo sourcing underlay and overlay ros workspace' > /opt/acfr/extra_ros_ws.sh && \
    if [ -f $UNDERLAY_WS/install/setup.bash ]; then echo source $UNDERLAY_WS/install/setup.bash >> /opt/acfr/extra_ros_ws.sh; fi && \
    if [ -f $OVERLAY_WS/install/setup.bash ]; then echo source $OVERLAY_WS/install/setup.bash >> /opt/acfr/extra_ros_ws.sh; fi && \
    echo 'exec "$@"' >> /opt/acfr/extra_ros_ws.sh && \
    chmod +x /opt/acfr/extra_ros_ws.sh

# test overlay build
ARG RUN_TESTS
ARG FAIL_ON_TEST_FAILURE=True
RUN if [ -n "$RUN_TESTS" ]; then \
    source install/setup.bash && \
    colcon test && \
    colcon test-result --verbose \
    || ([ -z "$FAIL_ON_TEST_FAILURE" ] || exit 1) \
    fi

# Use this to run the container using docker compose
FROM tester AS compose
ENTRYPOINT ["/ros_entrypoint.sh", "/opt/acfr/extra_ros_ws.sh"]
CMD ["bash"]
