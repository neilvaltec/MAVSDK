#
# Development environment for MAVSDK based on Ubuntu 20.04.
#
# Author: Neil Chou <neil@valtec.io>
#

FROM ubuntu:20.04

ENV DEBIAN_FRONTEND noninteractive


RUN apt-get update \
    && apt-get -y --quiet --no-install-recommends install \
        autoconf \
        automake \
        autotools-dev \
        build-essential \
        ca-certificates \
        ccache \
        clang-format-10 \
        cmake \
        colordiff \
        doxygen \
        git \
        golang-go \
        libcurl4-openssl-dev \
        libltdl-dev \
        libtinyxml2-dev \
        libtool \
        libz-dev \
        ninja-build \
        python3 \
        python3-pip \
        python3-future \
        ruby-dev \
        software-properties-common \
        sudo \
        wget \
    && apt-get -y autoremove \
    && apt-get clean autoclean \
    && rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*


WORKDIR "/workspace"

RUN git clone https://github.com/neilvaltec/MAVSDK.git

WORKDIR "/workspace/MAVSDK"

RUN git checkout valtec_dev_mavlink-message-direct-forward
RUN git submodule update --init --recursive && \
        cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_INSTALL_PREFIX=install -Bbuild/default -H. && \
        cmake --build build/default --target install

ENTRYPOINT ["/bin/bash"]

# cd examples/takeoff_and_land
# cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_PREFIX_PATH=$(pwd)/../../install -Bbuild -H.
# cmake --build build -j8

