FROM ubuntu:14.04

# install bootstrap
#   and dev tools
RUN apt-get update && \
    apt-get install -y \
      git \
      cmake \
      libboost-all-dev

ENV WS=/root/ws

# download and unzip gtsam
ENV GTSAM_VERSION 3.2.1
RUN mkdir -p $WS/gtsam
WORKDIR $WS
RUN git clone https://bitbucket.org/gtborg/gtsam.git && \
    git checkout b7c695fa71efd43b40972eec154df265617fc07d -b admm

# build and install gtsam
RUN mkdir -p $WS/gtsam/build
WORKDIR $WS/gtsam/build
RUN cmake .. && \
    make -j8 && \
    make install && \
    ldconfig

# clone admm-slam
WORKDIR $WS
RUN git clone https://github.com/itzsid/admm-slam.git

# build and install admm-slam
RUN mkdir -p $WS/admm-slam/cpp/build
WORKDIR $WS/admm-slam/cpp/build
RUN cmake .. && \
    make -j8 && \
    make check && \
    make install && \
    ldconfig
