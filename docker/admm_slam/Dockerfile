FROM ubuntu:14.04

# install bootstrap
#   and dev tools
RUN apt-get update && \
    apt-get install -y \
      build-essential \
      cmake \
      libboost-all-dev \
        bash-completion \
        byobu \
        git \
        less \
        tree \
        wget && \
    rm -rf /var/lib/apt/lists/*

ENV WS=/root/ws

# download and unzip gtsam
ENV GTSAM_VERSION 3.2.1
RUN mkdir -p $WS/gtsam
WORKDIR $WS
RUN wget https://research.cc.gatech.edu/borg/sites/edu.borg/files/downloads/gtsam-${GTSAM_VERSION}.tgz && \
    tar -xvzf gtsam-${GTSAM_VERSION}.tgz --strip=1 -C gtsam/ && \
    rm gtsam-${GTSAM_VERSION}.tgz

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
