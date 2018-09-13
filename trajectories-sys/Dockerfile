FROM rust:1.28-jessie
# FROM ubuntu:trusty

COPY . /opt
WORKDIR /opt

# ENV DEBIAN_FRONTEND=noninteractive

RUN echo 'deb http://ppa.launchpad.net/ubuntu-toolchain-r/test/ubuntu trusty main ' >> /etc/apt/sources.list && \
	apt-get --allow-unauthenticated update

RUN apt-get --allow-unauthenticated install -y pkg-config gcc-5 g++-5 curl libclang-3.5-dev

RUN curl http://de.archive.ubuntu.com/ubuntu/pool/universe/e/eigen3/libeigen3-dev_3.3~beta1-2_all.deb >> libeigen3.deb && \
	dpkg -i libeigen3.deb

# RUN curl https://sh.rustup.rs -sSf | sh -s -- -y && $HOME/.cargo/env

RUN cargo build --release

RUN cargo test --release