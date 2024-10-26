# Build stage
FROM ubuntu:22.04 AS build

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    curl \
    sudo \
    cmake \
    libeigen3-dev \
    libglfw3-dev \
    libzip-dev \
    libboost-dev \
    libx11-dev \
    build-essential \
    git \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
RUN git clone --recursive https://github.com/ETSTribology/primal_dual_contacts_miror primal-dual

WORKDIR /app/primal-dual
RUN git submodule update --init --recursive

RUN mkdir -p build

WORKDIR /app/primal-dual/build

RUN cmake .. -DCMAKE_BUILD_TYPE=Release
RUN make -j4

WORKDIR /app/primal-dual/build/Release/bin

CMD ["./CLIContactSimulation"]
