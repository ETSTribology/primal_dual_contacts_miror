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
    libomp-dev \
    git \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

RUN git clone --recursive https://github.com/ETSTribology/primal_dual_contacts_miror primal-dual

WORKDIR /app/primal-dual
RUN mkdir -p build && cd build \
    && cmake -DBUILD_NIX_PACKAGE=OFF .. \
    && make -j

FROM ubuntu:22.04 AS runtime

RUN apt-get update && apt-get install -y \
    libeigen3-dev \
    libglfw3-dev \
    libzip-dev \
    libboost-dev \
    libx11-dev \
    libomp-dev \
    && rm -rf /var/lib/apt/lists/*

COPY --from=build /app/primal-dual/build/Release/bin /app/bin

WORKDIR /app/bin

CMD ["./CLIContactSimulation"]
