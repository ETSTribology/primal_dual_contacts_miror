FROM ubuntu:22.04
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

RUN curl -L https://nixos.org/nix/install | sh

RUN . /root/.nix-profile/etc/profile.d/nix.sh

WORKDIR /app

RUN git clone --recursive-submodules git@git.ista.ac.at:yichen/primal-dual-friction-public.git

RUN nix-shell --run "nix-shell"

RUN mkdir build && cd build \
    && cmake -DBUILD_NIX_PACKAGE=ON .. \
    && make -j

WORKDIR /app/build/Release/bin

CMD ["./ContactSimulation"]
