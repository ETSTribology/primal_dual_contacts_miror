FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive

# Install required dependencies in a single layer
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

# Set up work directory
WORKDIR /app

# Clone the repository without submodules
RUN git clone https://github.com/ETSTribology/primal_dual_contacts_miror primal-dual

# Initialize and update submodules
WORKDIR /app/primal-dual
RUN git submodule init && git submodule update --recursive

# Build the project
RUN mkdir build && cd build \
    && cmake -DBUILD_NIX_PACKAGE=OFF .. \
    && make -j

# Set binary directory as the working directory
WORKDIR /app/primal-dual/build/Release/bin

# Define entrypoint for the main application
CMD ["./CLIContactSimulation"]
