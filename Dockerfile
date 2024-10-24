# Use a base image with necessary tools
FROM ubuntu:22.04

# Set environment variables to non-interactive (prevents prompts during installation)
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    libeigen3-dev \
    libglfw3-dev \
    libzip-dev \
    libboost-dev \
    libx11-dev \
    libomp-dev \
    git \
    wget \
    unzip \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Create a working directory
WORKDIR /app

# Clone the project repository with submodules
RUN git clone --recursive-submodules https://github.com/ETSTribology/primal_dual_contacts_miror.git .

# Create build directory and run CMake to configure the project
RUN mkdir -p build \
    && cd build \
    && cmake ../ \
    && make -j

# Set the working directory to the built binaries
WORKDIR /app/build/Release/bin

# Default command to run the GUI application (you can adjust based on needs)
CMD ["./ContactSimulation"]
