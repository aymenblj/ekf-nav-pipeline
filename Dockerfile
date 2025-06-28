FROM ubuntu:24.04

# Install build tools and dependencies for OpenGL GUI
RUN apt-get update && \
    apt-get install -y \
        ninja-build \
        build-essential \
        cmake \
        git \
        libgl1-mesa-dev \
        libx11-dev \
        libxrandr-dev \
        libxinerama-dev \
        libxcursor-dev \
        libxi-dev \
        libglu1-mesa-dev \
        doxygen \
        pkg-config \
        wget && \
    rm -rf /var/lib/apt/lists/*

# Set the working directory in the container
WORKDIR /app

# Copy all project files to the container
COPY . .

# Configure and build the project
RUN cmake -S . -B build -G Ninja
RUN cmake --build build

# Set up the default command
ENTRYPOINT ["build/NavStream"]
CMD []