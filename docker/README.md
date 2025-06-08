# ROS2 Docker Development Environment

This setup provides a complete ROS2 development environment with CUDA support, multi-stage builds, Devcontainer integration for Visual Studio Code, and multiple Compose profiles.

---

## Overview

The Docker infrastructure consists of several files that together enable a flexible and reproducible development environment for ROS2 projects. It supports both classic Docker workflows and direct integration into VS Code via Devcontainer.

```
docker/
└── 📁docker
    └── bashrc
    └── dependencies_lite.bash
    └── dependencies.bash
    └── Dockerfile.cpu
    └── Dockerfile.cuda
    └── Dockerfile.lite
    └── entrypoint.sh
    └── README.md           # you are here
    └── requirement_lite.txt
    └── requirement.txt
└── 📁.devcontainer
    └── devcontainer.json
docker-compose.yml
docker-compose-devcontainer.yml
```
---

## Usage

### Build & Run (classic)

```bash
# cuda profile
docker compose -f docker-compose.yml --profile cuda build
docker compose -f docker-compose.yml --profile cuda up -d

# cpu profile
docker compose -f docker-compose.yml --profile cpu build
docker compose -f docker-compose.yml --profile cpu up -d
```

### VSCode Devcontainer

1. Open the project in VS Code.
2. Select "Reopen in Container".
3. The container will be built and started automatically.

---

## Important files

### 1. [Dockerfile.cuda](docker/Dockerfile.cuda)

Base Dockerfile for ROS2 with CUDA support and a non-root user.

- **Multi-stage build:** Separates base, build, and runtime stages.
- **CUDA & ROS2:** Uses an NVIDIA CUDA image and installs ROS2 Humble.
- **User:** Creates a non-root user (`ros`) for development.
- **Packages:** Installs all system and ROS2 dependencies.
- **Workspace:** Copies source code and builds the ROS2 workspace with `colcon`.

### 2. [Dockerfile.cpu](docker/Dockerfile.cpu)

Base Dockerfile for ROS2 (CPU only) with a non-root user.

- **No CUDA:** Uses a standard Ubuntu base image and installs ROS2 Humble.
- **User:** Creates a non-root user (`ros`) for development.
- **Packages:** Installs all system and ROS2 dependencies.
- **Workspace:** Copies source code and builds the ROS2 workspace with `colcon`.

### 3. [Dockerfile.lite](docker/Dockerfile.lite)

Minimal Dockerfile for lightweight ARM64 deployments (e.g., Raspberry Pi 4).

- **ARM64 only:** Designed for ARM64 architecture.
- **Minimal dependencies:** Installs only essential ROS2 packages and dependencies.
- **No build step:** Intended to be used with the pre-built image `mshahrour/ros2-lite:arm64` from Docker Hub.

### 4. [`docker-compose.yml`](/docker-compose.yml)

Docker Compose file for different profiles (cuda, cpu, lite).

- Enables starting different containers depending on the profile.
- Contains all relevant mounts, devices, and environment variables.
- For classic Docker workflows outside of VS Code.

### 5. [docker-compose-devcontainer.yml](/docker-compose-devcontainer.yml)

Docker Compose file for Devcontainer operation for different profiles (cuda, cpu) in VS Code.

- Defines the `ros2-cuda` & `ros2-cpu` services for CUDA and cpu development.
- Mounts relevant volumes and devices.
- Sets environment variables for ROS and X11.
- Referenced by `.devcontainer/devcontainer.json`.


### 6. [devcontainer.json](/.devcontainer/devcontainer.json)

Configures the VS Code Devcontainer.

- References the Compose file and service.
- Mounts the workspace directory.
- Installs recommended extensions.

---


## Notes

- For CUDA support, the host must have an NVIDIA GPU and the appropriate drivers installed (tested with NVIDIA-SMI 550.144.03, Driver Version: 550.144.03, CUDA Version: 12.4).
- The service `ros2-lite` with the `lite` profile in [`docker-compose.yml`](docker-compose.yml) is intended for ARM64 devices such as the Raspberry Pi 4. This service does **not** include a build step, as the container image `mshahrour/ros2-lite:arm64` is pre-built and available on Docker Hub. It is not meant to be built on AMD64 hosts, but rather to be pulled and run directly on ARM64 hardware.