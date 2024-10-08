# RP2040-FreeRTOS Template 1.4.2

This repo contains my base project for [FreeRTOS](https://freertos.org/) on the [Raspberry Pi RP2040 microcontroller](https://www.raspberrypi.com/products/rp2040/). It can be run as a demo and then used as the basis of a new project.

More details [in this blog post](https://blog.smittytone.net/2022/02/24/how-to-use-freertos-with-the-raspberry-pi-pico/).

## Project Structure

```
/RP2040-FreeRTOS
|
|___/App-DiffDrive          // Differential driver controller
|   |___CMakeLists.txt      // Application-level CMake config file
|
|___/Common                 // Source code common to applications 2-4 (C++)
|
|___/Config
|   |___FreeRTOSConfig.h    // FreeRTOS project config file
|
|___/FreeRTOS-Kernel        // FreeRTOS kernel files, included as a submodule
|___/pico-sdk               // Raspberry Pi Pico SDK, included as a submodule
|
|___CMakeLists.txt          // Top-level project CMake config file
|___pico_sdk_import.cmake   // Raspberry Pi Pico SDK CMake import script
|___deploy.sh               // Build-and-deploy shell script
|
|___rp2040.code-workspace   // Visual Studio Code workspace
|___rp2040.xcworkspace      // Xcode workspace
|
|___README.md
|___LICENSE.md
```

## Prerequisites

To use the code in this repo, your system must be set up for RP2040 C/C++ development. See [this blog post of mine](https://blog.smittytone.net/2021/02/02/program-raspberry-pi-pico-c-mac/) for setup details.

## Usage

1. Clone the repo: `git clone --recursive https://github.com/CNHGit/DiffDriveBaseControlelr_RP2040`.
1. Enter the repo: `cd DiffDriveBaseControlelr_RP2040`.
1. Install the submodules: `git submodule update --init --recursive`.
1. Optionally, manually configure the build process: `cmake -S . -B build/`.
1. Optionally, manually build the app: `cmake --build build`.
1. Connect your device so it’s ready for file transfer.
1. Install the app: `./deploy.sh`.
    * Pass the app you wish to deplopy:
        * `./deploy.sh build/App-DiffDrive/DIFF_DRIVE.uf2`.
    * To trigger a build, include the `--build` or `-b` flag: `./deploy.sh -b`.

## Debug vs Release

You can switch between build types when you make the `cmake` call in step 5, above. A debug build is made explicit with:

```shell
cmake -S . -B build -D CMAKE_BUILD_TYPE=Debug
```

For a release build, which among various optimisations omits UART debugging code, call:

```shell
cmake -S . -B build -D CMAKE_BUILD_TYPE=Release
```

Follow both of these commands with the usual

```shell
cmake --build build
```


## The Apps

### App-DiffDrive

This C++ app provides a Differentail drive controller with Quadrature encoder support and BCM6050 IMU

#### Circuit Diagram
The Circuit diagram is as shown in the figure below.

![Circuit layout](./images/CircuitDiagram.png)

#### Activity Diagram
![Activity Diagram](./images/ActivityDiagram.png)

#### Sequence Diagram
This diagram shows different parallel tasks being executed in the controller.
* Reading encoder interrupts
* Handling serial commands read and state write
* PID control loop
![Sequence Diagram](./images/SequenceDiagram.png)


