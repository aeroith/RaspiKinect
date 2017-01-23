# RaspiKinect

[IN DEVELOPMENT]
This repository holds the code for the EE491 - Project class "Full 3D Model Reconstruction using Multiple Kinects"
Project is still under development. The purpose of this repository is to show progress of the project.

To drive Multiple Kinects, Raspberry Pi's are used for each Kinect. Each Raspberry Pi is accessed using SSH. Currently
only Kinect v1 is supported. The data acquired by Raspberry Pi is then converted into Point Cloud using Point Cloud Library at the server.

# Build Instructions

To build the project there are two parts of the project to take into consideration. One is Raspberry Pi part and the other one is
the local part.

## Raspberry Pi

### Prerequisites

To build the kinectRun you'll need:

- [libfreenect](https://github.com/OpenKinect/libfreenect)
- [Boost](http://www.boost.org/)

### Build

        git clone https://github.com/aeroith/RaspiKinect
        cd src/raspi
        mkdir build
        cd build
        cmake ..
        make
        # it is suggested that you put the executable at home dir
        cp kinectRun /home/${USER}

## Local part

### Prerequisites

- [Boost](http://www.boost.org/)
- [Point Cloud Library](http://pointclouds.org/)
- [LibSSH](https://www.libssh.org/)
- [OpenMP](http://www.openmp.org/)

### Build

        git clone https://github.com/aeroith/RaspiKinect
        cd src/
        mkdir build
        cd build
        cmake ..
        make

# Licensing

OpenKinect is released under a dual Apache v2/GPL v2 license. PCL is released under BSD License.
Libssh is open source licensed under the GNU Library (or: Lesser) General Public License (LGPL).
All of the code here is released under Apache 2.0 License.