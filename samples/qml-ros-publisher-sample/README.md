### qml-ros-publisher-sample

Example use of qml-ros-publisher. Follow the build
instructions in [the qml-ros-publisher README](../../README.md) before trying to run this sample. It has been tested with
Qt 5.11 on the following:
  - ArchLinux (rolling)
  - Android 6.0.1 (arm-v7) built with SDK API 18 and NDK r10e on ArchLinux host


### Prerequisites
 - ROS libraries and their headers. If cross-compiling for arm-v7, follow the instructions [here](http://wiki.ros.org/android_ndk/Tutorials/BuildingNativeROSPackages) and adapt the `.pro` file using the `lib` and `include` directories found in `roscpp_android/output/target`.
 - Chili ROS message types. Clone [this](https://github.com/chili-epfl/ros-chili-msgs) repo inside your catkin workspace's `src` directory and run `catkin_make` from the root. Then add the `devel/include` directory to your compiler's search path (or copy it's contents to your ROS `include` directory).

### Building
```
mkdir build
cd build
qmake ..
make
```
