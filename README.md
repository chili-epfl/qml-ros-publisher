qml-ros-publisher
===================

QML plugin for publishing messages to a ROS master. Tested with Qt 5.11 on
the following:

  - Android 6.0.1 (arm-v7) built with SDK API 18 and NDK r10e on ArchLinux host
  - ArchLinux (rolling)

Prerequisites
-------------

 - ROS libraries and their headers. If building for ARM, ROS libraries cross-compiled for arm-v7 and their headers. Follow the instructions [here](http://wiki.ros.org/android_ndk/Tutorials/BuildingNativeROSPackages) and adapt the `.pro` file using the `lib` and `include` directories found in `roscpp_android/output/target`.
 - Chili ROS message types. Clone [this](https://github.com/chili-epfl/ros-chili-msgs) repo inside your catkin workspace's `src` directory and run `catkin_make` from the root. Then add the `devel/include` directory to your compiler's search path (or copy it's contents to your ROS `include` directory).

build
-----

```
    $ mkdir build && cd build
    $ qmake ..
    $ make install
```

Make sure to use the correct `qmake` for your architecture.
