qml-ros
===================

QML plugin for publishing messages to a ROS master. Tested with Qt 5.10.0 on
the following:

  - Android 6.0.1 (arm-v7) built with SDK API 18 and NDK r10e on ArchLinux host

build
-----

```
    $ mkdir build && cd build
    $ qmake ..
    $ make install
```

Make sure to use the `qmake` binary from the arm-v7 Qt installation.
