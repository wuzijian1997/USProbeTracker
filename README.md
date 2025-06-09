## US Probe Tracker

### Getting Started

This repository used vcpkg for dependency management. To get started, you need to install vcpkg and then install the
required dependencies. By default, it uses CLion build-in vcpkg integration, but you can also use the command line.

#### Update vcpkg

1. Go to the vcpkg directory:

```bash
cd C:\Users\{$USER}\.vcpkg-clion\vcpkg
git pull # update to the latest master
.\bootstrap-vcpkg.bat # bootstrap vcpkg
```

2. Under the project root directory, update the vcpkg baseline

```bash
C:\Users\{$USER}\.vcpkg-clion\vcpkg\vcpkg.exe x-update-baseline
```

Features of specific libraries
- [opencv](https://vcpkg.io/en/package/opencv)
  - "features" : [ "gstreamer", "ffmpeg", "tbb", "openmp", "msmf", "highgui"]

#### Screen Capture with Scrcpy

Reference Docs:
- [Scrcpy](https://github.com/Genymobile/scrcpy/tree/master)
- [Scrcpy - Connection](https://github.com/Genymobile/scrcpy/blob/master/doc/connection.md#tcpip-wireless)
- [Scrcpy - Windows](https://github.com/Genymobile/scrcpy/blob/master/doc/windows.md#run)
- [Android Debug Bridge (ADB)](https://developer.android.com/tools/adb#wireless)

Steps:
1. Download the latest Scrcpy release
2. Connect the tablet to your computer via USB with developer mode enabled
3. Connect the tablet with the same local network
4. Run the following command to enable TCP/IP mode:

```bash
./scrcpy.exe # Init ADB connection
adb tcpip 5555 # Enable TCP/IP mode
./scrcpy.exe --tcpip=<IP_ADDRESS> # Connect to the tablet wirelessly
```