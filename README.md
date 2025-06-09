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
