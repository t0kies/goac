# GOAC (gif-overlay-aruco-cpp)

This project uses OpenCV to detect ArUco markers and overlay a GIF onto the detected marker. Follow the steps below to set up and build this project on Windows.

---

## Prerequisites
- [OpenCV](https://opencv.org/)
- [CMake](https://cmake.org/)



## Installation

### Manual Installation

1. Install prerequisites
```bash
sudo apt-get install libopencv-dev cmake
```
2. Clone the repository
```bash
git clone https://github....
```

3. Build the project
```bash
mkdir build && cd build
cmake ..
make
```

4. Run the project 
From the build directory, run the executable
```bash
cd ..
./build/goac
```

### On an Ubuntu-VM with script
1. Clone the repository
```bash
git clone 
```

2. Run the installation script
```bash
sudo sh install.sh
```

3. Run the project
```bash
./build/goac
```



