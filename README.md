# Python bindings for ORB_SLAM2

This is a python binding for ORB_SLAM2. This project is inspired by [this](https://github.com/jskinn/ORB_SLAM2-PythonBindings.git) and contains some of their codes. The biggest difference is we use pybind11 instead of Boost-python, which makes this module more light weight and portable.

## Prerequisites

* Tested on Ubuntu 20.04
* gcc with C++11 support
* CMake >= 3.4 or Pip 10+
* Pip 10+

## Installation

First install ORB_SLAM2 as per instructed by the [original pybind repo](https://github.com/jskinn/ORB_SLAM2-PythonBindings.git). You could also clone my [repo](https://github.com/xingruiy/ORB_SLAM2.git) and checkout to the `pybind` branch.

Clone this repository and pip install. Note the `--recursive` option which is needed for the pybind11 submodule:

```bash
git clone --recursive https://github.com/xingruiy/py_orbslam2.git
pip install ./py_orbslam2
```

With the `setup.py` file included in this example, the `pip install` command will
invoke CMake and build the pybind11 module as specified in `CMakeLists.txt`.

## FAQ
1. `fatal error: numpy/ndarrayobject.h: No such file or directory`: install numpy with `sudo apt-get install python-numpy`