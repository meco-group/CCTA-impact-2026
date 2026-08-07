# CCTA Impact Workshop 2026
Numerical Optimization and Model Predictive Control: A Hands On Journey with CasADi and Impact.

## Introduction

This repository contains source of training material for the `Impact Workshop 2026`.

Program to be announced.

## Software installation

The easiest way to follow and complete all assignments in this hands-on workshop is to bring a machine with linux or WSL, along with Docker.

### Furuta Pendulum Assignment
This repo contains a `Dockerfile` and a `docker-compose.yml`. To pull the required image, navigate to `furuta_pendulum` and run

```
docker compose up -d
```

This can take a few minutes. Within the container, navigate to /home/furuta-stepper-cpp/.

If you prefer not to use docker and do the installation yourself, you will need the following python packages
```
python -m pip install numpy matplotlib pandas scipy control pin meshcat
python -m pip install impact-meco
```
We install a newer version of CasADi compared to the one that is shipped with impact-meco to obtain a newer and more robust version of the FATROP solver. This will automatically be included in the next pip release of impact-meco.
```
python -m pip install --no-deps https://github.com/casadi/casadi/releases/download/nightly-lacemodelica/casadi-3.7.2.dev+lacemodelica-cp312-none-manylinux2014_x86_64.whl
```

To obtain the code to prototype the furuta pendulum controller, clone the following repo
```
git clone https://gitlab.kuleuven.be/meco-setups/furuta-stepper-cpp.git
cd furuta-stepper-cpp
git checkout minimal
```

To use ImpaC++t and simulate a real-time system, some source builds are required (we assume you have a linux/WSL system)
Get a CasADi source build (for use from within C++ scripts)
```
apt install gfortran swig libmetis-dev -y
git clone https://github.com/casadi/casadi.git && cd casadi && mkdir build
cd casadi/build && cmake \
    -DWITH_METIS=ON \
    -DWITH_OPENMP=ON \
    -DWITH_THREAD=ON ..
cd casadi/build && make && make install
ldconfig
apt-get install libeigen3-dev -y

# To make sure that the C++ casadi (simulator) finds the source build, but impact-mpc finds pip installed casadi with fatrop.
export LD_LIBRARY_PATH="/usr/local/lib:/usr/local/lib/python3.12/site-packages/casadi:${LD_LIBRARY_PATH}"
```
Get SEOM and CoE SOEM (for interfacing with the hardware)
```
git clone https://github.com/OpenEtherCATsociety/SOEM.git
cd SOEM
git checkout 7271d36
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local ..
make -j
make install

cd ../..
git clone https://gitlab.kuleuven.be/meco-software/coe-soem.git
cd coe-soem
mkdir build
cd build
cmake .. -DBUILD_TESTS=ON
make -j
make install
```
and get ImpaC++t
```
git clone https://gitlab.kuleuven.be/meco-software/impact-cpp.git
cd impact-cpp
git checkout debug_louis
mkdir build
cd build
cmake .. -DBUILD_EXAMPLES=ON -DBUILD_TESTS=ON -DENABLE_RT=OFF
make -j
make install
```
