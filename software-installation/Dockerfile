FROM python:3.12

USER root

SHELL ["/bin/bash", "-c"]

ARG DEBIAN_FRONTEND=noninteractive

# Get basics
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y build-essential cmake -y
RUN apt-get install git wget -y
RUN apt-get install python3-pip -y
RUN apt-get install x11-xserver-utils -y

RUN python -m pip install numpy matplotlib pandas scipy control pin meshcat

# Get impact
RUN python -m pip install impact-meco

# grab later casadi wheel (with new version of Fatrop)
RUN python -m pip install --no-deps https://github.com/casadi/casadi/releases/download/nightly-lacemodelica/casadi-3.7.2.dev+lacemodelica-cp312-none-manylinux2014_x86_64.whl

# get furuta pendulum repository
WORKDIR /home
RUN git clone https://gitlab.kuleuven.be/meco-setups/furuta-stepper-cpp.git
WORKDIR /home/furuta-stepper-cpp
RUN git checkout minimal

##########################################################
### Remaining requirements for C++ implementation only ###
##########################################################
# Casadi from source (for use in C++)
WORKDIR /opt
RUN apt install gfortran swig libmetis-dev -y
RUN git clone https://github.com/casadi/casadi.git && cd casadi && mkdir build
RUN cd casadi/build && cmake \
    -DWITH_METIS=ON \
    -DWITH_OPENMP=ON \
    -DWITH_THREAD=ON ..
RUN cd casadi/build && make && make install
RUN ldconfig

RUN apt-get install libeigen3-dev -y

# To make sure that the C++ casadi (simulator) finds the source build, but impact-mpc finds pip installed casadi with fatrop.
ENV LD_LIBRARY_PATH="/usr/local/lib:/usr/local/lib/python3.12/site-packages/casadi:${LD_LIBRARY_PATH}"

# get SOEM
WORKDIR /opt
RUN git clone https://github.com/OpenEtherCATsociety/SOEM.git
WORKDIR /opt/SOEM
RUN git checkout 7271d36
RUN mkdir build
WORKDIR /opt/SOEM/build
RUN cmake -DCMAKE_INSTALL_PREFIX=/usr/local ..
RUN make -j
RUN make install

# get CoE SOEM
WORKDIR /opt
RUN git clone https://gitlab.kuleuven.be/meco-software/coe-soem.git
WORKDIR /opt/coe-soem
RUN mkdir build
WORKDIR /opt/coe-soem/build
RUN cmake .. -DBUILD_TESTS=ON
RUN make -j
RUN make install

# get impac++t (used for C++ implementations)
WORKDIR /home
RUN git clone https://gitlab.kuleuven.be/meco-software/impact-cpp.git
WORKDIR /home/impact-cpp
RUN git checkout debug_louis
RUN mkdir build
WORKDIR /home/impact-cpp/build
RUN cmake .. -DBUILD_EXAMPLES=ON -DBUILD_TESTS=ON -DENABLE_RT=OFF
RUN make -j
RUN make install

# bring us home
WORKDIR /home/furuta-stepper-cpp