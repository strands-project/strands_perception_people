### libcudaHOG installation
This library is provided by the RWTH Aachen and will be downloaded from http://www.vision.rwth-aachen.de/projects/
This directory contains a cmake wrapper file that will take care of downloading, unpacking, building and installing the tar-ball. It will also create a pkg-config file to allow cmake to find it in other projects.
At the moment this library is only needed to build the ground_hog package.

## Dependencies
* NVIDIA - CUDA: Please follow instructions: http://developer.download.nvidia.com/compute/cuda/5_5/rel/docs/CUDA_Getting_Started_Linux.pdf or go directly to: https://developer.nvidia.com/cuda-downloads to find the latest version (tested with version 5.5)
	* This requires a NVIDIA graphics card
	* Make sure to follow the instructions espacially the part about exporting the PATH and LD_LIBRARY_PATH variables. Add these statements to your `.bashrc`.
* qmake (Qt4)

## Installation
As mentioned the cmake file will take care of almost everything. Just follow these simple instructions:
* Change into the `strands_perception_people/3rd_party` directory
* Create a build directory to keep it clean: `mkdir build; cd build`
* Run cmake: `cmake ..` _This will install everything to /usr/local and requires sudo rights._
	* To install it in a custom location: `cmake .. -DCMAKE_INSTALL_PREFIX=/my/path/`
* Run make: `make`. _This will download, unpack and build the files._
* Install library and headers: `sudo make install` _Omit the sudo if yoy chose a custom destination that does not require sudo rights._
