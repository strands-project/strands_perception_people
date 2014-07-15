# libcudaHOG 
This library is provided by the RWTH Aachen and will be downloaded from http://www.vision.rwth-aachen.de/projects/

This directory contains a cmake wrapper file that will take care of downloading, unpacking, building and installing the tar-ball. It will also create a pkg-config file to allow cmake to find it in other projects.
At the moment this library is only needed to build the ground_hog package.

## Dependencies
* NVIDIA - CUDA: Please follow instructions: http://developer.download.nvidia.com/compute/cuda/5_5/rel/docs/CUDA_Getting_Started_Linux.pdf or go directly to: https://developer.nvidia.com/cuda-downloads to find the latest version (tested with CUDA 5.5 (nvidia-319) and CUDA 6.0 (nvidia-331))
	* This requires a NVIDIA graphics card
	* Make sure to follow the instructions especially the part about exporting the PATH and LD_LIBRARY_PATH variables. Add these statements to your `.bashrc`.
* QT4
* qmake

## Installation
As mentioned the cmake file will take care of almost everything. Just follow these simple instructions:
* Change into the `strands_perception_people/3rd_party` directory
* Create a build directory to keep it clean: `mkdir build; cd build`
* Run cmake: `cmake ..` _This will install everything to /usr/local and requires sudo rights._
	* To install it in a custom location: `cmake .. -DCMAKE_INSTALL_PREFIX=/my/path/`
	* If you choose to install it in a custom location, you have to make sure that pkg-config finds it: `export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/my/path/lib/pkgconfig` _You can add this to your .bashrc because other projects may need to find it during compile time. If you installed the library to `/usr/local`, you do not have to export any paths._
* Run make: `make`. _This will download, unpack and build the files._
* Install library and headers: `sudo make install` _Omit the sudo if you chose a custom destination that does not require sudo rights._

## Troubleshooting
* If you are on a 32bit system, you will get an error when linking the library: `/usr/bin/ld: cannot find -lcudart`. To get rid of this you have to:
	* Edit the `build/libcudaHOG/src/libcudaHOG/cudaHOG/cudaHOG.pro` file. _Note: you have to run make before doing so because this will download and unzip the tarball. Otherwise you do not have that file._ 
	* Find the line that says: `LIBS += -lcudart -L/usr/local/cuda/lib64` and change it to: `LIBS += -lcudart -L/usr/local/cuda/lib`.
	* Now run `make` again.
