## Image Segmentation based 2D-3D Fusion for 3D Object Filtering, Segmentation and Recognition  (Editting)
This is the source code of segmentation based 2d-3d fusion for 3D Object filtering, segmentation and recognition.  This code is compiled under ubuntu 18.04 with cmake.

## Depenencies
* OpenCV  
* PCL
* Eigen
```c++
// step one
// step 1-1: download opencv from https://opencv.org/releases/
// step 1-2: install opencv
# mkdir build
# cd build
# cmake ..
# make -j8
# make install

// step two : install pcl
# sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
# sudo apt-get update
# sudo apt-get install libpcl-dev

// step three: install eigen
# sudo apt-get install libeigen3-dev
# sudo cp -r /usr/local/include/eigen3 /usr/include 
```

## Class Description  
* [*Orientations Class*]()  
【**Description**】 This class is used to generate suitable projection orientations.  
【&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;**Input**&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;】 upright orientation, initial vector, interval arc.  
【&nbsp;&nbsp;&nbsp;**Output**&nbsp;&nbsp;&nbsp;&nbsp;】 projecton orientations.  

* [*SegFSR Class*]()  
【**Description**】 This class realized the function for 3D object filtering, segmentation and recognition.


## File Descriptions  
* [*SegFSR.h, SegFSR.cpp*]()  
This file realize the main part.   
* [*global.h, global.cpp*]()   
Those files contain   
* [*BoundingBox.h, BoundingBox.cpp*]()  
* [*zBuffer.h, zBuffer.cpp*]()  


## Pipeline












