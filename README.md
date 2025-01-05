# VideoToBag

ROS package to generate a rosbag from a standard video format. The default topic is `/camera/image_raw`, but you can change it to anything you like.

Tested in ROS Noetic and Ubuntu 20.04

## Installation

In your favourite cmake directory:
```
git clone https://github.com/eozkzoe/VideoToBag.git VideoToBag
cd VideoToBag
mkdir build && cd build
cmake ..
make
```

## Usage:
    
`rosrun VideoToBag VideoToBag -i <video_file> -o <bag_file> -f <frequency> [-n <topic>]`
  
 - `video_file`: Absolute path with file extension
 - `bag_file`: Absolute path to save the bag with .bag extension
 - `frequency`: Frames per second
 - `topic`: Where to publish the frames
