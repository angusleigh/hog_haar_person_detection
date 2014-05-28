hog_haar_person_detection_ros
=============================

A ROS Hydro package implementing the OpenCV HOG pedestrian detector and HAAR face detector.
Subscribes to an image being published, detects faces and standing pedestrians in the image,
draws the detections on a copy of the image and publishes results as a rostopic. Also displays
detections on an OpenCV window.

## Usage

* $ cd [your hydro catkin src folder]
* $ git clone https://github.com/angusleigh/hog_haar_person_detection_ros.git
* $ cd ..
* $ catkin_make
* $ roslaunch hog_haar_person_detection_ros hog_haar_person_detection_ros.launch

You will probably also have to edit the "image_topic" parameter in the launch file depending on your setup.

