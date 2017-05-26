# This directory is used to test computer vision and autonomous capabilities before they are to 
# be implemented into ROS packages. To use the example code, make sure to have the following dependencies:

OpenCV
- The latest version of OpenCV
- At the time this was used, OpenCV 3.2.0-dev was the latest.
- You will need the development branch of OpenCV as well. This branch contains all of the latest and 
  newest packages by OpenCV such as deep learning.
- To properly install both, follow this tutorial: https://www.youtube.com/watch?v=JLohXMh6XZY
- Once properly installed, you can compile the directory using: make
- The executables will all be in the /bin directory. 
- When creating new examples, place them in the /src directory and add it to the CMakeLists.txt file and make

DeepLearning
- This is a very rapidly advancing field in the field of computer science. 
- It allows for machines to create models off of data sets using deep neural networks.
- It is able to create models unsupervised, this means that the user does not have to 
  initialy assign any features to begin with. All that is needed is a data set from the user.
- To learn more about Deep learning, here are some resources:
  - http://caffe.berkeleyvision.org/
  - http://docs.opencv.org/trunk/d5/de7/tutorial_dnn_googlenet.html (Read this to understand how to use the deeplearning example)
  - http://neuralnetworksanddeeplearning.com/index.html
  - https://www.mathworks.com/videos/object-recognition-deep-learning-and-machine-learning-for-computer-vision-121144.html
  - https://developer.nvidia.com/digits (DIY example)
  - http://demo.caffe.berkeleyvision.org/ (Deep Learning Demo)
- Once you understand what Deep Learning is, you can run the example by ./caffe_googlenet

Build Instructions:
- Within the Computer Vision directory, if this is the first time that this package is being built, ensure that
  there are no cmake files or directories. 
- The only contents that should exist with a prebuilt package should be a bin directory, src directory, CMakeLists.txt, Makefile, and READ_ME.txt
- Remove any other files. 
- To build, simply run: 
  cmake .
  make
- All examples should then be built and executables stored within the bin directory.
