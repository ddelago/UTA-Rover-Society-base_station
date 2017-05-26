# cmake needs this line
cmake_minimum_required(VERSION 2.8)

project(opencv_examples)

find_package(OpenCV REQUIRED)

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

if(CMAKE_VERSION VERSION_LESS "2.8.11")
  # Add OpenCV headers location to your include paths
  include_directories(${OpenCV_INCLUDE_DIRS})
endif()

# Declare the executable target built from your sources
add_executable(bin/video_brightness src/video_brightness.cpp)
target_link_libraries(bin/video_brightness ${OpenCV_LIBS})

add_executable(bin/webcam_example src/webcam_example.cpp)
target_link_libraries(bin/webcam_example ${OpenCV_LIBS})

add_executable(bin/contrast_example src/contrast_example.cpp)
target_link_libraries(bin/contrast_example ${OpenCV_LIBS})

add_executable(bin/color_detection src/color_detection.cpp)
target_link_libraries(bin/color_detection ${OpenCV_LIBS})

add_executable(bin/facedetect src/facedetect.cpp)
target_link_libraries(bin/facedetect ${OpenCV_LIBS})

add_executable(bin/histogram_eq src/histogram_eq.cpp)
target_link_libraries(bin/histogram_eq ${OpenCV_LIBS})

add_executable(bin/hist_eq_color src/hist_eq_color.cpp)
target_link_libraries(bin/hist_eq_color ${OpenCV_LIBS})

add_executable(bin/trackbar src/trackbar.cpp)
target_link_libraries(bin/trackbar ${OpenCV_LIBS})

add_executable(bin/mouse src/mouse.cpp)
target_link_libraries(bin/mouse ${OpenCV_LIBS})

add_executable(bin/image_rotation src/image_rotation.cpp)
target_link_libraries(bin/image_rotation ${OpenCV_LIBS})

add_executable(bin/video_rotation src/video_rotation.cpp)
target_link_libraries(bin/video_rotation ${OpenCV_LIBS})

add_executable(bin/eroding src/eroding.cpp)
target_link_libraries(bin/eroding ${OpenCV_LIBS})

#add_executable(bin/hough_circle_transform src/hough_circle_transform.cpp)
#target_link_libraries(bin/hough_circle_transform ${OpenCV_LIBS})

add_executable(bin/track_color src/track_color.cpp)
target_link_libraries(bin/track_color ${OpenCV_LIBS})

add_executable(bin/dilating src/dilating.cpp)
target_link_libraries(bin/dilating ${OpenCV_LIBS})

add_executable(bin/inverting src/inverting.cpp)
target_link_libraries(bin/inverting ${OpenCV_LIBS})

add_executable(bin/shape_detection src/shape_detection.cpp)
target_link_libraries(bin/shape_detection ${OpenCV_LIBS})

add_executable(bin/shape_detection2 src/shape_detection2.cpp)
target_link_libraries(bin/shape_detection2 ${OpenCV_LIBS})

add_executable(bin/camshiftdemo src/camshiftdemo.cpp)
target_link_libraries(bin/camshiftdemo ${OpenCV_LIBS})

add_executable(bin/houghcircles src/houghcircles.cpp)
target_link_libraries(bin/houghcircles ${OpenCV_LIBS})

add_executable(bin/caffe_googlenet src/caffe_googlenet.cpp)
target_link_libraries(bin/caffe_googlenet ${OpenCV_LIBS})
