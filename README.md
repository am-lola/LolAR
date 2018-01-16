# LolAR

Augmented-Reality Visualization for Lola

# Dependencies

* [ARVisualizer](https://github.com/am-lola/ARVisualizer)
* [OpenCV 3+](http://opencv.org/) and the [contrib module](https://github.com/Itseez/opencv_contrib)
  * Instructions: http://docs.opencv.org/3.1.0/d7/d9f/tutorial_linux_install.html
  * Be sure to install `libgtk2.0-dev` and `pkg-config` before building OpenCV
* [PCL](http://pointclouds.org/), compiled with C++11 support (NOT a packaged release!)
  * Instructions: http://www.pointclouds.org/documentation/tutorials/compiling_pcl_posix.php (follow the instructions under `experimental`)
  * NOTE: Before calling `cmake`, you will *also* need to add the following to PCL's `CMakeLists.txt` in order to ensure it builds with C++11 support:

      `SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")`
  * For detailed instructions, see lepp wiki: https://gitlab.lrz.de/AMCode/lepp3/wikis/Installation%20Instructions
