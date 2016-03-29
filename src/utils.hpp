#ifndef _UTILS_H
#define _UTILS_H

#include <opencv2/opencv.hpp>

// Converts degrees to radians.
#define RADIANS(angleDegrees) (angleDegrees * M_PI / 180.0)

// Converts radians to degrees.
#define DEGREES(angleRadians) (angleRadians * 180.0 / M_PI)

/*
  Copies the raw data from the given cv::Mat to dst_array
  Based on the sample code from: docs.opencv.org/2.4/doc/tutorials/core/how_to_scan_images/how_to_scan_images.html

  Returns True on success, False if the conversion could not be performed

  NOTE: dst_array must already be allocated with enough space to store the image!
*/
bool Mat2Arr(cv::Mat inputImage, unsigned char* dst_array)
{
  if (inputImage.depth() != CV_8U)
  {
    std::cout << "ERROR: Expected input image type: CV_8U! Got cv type: " << inputImage.depth() << std::endl;
    return false;
  }

  int channels = inputImage.channels();
  int nRows = inputImage.rows;
  int nCols = inputImage.cols * channels;

  if (inputImage.isContinuous())
  {
    nCols *= nRows;
    nRows = 1;
  }

  int p = 0;
  for (int i = 0; i < nRows; i++)
  {
    uint8_t* rowPtr = inputImage.ptr<uchar>(i);
    for (int j = 0; j < nCols; j += channels)
    {
      // note the order here, because the cv::Mat is BGR and we need RGB
      dst_array[p + 0] = rowPtr[j + 2]; // R
      dst_array[p + 1] = rowPtr[j + 1]; // G
      dst_array[p + 2] = rowPtr[j + 0]; // B

      p += 3;
    }
  }

  return true;
}

#endif
