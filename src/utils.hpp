#ifndef _UTILS_H
#define _UTILS_H

#include <boost/filesystem.hpp>
#include <boost/iterator/filter_iterator.hpp>

#include <opencv2/opencv.hpp>
#include <sys/stat.h>

// Converts degrees to radians.
#define RADIANS(angleDegrees) (angleDegrees * M_PI / 180.0)

// Converts radians to degrees.
#define DEGREES(angleRadians) (angleRadians * 180.0 / M_PI)


// linear interpolation from start to end
double lerp(double start, double end, double d)
{
  if (d >= 1.0)
    return end;
  else if (d <= 0.0)
    return start;
  else
    return ((1.0 - d) * start) + (d * end);
}

// smooth interpolation between start & end with easing on both sides
double easeInOut(double start, double end, double d)
{
  if (d >= 1.0)
    return end;
  else if (d <= 0.0)
    return start;
  else
    return lerp(start, end, d * d * (3.0 - 2.0 * d));
}

// checks for the existence of a path
bool checkPath(std::string filename)
{
  struct stat buffer;
  return (stat (filename.c_str(), &buffer) == 0);
}

// creates a directory with a name based on current timestamp
std::string static makeStampedDirectory(std::string prefix)
{
    auto now = std::time(nullptr);
    char buf[sizeof("YYYY-MM-DD_HHMMSS")];
    std::string dirname = prefix + std::string(buf, buf + std::strftime(buf, sizeof(buf), "%F_%H%M%S", std::gmtime(&now)));
#ifdef USE_BOOST_FILESYSTEM
    if (boost::filesystem::create_directories(dirname))
        return dirname;
    else
        throw std::runtime_error("Could not create directory: " + dirname);
#else
    const int dir_err = mkdir(dirname.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (-1 == dir_err)
    {
        throw std::runtime_error("Could not create directory: " + dirname + "\n\t\t" + strerror(errno));
    }
    return dirname;
#endif
}

bool comparator(const std::string& s1, const std::string& s2)
{
  size_t first = s1.find_last_of("_");
  // size_t last = s1.find_last_of(ext);
  std::string num1 = s1.substr(first+1, (s1.length()-4 - first) - 1);

  first = s2.find_last_of("_");
  // last = s2.find_last_of(ext);
  std::string num2 = s2.substr(first+1, (s2.length()-1 - first) - 1);

  return (std::stoi(num1) < std::stoi(num2));
}

// retrieves all files with the given extension in a directory
std::vector<std::string> getFilesInDirectory(const std::string& dir, const std::string& ext)
{
	int current_file_counter_ = 0;
  std::vector<std::string> file_names;
  // Default constructor for an iterator is the end iterator
	boost::filesystem::directory_iterator end_iter;
	for (boost::filesystem::directory_iterator iter(dir); iter != end_iter; ++iter)
  {
		if (iter->path().extension() == ext)
    {
      std::string const fn = iter->path().string();
      file_names.push_back(fn);
    	++current_file_counter_;
    }
  }

  // sort files
  std::sort(file_names.begin(), file_names.end(), comparator);
  std::cout << "found " << current_file_counter_
            << " " << ext
            << " files in directory " << dir << std::endl;

	return file_names;
}

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
