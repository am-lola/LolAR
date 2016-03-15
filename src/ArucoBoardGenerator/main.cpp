#include <iostream>
#include <fstream>
#include <string>
#include <getopt.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

struct MarkerParams
{
  int markerRows         = 1;    // # of markers in each row of the board
  int markerColumns      = 1;    // # of markers in each col of the board
  int markerBits         = 5;    // # of bits on each side of each marker (6x6, 5x5, etc)
  float markerSize       = 0.05; // Size, in meters, of each marker
  float markerSeparation = 0.01; // Distance, in meters, between adjacent markers
  int imageWidth         = 1191; // Width, in pixels, of output image
  int imageHeight        = 842;  // Height, in pixels, of output image (default size corresponds to A3 @ 72dpi)
  std::string filename;          // Output filename
};


bool parse_args(int argc, char* argv[], MarkerParams* params)
{
  int c;

  while (1)
  {
    static struct option long_options[] =
      {
        {"rows", required_argument, 0, 'r'},
        {"cols", required_argument, 0, 'c'},
        {"bits", required_argument, 0, 'b'},
        {"size", required_argument, 0, 's'},
        {"dist", required_argument, 0, 'd'},
        {"file", required_argument, 0, 'f'},
        {"help", no_argument,       0, 'h'},
        {0, 0, 0, 0}
      };

      int option_index = 0;
      c = getopt_long (argc, argv, "r:c:b:s:d:f:h",
                       long_options, &option_index);

      /* Detect the end of the options. */
      if (c == -1)
        break;

      switch (c)
      {
        case 'r':
          params->markerRows = std::stoi(optarg);
          break;

        case 'c':
          params->markerColumns = std::stoi(optarg);
          break;

        case 'b':
          params->markerBits = std::stoi(optarg);
          if (params->markerBits < 4 || params->markerBits > 7)
            return false;
          break;

        case 's':
          params->markerSize = std::stod(optarg);
          break;

        case 'd':
          params->markerSeparation = std::stod(optarg);
          break;

        case 'f':
          params->filename = std::string(optarg);
          break;

        case 'h':
          return false;
          break;

        default:
          abort ();
      }
  }

  /* Print any remaining command line arguments (not options). */
  if (optind < argc)
  {
    std::cout << "Unrecognized arguments: ";
    while (optind < argc)
      std::cout << argv[optind++] << ", ";
    std::cout << std::endl;
    return false;
  }

  return true;
}

void printHelp(std::string name)
{
  std::cout << "Usage:" << std::endl;
  std::cout << "\t" << name << " [--rows rows] [--cols cols] [--bits bits] [--size size] [--dist distance] [--file output_filename] [--help]" << std::endl;
  std::cout << "\t" << name << " [-r rows] [-c cols] [-b bits] [-s size] [-d distance] [-f output_filename] [-h]" << std::endl;
  std::cout << std::endl;
  std::cout << "\t" << "     rows: Number of markers in each row of the board" << std::endl;
  std::cout << "\t" << "     cols: Number of markers in each column of the board" << std::endl;
  std::cout << "\t" << "     bits: Number of bits on each side of the markers (4, 5, 6, or 7)" << std::endl;
  std::cout << "\t" << "     size: Size, in meters, of each marker" << std::endl;
  std::cout << "\t" << " distance: Distance, in meters, between adjacent markers" << std::endl;
  std::cout << "\t" << " filename: File to output result to" << std::endl;
  std::cout << "\t" << "     help: Display this message" << std::endl;
}

int main(int argc, char* argv[])
{
  MarkerParams marker;

  if (!parse_args(argc, argv, &marker))
  {
    printHelp(argv[0]);
    return 0;
  }

  if (marker.filename.size() == 0)
  {
    marker.filename = "ArucoBoard." +
                      std::to_string(marker.markerBits) + "X" + std::to_string(marker.markerBits) +
                      "." + std::to_string(marker.markerSize) +
                      ".jpg";
  }

  std::cout << "Generating Aruco MarkerBoard with the following settings:" << std::endl;
  std::cout << "---------------------------------------------------------" << std::endl;
  std::cout << "        Rows : " << marker.markerRows << std::endl;
  std::cout << "        Cols : " << marker.markerColumns << std::endl;
  std::cout << "        Bits : " << marker.markerBits << "x" << marker.markerBits << std::endl;
  std::cout << "        Size : " << marker.markerSize << "x" << marker.markerSize << " meters" << std::endl;
  std::cout << "  Separation : " << marker.markerSeparation << " meters" << std::endl;
  std::cout << " Output File : " << marker.filename << std::endl << std::endl;

  cv::Ptr<cv::aruco::Dictionary> dictionary;
  switch (marker.markerBits)
  {
    case 4:
      dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000);
      break;
    case 5:
      dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
      break;
    case 6:
      dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
      break;
    case 7:
      dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_1000);
      break;
  }

  std::cout << "Genrating marker board..." << std::endl;
  auto gridboard = cv::aruco::GridBoard::create(marker.markerColumns,
                                                marker.markerRows,
                                                marker.markerSize,
                                                marker.markerSeparation,
                                                dictionary);

  cv::Mat boardImage;
  gridboard->draw( cv::Size(marker.imageWidth, marker.imageHeight), boardImage, 10, 1 );

  std::cout << "Writing image to " << marker.filename << "..." << std::endl;
  cv::imwrite(marker.filename, boardImage);

  std::cout << "Done!" << std::endl;
  return 0;
}
