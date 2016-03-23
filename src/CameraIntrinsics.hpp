#ifndef _CAMERA_INTRINSICS_H_
#define _CAMERA_INTRINSICS_H_

struct CameraIntrinsics
{
  double intrinsics[3][3] = {
    {5.25, 0.0,  3.0},
    {0.0,  5.25, 2.0},
    {0.0,  0.0,  1.0}
  };
  std::vector<double> distortion = {
    0.0, 0.0, 0.0, 0.0, 0.0
  };

  CameraIntrinsics(double intrinsics_matrix[3][3], std::vector<double> distortion_coefficients)
  {
    for (size_t i = 0; i < 3; i++)
    {
      for (size_t j = 0; j < 3; j++)
      {
        intrinsics[i][j] = intrinsics_matrix[i][j];
      }
    }
    distortion = distortion_coefficients;
  }
};

#endif // _CAMERA_INTRINSICS_H_
