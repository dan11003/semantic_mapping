#ifndef SEMANTICSCREATOR_H
#define SEMANTICSCREATOR_H
#include "visualization_msgs/MarkerArray.h"
#include "Eigen/Eigen"
#define EURO_PALLET_SIZE_X 0.9
#define EURO_PALLET_SIZE_Y 1.32
#define EURO_PALLET_SIZE_Z 0.15
#define AREA_BORDER_SIZE 0.2
#include<Eigen/StdVector>
typedef std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > std_vector3d;

namespace semantics{

class SemanticsCreator{

public:
  SemanticsCreator();
  void GetEuroPallet(const Eigen::Affine3d &pose, visualization_msgs::MarkerArray &object_arr, const unsigned int id, const std::__cxx11::string &name);
  void GetPickingArea(visualization_msgs::MarkerArray &arr, const std_vector3d &vek, const unsigned int id, const std::__cxx11::string &name);
};
}



#endif // SEMANTICSCREATOR_H
