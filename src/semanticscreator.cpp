#include "semanticscreator.h"
namespace semantics {

SemanticsCreator::SemanticsCreator(){}
void SemanticsCreator::GetEuroPallet(const Eigen::Affine3d &pose,visualization_msgs::MarkerArray &object_arr,const unsigned int id, const std::string &name){
  visualization_msgs::Marker object;
  Eigen::Vector3d trans=pose.translation();
  Eigen::Quaterniond rot(pose.rotation());
  object.header.frame_id = "world";
  object.header.stamp = ros::Time();
  object.id = id;
  object.type = visualization_msgs::Marker::CUBE;
  object.action = visualization_msgs::Marker::ADD;
  object.pose.position.x =trans(0);
  object.pose.position.y = trans(1);
  object.pose.position.z = trans(2);
  object.pose.orientation.x = rot.x();
  object.pose.orientation.  y = rot.y();
  object.pose.orientation.z = rot.z();
  object.pose.orientation.w = rot.w();
  object.scale.x = EURO_PALLET_SIZE_X;
  object.scale.y = EURO_PALLET_SIZE_Y;
  object.scale.z = EURO_PALLET_SIZE_Z;
  object.color.a = 0.6; // Don't forget to set the alpha!
  object.color.r = 1.0;
  object.color.g = 0.8;
  object.color.b = 0.8;
  object.text=name;
  object.ns="Pallet";
  object_arr.markers.push_back(object);

}
void SemanticsCreator::   GetPickingArea(visualization_msgs::MarkerArray &arr,const std_vector3d &vek,const unsigned int id, const std::string &name){
  visualization_msgs::Marker object;
  object.header.frame_id = "world";
  object.header.stamp = ros::Time();
  object.id = id;
  object.type = visualization_msgs::Marker::LINE_STRIP;
  object.action = visualization_msgs::Marker::ADD;
  object.pose.position.x=0;
  object.pose.position.y=0;
  object.pose.position.z=0;
  object.scale.x = AREA_BORDER_SIZE;
  object.color.a = 0.6; // Don't forget to set the alpha!
  object.color.r = 0.8;
  object.color.g = 0.0;
  object.color.b = 0.30;
  object.text=name;
  object.ns="Picking_area";
  geometry_msgs::Point p;
  for(int i=0;i<vek.size();i++){
    p.x=vek[i](0);
    p.y=vek[i](1);
    p.z=vek[i](2);
    object.points.push_back(p);
  }
  arr.markers.push_back(object);

}
}
