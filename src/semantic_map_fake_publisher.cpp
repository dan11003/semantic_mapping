#include <ros/ros.h>
#include <cstdio>
#include <Eigen/Eigen>
#include "stdio.h"
#include "iostream"
#include "message_filters/subscriber.h"
#include "visualization_msgs/MarkerArray.h"
#include "std_srvs/Empty.h"
#include "semanticscreator.h"

#define PALLET_THETA 0.01564559473350094
/*   
BIG Pick items
      <x>-4.252623558044434</x>
      <y>0.051172733306884766</y>
      <theta>-0.005226923516850496</theta>*/
#define P1 -3.45,-0.75,0
#define P2 -4.95,-0.75,0
#define P3 -4.95, 0.75,0
#define P4 -3.45, 0.75,0



/*small item picking site 2
    <Pose name="toPose">
      <x>-21.755008697509766</x>
      <y>2.7174549102783203</y>
      <theta>-1.5736400093450944</theta>*/
#define P11 -22.5,2.0  ,0
#define P12 -21  ,2.0  ,0
#define P13 -21  ,3.5  ,0
#define P14 -22.5,3.5  ,0

/*      <x>-22.966995239257812</x>
      <y>-5.024758815765381</y>
      <theta>0.01564559473350094</theta>*/
using namespace std;

using namespace Eigen;
class semantic_map_publisher {

public:
  // Constructor
  semantic_map_publisher(ros::NodeHandle param_nh)
  {

    param_nh.param<std::string>("topic_prefix",topic_prefix_,"");

    marker_publisher_=nh_.advertise<visualization_msgs::MarkerArray>(topic_prefix_+"semantics_output",100);
    // marker_publisher_text_=nh_.advertise<visualization_msgs::MarkerArray>(topic_prefix_+"semantics_output_text",100);
    publish_service_ = param_nh.advertiseService("publish_semantics", &semantic_map_publisher::PublishSemantics, this);
    semantics_creator_=new  semantics::SemanticsCreator();

  }
  bool PublishSemantics(std_srvs::Empty::Request  &req,
                        std_srvs::Empty::Response &res ) {
    unsigned id=0;
    cout<<"Plotting semantics "<<endl;
    Eigen::Affine3d pose_pallet=Eigen::Affine3d::Identity();
    pose_pallet.translation()<<-22.966995239257812,-5.024758815765381,0.075;

    Eigen::AngleAxisd rollAngle(0, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(PALLET_THETA, Eigen::Vector3d::UnitY());  
    Eigen::AngleAxisd pitchAngle(0, Eigen::Vector3d::UnitX());
    Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
       pose_pallet.linear()=q.matrix();
 visualization_msgs::MarkerArray pallet_marker;
    semantics_creator_->GetEuroPallet(pose_pallet,pallet_marker,1,"pallet1");
    publishSemanticsWithText(pallet_marker);
    {
      visualization_msgs::MarkerArray arr_picking_site1;
      std_vector3d points;
      Eigen::Vector3d point1(P1);
      points.push_back(point1);
      Eigen::Vector3d point2(P2);
      points.push_back(point2);
      Eigen::Vector3d point3(P3);
      points.push_back(point3);
      Eigen::Vector3d point4(P4);
      points.push_back(point4);
      points.push_back(point1);
      semantics_creator_->GetPickingArea(arr_picking_site1,points,2,"Picking site: small items");
      publishSemanticAreaWithText(arr_picking_site1);
    }
    {
      visualization_msgs::MarkerArray arr_picking_site2;
      std_vector3d points;
      Eigen::Vector3d point1(P11);
      points.push_back(point1);
      Eigen::Vector3d point2(P12);
      points.push_back(point2);
      Eigen::Vector3d point3(P13);
      points.push_back(point3);
      Eigen::Vector3d point4(P14);
      points.push_back(point4);
      points.push_back(point1);
      semantics_creator_->GetPickingArea(arr_picking_site2,points,3,"Picking site: large items");
      publishSemanticAreaWithText(arr_picking_site2);
    }
    return true;

  }

  void publishSemanticAreaWithText( visualization_msgs::MarkerArray &arr){
    visualization_msgs::MarkerArray markers=arr;
    marker_publisher_.publish(markers);

    Eigen::Vector3d sum(0,0,0);
    for(int i=0;i<markers.markers[0].points.size();i++){
      sum(0)=sum(0)+markers.markers[0].points[i].x;
      sum(1)=sum(1)+markers.markers[0].points[i].y;
      sum(2)=sum(2)+markers.markers[0].points[i].z;
    }
    sum/=markers.markers[0].points.size();

    markers.markers[0].type=visualization_msgs::Marker::TEXT_VIEW_FACING;
    markers.markers[0].ns=markers.markers[0].ns+"_text";
    markers.markers[0].pose.position.x=sum(0);
    markers.markers[0].pose.position.y=sum(1);
    markers.markers[0].pose.position.z=sum(2)+markers.markers[0].scale.z/2.0+0.5;
    markers.markers[0].scale.z=0.4;
    std::cout<<"ns="<<markers.markers[0].ns<<", text="<<markers.markers[0].text<<endl;
    marker_publisher_.publish(markers);
  }

  void publishSemanticsWithText( visualization_msgs::MarkerArray &arr){
    visualization_msgs::MarkerArray markers=arr;
    marker_publisher_.publish(markers);
    markers.markers[0].type=visualization_msgs::Marker::TEXT_VIEW_FACING;
    markers.markers[0].ns=markers.markers[0].ns+"_text";
    markers.markers[0].pose.position.z=markers.markers[0].pose.position.z+markers.markers[0].scale.z/2.0+0.5;
    markers.markers[0].scale.z=0.4;
    marker_publisher_.publish(markers);
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher marker_publisher_,marker_publisher_text_;
  std::string topic_prefix_;
  semantics::SemanticsCreator *semantics_creator_;
  ros::ServiceServer publish_service_;


};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ndt_fuser_node");

  ros::NodeHandle param("~");
  semantic_map_publisher t(param);
  ros::spin();

  return 0;
}

