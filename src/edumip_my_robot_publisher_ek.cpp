/*WHOI ROS Short Course Module 4 Assignment
 *Emmett Krupczak
 *30 Nov 2017
 *Subscribe to /edumip/state and publish sensor_msgs/JointState on /joint_states
 *publish tf transform from world frame to robot_base frame
 */

#include <ros/ros.h>
#include <edumip_msgs/EduMipState.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

class EduMipStateNode
{
 public:
  EduMipStateNode();

 private:
  void EduMipState_Callback(const edumip_msgs::EduMipState::ConstPtr& EduMipState);
  ros::NodeHandle node_;
  ros::Publisher * pub1_;
  ros::Publisher pub2_;
  ros::Subscriber sub_;
};

void EduMipStateNode::EduMipState_Callback(const edumip_msgs::EduMipState::ConstPtr& EduMipState)
{
  //Transforms to publish
  static tf::TransformBroadcaster tf_broadcaster;
  static tf::Transform world_to_edumip_transform;
  //Joint state to publish
  static sensor_msgs::JointState joint_state;

  //  static double angle = 0.0;

  world_to_edumip_transform.setOrigin(tf::Vector3(EduMipState->body_frame_northing,
						  -EduMipState->body_frame_easting,
						  0.034));
  tf::Quaternion q;
  q.setRPY(0.0, EduMipState->theta, -EduMipState->body_frame_heading);
  world_to_edumip_transform.setRotation(q);
  
  joint_state.name.resize(2);
  joint_state.position.resize(2);
  joint_state.name[0]="jointL";
  joint_state.name[1]="jointR";

  joint_state.header.stamp = ros::Time::now();
  joint_state.position[0]  = EduMipState->wheel_angle_L;
  joint_state.position[1]  = EduMipState->wheel_angle_R;

  //send the joint state and transform
  pub1_->publish(joint_state);
  tf_broadcaster.sendTransform(tf::StampedTransform(world_to_edumip_transform,  ros::Time::now(), "world", "edumip_body"));
  

}

EduMipStateNode::EduMipStateNode()
{
  //Subscribe to /edumip/state
  sub_=node_.subscribe<edumip_msgs::EduMipState("edumip/state", 10, &EduMipStateNode::EduMipState_Callback, this);

  //Publish to /joint_states
  pub2_=node_.advertise<sensor_msgs::JointState>("joint_states", 1);
  pub1_ = &pub2_;

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "edumip_state");
  EduMipStateNode edumip_state;
  ros::spin();

}
