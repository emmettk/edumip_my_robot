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
  void EduMipState_Callback(const edumip_msgs::EduMipState::ConstPtr& msg);
  ros::NodeHandle node_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
};

void EduMipStateNode::EduMipState_Callback(const edumip_msgs::EduMipState::ConstPtr& msg)
{
  //Transforms to publish
  static tf::TransformBroadcaster tf_br;
  tf::Transform world_to_edumip_transform;
  //Joint state to publish
  static sensor_msgs::JointState joint_state;
  int wheel_rad = 0.034;
  


  world_to_edumip_transform.setOrigin(tf::Vector3(msg->body_frame_northing,
						  -msg->body_frame_easting,
						  wheel_rad));
  tf::Quaternion q;
  q.setRPY(0.0, msg->theta, -msg->body_frame_heading);
  world_to_edumip_transform.setRotation(q);
  
  joint_state.name.resize(2);
  joint_state.position.resize(2);
  joint_state.name[0]="wheelL_joint";
  joint_state.name[1]="wheelR_joint";

  joint_state.header.stamp = ros::Time::now();
  joint_state.position[0]  = msg->wheel_angle_L;
  joint_state.position[1]  = msg->wheel_angle_R;

  //ROS_INFO("We made it into the callback, %f", msg->wheel_angle_L);

  //publish the joint state and transform
  pub_.publish(joint_state);
  tf_br.sendTransform(tf::StampedTransform(world_to_edumip_transform,  ros::Time::now(), "world", "edumip_body"));
 
}

EduMipStateNode::EduMipStateNode()
{
  //Subscribe to /edumip/state
  sub_=node_.subscribe<edumip_msgs::EduMipState>("edumip/state", 10, &EduMipStateNode::EduMipState_Callback, this);
  //ros::Subscriber sub=node_.subscribe("edumip/state", 100,&EduMipStateNode::EduMipState_Callback, this);

  //Publish to /joint_states
  pub_=node_.advertise<sensor_msgs::JointState>("joint_states", 1);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "edumip_state");
  EduMipStateNode edumip_state;
  ros::spin();
  return 0;
}
