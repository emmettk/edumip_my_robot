

#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <edumip_msgs/EduMipState.h>
#include <tf/transform_broadcaster.h>

// global variables
ros::Publisher  * joint_publisher_ptr;

void EduMipState_Callback(const edumip_msgs::EduMipState::ConstPtr& EduMipState)
{

  static tf::TransformBroadcaster        tf_broadcaster;
  static tf::Transform                   world_to_edumip_trans;
  static sensor_msgs::JointState         joint_state;

  ROS_INFO("Got an EduMipState.");
  printf("Got an EduMipState.");

  static double angle = 0.0;

  // update transform
  // (moving in a circle with radius=2)
  //  world_to_edumip_trans.header.stamp = ros::Time::now();
  world_to_edumip_trans.setOrigin( tf::Vector3( EduMipState->body_frame_northing,
						-EduMipState->body_frame_easting,
					        0.034));
  //world_to_edumip_trans.setRotation( tf::createQuaternionMsgFromYaw( EduMipState->body_frame_heading ));
  tf::Quaternion q;
  q.setRPY(0.0 , EduMipState->theta, -EduMipState->body_frame_heading);
  world_to_edumip_trans.setRotation(q);

  // update the joint state
  joint_state.name.resize(2);
  joint_state.position.resize(2);
  joint_state.name[0]      ="jointL";
  joint_state.name[1]      ="jointR";

  joint_state.header.stamp = ros::Time::now();
  joint_state.position[0]  = EduMipState->wheel_angle_L;
  joint_state.position[1]  = EduMipState->wheel_angle_R;

  //send the joint state and transform
  joint_publisher_ptr->publish(joint_state);
  tf_broadcaster.sendTransform(tf::StampedTransform(world_to_edumip_trans,  ros::Time::now(), "world", "edumip_body"));
  
}




int main(int argc, char** argv)
{

  edumip_msgs::EduMipState EduMipState;

  ros::init(argc, argv, "edumip_my_robot_state_publisher");
    
  ros::NodeHandle n;

  ros::Publisher  joint_publisher  = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  // ros::Publisher  edumip_publisher = n.advertise<edumip_msgs::EduMipState>("edumip/state", 1);

  joint_publisher_ptr = &joint_publisher;

  ros::Subscriber sub = n.subscribe("edumip/state", 100, EduMipState_Callback);


  ros::spin();

  /*
  ros::Rate loop_rate(10);

  while (ros::ok())
    {
      ros::spinOnce();
      edumip_publisher.publish(EduMipState);
      ros::spinOnce();
      loop_rate.sleep();

    }
  */

  /*
    ros::Rate loop_rate(10);
    const double degree = M_PI/180;

    // robot state
    double tilt = 0, tinc = degree, swivel=0, angle=0, height=0, hinc=0.005;

    // message declarations
    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "axis";

    while (ros::ok())
    {
    //update joint_state
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(3);
    joint_state.position.resize(3);
    joint_state.name[0] ="swivel";
    joint_state.position[0] = swivel;
    joint_state.name[1] ="tilt";
    joint_state.position[1] = tilt;
    joint_state.name[2] ="periscope";
    joint_state.position[2] = height;


    // update transform
    // (moving in a circle with radius=2)
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.transform.translation.x = cos(angle)*2;
    odom_trans.transform.translation.y = sin(angle)*2;
    odom_trans.transform.translation.z = .7;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle+M_PI/2);

    //send the joint state and transform
    joint_pub.publish(joint_state);
    broadcaster.sendTransform(odom_trans);

    // Create new robot state
    tilt += tinc;
    if (tilt<-.5 || tilt>0) tinc *= -1;
    height += hinc;
    if (height>.2 || height<0) hinc *= -1;
    swivel += degree;
    angle += degree/4;

    // This will adjust as needed per iteration
    loop_rate.sleep();
    }

  */
    

  return 0;
}

