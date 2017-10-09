// @file aruco_helper.cpp
// @brief Backtraces transform based on camera input to snap odom to world map
// @author Matthew Wilson (matwilso)
//

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "aruco_patch");
  ros::NodeHandle node;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  static tf::TransformBroadcaster br; 

  //tf::Transform odom_map_tf;

  // flag to indicate we have received at least 1 transform,
  // so we can start republishing that information
  bool transform_is_set = false;
  double last_yaw;

  double roll, pitch, yaw;
  ros::param::get("/aruco_mapping/roll" , roll);
  ros::param::get("/aruco_mapping/pitch", pitch);
  ros::param::get("/aruco_mapping/yaw"  , yaw);

  geometry_msgs::TransformStamped odom_tree_tf;
  geometry_msgs::TransformStamped map_tree_tf;

  ros::Rate rate(10.0);
  while (node.ok()){
    //tf::StampedTransform odom_tree_transform;
    //tf::StampedTransform map_tree_transform;

    if (listener.canTransform("camera_position", "map", ros::Time::now()-ros::Duration(0.1)) &&
     listener.canTransform("ZED_left_camera", "odom", ros::Time::now()-ros::Duration(0.1))) {

       listener.lookupTransform("ZED_left_camera", "odom", ros::Time(0), odom_tree_transform);
    //    listener.lookupTransform("odom", "ZED_left_camera", ros::Time(0), odom_tree_transform);
       listener.lookupTransform("camera_position", "map", ros::Time(0), map_tree_transform);
    //    listener.lookupTransform("map", "camera_position", ros::Time(0), map_tree_transform);

    //    Calculated the different between the map and odom frames
    //    (map -> camera_position) - (odom -> ZED_left_camera) = map -> odom
        odom_map_tf = map_tree_transform.inverseTimes(odom_tree_transform);
        // odom_map_tf = odom_tree_transform.inverseTimes(map_tree_transform);


        // Create a quaternion from the parameters passed in that will be used
        // to rotate the transform
        // The launch file has tranform of 0 degrees.  This should probably
        // be taken out when the system can be tested again
        // TODO ^
        tf::Quaternion rot = tf::Quaternion(qx, qy, qz, qw);

        // Rotate the odom map difference and normalize the quaternion
        odom_map_tf.setRotation((odom_map_tf.getRotation()*rot).normalize());
        tf::Quaternion quat = odom_map_tf.getRotation();

        // Conver the quaterion to RPY so we can check the limits and remove
        // everything besides the rotation about Z axis.

        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        if (transform_is_set)  {
            last_yaw = yaw;
        }
        // This check was necessary because the rotation causes the tranform
        // to have weird discontinuites where the robot position completely flips
        //if (abs(yaw - last_yaw) > 0.785) {
        //    ROS_WARN("Current Yaw: %.2f, Last: %.2f", yaw, last_yaw);
        //   rate.sleep();
        //   continue;
        //}

        odom_map_tf.setRotation(tf::Quaternion(0.0, 0.0, yaw));
        tf::Vector3 translation = odom_map_tf.getOrigin();
        odom_map_tf.setOrigin(tf::Vector3(translation.getX(), translation.getY(), 0.0));
        br.sendTransform(tf::StampedTransform(odom_map_tf, ros::Time::now(), "map", "odom"));
        last_yaw = yaw;
		transform_is_set = true;
    }
    else {
        // If we have received a valid transform, keep publishing that until you see a new one
        // This allows us to retain state when we are facing away.
      if (transform_is_set) {
        br.sendTransform(tf::StampedTransform(odom_map_tf, ros::Time::now(), "map", "odom"));
      }
      // We haven't received anything yet, publish 0,0.
      else {
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "odom"));
      }
    }
    rate.sleep();
  }
  return 1;
};
