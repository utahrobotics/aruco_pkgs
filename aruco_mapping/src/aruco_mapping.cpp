/*********************************************************************************************//**
* @file aruco_mapping.cpp
*
* Copyright (c)
* Smart Robotic Systems
* March 2015
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

/* Author: Jan Bacik 
 * Editted by Matthew Wilson (<patch> tags around what I changed)
 * * */

#ifndef ARUCO_MAPPING_CPP
#define ARUCO_MAPPING_CPP

#include <aruco_mapping.h>

namespace aruco_mapping
{

ArucoMapping::ArucoMapping(ros::NodeHandle *nh) :
  listener_ (new tf::TransformListener),  // Initialize TF Listener  
  num_of_markers_ (10),                   // Number of used markers
  marker_size_(0.1),                      // Marker size in m
  calib_filename_("empty"),               // Calibration filepath
  space_type_ ("plane"),                  // Space type - 2D plane 
  roi_allowed_ (false),                   // ROI not allowed by default
  first_marker_detected_(false),          // First marker not detected by defualt
  lowest_marker_id_(-1),                  // Lowest marker ID
  marker_counter_(0),                     // Reset marker counter
  closest_camera_index_(0)                // Reset closest camera index 
  
{
  double temp_marker_size;  
  
  map_odom_tf_set_ = false; // we don't know this tf yet

  //Parse params from launch file 
  nh->getParam("/aruco_mapping/calibration_file", calib_filename_);
  nh->getParam("/aruco_mapping/marker_size", temp_marker_size); 
  nh->getParam("/aruco_mapping/num_of_markers", num_of_markers_);
  nh->getParam("/aruco_maping/space_type",space_type_);
  nh->getParam("/aruco_mapping/roi_allowed",roi_allowed_);
  nh->getParam("/aruco_mapping/roi_x",roi_x_);
  nh->getParam("/aruco_mapping/roi_y",roi_y_);
  nh->getParam("/aruco_mapping/roi_w",roi_w_);
  nh->getParam("/aruco_mapping/roi_h",roi_h_);
  nh->getParam("/aruco_mapping/gui",gui_);
  nh->getParam("/aruco_mapping/two_d_mode",two_d_mode_);
  nh->getParam("/aruco_mapping/marker_id_list",marker_id_list_);
  nh->getParam("/aruco_mapping/tag_offset",tag_offset_);
  nh->getParam("/aruco_mapping/cartx",cartx_);
  nh->getParam("/aruco_mapping/carty",carty_);
  nh->getParam("/aruco_mapping/cartz",cartz_);
  nh->getParam("/aruco_mapping/roll",roll_);
  nh->getParam("/aruco_mapping/pitch",pitch_);
  nh->getParam("/aruco_mapping/yaw",yaw_);
  nh->getParam("/aruco_mapping/tf_delay",tf_delay_);
  //nh->getParam("/aruco_mapping/quatx",quatx_);
  //nh->getParam("/aruco_mapping/quaty",quaty_);
  //nh->getParam("/aruco_mapping/quatz",quatz_);
  //nh->getParam("/aruco_mapping/quatw",quatw_);
     
  // Double to float conversion
  marker_size_ = float(temp_marker_size);
  
  if(calib_filename_ == "empty")
    ROS_WARN("Calibration filename empty! Check the launch file paths");
  else
  {
    ROS_INFO_STREAM("Calibration file path: " << calib_filename_ );
    ROS_INFO_STREAM("Number of markers: " << num_of_markers_);
    ROS_INFO_STREAM("Marker Size: " << marker_size_);
    ROS_INFO_STREAM("Type of space: " << space_type_);
    ROS_INFO_STREAM("ROI allowed: " << roi_allowed_);
    ROS_INFO_STREAM("ROI x-coor: " << roi_x_);
    ROS_INFO_STREAM("ROI y-coor: " << roi_x_);
    ROS_INFO_STREAM("ROI width: "  << roi_w_);
    ROS_INFO_STREAM("ROI height: " << roi_h_);      
    ROS_INFO_STREAM("GUI: " << gui_);
	for(std::vector<int>::size_type i = 0; i != marker_id_list_.size(); i++) {
	  ROS_INFO_STREAM("#" << i+1 <<  " Marker ID: " << marker_id_list_[i]);
	}
    ROS_INFO_STREAM("Tag spacing: " << tag_offset_);
  }
    
  //ROS publishers
  marker_msg_pub_           = nh->advertise<aruco_mapping::ArucoMarker>("aruco_poses",1);
  marker_visualization_pub_ = nh->advertise<visualization_msgs::Marker>("aruco_markers",1);
          
  //Parse data from calibration file
  parseCalibrationFile(calib_filename_);

  if (gui_) {
    //Initialize OpenCV window
    cv::namedWindow("Mono8", CV_WINDOW_AUTOSIZE);       
  }
      
  //Resize marker container
  markers_.resize(num_of_markers_);
  
  // Default markers_ initialization
  for(size_t i = 0; i < num_of_markers_;i++)
  {
    markers_[i].previous_marker_id = -1;
    markers_[i].visible = false;
    markers_[i].marker_id = -1;
  }
}

ArucoMapping::~ArucoMapping()
{
 delete listener_;
}

bool
ArucoMapping::parseCalibrationFile(std::string calib_filename)
{
  sensor_msgs::CameraInfo camera_calibration_data;
  std::string camera_name = "camera";

  camera_calibration_parsers::readCalibrationIni(calib_filename, camera_name, camera_calibration_data);

  // Alocation of memory for calibration data
  cv::Mat  *intrinsics       = new(cv::Mat)(3, 3, CV_64F);
  cv::Mat  *distortion_coeff = new(cv::Mat)(5, 1, CV_64F);
  cv::Size *image_size       = new(cv::Size);

  image_size->width = camera_calibration_data.width;
  image_size->height = camera_calibration_data.height;

  for(size_t i = 0; i < 3; i++)
    for(size_t j = 0; j < 3; j++)
    intrinsics->at<double>(i,j) = camera_calibration_data.K.at(3*i+j);

  for(size_t i = 0; i < 5; i++)
    distortion_coeff->at<double>(i,0) = camera_calibration_data.D.at(i);

  ROS_DEBUG_STREAM("Image width: " << image_size->width);
  ROS_DEBUG_STREAM("Image height: " << image_size->height);
  ROS_DEBUG_STREAM("Intrinsics:" << std::endl << *intrinsics);
  ROS_DEBUG_STREAM("Distortion: " << *distortion_coeff);


  //Load parameters to aruco_calib_param_ for aruco detection
  aruco_calib_params_.setParams(*intrinsics, *distortion_coeff, *image_size);

  //Simple check if calibration data meets expected values
  if ((intrinsics->at<double>(2,2) == 1) && (distortion_coeff->at<double>(0,4) == 0))
  {
    ROS_INFO_STREAM("Calibration data loaded successfully");
    return true;
  }
  else
  {
    ROS_WARN("Wrong calibration data, check calibration file and filepath");
    return false;
  }
}

void
ArucoMapping::imageCallback(const sensor_msgs::ImageConstPtr &original_image)
{
  //Create cv_bridge instance
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr=cv_bridge::toCvCopy(original_image, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Not able to convert sensor_msgs::Image to OpenCV::Mat format %s", e.what());
    return;
  }
  
  // sensor_msgs::Image to OpenCV Mat structure
  cv::Mat I = cv_ptr->image;
  
  // region of interest
  if(roi_allowed_==true)
    I = cv_ptr->image(cv::Rect(roi_x_,roi_y_,roi_w_,roi_h_));

  //Marker detection
  processImage(I,I);
  
  if (gui_) {
    // Show image
    cv::imshow("Mono8", I);
    cv::waitKey(10);  
  }
}


bool
ArucoMapping::processImage(cv::Mat input_image,cv::Mat output_image)
{
  aruco::MarkerDetector Detector;
  std::vector<aruco::Marker> temp_markers;

  //Set visibility flag to false for all markers
  for(size_t i = 0; i < num_of_markers_; i++)
      markers_[i].visible = false;

  // Save previous marker count
  marker_counter_previous_ = marker_counter_;

  // Detect markers
  Detector.detect(input_image, temp_markers, aruco_calib_params_, marker_size_);
    
  // If no marker found, print statement
  if(temp_markers.size() == 0)
    ROS_DEBUG("No marker found!");

  //------------------------------------------------------
  // FIRST MARKER DETECTED
  //------------------------------------------------------
  if((temp_markers.size() > 0) && (first_marker_detected_ == false))
  {
    //Set flag
    first_marker_detected_ = true;

    // Detect lowest marker ID
    lowest_marker_id_ = temp_markers[0].id;
    for(size_t i = 0; i < temp_markers.size();i++)
    {
      if(temp_markers[i].id < lowest_marker_id_)
        lowest_marker_id_ = temp_markers[i].id;
    }


    ROS_DEBUG_STREAM("The lowest Id marker " << lowest_marker_id_ );

	// <patch>

	// The first marker we see is set as the origin. So depending on which 
	// one we see first, we have to adjust for its actual position
	int marker_count = marker_id_list_.size();
	int list_middle = marker_count / 2;
	bool is_odd = marker_count % 2 == 1;
	double list_middle_f = list_middle - 0.5;

	int j = 0;
	for(std::vector<int>::size_type i = 0; i != marker_count; i++) {
		if (lowest_marker_id_ == marker_id_list_[i]) {
		  if (is_odd) {
		    carty_ = carty_ - tag_offset_ * (list_middle - j);
		  }
		  else {
		    carty_ = carty_ - tag_offset_ * (list_middle_f - j);
		  }

		  }
		j++;
	}

	// Create quat from passed in rpy
	// This is used to rotate the camera frame to standard ros frame
	// (x is out, instead of z, etc)
	tf::Matrix3x3 rpy_mat;
	rpy_mat.setEulerYPR(yaw_, pitch_, roll_);
	tf::Quaternion quat;
	rpy_mat.getRotation(quat);

    // Identify lowest marker ID with map's origin
    markers_[0].marker_id = lowest_marker_id_;

    markers_[0].geometry_msg_to_map.position.x = cartx_;
    markers_[0].geometry_msg_to_map.position.y = carty_;
    markers_[0].geometry_msg_to_map.position.z = cartz_;

    markers_[0].geometry_msg_to_map.orientation.x = quat.getX();
    markers_[0].geometry_msg_to_map.orientation.y = quat.getY();
    markers_[0].geometry_msg_to_map.orientation.z = quat.getZ();
    markers_[0].geometry_msg_to_map.orientation.w = quat.getW();

    // Relative position and Global position
    markers_[0].geometry_msg_to_previous.position.x = cartx_;
    markers_[0].geometry_msg_to_previous.position.y = carty_;
    markers_[0].geometry_msg_to_previous.position.z = cartz_;

    markers_[0].geometry_msg_to_previous.orientation.x = quat.getX();
    markers_[0].geometry_msg_to_previous.orientation.y = quat.getY();
    markers_[0].geometry_msg_to_previous.orientation.z = quat.getZ();
    markers_[0].geometry_msg_to_previous.orientation.w = quat.getW();

    // Transformation Pose to TF
    tf::Vector3 position;
    position.setX(cartx_);
    position.setY(carty_);
    position.setZ(cartz_);

    tf::Quaternion rotation;
    rotation.setX(quat.getX());
    rotation.setY(quat.getY());
    rotation.setZ(quat.getZ());
    rotation.setW(quat.getW());

	// </patch>

    markers_[0].tf_to_previous.setOrigin(position);
    markers_[0].tf_to_previous.setRotation(rotation);

    // Relative position of first marker equals Global position
    markers_[0].tf_to_map = markers_[0].tf_to_previous;

    // Increase count
    marker_counter_++;

    // Set sign of visibility of first marker
    markers_[0].visible = true;

    ROS_INFO_STREAM("First marker with ID: " << markers_[0].marker_id << " detected");

    //First marker does not have any previous marker
    markers_[0].previous_marker_id = THIS_IS_FIRST_MARKER;
  }

  //------------------------------------------------------
  // FOR EVERY MARKER DO
  //------------------------------------------------------
  for(size_t i = 0; i < temp_markers.size();i++)
  {
    int index;
    int current_marker_id = temp_markers[i].id;

	if (gui_) {
      //Draw marker convex, ID, cube and axis
      temp_markers[i].draw(output_image, cv::Scalar(0,0,255),2);
      aruco::CvDrawingUtils::draw3dCube(output_image,temp_markers[i], aruco_calib_params_);
      aruco::CvDrawingUtils::draw3dAxis(output_image,temp_markers[i], aruco_calib_params_);
	}

    // Existing marker ?
    bool existing = false;
    int temp_counter = 0;

    while((existing == false) && (temp_counter < marker_counter_))
    {
      if(markers_[temp_counter].marker_id == current_marker_id)
      {
        index = temp_counter;
        existing = true;
        ROS_DEBUG_STREAM("Existing marker with ID: " << current_marker_id << "found");
      }
        temp_counter++;
    }

    //New marker ?
    if(existing == false)
    {
      index = marker_counter_;
      markers_[index].marker_id = current_marker_id;
      existing = true;
      ROS_DEBUG_STREAM("New marker with ID: " << current_marker_id << " found");
    }

    // Change visibility flag of new marker
    for(size_t j = 0;j < marker_counter_; j++)
    {
      for(size_t k = 0;k < temp_markers.size(); k++)
      {
        if(markers_[j].marker_id == temp_markers[k].id)
          markers_[j].visible = true;
      }
    }

    //------------------------------------------------------
    // For existing marker do
    //------------------------------------------------------
    if((index < marker_counter_) && (first_marker_detected_ == true))
    {
      markers_[index].current_camera_tf = arucoMarker2Tf(temp_markers[i]);
      markers_[index].current_camera_tf = markers_[index].current_camera_tf.inverse();

      const tf::Vector3 marker_origin = markers_[index].current_camera_tf.getOrigin();
      markers_[index].current_camera_pose.position.x = marker_origin.getX();
      markers_[index].current_camera_pose.position.y = marker_origin.getY();
      markers_[index].current_camera_pose.position.z = marker_origin.getZ();

      const tf::Quaternion marker_quaternion = markers_[index].current_camera_tf.getRotation();
      markers_[index].current_camera_pose.orientation.x = marker_quaternion.getX();
      markers_[index].current_camera_pose.orientation.y = marker_quaternion.getY();
      markers_[index].current_camera_pose.orientation.z = marker_quaternion.getZ();
      markers_[index].current_camera_pose.orientation.w = marker_quaternion.getW();
    }

    //------------------------------------------------------
    // For new marker do
    //------------------------------------------------------
    if((index == marker_counter_) && (first_marker_detected_ == true))
    {
      markers_[index].current_camera_tf=arucoMarker2Tf(temp_markers[i]);

      tf::Vector3 marker_origin = markers_[index].current_camera_tf.getOrigin();
      markers_[index].current_camera_pose.position.x = marker_origin.getX();
      markers_[index].current_camera_pose.position.y = marker_origin.getY();
      markers_[index].current_camera_pose.position.z = marker_origin.getZ();

      tf::Quaternion marker_quaternion = markers_[index].current_camera_tf.getRotation();
      markers_[index].current_camera_pose.orientation.x = marker_quaternion.getX();
      markers_[index].current_camera_pose.orientation.y = marker_quaternion.getY();
      markers_[index].current_camera_pose.orientation.z = marker_quaternion.getZ();
      markers_[index].current_camera_pose.orientation.w = marker_quaternion.getW();

      // Naming - TFs
      std::stringstream camera_tf_id;
      std::stringstream camera_tf_id_old;
      std::stringstream marker_tf_id_old;

      camera_tf_id << "camera_" << index;

      // Flag to keep info if any_known marker_visible in actual image
      bool any_known_marker_visible = false;

      // Array ID of markers, which position of new marker is calculated
      int last_marker_id;

      // Testing, if is possible calculate position of a new marker to old known marker
      for(int k = 0; k < index; k++)
      {
        if((markers_[k].visible == true) && (any_known_marker_visible == false))
        {
          if(markers_[k].previous_marker_id != -1)
          {
            any_known_marker_visible = true;
            camera_tf_id_old << "camera_" << k;
            marker_tf_id_old << "marker_" << k;
            markers_[index].previous_marker_id = k;
            last_marker_id = k;
           }
         }
       }

     // New position can be calculated
     if(any_known_marker_visible == true)
     {
       // Generating TFs for listener
       for(char k = 0; k < 10; k++)
       {
		 //tf::Transform cam_tf(markers_[last_marker_id].current_camera_tf);
		 //tf::Quaternion quat(0.0, -1.5708, 0.0);
		 //cam_tf.setRotation(cam_tf.getRotation() * quat);

         // TF from old marker and its camera
         //broadcaster_.sendTransform(tf::StampedTransform(cam_tf, ros::Time::now(), marker_tf_id_old.str(),camera_tf_id_old.str()));

         // TF from old marker and its camera
         broadcaster_.sendTransform(tf::StampedTransform(markers_[last_marker_id].current_camera_tf,ros::Time::now(),
                                                         marker_tf_id_old.str(),camera_tf_id_old.str()));

         // TF from old camera to new camera
         broadcaster_.sendTransform(tf::StampedTransform(markers_[index].current_camera_tf,ros::Time::now(),
                                                         camera_tf_id_old.str(),camera_tf_id.str()));

         ros::Duration(BROADCAST_WAIT_INTERVAL).sleep();
       }

        // Calculate TF between two markers
        listener_->waitForTransform(marker_tf_id_old.str(),camera_tf_id.str(),ros::Time(0),
                                    ros::Duration(WAIT_FOR_TRANSFORM_INTERVAL));
        try
        {
          broadcaster_.sendTransform(tf::StampedTransform(markers_[last_marker_id].current_camera_tf,ros::Time::now(),
                                                          marker_tf_id_old.str(),camera_tf_id_old.str()));

          broadcaster_.sendTransform(tf::StampedTransform(markers_[index].current_camera_tf,ros::Time::now(),
                                                          camera_tf_id_old.str(),camera_tf_id.str()));

          listener_->lookupTransform(marker_tf_id_old.str(),camera_tf_id.str(),ros::Time(0),
                                     markers_[index].tf_to_previous);
        }
        catch(tf::TransformException &e)
        {
          ROS_ERROR("Not able to lookup transform");
        }

        // Save origin and quaternion of calculated TF
        marker_origin = markers_[index].tf_to_previous.getOrigin();
        marker_quaternion = markers_[index].tf_to_previous.getRotation();

        // If plane type selected roll, pitch and Z axis are zero
        if(space_type_ == "plane")
        {
          double roll, pitch, yaw;
          tf::Matrix3x3(marker_quaternion).getRPY(roll,pitch,yaw);
          roll = 0;
          pitch = 0;
          marker_origin.setZ(0);
          marker_quaternion.setRPY(pitch,roll,yaw);
        }

        markers_[index].tf_to_previous.setRotation(marker_quaternion);
        markers_[index].tf_to_previous.setOrigin(marker_origin);

        marker_origin = markers_[index].tf_to_previous.getOrigin();
        markers_[index].geometry_msg_to_previous.position.x = marker_origin.getX();
        markers_[index].geometry_msg_to_previous.position.y = marker_origin.getY();
        markers_[index].geometry_msg_to_previous.position.z = marker_origin.getZ();

        marker_quaternion = markers_[index].tf_to_previous.getRotation();
        markers_[index].geometry_msg_to_previous.orientation.x = marker_quaternion.getX();
        markers_[index].geometry_msg_to_previous.orientation.y = marker_quaternion.getY();
        markers_[index].geometry_msg_to_previous.orientation.z = marker_quaternion.getZ();
        markers_[index].geometry_msg_to_previous.orientation.w = marker_quaternion.getW();

        // increase marker count
        marker_counter_++;

        // Invert and position of new marker to compute camera pose above it
        markers_[index].current_camera_tf = markers_[index].current_camera_tf.inverse();

        marker_origin = markers_[index].current_camera_tf.getOrigin();
        markers_[index].current_camera_pose.position.x = marker_origin.getX();
        markers_[index].current_camera_pose.position.y = marker_origin.getY();
        markers_[index].current_camera_pose.position.z = marker_origin.getZ();

        marker_quaternion = markers_[index].current_camera_tf.getRotation();
        markers_[index].current_camera_pose.orientation.x = marker_quaternion.getX();
        markers_[index].current_camera_pose.orientation.y = marker_quaternion.getY();
        markers_[index].current_camera_pose.orientation.z = marker_quaternion.getZ();
        markers_[index].current_camera_pose.orientation.w = marker_quaternion.getW();

        // Publish all TFs and markers
        publishTfs(false);
      }
    }

    //------------------------------------------------------
    // Compute global position of new marker
    //------------------------------------------------------
    if((marker_counter_previous_ < marker_counter_) && (first_marker_detected_ == true))
    {
      // Publish all TF five times for listener
      for(char k = 0; k < 5; k++)
        publishTfs(false);

      std::stringstream marker_tf_name;
      marker_tf_name << "marker_" << index;

      listener_->waitForTransform("map",marker_tf_name.str(),ros::Time(0),
                                  ros::Duration(WAIT_FOR_TRANSFORM_INTERVAL));
      try
      {
        listener_->lookupTransform("map",marker_tf_name.str(),ros::Time(0),
                                   markers_[index].tf_to_map);
      }
      catch(tf::TransformException &e)
      {
        ROS_ERROR("Not able to lookup transform");
      }

      // Saving TF to Pose
      const tf::Vector3 marker_origin = markers_[index].tf_to_map.getOrigin();
      markers_[index].geometry_msg_to_map.position.x = marker_origin.getX();
      markers_[index].geometry_msg_to_map.position.y = marker_origin.getY();
      markers_[index].geometry_msg_to_map.position.z = marker_origin.getZ();

      tf::Quaternion marker_quaternion=markers_[index].tf_to_map.getRotation();
      markers_[index].geometry_msg_to_map.orientation.x = marker_quaternion.getX();
      markers_[index].geometry_msg_to_map.orientation.y = marker_quaternion.getY();
      markers_[index].geometry_msg_to_map.orientation.z = marker_quaternion.getZ();
      markers_[index].geometry_msg_to_map.orientation.w = marker_quaternion.getW();
    }
  }

  //------------------------------------------------------
  // Compute which of visible markers is the closest to the camera
  //------------------------------------------------------
  bool any_markers_visible=false;
  int num_of_visible_markers=0;

  if(first_marker_detected_ == true)
  {
    double minimal_distance = INIT_MIN_SIZE_VALUE;
    for(int k = 0; k < num_of_markers_; k++)
    {
      double a,b,c,size;

      // If marker is visible, distance is calculated
      if(markers_[k].visible==true)
      {
        a = markers_[k].current_camera_pose.position.x;
        b = markers_[k].current_camera_pose.position.y;
        c = markers_[k].current_camera_pose.position.z;
        size = std::sqrt((a * a) + (b * b) + (c * c));
        if(size < minimal_distance)
        {
          minimal_distance = size;
          closest_camera_index_ = k;
        }

        any_markers_visible = true;
        num_of_visible_markers++;
      }
    }
  }

  //------------------------------------------------------
  // Publish all known markers
  //------------------------------------------------------
  if(first_marker_detected_ == true)
    publishTfs(true);

  //------------------------------------------------------
  // Compute global camera pose
  //------------------------------------------------------
  if((first_marker_detected_ == true) && (any_markers_visible == true))
  {
    std::stringstream closest_camera_tf_name;
    closest_camera_tf_name << "camera_" << closest_camera_index_;

    listener_->waitForTransform("map",closest_camera_tf_name.str(),ros::Time(0),
                                ros::Duration(WAIT_FOR_TRANSFORM_INTERVAL));
    try
    {
      listener_->lookupTransform("map",closest_camera_tf_name.str(),ros::Time(0),
                                 map_position_transform_);
    }
    catch(tf::TransformException &ex)
    {
      ROS_ERROR("Not able to lookup transform");
    }

    // Saving TF to Pose
    const tf::Vector3 marker_origin = map_position_transform_.getOrigin();
    map_position_geometry_msg_.position.x = marker_origin.getX();
    map_position_geometry_msg_.position.y = marker_origin.getY();
    map_position_geometry_msg_.position.z = marker_origin.getZ();

    tf::Quaternion marker_quaternion = map_position_transform_.getRotation();
    map_position_geometry_msg_.orientation.x = marker_quaternion.getX();
    map_position_geometry_msg_.orientation.y = marker_quaternion.getY();
    map_position_geometry_msg_.orientation.z = marker_quaternion.getZ();
    map_position_geometry_msg_.orientation.w = marker_quaternion.getW();
  }

  //------------------------------------------------------
  // Publish all known markers
  //------------------------------------------------------
  if(first_marker_detected_ == true)
    publishTfs(true);

  //------------------------------------------------------
  // Publish custom marker message
  //------------------------------------------------------
  aruco_mapping::ArucoMarker marker_msg;

  if((any_markers_visible == true))
  {
    marker_msg.header.stamp = ros::Time::now();
    marker_msg.header.frame_id = "map";
    marker_msg.marker_visibile = true;
    marker_msg.num_of_visible_markers = num_of_visible_markers;
    marker_msg.global_camera_pose = map_position_geometry_msg_;
    marker_msg.marker_ids.clear();
    marker_msg.global_marker_poses.clear();
    for(size_t j = 0; j < marker_counter_; j++)
    {
      if(markers_[j].visible == true)
      {
        marker_msg.marker_ids.push_back(markers_[j].marker_id);
        marker_msg.global_marker_poses.push_back(markers_[j].geometry_msg_to_map);       
      }
    }
  }
  else
  {
    marker_msg.header.stamp = ros::Time::now();
    marker_msg.header.frame_id = "map";
    marker_msg.num_of_visible_markers = num_of_visible_markers;
    marker_msg.marker_visibile = false;
    marker_msg.marker_ids.clear();
    marker_msg.global_marker_poses.clear();
  }

  // Publish custom marker msg
  marker_msg_pub_.publish(marker_msg);

  return true;
}


////////////////////////////////////////////////////////////////////////////////////////////////

tf::Transform ArucoMapping::averageTF(std::vector<std::string> frame_vec) {
  int n = 0;
  double tx_sum = 0.0;
  double ty_sum = 0.0;
  double tz_sum = 0.0;
  double qx_sum = 0.0;
  double qy_sum = 0.0;
  double qz_sum = 0.0;
  double qw_sum = 0.0;

  tf::Quaternion avg_quat(0, 0, 0, 0);

  for (std::vector<std::string>::iterator it = frame_vec.begin(); it != frame_vec.end(); it++) {

    tf::StampedTransform transform;
    ros::Time now = ros::Time::now() -  ros::Duration(tf_delay_);
    //listener_->waitForTransform(*it, "map", now, ros::Duration(0.2));
    listener_->lookupTransform(*it, "map", now, transform);

	tf::Vector3 trans = transform.getOrigin();
	tf::Quaternion quat = transform.getRotation();
	tx_sum += trans.getX();
	ty_sum += trans.getY();
	tz_sum += trans.getZ();

	// += just adds the x,y,z,w components 
	avg_quat += quat;
 
	n++;
  }

  tf::Vector3 avg_trans(tx_sum/n, ty_sum/n, tz_sum/n);

  avg_quat /= n;
  avg_quat.normalize();

  return tf::Transform(avg_quat, avg_trans);
}

////////////////////////////////////////////////////////////////////////////////////////////////

void
ArucoMapping::publishTfs(bool map_option)
{
  std::vector<std::string> visible_camera_patch_ids;

  for(int i = 0; i < marker_counter_; i++)
  {
    // Actual Marker
    std::stringstream marker_tf_id;
    marker_tf_id << "marker_" << i;
    // Older marker - or World
    std::stringstream marker_tf_id_old;
    if(i == 0) {
      marker_tf_id_old << "map";
	}

    else
      marker_tf_id_old << "marker_" << markers_[i].previous_marker_id;


    broadcaster_.sendTransform(tf::StampedTransform(markers_[i].tf_to_previous,ros::Time::now(),marker_tf_id_old.str(),marker_tf_id.str()));

    // Position of camera to its marker
    std::stringstream camera_tf_id;
    camera_tf_id << "camera_" << i;

	// <patch>
    std::stringstream camera_patch_id;
    camera_patch_id << "camera_patch_" << i;


	if (markers_[i].visible) {
		broadcaster_.sendTransform(tf::StampedTransform(markers_[i].current_camera_tf,ros::Time::now(),marker_tf_id.str(),camera_tf_id.str()));

		tf::Transform current_patch_tf(markers_[i].current_camera_tf);
		tf::Quaternion og_quat = current_patch_tf.getRotation();
		tf::Quaternion rot_quat;

		// Magic number. Not sure why, probably have an axis flipped
		rot_quat.setRPY(3.1415, pitch_, yaw_); 
		current_patch_tf.setRotation((og_quat * rot_quat).normalized());

		broadcaster_.sendTransform(tf::StampedTransform(current_patch_tf,ros::Time::now(),marker_tf_id.str(),camera_patch_id.str()));

		visible_camera_patch_ids.push_back(camera_patch_id.str());
	}

	// </patch>

    if(map_option == true)
    {
      // Global position of marker TF
      std::stringstream marker_globe;
      marker_globe << "marker_globe_" << i;


      broadcaster_.sendTransform(tf::StampedTransform(markers_[i].tf_to_map,ros::Time::now(),"map",marker_globe.str()));
    }

    // Cubes for RVIZ - markers
    publishMarker(markers_[i].geometry_msg_to_previous,markers_[i].marker_id,i);
  }

  // Global Position of object
  if(map_option == true) {
    broadcaster_.sendTransform(tf::StampedTransform(map_position_transform_,ros::Time::now(),"map","camera_position"));
  }

  // <patch>

  // TODO: might consider adding cov

  if (visible_camera_patch_ids.size() > 0) {
	tf::Transform map_tree_tf;
	tf::StampedTransform odom_tree_tf;

	try {
		map_tree_tf = averageTF(visible_camera_patch_ids);
	}
    catch (tf::TransformException &ex) {
        ROS_ERROR("Visible camera map lookup failed + %s",ex.what());
		return;
    }

    try{
		ros::Time now = ros::Time::now() - ros::Duration(tf_delay_);
		//listener_->waitForTransform("ZED_left_camera", "odom", 
        //                      now, );

        listener_->lookupTransform("ZED_left_camera", "odom", 
                                 now, odom_tree_tf);
      }
      catch (tf::TransformException &ex) {
        ROS_ERROR("odom to ZED_left_camera lookup failed + %s",ex.what());
		return;
    }


	  map_odom_tf_ = map_tree_tf.inverseTimes(odom_tree_tf);
	  if (two_d_mode_) {
		  // remove roll and pitch components and z to 0
		  double roll, pitch, yaw;
		  tf::Matrix3x3(map_odom_tf_.getRotation()).getRPY(roll, pitch, yaw);
		  tf::Quaternion quat;
		  quat.setRPY(0.0, 0.0, yaw);

		  tf::Vector3 trans = map_odom_tf_.getOrigin();
		  trans.setZ(0.0);

		  map_odom_tf_.setRotation(quat);
		  map_odom_tf_.setOrigin(trans);
	  }
	  map_odom_tf_set_ = true;
  }

  // Once we have found the map -> odom, keep publishing it 
  if (map_odom_tf_set_) {
	  broadcaster_.sendTransform(tf::StampedTransform(map_odom_tf_, ros::Time::now(), "map", "odom"));
  }
  // </patch>
}

////////////////////////////////////////////////////////////////////////////////////////////////

void
ArucoMapping::publishMarker(geometry_msgs::Pose marker_pose, int marker_id, int index)
{
  visualization_msgs::Marker vis_marker;

  if(index == 0)
    vis_marker.header.frame_id = "map";
  else
  {
    std::stringstream marker_tf_id_old;
    marker_tf_id_old << "marker_" << markers_[index].previous_marker_id;
    vis_marker.header.frame_id = marker_tf_id_old.str();
  }

  vis_marker.header.stamp = ros::Time::now();
  vis_marker.ns = "basic_shapes";
  vis_marker.id = marker_id;
  vis_marker.type = visualization_msgs::Marker::CUBE;
  vis_marker.action = visualization_msgs::Marker::ADD;

  vis_marker.pose = marker_pose;
  vis_marker.scale.x = marker_size_;
  vis_marker.scale.y = marker_size_;
  vis_marker.scale.z = RVIZ_MARKER_HEIGHT;

  vis_marker.color.r = RVIZ_MARKER_COLOR_R;
  vis_marker.color.g = RVIZ_MARKER_COLOR_G;
  vis_marker.color.b = RVIZ_MARKER_COLOR_B;
  vis_marker.color.a = RVIZ_MARKER_COLOR_A;

  vis_marker.lifetime = ros::Duration(RVIZ_MARKER_LIFETIME);

  marker_visualization_pub_.publish(vis_marker);
}

////////////////////////////////////////////////////////////////////////////////////////////////

tf::Transform
ArucoMapping::arucoMarker2Tf(const aruco::Marker &marker)
{
  cv::Mat marker_rotation(3,3, CV_32FC1);
  cv::Rodrigues(marker.Rvec, marker_rotation);
  cv::Mat marker_translation = marker.Tvec;

  cv::Mat rotate_to_ros(3,3,CV_32FC1);
  rotate_to_ros.at<float>(0,0) = -1.0;
  rotate_to_ros.at<float>(0,1) = 0;
  rotate_to_ros.at<float>(0,2) = 0;
  rotate_to_ros.at<float>(1,0) = 0;
  rotate_to_ros.at<float>(1,1) = 0;
  rotate_to_ros.at<float>(1,2) = 1.0;
  rotate_to_ros.at<float>(2,0) = 0.0;
  rotate_to_ros.at<float>(2,1) = 1.0;
  rotate_to_ros.at<float>(2,2) = 0.0;

  marker_rotation = marker_rotation * rotate_to_ros.t();

  //cv::Mat permute(3,3,CV_32FC1);
  //rotate_to_ros.at<float>(0,0) = 0;
  //rotate_to_ros.at<float>(0,1) = 0;
  //rotate_to_ros.at<float>(0,2) = 1.0;
  //rotate_to_ros.at<float>(1,0) = 1.0;
  //rotate_to_ros.at<float>(1,1) = 0;
  //rotate_to_ros.at<float>(1,2) = 1.0;
  //rotate_to_ros.at<float>(2,0) = 0.0;
  //rotate_to_ros.at<float>(2,1) = 1.0;
  //rotate_to_ros.at<float>(2,2) = 0.0;

  //marker_rotation = marker_rotation * permute();

  // Origin solution
  tf::Matrix3x3 marker_tf_rot(marker_rotation.at<float>(0,0),marker_rotation.at<float>(0,1),marker_rotation.at<float>(0,2),
                              marker_rotation.at<float>(1,0),marker_rotation.at<float>(1,1),marker_rotation.at<float>(1,2),
                              marker_rotation.at<float>(2,0),marker_rotation.at<float>(2,1),marker_rotation.at<float>(2,2));

  tf::Vector3 marker_tf_tran(marker_translation.at<float>(0,0),
                             marker_translation.at<float>(1,0),
                             marker_translation.at<float>(2,0));

  tf::Transform transform(marker_tf_rot, marker_tf_tran);

  //tf::Quaternion rot_quat(0.0, -1.5708, 0.0);
  //transform.setRotation(transform.getRotation() * rot_quat);

  return transform;
}



}  //aruco_mapping

#endif  //ARUCO_MAPPING_CPP
