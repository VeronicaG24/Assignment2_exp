/*****************************
 Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are
 permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this list of
 conditions, and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice, this list
 of conditions, and the following disclaimer in the documentation and/or other materials
 provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 The views and conclusions contained in the software and documentation are those of the
 authors and should not be interpreted as representing official policies, either expressed
 or implied, of Rafael Muñoz Salinas.
 ********************************/
/**
 * @file marker_publish.cpp
 * @author Bence Magyar
 * @date June 2014
 * @brief Modified copy of simple_single.cpp to publish all markers visible
 * (modified by Josh Langsfeld, 2014)
 */

#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

class ArucoMarkerPublisher {
	private:
	  // ArUco stuff
	  aruco::MarkerDetector mDetector_;
	  std::vector<aruco::Marker> markers_;
	  aruco::CameraParameters camParam_;
	  
	  // Message
	  geometry_msgs::Point marker_center;
	  std_msgs::Float64 pixel_msg;
	  std_msgs::Int32 marker_id_msg;

	  // Node parameters
	  double marker_size_;
	  bool useCamInfo_;
	  std::vector<int> marker_detect;

	  // ROS pub-sub
	  ros::NodeHandle nh_;
	  image_transport::ImageTransport it_;
	  image_transport::Subscriber image_sub_;
	 
	  // Publisher for marker point
	  ros::Publisher marker_point_publisher;
	  // Publisher for number of pixel of the side of the marker
	  ros::Publisher pixel_side_publisher;
	  // Publisher for id of the marker
	  ros::Publisher id_pub;

	  image_transport::Publisher image_pub_;
	  image_transport::Publisher debug_pub_;

	  cv::Mat inImage_;
  
	public:
	  ArucoMarkerPublisher() : 
	  nh_("~"), it_(nh_), useCamInfo_(true) {
		    // Subscribe to the input image topic
		    image_sub_ = it_.subscribe("/image", 1, &ArucoMarkerPublisher::image_callback, this);
		    
		    // Advertise publishers for marker-related messages
		    image_pub_ = it_.advertise("result", 1);
		    debug_pub_ = it_.advertise("debug", 1);
		    
		    // Get parameters from the parameter server
		    nh_.param<bool>("use_camera_info", useCamInfo_, false);
		    camParam_ = aruco::CameraParameters();
		    
		    // Advertise publishers for marker-related messages
		    marker_point_publisher = nh_.advertise<geometry_msgs::Point>("/marker_point", 10);
		    pixel_side_publisher = nh_.advertise<std_msgs::Float64>("/pixel_side_marker", 10);
		    id_pub = nh_.advertise<std_msgs::Int32>("/id_publisher", 10);
	  }

	  // Callback function for processing input images
	  void image_callback(const sensor_msgs::ImageConstPtr& msg) {
	  
	    bool publishImage = image_pub_.getNumSubscribers() > 0;
	    bool publishDebug = debug_pub_.getNumSubscribers() > 0;

	   // Get marker detection list from the parameter server
	    if (!nh_.getParam("/marker_publisher/marker_list", marker_detect)) {
	      ROS_ERROR("Unable to get the parameter '/marker_publisher/marker_list'");
        }

	    // Get the timestamp of the current image
	    ros::Time curr_stamp = msg->header.stamp;
	    cv_bridge::CvImagePtr cv_ptr;
	    
	    try {
	      // Convert the ROS image message to a CV image
	      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	      inImage_ = cv_ptr->image;
	   
	      // Clear out previous detection results
	      markers_.clear();

	      // Detect markers
	      mDetector_.detect(inImage_, markers_, camParam_, marker_size_, false);

	      // Process each detected marker
	      for (std::size_t i = 0; i < markers_.size(); ++i) {
		std::cout << "The id of the detected marker is: ";
		std::cout << markers_.at(i).id << " ";
		
		// Send the current marker ID
		marker_id_msg.data = markers_.at(i).id;
		id_pub.publish(marker_id_msg);
		
		// Send the current marker center position
		marker_center.x = markers_.at(i).getCenter().x;
		marker_center.y = markers_.at(i).getCenter().y;
		marker_point_publisher.publish(marker_center);
		
		// Send the side pixel
		pixel_msg.data = (markers_.at(i).getPerimeter()) / 4.0;
		pixel_side_publisher.publish(pixel_msg);
	      }
	      
	      std::cout << std::endl;

	      // Draw detected markers on the image for visualization
	      for (std::size_t i = 0; i < markers_.size(); ++i) {
		markers_[i].draw(inImage_, cv::Scalar(0, 0, 255), 2);
	      }

	      // Publish the input image with markers drawn on it
	      if (publishImage) {
		cv_bridge::CvImage out_msg;
		out_msg.header.stamp = curr_stamp;
		out_msg.encoding = sensor_msgs::image_encodings::RGB8;
		out_msg.image = inImage_;
		image_pub_.publish(out_msg.toImageMsg());
	      }

	      // Publish the image after internal image processing
	      if (publishDebug) {
		cv_bridge::CvImage debug_msg;
		debug_msg.header.stamp = curr_stamp;
		debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
		debug_msg.image = mDetector_.getThresholdedImage();
		debug_pub_.publish(debug_msg.toImageMsg());
	      }

	      // Shut down if the marker detection list is empty
	      if (!marker_detect.size()) {
		ROS_INFO("Shutdown requested");
		ros::shutdown();
	      }
	    }
	    
	    catch (cv_bridge::Exception& e) {
	      ROS_ERROR("cv_bridge exception: %s", e.what());
	    }
  	}
};

// Main function to initialize the node and start processing
int main(int argc, char **argv) {
  ros::init(argc, argv, "aruco_marker_publisher");

  // Create an instance of the ArucoMarkerPublisher class
  ArucoMarkerPublisher node;

  // Start the ROS event loop
  ros::spin();
}

