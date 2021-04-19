#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <sensor_msgs/Image.h>
#include <gps_common/GPSFix.h>
#include <gps_common/conversions.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/tf.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen3/Eigen/Dense>

#include <geometry_msgs/Twist.h>


// Dynamic reconfigure headers
#include <dynamic_reconfigure/server.h>
#include <line_following/Lane_KeepingConfig.h>
// #include <test_q2/LaneKeepingConfig.h>

// TF lookup headers
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

// Image processing and camera geometry headers
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

// PCL headers
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

//Lidar 3D Bbox custom message type from Michos class
#include <avs_lecture_msgs/TrackedObjectArray.h>

namespace lanekeeping 
{


typedef struct {
  std::vector<geometry_msgs::Point> traj_line; //centerd or lane change line
  std::string traj_type; //used to distinguish color of lane change trajectory and centered lane
} Trajectory;

typedef struct {
  std::vector<double> poly_coeff; // Coefficients of the polynomial
  double min_x; // Minimum x value where polynomial is defined
  double max_x; // Maximum x value where polynomial is defined
} CurveFit;


class LaneDetection
{
  public:
    LaneDetection(ros::NodeHandle n, ros::NodeHandle pn);

  private:
    // void reconfig(LaneKeepingConfig& config, uint32_t level);
    void recvFix(const gps_common::GPSFixConstPtr& msg);

    void recvLanes(const visualization_msgs::MarkerArrayConstPtr& msg);
    void smallestLines(std::vector<std::vector<geometry_msgs::Point>>& array_lines, int arr_size, std::vector<int>& index);
    double pid_controller(geometry_msgs::Point path_point); 
    double bez_pid_controller(geometry_msgs::Point path_point);
    void TimeCallBack(const ros::TimerEvent& event);
  
    void populateMarkers(const std::vector<Trajectory>& curves);
    
    void reconfig(Lane_KeepingConfig& config, uint32_t level);
    double theta_calculation(geometry_msgs::Point theta_path_point);
    void generate_bezier_curve(const std::string& direction, std::vector<geometry_msgs::Point>& bez_curve, const std::vector<geometry_msgs::Point>& working_lane, std::vector<geometry_msgs::Point>& first_tangent_line,std::vector<geometry_msgs::Point>& second_tangent_line );
    void to_global_frame(const std::vector<geometry_msgs::Point>& points_, std::vector<geometry_msgs::Point>& bez_curve);
    void to_vehicle_frame(const std::vector<geometry_msgs::Point>& bez_curve, std::vector<geometry_msgs::Point>& vehicle_bez_curve);
    void follow_centerLine(const std::vector<geometry_msgs::Point>& centerLine);
    void change_lane(std::vector<geometry_msgs::Point>& bez_curve, double& journy_time, geometry_msgs::Point& path_point_to_follow);
    bool fitPoints(const std::vector<geometry_msgs::Point>& path, int order, CurveFit& curve);
    bool checkCurve(const std::vector<geometry_msgs::Point>& path, const CurveFit& curve);
    void TimeCallBack2(const ros::TimerEvent& event);



    tf2_ros::TransformListener listener_;
    tf2_ros::Buffer buffer_;

  


    //ros::Subscriber sub_image_;
    //ros::Subscriber sub_cam_info_;
    ros::Publisher pub_markers_;
    ros::Publisher pub_speed_data_;
    ros::Subscriber sub_fix;
    ros::Subscriber sub_markers;
    ros::Timer marker_timer;
    ros::Timer error_ploting_timer;
    // error publication for plotting 
    ros::Publisher pub_error;

    //sensor_msgs::CameraInfo camera_info_;
    //tf2::Transform camera_transform_; // Coordinate transformation from footprint to camera
    //bool looked_up_camera_transform_;
    pcl::search::Search<pcl::PointXYZ>::Ptr kd_tree_;
    double current_utm_y = 0;
    double current_utm_x = 0;
    std::vector<std::vector<geometry_msgs::Point>> array_lines;
   
    std::vector<geometry_msgs::Point> centered_line;
    std::vector<geometry_msgs::Point> vehicle_all_center_lines;
    std::vector<geometry_msgs::Point> global_all_center_lines;
    size_t size;
    tf::Transform utm_to_vehicle;
    // std::vector<geometry_msgs::Point> right_lane;

    Trajectory generic_lane;
    double utm_heading;

    // PID variables
    double last_error;
    double T= 0.02; //sample time
    // double kd = 2;//4
    // double kp = 10;//6
    // double ki = 4;
    double total_error, error;
    double desired_y_center= -0.3;
    double desired_x_center;
    

    // adding the cfg controller here 
    dynamic_reconfigure::Server<Lane_KeepingConfig> srv_;
    Lane_KeepingConfig cfg_;

    // // Bezuer curve 
    std::vector<geometry_msgs::Point> first_tangent_line;
    std::vector<geometry_msgs::Point> second_tangent_line;
    std::vector<geometry_msgs::Point> bez_curve;
    double jTime = 0; // time the vehicle is following the bez curve
    size_t bez_target_point; 
    Trajectory lane_change_bezcurve;
    std::string drive_direction = "center";
    double yaw_rate;
    std_msgs::Float64 error_msg;

  std::vector<Trajectory> lanes;
  Trajectory tangents;
};

}
