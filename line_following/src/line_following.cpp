#include "line_following.hpp"
#include "bezier.hpp"

#define DEBUG 1
#define LEFT "left"
#define RIGHT "right"

#define FOOTPPRINT_TO_HOOD 2.5

//Half of 1.2 meter wide lane
#define HALFLANEWIDTH 0.6
using namespace cv;

namespace lanekeeping
{

LaneDetection::LaneDetection(ros::NodeHandle n, ros::NodeHandle pn) :
  listener_(buffer_),
  kd_tree_(new pcl::search::KdTree<pcl::PointXYZ>)
{
  //sub_cam_info_ = n.subscribe("camera_info", 1, &LaneDetection::recvCameraInfo, this);
  //sub_image_ = n.subscribe("image_rect_color", 1, &LaneDetection::recvImage, this);
  sub_markers = n.subscribe("projected_lines",1, &LaneDetection::recvLanes, this);

  pub_markers_ = n.advertise<visualization_msgs::MarkerArray>("trajectory", 1);
 
  sub_fix = n.subscribe("/enhanced_fix",1, &LaneDetection::recvFix,this);
  pub_speed_data_ = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  srv_.setCallback(boost::bind(&LaneDetection::reconfig, this, _1, _2));
  // looked_up_camera_transform_ = false;
  marker_timer = n.createTimer(ros::Duration(0.02),&LaneDetection::TimeCallBack, this);//50hz
  error_ploting_timer = n.createTimer(ros::Duration(0.1),&LaneDetection::TimeCallBack2, this);//10 hz
  pub_error = n.advertise<std_msgs::Float64>("/error_data",1 );

 array_lines.clear();
 bez_curve.clear();

#if DEBUG
  namedWindow("Binary", CV_WINDOW_NORMAL);
#endif
}

void LaneDetection::TimeCallBack2(const ros::TimerEvent& event) // called every 0.5 sec (T = 2 s)
{
  error_msg.data = desired_y_center;
  pub_error.publish(error_msg);
}
void LaneDetection::TimeCallBack(const ros::TimerEvent& event) // called every 0.02 sec (T = 0.02 s)
{
  geometry_msgs::Twist cmd_vel;
  geometry_msgs::Point path_point_to_follow;
  if (drive_direction == "center" && generic_lane.traj_line.size()>1)
  {
  
  // normal approach [ taking the first point of the center line to follow]
  // path_point_to_follow = generic_lane.traj_line.at(0);

  // accumulating the centrlines approach
  path_point_to_follow.y = desired_y_center;
  path_point_to_follow.x = desired_x_center;
  // double x_point_to_follow,y_point_to_follow;
  // x_point_to_follow = path_point_to_follow.x;
  // y_point_to_follow = path_point_to_follow.y;
  // ROS_INFO_STREAM("the x position is = " << path_point_to_follow << "the y position is = \n" << path_point_to_follow);
  // theta_calculation(path_point_to_follow);
  
  
  
  yaw_rate = pid_controller(path_point_to_follow);
  cmd_vel.angular.z = yaw_rate;
  // ROS_INFO("the first waypoint y is %f and the yaw_rate is %f and the x value is %f", path_point_to_follow.y, yaw_rate, desired_x_center);
  cmd_vel.linear.x = cfg_.speed;
  pub_speed_data_.publish(cmd_vel);
  }

  else if (drive_direction == "langeChange" && bez_curve.size() >1)
  {
    // ROS_INFO("the bezcurve size is %d", int( bez_curve.size()));
    std::vector<geometry_msgs::Point> vehicle_bez_curve;
    // how to get the error from the global points minus the (0,0) the vehicle frame points
    to_vehicle_frame(bez_curve, vehicle_bez_curve); // convert it to vehicle frame
    // ROS_INFO("the x value of vehicle frame of bez is %f the y is %f", vehicle_bez_curve.at(0).x, vehicle_bez_curve.at(0).y);
    // which point to follow at each time 
    // when to stop following the bez and back to the center 
    change_lane(vehicle_bez_curve, jTime, path_point_to_follow );
    yaw_rate = bez_pid_controller(path_point_to_follow);
    // ROS_INFO( "the yaw rate of the bez is %f ", yaw_rate);
    
    cmd_vel.angular.z = yaw_rate;
    cmd_vel.linear.x = cfg_.speed;
    pub_speed_data_.publish(cmd_vel);
    if(jTime == INT_MAX)
     {
       ROS_INFO("now we should go to center line");
       bez_curve.resize(0);
       jTime = 0;
       drive_direction = "center";
       
      }
      global_all_center_lines.clear();
  }


}

// TODO add extract the vehicle heading and position and work on it here
void LaneDetection::recvFix(const gps_common::GPSFixConstPtr& msg)
{
  std::string current_utm_zone;
  // speed = msg->speed;


  
  double enu_heading =  ( msg->track); 
  // lamda_node = Longitude of the central meridian of the UTM zone = in michigan it is -81.
  // the general law for the converge_angle is atan(tan(geodetic cordinates (lamda) - lamda_node)*sin(theta) )
  double convergence_angle = atan(tan(M_PI/180 * ((msg->longitude)+81))*sin(M_PI/180 * msg->latitude)) ;
  utm_heading = enu_heading + convergence_angle;
  // ROS_INFO_STREAM("the heading of the car is = \n" <<utm_heading * 180 / M_PI);
  
  // Utm coordinates system considers the north is the starting point and positive towards the east side. 
  // ENU corrdinates system considers the east is the starting point and positive towards north SO counter clockwise. 

  gps_common::LLtoUTM(msg->latitude, msg->longitude,current_utm_y, current_utm_x, current_utm_zone);
  
  utm_to_vehicle.setOrigin(tf::Vector3(current_utm_x,current_utm_y,0.0));
  utm_to_vehicle.setRotation(tf::createQuaternionFromYaw(utm_heading));


}

// This function is called whenever a new image is received from either
// the live running camera driver, a bag file recording, or the simulated
// image coming from Gazebo


void LaneDetection::recvLanes(const visualization_msgs::MarkerArrayConstPtr& msg)
{



  lanes.clear();

  array_lines.clear();

   for(size_t i=0; i< msg->markers.size(); i++){
     array_lines.push_back(msg->markers[i].points);
    }


    if (array_lines.size() < 2)
    {
         //ROS_INFO(" really not enough lines ");

         populateMarkers(lanes);


        return;
  
    }


 

  std::vector<int> index;

  //This function uses the first point in each generated line to find the closest lanes to the vehicle frame and assigns them in an index of 0 or 1
  LaneDetection::smallestLines(array_lines, array_lines.size(), index);

  //These two variables represent the first y coordinates of the closest lanes
  double y00 = array_lines.at(index[0]).at(0).y;
  double y01 = array_lines.at(index[1]).at(0).y;
  // ROS_INFO("y00 = %f, y01 = %f ", y00, y01);

  //Assign the variable 'size' to the size of the shortest of the two closest lanes
  if ( array_lines.at(index[0]).size() < array_lines.at(index[1]).size())
    { size = array_lines.at(index[0]).size(); }
  else
    {
    size = array_lines.at(index[1]).size();
    }

  //The centered line to be followed takes the size of the smallest of the close lanes
  centered_line.resize(size);



std::vector<geometry_msgs::Point> the_left_line;
std::vector<geometry_msgs::Point> the_right_line;

//Check the sign of the closest lanes (index 0 and 1) Negative Sign indicates the line is to the right of the car frame, Positive is the left
//If the sign concept seems fuzzy, run the program and look at the Y-axis green arrow of base footprint
if (array_lines.at(index[0]).at(0).y < array_lines.at(index[1]).at(0).y)
{
  the_left_line = array_lines.at(index[1]);
  the_right_line = array_lines.at(index[0]);
}
else
{
  the_left_line = array_lines.at(index[0]);
  the_right_line = array_lines.at(index[1]);
}
// generate the center lane points here 
  for (size_t i=0; i < size ;i++)
  {
    centered_line[int(i)].x = (array_lines.at(index[0]).at(i).x + array_lines.at(index[1]).at(i).x) / 2;
    centered_line[int(i)].y = (array_lines.at(index[0]).at(i).y + array_lines.at(index[1]).at(i).y) / 2;
   }

  //  //  // fit the center line curve to find the value of the curve at x =0(vehicle frame) which is where the footprint is 
  // CurveFit new_curve;
  // bool successful_fit = fitPoints(centered_line, 2, new_curve);
  // ROS_INFO("The point that the footprint should follow is %f", new_curve.poly_coeff.at(0));
  // if (successful_fit) {
  //     bool good_fit = checkCurve(centered_line, new_curve);
  //     if (good_fit) {
  //         geometry_msgs::Point a0;
  //         a0.x =0;
  //         a0.y = new_curve.poly_coeff.at(0);
  //         centered_line.at(0) = a0;
  //   }
  //   }
  // // didn't work cuz it's not stable prediction and sometime it goes very far (the error)



  // ////  Acumulating Center lines Approach
  std::vector<geometry_msgs::Point> global_centered_line;
  to_global_frame(centered_line, global_centered_line);
  // add the (global frame) centered_line to global_all_center_lines 
  global_all_center_lines.insert(global_all_center_lines.end(), global_centered_line.begin(), global_centered_line.end());
  // convert global_all_center_line to vehicle frame to find the lowest X value and take it's y to follow
  to_vehicle_frame(global_all_center_lines, vehicle_all_center_lines);
  // find the lowest X value and take it's y to follow
  double x_min = INFINITY;
  int point_index;
  for (size_t i=0; i < vehicle_all_center_lines.size(); i++)
  {
    if (vehicle_all_center_lines[i].x < x_min && vehicle_all_center_lines[i].x>=4.5 && vehicle_all_center_lines[i].y <= 2 && vehicle_all_center_lines[i].y >= -2) // to element noise
    {
      point_index = int(i);
      x_min = vehicle_all_center_lines[i].x;
      desired_y_center = vehicle_all_center_lines[i].y; 
    }
  }

  // desired_y_center = vehicle_all_center_lines[point_index].y;
  desired_x_center = x_min;
  // ROS_INFO("the desired y is %f", desired_y_center);
  geometry_msgs::Point pp; pp.x = desired_x_center; pp.y = desired_y_center;
  centered_line[0] = pp;
  

  // clearing the unwanted values in global_all_center_lines
  for (size_t i = 0; i< vehicle_all_center_lines.size(); i++)
  {
    if (vehicle_all_center_lines[i].x < 4)
    {
      global_all_center_lines.erase(global_all_center_lines.begin() + i);
    }
  }

  // ROS_INFO("the size of the global_all_center is %d", (int)global_all_center_lines.size());
  vehicle_all_center_lines.clear();

  generic_lane.traj_line = centered_line;
  generic_lane.traj_type = "center";
  lanes.push_back(generic_lane);


  // LaneDetection::follow_centerLine(generic_lane.traj_line);
  
  // Change lane starting from here
  if (cfg_.turn_left) {
    // generate bez curve once
    generate_bezier_curve(LEFT, bez_curve, the_left_line, first_tangent_line, second_tangent_line);
    ROS_INFO("we started");
    cfg_.turn_left = false;
    
    drive_direction = "langeChange";

  }
  else if(cfg_.turn_right)
  {
    generate_bezier_curve(RIGHT, bez_curve, the_right_line, first_tangent_line, second_tangent_line);
    cfg_.turn_right = false;
   
    drive_direction = "langeChange";
  }

  // // visualizing bez tangents 
  // if (first_tangent_line.size() > 1)
  //   {tangents.traj_line = first_tangent_line;
  // tangents.traj_type = "bez tangent 1";
  // lanes.push_back(tangents);
  // tangents.traj_line = second_tangent_line;
  // tangents.traj_type = "bez tangent 2";
  // lanes.push_back(tangents);
  // }
  
  


  std::vector<geometry_msgs::Point> vehicle_bez_curve;
  if(bez_curve.size() > 0) // means we did generate bez curve
  {
    
    to_vehicle_frame(bez_curve, vehicle_bez_curve); // convert it to vehicle frame

    
    lane_change_bezcurve.traj_line = vehicle_bez_curve;
    lane_change_bezcurve.traj_type = "lane_switch_bezcurve";
    lanes.push_back(lane_change_bezcurve);
  
  }


  populateMarkers(lanes);
}

// calclatte the angle of the path point here
double LaneDetection::theta_calculation(geometry_msgs::Point theta_path_point)
{
  
  double X_point = (theta_path_point.x);
  double Y_point = (theta_path_point.y);
  double angle_rad = atan2(Y_point,X_point);
  double angle_deg = angle_rad * 180/M_PI;
  // ROS_INFO_STREAM("the angle in rad is = " << angle_rad <<"the angle in drgree is = " <<angle_deg);
  return (angle_rad);

}


double LaneDetection::pid_controller(geometry_msgs::Point path_point)
{
  // bcuz the car is always at (0,0,0) point we dont need to substract it from the point of the path 
  double error = path_point.y;
  
  total_error += error;
  double delta_error = error - last_error; //difference of error for derivative term

  last_error = error;
  // ROS_INFO("the erro is %f", error);
  
  // return ((cfg_.kp * error) + ( (cfg_.kd  / T) * delta_error) + ((cfg_.ki *T)*total_error)) ; // PID
  double control_cmd = (cfg_.kp * error) + ( (cfg_.kd / T) * delta_error);
  if (control_cmd > cfg_.up_sat_value){control_cmd = cfg_.up_sat_value;}

  return (control_cmd)  ; // PD
}

double LaneDetection::bez_pid_controller(geometry_msgs::Point path_point)
{
  // bcuz the car is always at (0,0,0) point we dont need to substract it from the point of the path 
  double error = path_point.y;
  // ROS_INFO( "the error of the bez is %f ", error);
  total_error += error;
  double delta_error = error - last_error; //difference of error for derivative term

  last_error = error;
  // ROS_INFO("the erro is %f", error);
  
  // return ((cfg_.kp * error) + ( (cfg_.kd  / T) * delta_error) + ((cfg_.ki *T)*total_error)) ; // PID
  double control_cmd = (cfg_.bez_kp * error) + ( (cfg_.bez_kd / T) * delta_error);
  if (control_cmd > cfg_.up_sat_value){control_cmd = cfg_.up_sat_value;}

  return (control_cmd)  ; // PD


}



//TODO: Add const to 'array_lines' for better readablity
void LaneDetection::smallestLines( std::vector<std::vector<geometry_msgs::Point>>& array_lines, int arr_size, std::vector<int>& index)
{
  int first, second;
  index.resize(2);

  double y0 ;
    index[0] = 0;
    index[1] = 0 ;
    first = second = INT_MAX;
    for (size_t i = 0; i < size_t(arr_size) ; i ++)
    {

        // Take the first point in the ith curve
        y0 = abs(array_lines.at(i).at(0).y);


        //  ROS_INFO("this is the avr %f , at i = %d",avr , int(i) );
        /* If current element is smaller than first
        then update both first and second */
        if (y0 < first )  // not to take faraway lines
        {
            second = first;
            first = y0;
            index[1] = index[0];
            index[0] = i;
        }
        /* If avr is in between first and second
        then update second */
        else if (y0 < second && y0 != first )
          {  second = y0;
            index[1] = i;}

    }
    }

void LaneDetection::follow_centerLine(const std::vector<geometry_msgs::Point>& centerLine)
{
  if (centerLine.size()<1) return;
  geometry_msgs::Twist cmd_vel;
  geometry_msgs::Point path_point_to_follow = centerLine.at(0);


  // ROS_INFO("the first waypoint y is %f", path_point_to_follow.y);
  double yaw_rate = pid_controller(path_point_to_follow);
  cmd_vel.angular.z = yaw_rate;
  cmd_vel.linear.x = cfg_.speed;
  pub_speed_data_.publish(cmd_vel);
}

// WE NEED MAGIC HERE
void LaneDetection::change_lane( std::vector<geometry_msgs::Point>& vehicle_bez_curve, double& journy_time, geometry_msgs::Point& path_point_to_follow )
{

  double hypotenuse = 0;
  double index = INT_MAX;

  for (size_t i = 0; i < vehicle_bez_curve.size(); i++ )
  {
  double x_distance = vehicle_bez_curve.at(i).x ; double y_distance = vehicle_bez_curve.at(i).y;
  hypotenuse = pow( (x_distance * x_distance + y_distance * y_distance) , 0.5);
  // x>0 to make sure that the part of the curve behind the car is not going to be selected
  if ( (hypotenuse - cfg_.look_Ahead_dist) > 0.1  && x_distance > 0)
  {
    index = i ;
    break;
  }
  }
    if(index == INT_MAX) // means that we did not find a point far from the car about look_Ahead_dist
  {
    index = vehicle_bez_curve.size() -1 ; // the last element in the curve
  }
  if (hypotenuse < cfg_.look_Ahead_dist  ) // now we are in the next lane
  {
    journy_time = INT_MAX;
  }
  //  ROS_INFO( "the index is %f , the hypo is %f", index, hypotenuse);

  // size_t target_point = (0.25 * bez_curve.size() * journy_time ) - 1;
  // bez_target_point = index;
  path_point_to_follow.x =  vehicle_bez_curve.at(index).x; path_point_to_follow.y  = vehicle_bez_curve.at(index).y;
  

}

void LaneDetection::generate_bezier_curve(const std::string& direction, std::vector<geometry_msgs::Point>& bez_curve, const std::vector<geometry_msgs::Point>& working_lane, std::vector<geometry_msgs::Point>& first_tangent_line,std::vector<geometry_msgs::Point>& second_tangent_line )
{

bool check_2p = false;
bool check_3p = false;
bool check_4p = false;
double third_point_x = 0;
double third_point_y = 0;
double fourth_point_y = 0;
double fourth_point_x = 0;
double second_point_x = 0;
double second_point_y = 0;

//The Bezier Curve is generated from 4 points corresponding at arbitrary points Y where X = 3, 4, 5, 6

//The three For loops below use pinpoint the Y location at 4, 5, and 6
//The first point @ X = 3 corresponds to the origin of the car
//The second point @ X = 4 corresponds to the center line the car is in
//For the third and fourth points @ X = 5, 6,We use the Y points of the lane we want to switch into 

  for ( size_t i = 0; i < working_lane.size(); i++) // 5
  {
    if ( (working_lane.at(i).x - 5) > 1 )
    {
      third_point_x =  working_lane.at(i).x;
      if(direction == LEFT)
      {third_point_y =  working_lane.at(i).y + (HALFLANEWIDTH);check_3p= true; break;}
      if(direction == RIGHT)
      {third_point_y =  working_lane.at(i).y - (HALFLANEWIDTH); check_3p= true; break;}
    }

  }


   for ( size_t i = 0; i < working_lane.size(); i++) // 6
  {
    if ( (working_lane.at(i).x - 6) > 1 )
    {
      fourth_point_x =  working_lane.at(i).x;
      if(direction == LEFT)
      {fourth_point_y =  working_lane.at(i).y + ( HALFLANEWIDTH); check_4p= true;  break; }
      if(direction == RIGHT)
      {fourth_point_y =  working_lane.at(i).y - (HALFLANEWIDTH); check_4p=true; break;}
      
    }
  }


  for ( size_t i = 0; i < centered_line.size(); i++) // 4
  {
    if ( (working_lane.at(i).x - 4) > 1 )
    {
      second_point_x =  centered_line.at(i).x;
      second_point_y =  centered_line.at(i).y;
      check_2p = true;
      break;
    }
  }



  // // visualize the first tangent
  // geometry_msgs::Point p;
  // p.x = cfg_.x_start;
  // p.y = cfg_.y_start;
  // first_tangent_line.push_back(p);
  // p.x = second_point_x;
  // p.y = second_point_y;
  // first_tangent_line.push_back(p);


  // //** visualize the second tangent
  // geometry_msgs::Point p2;
  // p2.x = third_point_x;
  // p2.y = third_point_y;
  // second_tangent_line.push_back(p2);
  // p2.x = fourth_point_x;
  // p2.y = fourth_point_y;
  // second_tangent_line.push_back(p2);
 


 if(check_2p && check_3p && check_4p)
 {
  double x[4] = {cfg_.x_start, second_point_x, third_point_x , fourth_point_x};
  double y[4] = {cfg_.y_start, second_point_y, third_point_y, fourth_point_y};
  // ROS_INFO("The 4th point is %f and the int of it is %d ", fourth_point_y, int(fourth_point_y));
  std::vector<geometry_msgs::Point> points;
  bezierCurve(x, y, points);
  // now I need to convert the curve to global frame
  to_global_frame(points, bez_curve);

  // viz_marker.points = bez_curve;
  // marker_msg.markers.push_back(viz_marker);
  // viz_marker.id++;

  // jTime = 1;
 }
 else
 {
   ROS_INFO("Could not get all 4 points to generate curve");

   ROS_INFO("Max X of point 3 & 4= %f", working_lane.back().x);

   ROS_INFO("Max X of point 2= %f", centered_line.back().x);

   return;
   
 }


}

void LaneDetection::to_global_frame(const std::vector<geometry_msgs::Point>& points_, std::vector<geometry_msgs::Point>& bez_curve)
{

   geometry_msgs::Point p;
  for ( size_t i=0; i<points_.size(); i++)
  {
  tf::Vector3 vehicle_frame_bezier_points(points_.at(i).x, points_.at(i).y, 0.0);
  tf::Vector3 global_frame_vect = utm_to_vehicle * vehicle_frame_bezier_points; // now I have the single waypoint position in vehicle frame
  p.x = global_frame_vect.getX();
  p.y = global_frame_vect.getY();
  p.z = 0.0;
  bez_curve.push_back(p );

  }
}

// global to vehicle curve
void LaneDetection::to_vehicle_frame(const std::vector<geometry_msgs::Point>& bez_curve, std::vector<geometry_msgs::Point>& vehicle_bez_curve)
{
  geometry_msgs::Point p;
  for(size_t i =0; i < bez_curve.size(); i++)
  {
  tf::Vector3 global_bez(bez_curve.at(i).x, bez_curve.at(i).y, 0);
  tf::Vector3 vehicle_frame_vect = utm_to_vehicle.inverse() * global_bez; // now I have the single waypoint position in vehicle frame
        p.x = vehicle_frame_vect.getX();
        p.y = vehicle_frame_vect.getY();
        vehicle_bez_curve.push_back(p );
  }
  // ROS_INFO("the x value of vehicle frame of bez is %f the y is %f", vehicle_bez_curve.at(0).x, vehicle_bez_curve.at(0).y);
}

// This function inputs a geometry points curve and applies a least squares curve fit
// to the points which fits an optimal polynomial curve of the desired order
bool LaneDetection::fitPoints(const std::vector<geometry_msgs::Point>& path, int order, CurveFit& curve)
{
  // Check if it is mathematically possible to fit a curve to the data
  if (path.size() <= order) {
    return false;
  }

  Eigen::MatrixXd regression_matrix(path.size(), order + 1);
  Eigen::VectorXd y_samples(path.size());

  curve.min_x = INFINITY;
  curve.max_x = 0.0;
  for (int i = 0; i < path.size(); i++) {
    y_samples(i) = path[i].y;

    // Fill in row of regression matrix
    // [1, x, x^2, ..., x^N]
    double tx = 1.0;
    for (int j = 0; j <= order; j++) {
      regression_matrix(i, j) = tx;
      tx *= path[i].x;
    }

    // Compute the minimum value of x to constrain
    // the polynomial curve
    if (path[i].x < curve.min_x) {
      curve.min_x = path[i].x;
    }

    // Compute the maximum value of x to constrain
    // the polynomial curve
    if (path[i].x > curve.max_x) {
      curve.max_x = path[i].x;
    }
  }

  // Invert regression matrix with left pseudoinverse operation
  Eigen::MatrixXd pseudoinverse_matrix = (regression_matrix.transpose() * regression_matrix).inverse() * regression_matrix.transpose();

  // Perform least squares estimation and obtain polynomial coefficients
  Eigen::VectorXd curve_fit_coefficients = pseudoinverse_matrix * y_samples;

  // Populate 'poly_coeff' field of the 'curve' argument output
  for (int i = 0; i < curve_fit_coefficients.rows(); i++) {
    curve.poly_coeff.push_back(curve_fit_coefficients(i));
  }

  return true; // Successful curve fit!
}

bool LaneDetection::checkCurve(const std::vector<geometry_msgs::Point>& path, const CurveFit& curve)
{
  // Compute error between each sample point y and the expected value of y
  std::vector<double> error_samples;
  for (size_t i = 0; i < path.size(); i++) {
    double new_error;

    // Compute expected y value based on the order of the curve
    // y = a0 + a1 * x + a2 * x^2 + ... + aM * x^M
    double y_hat = 0.0;
    double t = 1.0;
    for (size_t j = 0; j < curve.poly_coeff.size(); j++) {
      // Add a term to y_hat for each coefficient in the polynomial
      y_hat += t * curve.poly_coeff[j];
      t *= path[i].x;
    }
    new_error = path[i].y - y_hat;
    error_samples.push_back(new_error);
  }

  // Compute mean squared error
  double mean_square = 0;
  for (size_t i = 0; i < path.size(); i++) {
    mean_square += error_samples[i] * error_samples[i];
  }
  mean_square /= (double)path.size();

  // RMS value is the square root of the mean squared error
  double rms = sqrt(mean_square);

  // Return boolean indicating success or failure
  return rms <  0.299;
}

void LaneDetection::populateMarkers(const std::vector<Trajectory>& curves)
{

  visualization_msgs::MarkerArray marker_msg;
  visualization_msgs::Marker viz_marker;
  viz_marker.header.frame_id = "base_footprint";
  viz_marker.header.stamp = ros::Time::now();
  viz_marker.action = visualization_msgs::Marker::ADD;
  viz_marker.pose.orientation.w = 1;
  viz_marker.id = 0;

  // Use the LINE_STRIP type to display a line connecting each point
  viz_marker.type = visualization_msgs::Marker::LINE_STRIP;

  // 0.1 meters thick line
  viz_marker.scale.x = 0.1;

  // populate and publish lane
  // for each separate cluster
  for (auto& curve : curves) {

    if(curve.traj_type == "center")
    {
      // green
      viz_marker.color.a = 1.0;
      viz_marker.color.r = 0.0;
      viz_marker.color.g = 1.0;
      viz_marker.color.b = 0.0;
    }
    else if (curve.traj_type == "lane_switch")
    {
      viz_marker.color.a = 1.0;
      viz_marker.color.r = 0.0;
      viz_marker.color.g = 0.0;
      viz_marker.color.b = 1.0;
    }
    else if (curve.traj_type == "lane_switch_bezcurve")
    {
      viz_marker.color.a = 1.0;
      viz_marker.color.r = 0.0;
      viz_marker.color.g = 1.0;
      viz_marker.color.b = 0.0;
    }
    else{
      viz_marker.color.a = 1.0;
      viz_marker.color.r = 1.0;
      viz_marker.color.g = 1.0;
      viz_marker.color.b = 0.0;
    }

    viz_marker.points = curve.traj_line;
    marker_msg.markers.push_back(viz_marker);
    viz_marker.id++;
  }

  // Delete markers to avoid ghost markers from lingering if
  // the number of markers being published changes
  visualization_msgs::MarkerArray delete_markers;
  delete_markers.markers.resize(1);
  delete_markers.markers[0].action = visualization_msgs::Marker::DELETEALL;
  pub_markers_.publish(delete_markers);

  // Publish for visualization
  pub_markers_.publish(marker_msg);
  
}

// adding the cfg function here 
void LaneDetection::reconfig(Lane_KeepingConfig& config, uint32_t level){
  cfg_ = config;
}

}
