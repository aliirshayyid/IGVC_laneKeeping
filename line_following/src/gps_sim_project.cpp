// first, We find the refrence UTM values which we took as a parameter and we have
// our position from the simulator topic we are subscribing to. 
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>
// Navigation Satellite fix for any Global Navigation Satellite System
#include <sensor_msgs/NavSatFix.h> 
#include <nav_msgs/Path.h> 
#include <ugv_course_libs/gps_conv.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <gps_common/conversions.h>


const double FB_WHEEL = 2.65; // L
const double GEAR_RATIO = 17.3; // gamma
int target_waypoint_index = 0;
UTMCoords ref_coords;
tf::Vector3 wayPoint_relative_position; 
visualization_msgs::MarkerArray marker_msg;
visualization_msgs::Marker marker;
ros::Publisher array_pub; 
ros::Publisher pub_speed;
ros::Subscriber sub_speed;
tf::Vector3 car_relative_position;
double car_heading_actual;
geometry_msgs::Twist cmd_vel;
double car_waypoint_angle;
double convergence_angle;
double central_meridian;
bool STOP_SIGNAL = false;
tf::Transform utm_to_vehicle;
double speed;

// PID variables
double last_error;
double T= 0.02; //sample time
double kd = 6;//4
double kp = 10;//6
double ki = 2;
double total_error, error;

// speed control parameters
double abs_distance_CarWp;
#define MAXCARACCEL 46//40,45
#define MINCARACCEL 13 //9, 12

// path variables
nav_msgs::Path gps_path;
ros::Publisher path_pub;

// tunning parameters
ros::Publisher pub_error;

tf::Vector3 utm_wayPoints(int x)
{
   double a[8][2] = { {42.851358,-83.069485}, {42.851383,-83.069007},{42.852443,-83.068013},{42.852021,-83.066888},{42.851525,-83.067044},{42.851344,-83.066344},{42.850836,-83.066440},{42.849644,-83.066060}};// first is the number of group the 2nd is howmany in each group
   double r= a[x][0];
   LatLon current(a[x][0],a[x][1],0.0);
   marker.id = x;
   UTMCoords current_coords(current);
   return (current_coords - ref_coords);

}

tf::Vector3 utm_wayPoints_vehicle_frame(int x)
{
  std::string current_utm_zone;
  double waypoint_utm_y;
  double waypoint_utm_x;
  tf::Vector3 latlon_point = utm_wayPoints(x);
  gps_common::LLtoUTM(latlon_point.getX(), latlon_point.getY(),waypoint_utm_y, waypoint_utm_x, current_utm_zone);
}

int update_waypoint_index()
{
  if (target_waypoint_index > 7) {target_waypoint_index = 7;}

  tf::Vector3 goal = utm_wayPoints(target_waypoint_index);
  tf::Vector3 x = goal - car_relative_position;

  abs_distance_CarWp = sqrt(pow(abs(x.getY()),2) + pow(abs(x.getX()),2));
  if (target_waypoint_index == 7 && abs_distance_CarWp <= 1)
  {
    STOP_SIGNAL = true; // the last waypoint is reached 
  }
 
  if (abs_distance_CarWp <= 7.5)
  {
    target_waypoint_index +=1;
    return target_waypoint_index;
  }
  else
  {
    return target_waypoint_index;
  }

}

void update_path(ros::Time t)
{
  geometry_msgs::PoseStamped current_pose;
  current_pose.pose.position.x = car_relative_position.x();
  current_pose.pose.position.y = car_relative_position.y();
  
  gps_path.poses.push_back(current_pose);
  gps_path.header.frame_id="world";
  gps_path.header.stamp = t;
  path_pub.publish(gps_path);
}


double update_speed_yaw_rate_wise(double yaw_rate)
{
  if (STOP_SIGNAL){ return 0;}

  if (abs_distance_CarWp <= 2){//2
    return  0;//0
  }
  else { 
    double expp = -( abs(yaw_rate) / 1.5);
    return (MINCARACCEL + (MAXCARACCEL - MINCARACCEL) * exp(expp));
  }
  
}

double update_yaw_rate_Equation() // It didn't work for now, will check later
{
  // atan2(FB_WHEEL / );
  // R = (xt^2 + yt^2)/2yt
  double nomenator = pow(abs(wayPoint_relative_position.getY()- car_relative_position.getY()),2) + pow(abs(wayPoint_relative_position.getX()- car_relative_position.getX()),2);
  
  double R =  nomenator/(2*(wayPoint_relative_position.getY()- car_relative_position.getY()));
  double steering_angle =  -atan2(FB_WHEEL , R);
  return tan(steering_angle) * speed / FB_WHEEL;
}



double pid_controller(double desired, double actual)
{
  // p controller
   std_msgs::Float64 error_msg;

  error = desired - actual; // please remember to check the phase shift of 90 between them.
  // total_error += error;
  double delta_error = error - last_error; //difference of error for derivative term

  last_error = error;
  // to be ploted 
  error_msg.data = error;
  pub_error.publish(error_msg);
  // return ((kp * error) + ( (kd / T) * delta_error) + ((ki*T)*total_error)) ; // PID
  return ((kp * error) + ( (kd / T) * delta_error))  ; // PD
}



double theta(tf::Vector3 car_p, tf::Vector3 wayPP) // angle between the car and the waypoint
{
  // y,x target-car
  return atan2(wayPP.getY()- car_p.getY(), wayPP.getX()- car_p.getX());
}

void recvFix(const sensor_msgs::NavSatFixConstPtr& msg)
{

    // msg contains the address of car's position 
    UTMCoords current_utm(*msg); // we put * to derefrence the value of msg pointer 
    car_relative_position = current_utm - ref_coords;
}

void recvHeading(const std_msgs::Float64ConstPtr& msg ) // the heading 
{
  car_heading_actual = msg->data;

  car_heading_actual = -1 * car_heading_actual + 90 ; // robotics coordinates
  // if ( car_heading_actual <= -180)
  // {
  //   car_heading_actual += 360;
  // }
  // if (car_heading_actual >= 270 )
  // {
  //   car_heading_actual -=360;
  // }

  // Using convergance angle 
  car_heading_actual = car_heading_actual + convergence_angle;
  car_heading_actual = car_heading_actual * M_PI / 180.0;  //radian
  


}
void recv_speed(const geometry_msgs::TwistStampedConstPtr& msg)
{
  speed = msg->twist.linear.x ;
  // ROS_INFO("The speed of the car is %f", speed);

  
}
// void update_speed()
// {
//   if (abs_distance_CarWp <= 5){
//     linear_speed_cmd = 12;
//   }
//   else { linear_speed_cmd = 21;}
// }


void TimeCallBack(const ros::TimerEvent& event) // called every 0.05 sec (T = 0.05 s)
{
  update_path(event.current_real);
  marker.header.stamp = event.last_real;
  array_pub.publish(marker_msg);
  // first, which waypoint the car should be heading to 
  target_waypoint_index = update_waypoint_index();
  
  wayPoint_relative_position= utm_wayPoints(target_waypoint_index); 

  // ROS_INFO("the waypoint index is %d" , target_waypoint_index);
  // 2nd, find the angle between the target and the car 
  car_waypoint_angle = theta(car_relative_position, wayPoint_relative_position);

  // 3rd,  use a controller to make the angle smaller and smaller while driving in the X axis.
  double rotation_cmd = pid_controller(car_waypoint_angle, car_heading_actual);

  cmd_vel.angular.z = rotation_cmd;
  if (target_waypoint_index == 7 && STOP_SIGNAL == false)
  {
    cmd_vel.linear.x = 100;
  }
  else if (target_waypoint_index == 6 && abs_distance_CarWp < 25)
  {
    cmd_vel.linear.x = 70;
  }
  else{
  if (abs_distance_CarWp<7){cmd_vel.linear.x = update_speed_yaw_rate_wise(rotation_cmd)/2.4;}
  else{
  // cmd_vel.linear.x = update_speed_anglewise();
  cmd_vel.linear.x = update_speed_yaw_rate_wise(rotation_cmd);}
  }
  // ROS_INFO("The cmd is %f", cmd_vel.linear.x);
  pub_speed.publish(cmd_vel);
  
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "gps_sim_project");
  ros::NodeHandle node;
  double ref_lat;
  double ref_lon;
  node.param("/audibot/gps/ref_lat",ref_lat, 0.0);
  node.param("/audibot/gps/ref_lon",ref_lon, 0.0); 

  // Instantiate a LatLon instance with the reference geodetic coordinates
  LatLon ref_lat_lon(ref_lat, ref_lon, 0.0);
  central_meridian = ref_coords.getCentralMeridian();
  convergence_angle = atan2(tan(ref_lon- central_meridian)* sin(ref_lat),1);

  
  // pass the LatLon object instance to the UTMCoords constructor to convert 
  // to UTM and store the resulting object in the global instance.
  ref_coords = UTMCoords(ref_lat_lon);
  
  array_pub = node.advertise<visualization_msgs::MarkerArray>("markerCylinder",1);
  ros::Timer marker_timer = node.createTimer(ros::Duration(0.02),TimeCallBack);//20hz
  pub_speed = node.advertise<geometry_msgs::Twist>("/audibot/cmd_vel",1);
  pub_error = node.advertise<std_msgs::Float64>("/audibot/error",1);
  ros::Subscriber sub_heading = node.subscribe("/audibot/gps/heading",1,recvHeading);
  ros::Subscriber sub_fix = node.subscribe("/audibot/gps/fix",1,recvFix);
  sub_speed = node.subscribe("/audibot/twist", 1, recv_speed);
  path_pub = node.advertise<nav_msgs::Path>("gps_path",1);

// showing the distenation wayPoints 
  marker_msg.markers.resize(8); // make 1 markers in the array msg
  
  marker.header.frame_id= "world";
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 2.0;
  marker.scale.y = 2.0;
  marker.scale.z = 0.2;
  marker.color.a = 1.0;  // transperancy if a=0 we can't see the marker
  marker.color.r = 1.0;  // red
  marker.color.g = 0.0;  // green 
  marker.color.b = 0.0;  // blue
  marker.pose.position.x= utm_wayPoints(0).getX();
  marker.pose.position.y= utm_wayPoints(0).getY();
  marker.pose.position.z= 0.0;
  marker.id = 0; // cuz we will put it in an array
  marker_msg.markers[0] = marker; // finally, put the marker in the array
  marker.pose.position.x= utm_wayPoints(1).getX() ;
  marker.pose.position.y= utm_wayPoints(1).getY() ;
  marker_msg.markers[1] = marker; 
  // 
  marker.pose.position.x= utm_wayPoints(2).getX() ;
  marker.pose.position.y= utm_wayPoints(2).getY() ;
  marker_msg.markers[2] = marker; 

  marker.pose.position.x= utm_wayPoints(3).getX() ;
  marker.pose.position.y= utm_wayPoints(3).getY() ;
  marker_msg.markers[3] = marker; 

  marker.pose.position.x= utm_wayPoints(4).getX() ;
  marker.pose.position.y= utm_wayPoints(4).getY() ;
  marker_msg.markers[4] = marker; 

  marker.pose.position.x= utm_wayPoints(5).getX() ;
  marker.pose.position.y= utm_wayPoints(5).getY() ;
  marker_msg.markers[5] = marker; 

  marker.pose.position.x= utm_wayPoints(6).getX() ;
  marker.pose.position.y= utm_wayPoints(6).getY() ;
  marker_msg.markers[6] = marker; 

  marker.pose.position.x= utm_wayPoints(7).getX() ;
  marker.pose.position.y= utm_wayPoints(7).getY() ;
  marker_msg.markers[7] = marker; 

  ros::spin();
}