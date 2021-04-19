#include <math.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
namespace lanekeeping
{
void bezierCurve(double x[] , double y[], std::vector<geometry_msgs::Point>& points) 
{ 
    points.clear();
    geometry_msgs::Point p;
    double xu = 0.0 , yu = 0.0 , u = 0.0 ; 
    int i = 0 ; 
    for(u = 0.0 ; u <= 1.0 ; u += 0.01) 
    { 
        xu = pow(1-u,3)*x[0]+3*u*pow(1-u,2)*x[1]+3*pow(u,2)*(1-u)*x[2] 
             +pow(u,3)*x[3]; 
        yu = pow(1-u,3)*y[0]+3*u*pow(1-u,2)*y[1]+3*pow(u,2)*(1-u)*y[2] 
            +pow(u,3)*y[3]; 
         p.x = xu;
         p.y = yu;
         points.push_back(p);
    } 
    

}


}
