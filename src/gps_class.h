// gps_class.h header file //
// wsn; Feb, 2015
// include this file in "gps_class.cpp"

// here's a good trick--should always do this with header files:
// create a unique mnemonic for this header file, so it will get included if needed,
// but will not get included multiple times
#ifndef gps_class_H_
#define gps_class_H_

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h> //ALWAYS need to include this

//message types used in this example code;  include more message types, as needed
#include <std_msgs/Bool.h> 
#include <std_msgs/Float32.h>
#include <sensor_msgs/MagneticField.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/NavSatFix.h>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include <tf/transform_broadcaster.h>


// define a class, including a constructor, member variables and member functions
class gpsRosClass
{
public:
    gpsRosClass(ros::NodeHandle* nodehandle); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    // may choose to define public methods or public variables, if desired
private:
    // put private member data here;  "private" data will only be available to member functions of this class;
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    // some objects to support subscriber, service, and publisher
    ros::Subscriber gps_subscriber_;
    ros::Publisher gps_publisher_;

    // Initialize variables        
    nav_msgs::Odometry odom_msg_;
    geometry_msgs::Point gps_origin;

    bool isfirst_gps = false;
    
    double val_from_subscriber_; //example member variable: better than using globals; convenient way to pass data from a subscriber to other member functions
    double val_to_remember_; // member variables will retain their values even as callbacks come and go
    
    // member methods as well:
    void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
    void initializePublishers();
    void initializeServices();

    std::vector<float> read_vec(const std::string& param_name, const int& exp_long, ros::NodeHandle &nh_private_);
        
    void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    //prototype for callback for example service
    
}; // note: a class definition requires a semicolon at the end of the definition

#endif  // this closes the header-include trick...ALWAYS need one of these to match #ifndef
