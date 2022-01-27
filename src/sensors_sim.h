// sensors_sim.h header file //
// wsn; Feb, 2015
// include this file in "sensors_sim.cpp"

// here's a good trick--should always do this with header files:
// create a unique mnemonic for this header file, so it will get included if needed,
// but will not get included multiple times
#ifndef sensors_sim_H_
#define sensors_sim_H_

#include <thread>

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <random>

// Eigen stuff
#include <eigen3/Eigen/Dense>

#include <ros/ros.h> //ALWAYS need to include this

//message types used in this example code;  include more message types, as needed
#include <std_msgs/Bool.h> 
#include <std_msgs/Float32.h>
#include <sensor_msgs/MagneticField.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/NavSatFix.h>

// GeographicLib stuff
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include <tf/transform_broadcaster.h>


// #include <example_srv/simple_bool_service_message.h> // this is a pre-defined service message, contained in shared "example_srv" package

// define a class, including a constructor, member variables and member functions
class sensors_class
{
public:
    sensors_class(ros::NodeHandle* nodehandle); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    // may choose to define public methods or public variables, if desired
private:

    std::thread magnetometer_thread_;    
    std::thread position_thread_;
    std::thread gps_thread_;

    // put private member data here;  "private" data will only be available to member functions of this class;
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    // some objects to support subscriber, service, and publisher    
    ros::Subscriber odom_subscriber_;
    ros::Subscriber magnetometer_subscriber_;
    ros::ServiceServer minimal_service_;    
    ros::Publisher magnetic_publisher_;
    ros::Publisher magnetic_from_q_publisher_;
    ros::Publisher odom_publisher_;
    ros::Publisher gps_publisher_;
    ros::Publisher gps_gt_publisher_;
    ros::Publisher rpy_publisher_;

    
    
    double val_from_subscriber_; //example member variable: better than using globals; convenient way to pass data from a subscriber to other member functions
    double val_to_remember_; // member variables will retain their values even as callbacks come and go

    bool isfirst_odom = false, isfirst_mag = false;

    // Params    
    geometry_msgs::Point position_std;
    geometry_msgs::Point position_mean;    
    geometry_msgs::Point magnetometer_std;
    geometry_msgs::Point magnetometer_mean;
    geometry_msgs::Point magnetometer_vector;
    geometry_msgs::Point gps_origin;
    geometry_msgs::Point gps_position_mean;
    geometry_msgs::Point gps_position_std;
    int magnetometer_freq, odometry_freq, gps_freq;   
    

    GeographicLib::LocalCartesian proj_local;
    double x_local0, y_local0, z_local0;

    // Msgs in callbacks
    sensor_msgs::MagneticField mag_msg;
    nav_msgs::Odometry odom_msg;

    // rand generator
    std::default_random_engine generator;
    
    // member methods as well:
    void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
    void initializePublishers();
    void initializeServices();
    void init_params(ros::NodeHandle &nh_private_);

    void ThreadFunc_magmetometer(void);
    void ThreadFunc_odometry(void);
    void ThreadFunc_gps(void);
    std::vector<float> read_vec(const std::string& param_name, const int& exp_long, ros::NodeHandle &nh_private_);

    // Callbacks    
    void magnetometer_callback(const sensor_msgs::MagneticField::ConstPtr& msg); 
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);     

}; // note: a class definition requires a semicolon at the end of the definition

#endif  // this closes the header-include trick...ALWAYS need one of these to match #ifndef

