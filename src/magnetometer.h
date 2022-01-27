// example_ros_class.h header file //
// wsn; Feb, 2015
// include this file in "example_ros_class.cpp"

// here's a good trick--should always do this with header files:
// create a unique mnemonic for this header file, so it will get included if needed,
// but will not get included multiple times
#ifndef EXAMPLE_ROS_CLASS_H_
#define EXAMPLE_ROS_CLASS_H_

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


// #include <example_srv/simple_bool_service_message.h> // this is a pre-defined service message, contained in shared "example_srv" package

// define a class, including a constructor, member variables and member functions
class magnetometer_class
{
public:
    magnetometer_class(ros::NodeHandle* nodehandle); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    // may choose to define public methods or public variables, if desired
private:
    // put private member data here;  "private" data will only be available to member functions of this class;
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    // some objects to support subscriber, service, and publisher
    ros::Subscriber minimal_subscriber_; //these will be set up within the class constructor, hiding these ugly details
    ros::Subscriber odom_subscriber_;
    ros::Subscriber magnetometer_subscriber_;
    ros::ServiceServer minimal_service_;
    ros::Publisher minimal_publisher_;
    ros::Publisher magnetic_publisher_;
    ros::Publisher magnetic_from_q_publisher_;
    ros::Publisher odom_publisher_;
    
    double val_from_subscriber_; //example member variable: better than using globals; convenient way to pass data from a subscriber to other member functions
    double val_to_remember_; // member variables will retain their values even as callbacks come and go
    std::string xyz_std_; // World frame ID.
    std::string robot_frame_id_; // Robot frame ID.
    std::string odom_in_frame_id_; // Odometry IN frame ID.        
    std::string odom_out_frame_id_; // Odometry OUT frame ID.
    std::string imu_frame_id_; // IMU frame ID.    
    std::string mag_frame_id_; // Magnetometer frame ID.


    geometry_msgs::Point magnetometer_noise;
    geometry_msgs::Point magnetometer_std;
    geometry_msgs::Point magnetometer_mean;
    geometry_msgs::Point magnetometer_vector;

    std::default_random_engine generator;
    
    // member methods as well:
    void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
    void initializePublishers();
    void initializeServices();    
    std::vector<float> read_vec(const std::string& param_name, const int& exp_long, ros::NodeHandle &nh_private_);
    
    void subscriberCallback(const std_msgs::Float32& message_holder); //prototype for callback of example subscriber
    void magnetometer_callback(const sensor_msgs::MagneticField::ConstPtr& msg);     
    

}; // note: a class definition requires a semicolon at the end of the definition

#endif  // this closes the header-include trick...ALWAYS need one of these to match #ifndef
