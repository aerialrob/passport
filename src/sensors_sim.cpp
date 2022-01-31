
// this header incorporates all the necessary #include files and defines the class "sensors_class"
#include "sensors_sim.h"


using namespace GeographicLib;

//CONSTRUCTOR:  this will get called whenever an instance of this class is created
// want to put all dirty work of initializations here
// odd syntax: have to pass nodehandle pointer into constructor for constructor to build subscribers, etc
sensors_class::sensors_class(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
    ROS_INFO("in class constructor of sensors_class");
    initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    initializePublishers();    

    // Init params
    ros::NodeHandle nh_private_("~"); // Private nodehandle for handling parameters        
    std::vector<float> dummyParam;
    dummyParam = read_vec("odometry/std_position", 3, nh_private_);
    position_std.x = dummyParam[0];
    position_std.y = dummyParam[1];
    position_std.z = dummyParam[2];
    dummyParam = read_vec("odometry/mean_normal_position", 3, nh_private_);
    position_mean.x = dummyParam[0];
    position_mean.y = dummyParam[1];
    position_mean.z = dummyParam[2];
    dummyParam = read_vec("magnetometer/std", 3, nh_private_);
    magnetometer_std.x = dummyParam[0];
    magnetometer_std.y = dummyParam[1];
    magnetometer_std.z = dummyParam[2];
    dummyParam = read_vec("magnetometer/mean_normal", 3, nh_private_);
    magnetometer_mean.x = dummyParam[0];
    magnetometer_mean.y = dummyParam[1];
    magnetometer_mean.z = dummyParam[2];
    dummyParam = read_vec("magnetometer/magnetic_vector", 3, nh_private_);
    magnetometer_vector.x = dummyParam[0];
    magnetometer_vector.y = dummyParam[1];
    magnetometer_vector.z = dummyParam[2];
    dummyParam = read_vec("magnetometer/frequency", 1, nh_private_);
    magnetometer_freq = dummyParam[0];    
    dummyParam = read_vec("odometry/frequency", 1, nh_private_);
    odometry_freq = dummyParam[0];  
    dummyParam = read_vec("gps/frequency", 1, nh_private_);
    gps_freq = dummyParam[0];  
    dummyParam = read_vec("gps/origin", 3, nh_private_);
    params_lat0 = dummyParam[0];
    params_lon0 = dummyParam[1];
    params_h0 = dummyParam[2];    
    dummyParam = read_vec("gps/std_position", 3, nh_private_);
    gps_position_std.x = dummyParam[0];
    gps_position_std.y = dummyParam[1];
    gps_position_std.z = dummyParam[2]; 
    dummyParam = read_vec("gps/mean_normal_position", 3, nh_private_);
    gps_position_mean.x = dummyParam[0];
    gps_position_mean.y = dummyParam[1];
    gps_position_mean.z = dummyParam[2]; 

    nh_private_.param<std::string>("gps/frame_id",gps_frame_id_,"");
    nh_private_.param<std::string>("magnetometer/frame_id",mag_frame_id_,"");
    nh_private_.param<std::string>("odometry/frame_id",odom_frame_id_,"");
    nh_private_.param<std::string>("odometry/child_frame_id",odom_child_frame_id_,"");

    // Initialize variables here, as needed
    // Local cartesian origin
    Geocentric earth(Constants::WGS84_a(), Constants::WGS84_f());        
    // Sample forward calculation        
    proj_local = GeographicLib::LocalCartesian(params_lat0, params_lon0, params_h0);    
    ROS_INFO("latitude_0 [%f] longitude_0 [%f] altitude_0 [%f]", params_lat0, params_lon0, params_h0);   
            
    // Initialize threads            
    magnetometer_thread_ = std::thread(&sensors_class::ThreadFunc_magmetometer, this);
    position_thread_ = std::thread(&sensors_class::ThreadFunc_odometry, this);
    gps_thread_ = std::thread(&sensors_class::ThreadFunc_gps, this);
}

//member helper function to set up subscribers;
// note odd syntax: &sensors_class::subscriberCallback is a pointer to a member function of sensors_class
// "this" keyword is required, to refer to the current instance of sensors_class
void sensors_class::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");    
    odom_subscriber_ = nh_.subscribe("odom_in", 1, &sensors_class::odom_callback, this);    
    // add more subscribers here, as needed
}

//member helper function to set up publishers;
void sensors_class::initializePublishers()
{
    ROS_INFO("Initializing Publishers");            
    odom_publisher_ = nh_.advertise<nav_msgs::Odometry>("sensors/odometry", 1, true); 
    gps_publisher_ = nh_.advertise<sensor_msgs::NavSatFix>("sensors/gps", 1, true);
    magnetic_publisher_ = nh_.advertise<sensor_msgs::MagneticField>("sensors/magnetic_field", 1, true); 
    gps_gt_publisher_ = nh_.advertise<sensor_msgs::NavSatFix>("/ground_truth/gps", 1, true);
    rpy_publisher_ = nh_.advertise<geometry_msgs::Vector3Stamped>("/ground_truth/orientation", 1, true);
    magnetic_gt_publisher_ = nh_.advertise<sensor_msgs::MagneticField>("/ground_truth/magnetic_field_enu", 1, true); 
}

// Read vector from yaml
std::vector<float> sensors_class::read_vec(const std::string& param_name, const int& exp_long, ros::NodeHandle &nh_private_)
{
    std::vector<double> params(exp_long);    

    XmlRpc::XmlRpcValue my_list;
    nh_private_.getParam(param_name, my_list);

    if (!nh_private_.hasParam(param_name))    
        ROS_ERROR_STREAM("ERROR loading " << param_name << ": Not declared");  

    if (my_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
        int vec_size = int(my_list.size());
        if (vec_size == exp_long)
        {
            for (int ii = 0; ii < vec_size; ii++)
            {
                params[ii] = my_list[ii];     
                // std::cout << params[ii] << std::endl;
            }                         
        }
        else
            ROS_ERROR_STREAM("ERROR loading " << param_name << ": Wrong elements number");            
    }
    else
        ROS_ERROR_STREAM("ERROR loading " << param_name << ": Wrong element type");                 
    
    std::vector<float> paramsf(params.begin(), params.end());
    return paramsf;
}

void sensors_class::ThreadFunc_odometry(void)
{
    ros::Rate rate(odometry_freq);
    ROS_WARN("On Pos Thread");    
    Geocentric earth(Constants::WGS84_a(), Constants::WGS84_f());
    int seq = 0;
    while(ros::ok())
    {
        if (isfirst_odom)
        {
            // Get msg from callback and add random generator with Gaussian distribution  
            nav_msgs::Odometry odom_noise_msg = odom_msg;    
            odom_noise_msg.header.frame_id = odom_frame_id_;    
            odom_noise_msg.child_frame_id = odom_child_frame_id_;    
            std::normal_distribution<double> dist_x(position_mean.x, position_std.x);
            std::normal_distribution<double> dist_y(position_mean.y, position_std.y);
            std::normal_distribution<double> dist_z(position_mean.z, position_std.z);
            geometry_msgs::Point position_noise;
            position_noise.x = dist_x(generator);
            position_noise.y = dist_y(generator);
            position_noise.z = dist_z(generator);    
                    
            odom_noise_msg.pose.pose.position.x = odom_msg.pose.pose.position.x + position_noise.x;
            odom_noise_msg.pose.pose.position.y = odom_msg.pose.pose.position.y + position_noise.y;
            odom_noise_msg.pose.pose.position.z = odom_msg.pose.pose.position.z + position_noise.z;
            odom_noise_msg.pose.covariance[0] = position_std.x * position_std.x;
            odom_noise_msg.pose.covariance[7] = position_std.y * position_std.y;
            odom_noise_msg.pose.covariance[14] = position_std.z * position_std.z;
            // Publisher
            odom_publisher_.publish(odom_noise_msg);  

            static tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setOrigin( tf::Vector3(odom_noise_msg.pose.pose.position.x,odom_noise_msg.pose.pose.position.y,odom_noise_msg.pose.pose.position.z) );
            tf::Quaternion q;
            q.setRPY(0, 0, 0);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), odom_frame_id_, odom_child_frame_id_));

            // Create orientation msg 
            geometry_msgs::Vector3Stamped _orient_rpy;                        
            _orient_rpy.header = odom_msg.header;
            _orient_rpy.header.frame_id = odom_frame_id_;            
            tf::Quaternion quat;
            double roll, pitch, yaw;
            tf::quaternionMsgToTF(odom_msg.pose.pose.orientation, quat);
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            _orient_rpy.vector.x = roll * 180/M_PI;
            _orient_rpy.vector.y = pitch * 180/M_PI;
            _orient_rpy.vector.z = yaw * 180/M_PI;
            // Publish rpy message
            rpy_publisher_.publish(_orient_rpy);         

        }
        ros::spinOnce();
        rate.sleep();
    }
}

void sensors_class::ThreadFunc_magmetometer(void)
{
    ros::Rate rate(magnetometer_freq);
    ROS_WARN("On Mag Thread");
    while (ros::ok()) 
    {
        if (isfirst_odom)
        {            
            // Get msg from callback and add random generator with Gaussian distribution            
            std::normal_distribution<double> dist_x(magnetometer_mean.x, magnetometer_std.x);
            std::normal_distribution<double> dist_y(magnetometer_mean.y, magnetometer_std.y);
            std::normal_distribution<double> dist_z(magnetometer_mean.z, magnetometer_std.z);
            geometry_msgs::Point magnetometer_noise;
            magnetometer_noise.x = dist_x(generator);
            magnetometer_noise.y = dist_y(generator);
            magnetometer_noise.z = dist_z(generator);
                    
            // ---------Create magnetometer from orientation------------
            // Get orientation msg
            Eigen::Quaternionf qt{odom_msg.pose.pose.orientation.w, odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z};
            Eigen::Matrix3f R_temp;
            R_temp = qt.matrix();
            Eigen::Vector3f magnetic_field_vector{magnetometer_vector.x, magnetometer_vector.y, magnetometer_vector.z};        
            // Magnetic field msg 
            Eigen::Vector3f magnetic_field_msg;    
            magnetic_field_msg = R_temp.transpose() * magnetic_field_vector; 
            
            sensor_msgs::MagneticField mag_msg_from_q;            
            mag_msg_from_q.header = odom_msg.header;
            mag_msg_from_q.header.frame_id = mag_frame_id_;
            mag_msg_from_q.magnetic_field.x = magnetic_field_msg[0];
            mag_msg_from_q.magnetic_field.y = magnetic_field_msg[1];
            mag_msg_from_q.magnetic_field.z = magnetic_field_msg[2];
            mag_msg_from_q.magnetic_field_covariance[0] = 0.0;
            mag_msg_from_q.magnetic_field_covariance[4] = 0.0;
            mag_msg_from_q.magnetic_field_covariance[8] = 0.0;
            // Publisher gt
            magnetic_gt_publisher_.publish(mag_msg_from_q);  

            // Add noise
            sensor_msgs::MagneticField mag_noise_msg;
            mag_noise_msg.header = odom_msg.header;
            mag_noise_msg.header.frame_id = mag_frame_id_;
            mag_noise_msg.magnetic_field.x = mag_msg_from_q.magnetic_field.x + magnetometer_noise.x;
            mag_noise_msg.magnetic_field.y = mag_msg_from_q.magnetic_field.y + magnetometer_noise.y;
            mag_noise_msg.magnetic_field.z = mag_msg_from_q.magnetic_field.z + magnetometer_noise.z;
            mag_noise_msg.magnetic_field_covariance[0] = magnetometer_std.x * magnetometer_std.x;
            mag_noise_msg.magnetic_field_covariance[4] = magnetometer_std.y * magnetometer_std.y;
            mag_noise_msg.magnetic_field_covariance[8] = magnetometer_std.z * magnetometer_std.z;
            // Publisher
            magnetic_publisher_.publish(mag_noise_msg);

        }        
        ros::spinOnce();
        rate.sleep();
    }
}

void sensors_class::ThreadFunc_gps(void)
{
    ros::Rate rate(gps_freq);
    ROS_WARN("On GPS Thread");
    double lat, lon, h;
    double x_local, y_local, z_local;
    sensor_msgs::NavSatFix gps_msg;     
    sensor_msgs::NavSatFix gps_gt_msg;     
    while (ros::ok()) 
    {
        if (isfirst_odom)
        {            
            // Get msg from callback and add random generator with Gaussian distribution  
            nav_msgs::Odometry odom_noise_msg = odom_msg;      
            std::normal_distribution<double> dist_x(gps_position_mean.x, gps_position_std.x);
            std::normal_distribution<double> dist_y(gps_position_mean.y, gps_position_std.y);
            std::normal_distribution<double> dist_z(gps_position_mean.z, gps_position_std.z);
            geometry_msgs::Point position_noise;
            position_noise.x = dist_x(generator);
            position_noise.y = dist_y(generator);
            position_noise.z = dist_z(generator);    
                    
            odom_noise_msg.pose.pose.position.x = odom_msg.pose.pose.position.x + position_noise.x;
            odom_noise_msg.pose.pose.position.y = odom_msg.pose.pose.position.y + position_noise.y;
            odom_noise_msg.pose.pose.position.z = odom_msg.pose.pose.position.z + position_noise.z;

            // -----------GPS Reverse calculation-------------------                   
            // * @param[in] X geocentric coordinate (meters).
            // * @param[in] Y geocentric coordinate (meters).
            // * @param[in] Z geocentric coordinate (meters).
            // * @param[out] lat latitude of point (degrees).
            // * @param[out] lon longitude of point (degrees).
            // * @param[out] h height of point above the ellipsoid (meters).            
            // msg GT
            gps_gt_msg.header = odom_msg.header;
            gps_gt_msg.header.frame_id = gps_frame_id_;
            x_local = odom_msg.pose.pose.position.x;
            y_local = odom_msg.pose.pose.position.y;
            z_local = odom_msg.pose.pose.position.z;
            proj_local.Reverse(x_local, y_local, z_local, lat, lon, h);            
            gps_gt_msg.latitude = lat;  
            gps_gt_msg.longitude = lon;
            gps_gt_msg.altitude = h;            
            gps_gt_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN;
            gps_gt_publisher_.publish(gps_gt_msg);

            // msg GT + noise             
            gps_msg.header = odom_msg.header; 
            gps_msg.header.frame_id = gps_frame_id_;
            x_local = odom_noise_msg.pose.pose.position.x;
            y_local = odom_noise_msg.pose.pose.position.y;
            z_local = odom_noise_msg.pose.pose.position.z;
            proj_local.Reverse(x_local, y_local, z_local, lat, lon, h);                                      
            gps_msg.latitude = lat;  
            gps_msg.longitude = lon;
            gps_msg.altitude = h;   
            gps_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN;
            gps_msg.position_covariance[0] = gps_position_std.x * gps_position_std.x;
            gps_msg.position_covariance[4] = gps_position_std.y * gps_position_std.y;
            gps_msg.position_covariance[8] = gps_position_std.z * gps_position_std.z;
            gps_publisher_.publish(gps_msg);
        }        
        ros::spinOnce();
        rate.sleep();
    }
}

void sensors_class::odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {

    if (!isfirst_odom)
        isfirst_odom = true;

    ROS_INFO_ONCE("main: odom_callback");
    odom_msg = *msg;
}

int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "sensors_class"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    ROS_INFO("main: instantiating an object of type sensors_class");
    sensors_class sensors_class(&nh);  //instantiate an sensors_class object and pass in pointer to nodehandle for constructor to use

    ROS_INFO("main: going into spin; let the callbacks do all the work");
    ros::spin();
    return 0;
} 

