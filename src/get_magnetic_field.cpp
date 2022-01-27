/**
 * @file set_datum.cpp
 * @brief Find convergence value using given location on .yaml file. Set datum param, origin of map to navsat_transform_node on robot_localization
 * @author Andres Palomino (apalomino@edgebrain.io)
 */

#include "ros/ros.h"
#include <geographic_msgs/GeoPose.h>
#include <robot_localization/SetDatum.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <GeographicLib/GeoCoords.hpp>
#include <GeographicLib/MagneticModel.hpp>
#include <GeographicLib/MagneticCircle.hpp>
#include <GeographicLib/UTMUPS.hpp>




using namespace GeographicLib;

/**
* @brief Client Node. Takes localization parameters to set orogin of map
*
******************************************************************/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "set_datum_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<robot_localization::SetDatum>("datum");
    robot_localization::SetDatum srv;

    double latitude0_param, longitude0_param, altitude0_param, convergence0_param;

    // Get Params of ENU origin
    if (ros::param::has("latitude0_deg") && ros::param::has("longitude0_deg") && ros::param::has("altitude0") )
    {        
        ros::param::get("latitude0_deg", latitude0_param);
        ros::param::get("longitude0_deg", longitude0_param);
        ros::param::get("altitude0", altitude0_param);                     
        ROS_INFO("Using params from .yaml ");
        ROS_INFO("latitude0_deg [%f] longitude0_deg [%f] altitude0 [%f]", latitude0_param, longitude0_param, altitude0_param);
    }

    // wmm2020, the World Magnetic Model 2020, which approximates the
    // main magnetic field for the period 2020-2025.  See
    // https://ngdc.noaa.gov/geomag/WMM/DoDWMM.shtml
    MagneticModel mag("wmm2020");
    double lat = 47.121, lon = 8.666, h = 1061, t = 2020; // FSR_Rothenthurm_lawnmower
    double Bx, By, Bz;  // easterly , northerly, vertical (up) nanotesla
    mag(t, lat, lon, h, Bx, By, Bz);
    ROS_INFO("Magnetic field");
    ROS_INFO_STREAM(lat << " " << lon << " " << Bx << " " << By << " " << Bz);
    // cout << lon << " " << Bx << " " << By << " " << Bz << "\n";
    
    // Sample forward calculation    
    int zone;
    bool northp;
    double x, y;
    UTMUPS::Forward(lat, lon, zone, northp, x, y);
    std::string zonestr = UTMUPS::EncodeZone(zone, northp);
    ROS_INFO_STREAM("zone " << zonestr << " x " << x << " y " << y );          
    
      // Fast method of evaluating the values at several points on a circle of
      // latitude using MagneticCircle.
    //   MagneticCircle circ = mag.Circle(t, lat, h);
    //   for (int i = -5; i <= 5; ++i) {
    //     double lon = lon0 + i * 0.2;
    //     double Bx, By, Bz;
    //     circ(lon, Bx, By, Bz);
    //     cout << lon << " " << Bx << " " << By << " " << Bz << "\n";
    //   }
    

    // Create geo_pose message
    geographic_msgs::GeoPose geo_pose;
    geo_pose.position.latitude = lat;
    geo_pose.position.longitude = lon;
    geo_pose.position.altitude = h;
    // Find convergence value from location
    GeoCoords conversion_data(lat, lon);
    float convergence_radians = conversion_data.Convergence() * M_PI/180.0;
    geo_pose.orientation = tf::createQuaternionMsgFromYaw( convergence_radians );  // Create this quaternion from yaw (in radians)
    ROS_INFO("convergence [%f] ", convergence_radians);

    // Call service
    // srv.request.geo_pose = geo_pose;
    // if (client.call(srv))
    // {   
    //     ROS_WARN("datum service sussecfull");     
    // }   
    // else
    // {
    //     ROS_ERROR("Failed to call datum service");
    //     return 1;
    // }

    return 0;
}

