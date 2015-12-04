#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include "src/gps_utils.h"




int main(int argc, char *argv[])
{

    string gps;
    string cams;
    string points;

    gps = argv[1];
    cams = argv[2];
    points = argv[3];


    /*TEST - generate an initial ply for both GPS and camera positions*/
    GPS_Utils::save(GPS_Utils::convert_XYZ(GPS_Utils::read_gps_list(gps)));
    pointcloud_utils::save_ply(pointcloud_utils::read_point_list(cams));


    /*Finds a rigid&scale transformation using cameras*/

    vector<pointcloud_utils::Point3D> cloud_gps;
    vector<pointcloud_utils::Point3D> cloud_cam;
    vector<pointcloud_utils::Point3D> cloud_points;

    double scale;
    Eigen::Matrix4f T;

    cloud_cam = pointcloud_utils::read_point_list(cams); //reading cameras
    cloud_points = pointcloud_utils::read_point_list(cams); //reading DEM points
    cloud_gps = GPS_Utils::convert_XYZ(GPS_Utils::read_gps_list(gps)); //reading GPS and converting to ECEF xyz
    pointcloud_utils::register_clouds(cloud_cam, cloud_gps,T,scale); //registering the model frame to the ECEF frame

    /*Apply transformation and converts cameras and points to GPS LLA*/
    pointcloud_utils::transform_points(T,scale,cloud_cam);
    pointcloud_utils::transform_points(T,scale,cloud_points);



	return 0;
}











