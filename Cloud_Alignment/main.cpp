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
    vector<pointcloud_utils::RGB> color_list;

    pair<vector<pointcloud_utils::Point3D>,vector<pointcloud_utils::RGB> > pt_colored;

    bool colored = false;

    double scale;
    Eigen::Matrix4f T;

    cloud_cam = pointcloud_utils::read_point_list(cams); //reading cameras

    if(points.find(".ply") ==std::string::npos)
    {
        cout<<"reading sparse points..."<<endl;
        cloud_points = pointcloud_utils::read_point_list(points); //reading sparse set of points
    }

    else
    {
        colored = true;
        cout<<"reading dense colored points..."<<endl;
        pt_colored = pointcloud_utils::read_ply(points); //reading dense set of PLY colored points
        cloud_points = pt_colored.first;
        color_list = pt_colored.second;

    }

    cloud_gps = GPS_Utils::convert_XYZ(GPS_Utils::read_gps_list(gps)); //reading GPS and converting to ECEF xyz
    pointcloud_utils::register_clouds(cloud_cam, cloud_gps,T,scale); //registering the model frame to the ECEF frame

    /*Apply transformation and converts cameras and points to GPS LLA*/
    pointcloud_utils::transform_points_double(T,scale,cloud_cam);
    pointcloud_utils::transform_points_double(T,scale,cloud_points);

    /*save registered ply to check alignment quality*/
    pointcloud_utils::save_registered_ply(cloud_gps, cloud_cam);

    //pointcloud_utils::save_ply(cloud_points); //test if points are OK


    /*saves lat long and alt for each 3D point*/
    if(!colored)
        GPS_Utils::save_LLA(GPS_Utils::convert_LLA(cloud_points));
    else
        GPS_Utils::save_LLA_RGB(GPS_Utils::convert_LLA(cloud_points), color_list);

	return 0;
}











