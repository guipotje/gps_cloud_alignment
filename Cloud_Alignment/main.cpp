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

    gps = argv[1];
    cams = argv[2];


    /*TEST - generate an initial ply for both GPS and camera positions*/
    GPS_Utils::save(GPS_Utils::convert_XYZ(GPS_Utils::read_gps_list(gps)));
    pointcloud_utils::save_ply(pointcloud_utils::read_camera_list(cams));


    /*aligning clouds*/

    vector<pointcloud_utils::Point3D> cloud_gps;
    vector<pointcloud_utils::Point3D> cloud_cam;
    double scale;
    Eigen::Matrix4f T;

    cloud_cam = pointcloud_utils::read_camera_list(cams);
    cloud_gps = GPS_Utils::convert_XYZ(GPS_Utils::read_gps_list(gps));

    pointcloud_utils::register_clouds(cloud_cam, cloud_gps,T,scale);
    pointcloud_utils::transform_cameras(T,scale,cloud_cam);



	return 0;
}











