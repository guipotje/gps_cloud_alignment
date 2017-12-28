#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include "src/gps_utils.h"
#include "src/similarity_transf.h"




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
    Eigen::Matrix4f _T;
    

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

    vector<int> inliers = pointcloud_utils::register_clouds(cloud_cam, cloud_gps,_T,scale); //get inliers using RANSAC and approximate similarity t
    pointcloud_utils::filter_outliers(cloud_cam,cloud_gps,inliers);

    std::vector<Eigen::Vector3d> e_cam = pointcloud_utils::cvt_to_Eigen(cloud_cam);
    std::vector<Eigen::Vector3d> e_gps = pointcloud_utils::cvt_to_Eigen(cloud_gps);

     Eigen::Matrix <double,3,4> T = similarity_transf::AlignPointsEigen(e_cam,e_gps); //least squares using only inliers
     cout<<"Residual: "<< similarity_transf::media(similarity_transf::compute_residuals_eigen(e_cam,e_gps,T)) <<endl;

    /*Apply transformation and converts cameras and points to GPS LLA*/

    //pointcloud_utils::transform_points_double(T,scale,cloud_cam);
    //pointcloud_utils::transform_points_double(T,scale,cloud_points);

     std::vector<Eigen::Vector3d> t_cam = similarity_transf::transform_points(e_cam,T);
     std::vector<Eigen::Vector3d> t_pts = similarity_transf::transform_points(pointcloud_utils::cvt_to_Eigen(cloud_points),T);

     cloud_cam = pointcloud_utils::cvt_to_PointCloud(t_cam);
     cloud_points = pointcloud_utils::cvt_to_PointCloud(t_pts);


   /* for(size_t i=0; i< cloud_points.size(); i++)
    {
            printf("%.8f %.8f %.8f\n",cloud_points[i].X,cloud_points[i].Y,cloud_points[i].Z);

            if(i%100 ==0)
                getchar();
    }
    */

    /*save registered ply to check alignment quality*/
    pointcloud_utils::save_registered_ply(cloud_gps, cloud_cam);

    if(colored)
        pointcloud_utils::save_ply_color(cloud_points, color_list);


    //pointcloud_utils::save_ply(cloud_points); //test if points are OK


    /*saves lat long and alt for each 3D point*/
    if(!colored)
        GPS_Utils::save_LLA(GPS_Utils::convert_LLA(cloud_points));
    else
        GPS_Utils::save_LLA_RGB(GPS_Utils::convert_LLA(cloud_points), color_list);

	return 0;
}











