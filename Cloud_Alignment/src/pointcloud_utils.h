#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <Eigen/Core>
#include <pcl/registration/icp.h>
//#include <pcl/common/transform.h>
#include "pcl/point_cloud.h"
#include "pcl/kdtree/kdtree_flann.h"
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transformation_from_correspondences.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/pca.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/registration/icp.h>

#include<vector>
#include <fstream>
#include <sstream>
#include <math.h>
#include <string>

using namespace std;

namespace pointcloud_utils
{

    class Point3D
        {
            public:
            double X,Y,Z;

            Point3D()
            {
            }

            Point3D(double _X, double _Y, double _Z)
            {
             X = _X;
             Y = _Y;
             Z = _Z;
            }

        };


    int RANSACRegister(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudA,
                const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudB,
                Eigen::Matrix4f& Tresult,
		vector<int> &inliers);


    void ScaleRANSACRegisterEx(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudA,
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudB,
                        Eigen::Matrix4f& Tresult,
                        double& in_out_s,
                        int num_iterations,
                        double iteration_scale_step);


    void ScaleRANSACRegister(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudA,
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudB,
                        Eigen::Matrix4f& Tresult,
                        double& max_s,
                        int num_iterations,
                        double iteration_scale_step);

    double norm_L2(pcl::PointXYZ p1, pcl::PointXYZ p2);

    double get_cloud_scale_pca(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudA,
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudB,
                         bool update
                        );

    double get_cloud_scale(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudA,
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudB,
                         bool update
                        );

     vector<Point3D> read_point_list(string filename);
     void save_ply(vector<Point3D> cam_list);
     void icp_AlignClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt, Eigen::Matrix4f &final_transformation_matrix);
     void register_clouds(vector<Point3D> pcl_CAM, vector<Point3D> pcl_GPS, Eigen::Matrix4f& T, double &scale);
     void transform_points(Eigen::Matrix4f T, double s, vector<Point3D> &cameras);/*transform points given an affine T matrix*/

}
