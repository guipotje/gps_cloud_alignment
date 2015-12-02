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




#include "gps_utils.h"


int RANSACRegister(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudA, 
            const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudB,
            Eigen::Matrix4f& Tresult);


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