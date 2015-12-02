#include "pointcloud_utils.h"


int RANSACRegister(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudA, 
            const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudB,
            Eigen::Matrix4f& Tresult) 
{
    pcl::SampleConsensusModelRegistration<pcl::PointXYZ>::Ptr sac_model(new pcl::SampleConsensusModelRegistration<pcl::PointXYZ>(cloudA));
    sac_model->setInputTarget(cloudB);
 
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(sac_model);
    //pcl::LeastMedianSquares<pcl::PointXYZ> ransac(sac_model); //might as well try these out too!
    //pcl::ProgressiveSampleConsensus<pcl::PointXYZ> ransac(sac_model);
    ransac.setDistanceThreshold(0.1);
 
    //upping the verbosity level to see some info
    pcl::console::VERBOSITY_LEVEL vblvl = pcl::console::getVerbosityLevel();
    pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);
    ransac.computeModel(1);
    pcl::console::setVerbosityLevel(vblvl);
 
    Eigen::VectorXf coeffs;
    ransac.getModelCoefficients(coeffs);
    assert(coeffs.size() == 16);
    Tresult = Eigen::Map<Eigen::Matrix4f>(coeffs.data(),4,4);
 
    vector<int> inliers; ransac.getInliers(inliers);
    return inliers.size();
}


void ScaleRANSACRegisterEx(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudA, 
                    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudB,
                    Eigen::Matrix4f& Tresult,
                    double& in_out_s,
                    int num_iterations,
                    double iteration_scale_step) 
{
    double s = in_out_s;
    int max_inliers = 0; Eigen::Matrix4f max_T; double max_s = s;
 
    for(int i=-(num_iterations/2);i<=(num_iterations/2);i++) 
    //int i=0;
    {
        double _s = (s + (double)i*(s*iteration_scale_step));
        cout << "scale synth to " << _s << endl;
        Eigen::Matrix4f T = Eigen::Matrix4f(Eigen::Matrix4f::Identity());
        T.topLeftCorner(3,3) *= Eigen::Matrix3f::Identity() * _s;
        cout << "apply scale"<<endl<<T<<endl;
 
        pcl::PointCloud<pcl::PointXYZ> cloudA_trans;
        pcl::transformPointCloud<pcl::PointXYZ>(*cloudA, cloudA_trans, T);
 
        int inliers_num = RANSACRegister(cloudA_trans.makeShared(),cloudB,Tresult);
        cout << "RANSAC rigid transform:"<<endl<<Tresult.transpose()<<endl;
        cout << "RANSAC inliers:"<<inliers_num<<endl;
        cout << "------------------------------------------------------------------------" << endl;
 
        if(inliers_num>max_inliers) {
            max_inliers = inliers_num;
            max_T = Tresult;
            max_s = _s;
        }
    }
    Tresult = max_T;
    in_out_s = max_s;
}



void ScaleRANSACRegister(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudA, 
                    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudB,
                    Eigen::Matrix4f& Tresult,
                    double& max_s,
                    int num_iterations,
                    double iteration_scale_step) 
{
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cloudA);
    Eigen::Vector4f v_A_mu = pca.getMean();
    Eigen::Vector3f ev_A = pca.getEigenValues();
 
    pca.setInputCloud(cloudB);
    Eigen::Vector4f v_B_mu = pca.getMean();
    Eigen::Vector3f ev_B = pca.getEigenValues();
 
    double s = sqrt(ev_B[0])/sqrt(ev_A[0]);
 
    //rough
    ScaleRANSACRegisterEx(cloudA,cloudB,Tresult,s,num_iterations,iteration_scale_step);
    max_s = s;
 
    //fine
    ScaleRANSACRegisterEx(cloudA,cloudB,Tresult,max_s,num_iterations,iteration_scale_step/10.0);
}