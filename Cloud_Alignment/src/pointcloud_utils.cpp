#include "pointcloud_utils.h"


namespace pointcloud_utils
{

    int RANSACRegister(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudA,
                const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudB,
                Eigen::Matrix4f& Tresult)
    {
        pcl::SampleConsensusModelRegistration<pcl::PointXYZ>::Ptr sac_model(new pcl::SampleConsensusModelRegistration<pcl::PointXYZ>(cloudA));
        sac_model->setInputTarget(cloudB);

        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(sac_model);
        //pcl::LeastMedianSquares<pcl::PointXYZ> ransac(sac_model); //might as well try these out too!
        //pcl::ProgressiveSampleConsensus<pcl::PointXYZ> ransac(sac_model);
        ransac.setDistanceThreshold(5.0);

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

    double get_cloud_scale_pca(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudA,
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudB,
                         bool update
                        )
    {
        pcl::PCA<pcl::PointXYZ> pca;
        pca.setInputCloud(cloudA);
        Eigen::Vector4f v_A_mu = pca.getMean();
        Eigen::Vector3f ev_A = pca.getEigenValues();

        pca.setInputCloud(cloudB);
        Eigen::Vector4f v_B_mu = pca.getMean();
        Eigen::Vector3f ev_B = pca.getEigenValues();

        double s = sqrt(ev_B[0])/sqrt(ev_A[0]);

        if(update)
          for(size_t i = 0; i< cloudA->points.size();i++)
          {
              cloudA->points[i].x*=s;
              cloudA->points[i].y*=s;
              cloudA->points[i].z*=s;

          }


        return s;
    }


   double norm_L2(pcl::PointXYZ p1, pcl::PointXYZ p2)
   {
       return sqrt(pow(p1.x-p2.x,2) + pow(p1.y-p2.y,2) + pow(p1.z-p2.z,2));
   }

    double get_cloud_scale(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudA,
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudB,
                         bool update
                        )
    {

     double s=0;
     double c=0;

     vector<double> scales;

        for(size_t i = 0; i< cloudA->points.size();i++)
           for(size_t j = i+1; j< cloudA->points.size();j++)
           {
              double d1 = norm_L2(cloudA->points[i],cloudA->points[j]);
              double d2 = norm_L2(cloudB->points[i],cloudB->points[j]);

                if(d1!=0)
                {
                s+=(d2/d1);
                scales.push_back(d2/d1);
                }
                c++;
           }

          //s/=c; //use mean

          std::sort(scales.begin(),scales.end());
          s = scales[scales.size()/2 -1]; //use median

        if(update)
          for(size_t i = 0; i< cloudA->points.size();i++)
          {
              cloudA->points[i].x*=s;
              cloudA->points[i].y*=s;
              cloudA->points[i].z*=s;

          }


        return s;
    }


    vector<Point3D> read_camera_list(string filename)
    {
        vector<Point3D> cam_list;

        ifstream myfile (filename.c_str());
        double x,y,z;

        myfile>>x>>y>>z;

        while(!myfile.eof())
        {
            cam_list.push_back(Point3D(x,y,z));
            myfile>>x>>y>>z;

        }
        isnan(x);
        return cam_list;
    }


    void save_ply(vector<Point3D> cam_list)
   {
   static char ply_header[] =
   "ply\n"
   "format ascii 1.0\n"
   "element face 0\n"
   "property list uchar int vertex_indices\n"
   "element vertex %ld\n"
   "property float x\n"
   "property float y\n"
   "property float z\n"
   "property uchar diffuse_red\n"
   "property uchar diffuse_green\n"
   "property uchar diffuse_blue\n"
   "end_header\n";
   long num_points_out = cam_list.size();

   FILE *f = fopen("CAMS_bundle_out.ply", "w");


       /* Print the ply header */
       fprintf(f, ply_header, num_points_out);

       /* X Y Z R G B for each line*/

       for(unsigned int i=0;i<cam_list.size();i++)
       {
           Point3D pt3d = cam_list[i];



           fprintf(f,"%.12f %.12f %.12f 0 255 0\n",pt3d.X, pt3d.Y, pt3d.Z);

       }

       fclose(f);


   }

   void register_clouds(vector<Point3D> pcl_CAM, vector<Point3D> pcl_GPS, Eigen::Matrix4f& T, double &scale)
   {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_CAM (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_GPS (new pcl::PointCloud<pcl::PointXYZ>);


        for (size_t i = 0; i < pcl_GPS.size (); ++i)
        {

           if(!isnan(pcl_CAM[i].X) && !isnan(pcl_CAM[i].Y) && !isnan(pcl_CAM[i].Z))
           {
               cloud_CAM->points.push_back(pcl::PointXYZ(pcl_CAM[i].X, pcl_CAM[i].Y, pcl_CAM[i].Z));
               cloud_GPS->points.push_back(pcl::PointXYZ(pcl_GPS[i].X, pcl_GPS[i].Y, pcl_GPS[i].Z));

           }
        }

       /* ScaleRANSACRegister(cloud_CAM,
                                cloud_GPS,
                                T,
                                scale,
                                1, //number of iterations
                                0.1); //step size

       */

        cout<<"cloud size "<<cloud_CAM->points.size()<<endl;
        scale = get_cloud_scale(cloud_CAM,cloud_GPS,true);
        //scale = get_cloud_scale_pca(cloud_CAM,cloud_GPS,true);
        cout<<"p1 "<<cloud_CAM->points[0]<<endl; getchar();
        RANSACRegister(cloud_CAM,cloud_GPS,T);
        cout<<scale<<endl;

   }


   void transform_cameras(Eigen::Matrix4f T, double s, vector<Point3D> &cameras)
   {

       pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cameras (new pcl::PointCloud<pcl::PointXYZ>);
       pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cameras_t (new pcl::PointCloud<pcl::PointXYZ>);

       for (size_t i = 0; i < cameras.size (); ++i)
       {
            pcl_cameras->points.push_back(pcl::PointXYZ(cameras[i].X*s, cameras[i].Y*s, cameras[i].Z*s)); //scaling

       }

       pcl::transformPointCloud(*pcl_cameras, *pcl_cameras_t, T);



       for (size_t i = 0; i < cameras.size (); ++i)
       {
           cameras[i].X = pcl_cameras_t->points[i].x;
           cameras[i].Y = pcl_cameras_t->points[i].y;
           cameras[i].Z = pcl_cameras_t->points[i].z;

         //  cout<<"Point "<< i << " :"<<pcl_cameras->points[i]<<endl;

       }

   }


} //END namespace


