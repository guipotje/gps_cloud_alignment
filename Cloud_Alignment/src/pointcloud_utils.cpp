#include "pointcloud_utils.h"


namespace pointcloud_utils
{

    int RANSACRegister(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudA,
                const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudB,
                Eigen::Matrix4f& Tresult,
		vector<int> &inliers)
    {
        pcl::SampleConsensusModelRegistration<pcl::PointXYZ>::Ptr sac_model(new pcl::SampleConsensusModelRegistration<pcl::PointXYZ>(cloudA));
        sac_model->setInputTarget(cloudB);

        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(sac_model);
        //pcl::LeastMedianSquares<pcl::PointXYZ> ransac(sac_model); //might as well try these out too!
        //pcl::ProgressiveSampleConsensus<pcl::PointXYZ> ransac(sac_model);
        ransac.setDistanceThreshold(4.0);
	      ransac.setProbability(0.99999);

        //upping the verbosity level to see some info
        pcl::console::VERBOSITY_LEVEL vblvl = pcl::console::getVerbosityLevel();
        //pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);
        ransac.computeModel(1);
	      ransac.refineModel(3.0,1000);
        pcl::console::setVerbosityLevel(vblvl);

        Eigen::VectorXf coeffs;
        ransac.getModelCoefficients(coeffs);
        assert(coeffs.size() == 16);
        Tresult = Eigen::Map<Eigen::Matrix4f>(coeffs.data(),4,4);

        ransac.getInliers(inliers);
        return inliers.size();
    }

    void FindSimilarityRANSAC(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudA,
                const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudB,
                Eigen::Matrix4f& Tresult,
    vector<int> &inliers, double &scale)
    {
      int iteration_step = 10;
      int num_iters = 7;
      vector<pair<int,double> > bins(iteration_step+1);

      double init_scale = scale;

      int best_dude = -1;
      int best_idx;

      double low = init_scale - init_scale*0.5;
      double high = init_scale + init_scale*0.5;

     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_As (new pcl::PointCloud<pcl::PointXYZ>());

     for(size_t i =0; i < cloudA->points.size();i++)
       cloud_As->points.push_back(cloudA->points[i]);


      for(int i=0;i<num_iters;i++)
      {
        double delta = (high-low)/(double)iteration_step;

        printf("low:%.5f high:%.5f\n",low,high);

        for(int j=0;j<=iteration_step;j++)
        {

          double loc_scale = low + delta*j;

            for(size_t k =0; k < cloudA->points.size();k++)
           {
            cloud_As->points[k].x = cloudA->points[k].x*loc_scale;
            cloud_As->points[k].y = cloudA->points[k].y*loc_scale;
            cloud_As->points[k].z = cloudA->points[k].z*loc_scale;

           }

          bins[j].first = RANSACRegister(cloud_As,cloudB,Tresult,inliers);
          bins[j].second = loc_scale;

          printf("Found %d inliers with scale %.5f\n",bins[j].first, bins[j].second);

        }

         best_dude = -1;

        for(int j=0;j<bins.size();j++)
          if(bins[j].first > best_dude)
          {
            best_dude = bins[j].first;
            best_idx = j;
          }

        if(best_idx >0 && best_idx < bins.size())
        {
          low = bins[best_idx-1].second;
          high = bins[best_idx+1].second;
        }
        else if(best_idx==0)
        {
          low = bins[best_idx].second;
          high = bins[best_idx+1].second;
        }
        else
        {
          low = bins[best_idx-1].second;
          high = bins[best_idx].second;
        }


        for(int j=0;j<bins.size();j++)
          printf("%d ",bins[j].first);

        printf("\n");
        //getchar();

      }

      scale = bins[best_idx].second;

      for(size_t i =0; i < cloudA->points.size();i++)
      {
        cloudA->points[i].x*=scale;
        cloudA->points[i].y*=scale;
        cloudA->points[i].z*=scale;
      }


      cout<<"Best model has "<< RANSACRegister(cloudA,cloudB,Tresult,inliers) << " inliers"<<endl;

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
	    vector<int> inliers;
            int inliers_num = RANSACRegister(cloudA_trans.makeShared(),cloudB,Tresult,inliers);
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
          s = scales[scales.size()/2]; //use median

        if(update)
          for(size_t i = 0; i< cloudA->points.size();i++)
          {
              cloudA->points[i].x*=s;
              cloudA->points[i].y*=s;
              cloudA->points[i].z*=s;

          }

        cout<<"Found scale: "<<s<<endl;
        getchar();

        return s;
    }


    vector<Point3D> read_point_list(string filename)
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
        return cam_list;
    }

        vector<Point3D> read_pset(string filename)
    {
        vector<Point3D> point_set;

        ifstream myfile (filename.c_str());
        double x,y,z,nx,ny,nz;

        myfile>>x>>y>>z>>nx>>ny>>nz;

        while(!myfile.eof())
        {
            point_set.push_back(Point3D(x,y,z));
            myfile>>x>>y>>z>>nx>>ny>>nz;

        }
        return point_set;
    }

      pair<vector<Point3D>,vector<RGB> > read_ply(string filename)
    {

        vector<Point3D> point_set;
        vector<RGB> color_set;

        ifstream myfile (filename.c_str());
        double x,y,z,nx,ny,nz,r,g,b;

        string aux;

        myfile>>aux;

        while (!myfile.eof() && aux != "end_header")
        {
          myfile>>aux;
          cout<<aux;
        }


        myfile>>x>>y>>z>>nx>>ny>>nz>>r>>g>>b;

        while(!myfile.eof())
        {
            point_set.push_back(Point3D(x,y,z));
            color_set.push_back(RGB(r,g,b));
            myfile>>x>>y>>z>>nx>>ny>>nz>>r>>g>>b;

        }

        return make_pair(point_set,color_set);
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

    void save_registered_ply(vector<Point3D> GPS_list, vector<Point3D> cam_list)
   {
   static char ply_header[] =
   "ply\n"
   "format ascii 1.0\n"
   "element face 0\n"
   "property list uchar int vertex_indices\n"
   "element vertex %ld\n"
   "property double x\n"
   "property double y\n"
   "property double z\n"
   "property uchar diffuse_red\n"
   "property uchar diffuse_green\n"
   "property uchar diffuse_blue\n"
   "end_header\n";
   long num_points_out = cam_list.size()*2;

   FILE *f = fopen("registered_cameras.ply", "w");

       /* Print the ply header */
       fprintf(f, ply_header, num_points_out);

       /* X Y Z R G B for each line*/
       for(unsigned int i=0;i<cam_list.size();i++)
       {
           Point3D pt3d1 = GPS_list[i];
           Point3D pt3d2 = cam_list[i];

           unsigned char R,G,B;

           R = rand()%256;
           G = rand()%256;
           B = rand()%256;

          fprintf(f,"%.12f %.12f %.12f %d %d %d\n",pt3d1.X, pt3d1.Y, pt3d1.Z,R,G,B);
          fprintf(f,"%.12f %.12f %.12f %d %d %d\n",pt3d2.X, pt3d2.Y, pt3d2.Z,R,G,B);

       }

       fclose(f);

   }


void icp_AlignClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt, Eigen::Matrix4f &final_transformation_matrix)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr Final (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputCloud(transformed_cloud);
  	icp.setInputTarget(cloud_tgt);

        //upping the verbosity level to see some info
        pcl::console::VERBOSITY_LEVEL vblvl = pcl::console::getVerbosityLevel();
        pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);
	icp.setMaximumIterations(1);
	icp.align(*Final);
        pcl::console::setVerbosityLevel(vblvl);

	for(int i=0;i<10;i++)
	{
		cout<<"Source: "<<transformed_cloud->points[i]<<" Target: "<< cloud_tgt->points[i]<<" Transformed:" << Final->points[i] <<endl;
	}

	std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
	final_transformation_matrix = icp.getFinalTransformation();

}


   void register_clouds(vector<Point3D> pcl_CAM, vector<Point3D> pcl_GPS, Eigen::Matrix4f& T, double &scale)
   {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_CAM (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_GPS (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_CAM_in (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_GPS_in (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cameras_t (new pcl::PointCloud<pcl::PointXYZ>());
	
	Eigen::Matrix4f final_transformation_matrix;


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
        scale = get_cloud_scale(cloud_CAM,cloud_GPS,false);
        //scale = get_cloud_scale_pca(cloud_CAM,cloud_GPS,true);
        cout<<"p1: "<<cloud_CAM->points[0]<<endl;
	       vector<int> inliers;

        //cout<<"found "<< RANSACRegister(cloud_CAM,cloud_GPS,T,inliers) << " inliers."<<endl;
         FindSimilarityRANSAC(cloud_CAM,cloud_GPS,T,inliers,scale);
	       cout<<"Initial transform: "<<endl<<T.transpose()<<endl;

	/*Getting RANSAC inliers*/
	for (size_t i = 0; i < inliers.size (); ++i)
	{
	       cloud_CAM_in->points.push_back(cloud_CAM->points[inliers[i]]);
         cloud_GPS_in->points.push_back(cloud_GPS->points[inliers[i]]);
	}

  vector<Point3D> gps_inliers;

  for (size_t i = 0; i < cloud_GPS_in->points.size (); ++i)
     gps_inliers.push_back(Point3D(cloud_GPS_in->points[i].x,cloud_GPS_in->points[i].y,cloud_GPS_in->points[i].z));

  //save_ply(gps_inliers); //test if points are OK

	//cout<<"new scale "<<get_cloud_scale(cloud_CAM_in,cloud_GPS_in,true)<<endl;	

	/****ICP alignment******/

	//pcl::transformPointCloud(*cloud_CAM_in, *pcl_cameras_t, T.transpose()); //transforming cloud using the coarse estimation
	//icp_AlignClouds(pcl_cameras_t, cloud_GPS_in,final_transformation_matrix); //fine alignment
	
	//cout<<"ICP transform: "<<endl<<final_transformation_matrix<<endl<<endl;

	Eigen::Matrix4f Tf;
	Tf = T.transpose();
	T = Tf;

	//T = Tf*final_transformation_matrix;

        //cout<<scale<<endl;

   }



   void transform_points_double(Eigen::Matrix4f T, double s, vector<Point3D> &cameras)
   {
     for (size_t i = 0; i < cameras.size (); ++i)
       {

          double X,Y,Z;

          cameras[i].X = cameras[i].X*s;
          cameras[i].Y = cameras[i].Y*s;
          cameras[i].Z = cameras[i].Z*s;

          X = (double)T(0,0)*cameras[i].X + (double)T(0,1)*cameras[i].Y + (double)T(0,2)*cameras[i].Z + (double)T(0,3);        
          Y = (double)T(1,0)*cameras[i].X + (double)T(1,1)*cameras[i].Y + (double)T(1,2)*cameras[i].Z + (double)T(1,3);         
          Z = (double)T(2,0)*cameras[i].X + (double)T(2,1)*cameras[i].Y + (double)T(2,2)*cameras[i].Z + (double)T(2,3);

          cameras[i].X = X;
          cameras[i].Y = Y;
          cameras[i].Z = Z;

          //printf("%.5f %.5f %.5f\n",X,Y,Z);

          //if(i%100==0)
          //  getchar();
       }
   }

   void transform_points(Eigen::Matrix4f T, double s, vector<Point3D> &cameras)
   {

       pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cameras (new pcl::PointCloud<pcl::PointXYZ>());
       pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cameras_t (new pcl::PointCloud<pcl::PointXYZ>());

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

          //cout<<"Point "<< i+1 << " :"<<pcl_cameras_t->points[i]<<endl;

       }

       cout<<T<<endl;

   }


} //END namespace


