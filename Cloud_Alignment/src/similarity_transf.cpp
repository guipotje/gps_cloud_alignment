#include "similarity_transf.h"


namespace similarity_transf
{
	Eigen::Matrix <double,3,4> AlignPointsEigen(std::vector<Eigen::Vector3d> src, std::vector<Eigen::Vector3d> dst)
	{

	  Eigen::Matrix <double,3,4> T;
	  Eigen::Matrix<double, 3, Eigen::Dynamic> src_mat(3, src.size());
	  Eigen::Matrix<double, 3, Eigen::Dynamic> dst_mat(3, dst.size());

	  for (size_t i = 0; i < src.size(); ++i) 
	    {
	      src_mat.col(i) = src[i];
	      dst_mat.col(i) = dst[i];
	    }

	    T = Eigen::umeyama(src_mat, dst_mat, true).topLeftCorner(3, 4);

	    return T;
	}

	vector<double> compute_residuals_eigen(std::vector<Eigen::Vector3d>src, std::vector<Eigen::Vector3d> dst, Eigen::Matrix <double,3,4> T)
	{
	  
	  vector<double> residuals;

	  for(size_t i=0;i<src.size();i++)
	  {
	    Eigen::Vector3d actual = dst[i];

	    Eigen::VectorXd measured = T*src[i].homogeneous();
	    residuals.push_back((actual-measured).norm());
	  }


	return residuals;
	}

	double media(vector<double> v)
	{
	  double sum = 0;

	  for(size_t i=0;i<v.size();i++)
	    sum+=v[i];

	  sum/=(double)v.size();

	  return sum;
	}

	std::vector<Eigen::Vector3d> transform_points(std::vector<Eigen::Vector3d>src, Eigen::Matrix <double,3,4> T)
	{
		vector<Eigen::Vector3d> dst(src.size());

	  for(size_t i=0;i<src.size();i++)
	  {
	    Eigen::VectorXd measured = T*src[i].homogeneous();
	    dst[i] = measured;
	  }

	  return dst;
	}




}