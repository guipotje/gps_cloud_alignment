#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>


using namespace std;

namespace similarity_transf
{

	Eigen::Matrix <double,3,4> AlignPointsEigen(std::vector<Eigen::Vector3d> src, std::vector<Eigen::Vector3d> dst);
	vector<double> compute_residuals_eigen(std::vector<Eigen::Vector3d>src, std::vector<Eigen::Vector3d> dst, Eigen::Matrix <double,3,4> T);
	double media(vector<double> v);
	std::vector<Eigen::Vector3d> transform_points(std::vector<Eigen::Vector3d>src, Eigen::Matrix <double,3,4> T);

}