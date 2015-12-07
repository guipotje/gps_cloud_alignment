#include <math.h>
#include <vector>
#include <fstream>
#include <sstream>
#include "pointcloud_utils.h"


using namespace std;

namespace GPS_Utils
{
	class LLA //Latitude, Longitude and Altitude struct
	{
		public:
		double lat,longit,alt;

		LLA(double _lat, double _longit, double _alt)
		{
		 lat = _lat;
		 longit = _longit;
		 alt = _alt;
		}

		LLA ()
		{
		}

	};


	// WGS84 ellipsoid constants
	const double a = 6378137; // radius
	const double e = 8.1819190842622e-2;  // eccentricity

	const double asq = pow(a,2);
	const double esq = pow(e,2);



    void ecef2lla(pointcloud_utils::Point3D ecef, LLA &lla);
    void lla2ecef(LLA lla, pointcloud_utils::Point3D &ret);
	vector<LLA> read_gps_list(string filename);
    vector<pointcloud_utils::Point3D> convert_XYZ (vector<LLA> gps_list);
    vector<LLA> convert_LLA(vector<pointcloud_utils::Point3D> pts_xyz);
    void save(vector<pointcloud_utils::Point3D> gps_list);
    void save_LLA(vector<LLA> lla_list);


}


