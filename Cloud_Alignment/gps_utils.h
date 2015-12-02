#include <math.h>
#include <vector>
#include <fstream>
#include <sstream>



using namespace std;

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


// WGS84 ellipsoid constants
const double a = 6378137; // radius
const double e = 8.1819190842622e-2;  // eccentricity

const double asq = pow(a,2);
const double esq = pow(e,2);

void ecef2lla(Point3D ecef, LLA &lla);
void lla2ecef(LLA lla, Point3D &ret);
vector<LLA> read_gps_list(string filename);
void convert_and_save(vector<LLA> gps_list);



