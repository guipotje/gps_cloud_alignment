#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include "gps_utils.h"



int main(int argc, char *argv[])
{

	string fileName;
	double lat,longit,alt,x,y,z;


	fileName = argv[1];

	convert_and_save(read_gps_list(fileName));

	/*ifstream myfile (fileName.c_str());

	myfile>>lat>>longit>>alt;
	myfile>>x>>y>>z;


	LLA query(lat,longit,alt);
	Point3D query2(x,y,z);
	LLA result_final;

	//query.lat = lat; query.longit = longit; query.alt = alt;
  // query2.X = x; query2.Y= y; query2.Z = z;

	LLA lla;
	Point3D pt3d;

	lla2ecef(query,pt3d);
	ecef2lla(query2,lla);
	ecef2lla(pt3d,result_final);
	//ecef2lla(query,lla);
	//lla2ecef(lla,pt3d);


	cout<<"Original LLA: "<<query.lat<<" "<<query.longit<<" "<<query.alt<<endl;
	cout<<"Original XYZ: "<<query2.X<<" "<<query2.Y<<" "<<query2.Z<<endl;
	cout<<"XYZ: "<<pt3d.X<<" "<<pt3d.Y<<" "<<pt3d.Z<<endl;
	cout<<"LLA: "<<lla.lat<<" "<<lla.longit<<" "<<lla.alt<<endl;
	cout<<"Estimated LLA: "<<result_final.lat<<" "<<result_final.longit<<" "<<result_final.alt<<endl;

    */

	return 0;
}











