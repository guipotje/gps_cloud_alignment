#include "gps_utils.h"


namespace GPS_Utils
{
    void ecef2lla(pointcloud_utils::Point3D ecef, LLA &lla) /*Converts ECEF to Latitude Longitude and Altitude*/
    {
      double x = ecef.X;
      double y = ecef.Y;
      double z = ecef.Z;

      double b = sqrt( asq * (1-esq) );
      double bsq = pow(b,2);
      double ep = sqrt( (asq - bsq)/bsq);
      double p = sqrt( pow(x,2) + pow(y,2) );
      double th = atan2(a*z, b*p);

      double lon = atan2(y,x);
      double lat = atan2( (z + pow(ep,2)*b*pow(sin(th),3) ), (p - esq*a*pow(cos(th),3)) );
      double N = a/( sqrt(1-esq*pow(sin(lat),2)) );
      double alt = p / cos(lat) - N;

      // mod lat to 0-2M_PI
       //lon = lon % (2*M_PI);
      lon = fmod(lon,2*M_PI);
      // correction for altitude near poles left out.

      lla.lat = lat*(180.0/M_PI);
      lla.longit = lon*(180.0/M_PI);
      lla.alt = alt;


    }


    void lla2ecef(LLA lla, pointcloud_utils::Point3D &ret) /*Converts Latitude Longitude and Altitude to ECEF*/
    {
      double lat = lla.lat*(M_PI/180.0);
      double lon = lla.longit*(M_PI/180.0);
      double alt = lla.alt;

      double N = a / sqrt(1 - esq * pow(sin(lat),2) );

      double x = (N+alt) * cos(lat) * cos(lon);
      double y = (N+alt) * cos(lat) * sin(lon);
      double z = ((1-esq) * N + alt) * sin(lat);

      ret.X = x;
      ret.Y = y;
      ret.Z = z;

    }


    vector<LLA> read_gps_list(string filename)
    {
        vector<LLA> gps_list;

        ifstream myfile (filename.c_str());
        double lat, lon, alt;

        myfile>>lat>>lon>>alt;

        while(!myfile.eof())
        {
            gps_list.push_back(LLA(lat,lon,alt));
            myfile>>lat>>lon>>alt;

        }

        return gps_list;
    }


    vector<pointcloud_utils::Point3D> convert_XYZ (vector<LLA> gps_list)
    {
        vector<pointcloud_utils::Point3D> gps_xyz;

        for(unsigned int i=0;i<gps_list.size();i++)
        {
            pointcloud_utils::Point3D pt3d;
            lla2ecef(gps_list[i],pt3d);
            gps_xyz.push_back(pt3d);

        }

        return gps_xyz;
    }

    vector<LLA> convert_LLA(vector<pointcloud_utils::Point3D> pts_xyz)
    {
        vector<LLA> gps_list;

        for(unsigned int i=0;i<pts_xyz.size();i++)
        {
            LLA gps;
            ecef2lla(pts_xyz[i],gps);
            gps_list.push_back(gps);

        }

        return gps_list;
    }

    void save(vector<pointcloud_utils::Point3D> gps_list)
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
    long num_points_out = gps_list.size();

    FILE *f = fopen("GPS_bundle_out.ply", "w");


        /* Print the ply header */
        fprintf(f, ply_header, num_points_out);

        /* X Y Z R G B for each line*/

        for(unsigned int i=0;i<gps_list.size();i++)
        {
            pointcloud_utils::Point3D pt3d = gps_list[i];


            fprintf(f,"%.12f %.12f %.12f 0 255 0\n",pt3d.X, pt3d.Y, pt3d.Z);

        }

        fclose(f);


    }

}
