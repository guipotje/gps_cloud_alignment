# gps_cloud_alignment


This code geo-references the points output by a Structure-from-Motion algorithm using the cameras' GPS information.

Required input:

3 files where each line of each respective file contains:

- GPS for each camera (Latitude Longitude Altitude)
- 3D poisition for each camera (X Y Z) in the local frame
- 3D points (X Y Z) in the local frame

Output:
- list of GPS for each point (Latitude Longitude  Altitude)

example with files that comes as example: 
"./cloud_align gps_list camera_list points_pos_list"

the later one can be a sparse set of points or a dense reconstruction by PMVS or CMVS.

-----------------
Dependencies:

- PCL library
