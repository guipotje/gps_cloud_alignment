# gps_cloud_alignment


This code geo-references the points output by a Structure-from-Motion algorithm using GPS information.

Required input:

A file where each line contains:

- list of GPS for each camera (Latitude Longitude Altitude)
- list of 3D poisition for each camera (X Y Z) in the local frame
- list of 3D points (X Y Z) in the local frame

Output:
- list of GPS for each point (Latitude Longitude  Altitude)


-----------------
Dependencies:

- PCL library
