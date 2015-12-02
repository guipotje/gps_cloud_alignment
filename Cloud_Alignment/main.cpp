#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include "src/gps_utils.h"



int main(int argc, char *argv[])
{

	string fileName;
	double lat,longit,alt,x,y,z;

	fileName = argv[1];

	convert_and_save(read_gps_list(fileName));


	return 0;
}











