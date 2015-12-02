#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include "src/gps_utils.h"



int main(int argc, char *argv[])
{

	string fileName;

	fileName = argv[1];

	GPS_Utils::convert_and_save(GPS_Utils::read_gps_list(fileName));


	return 0;
}











