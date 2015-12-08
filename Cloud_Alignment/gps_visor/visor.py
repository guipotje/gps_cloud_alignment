#! /usr/bin/perl

import random
import argparse

gps_file_a = {
	'gps_list' : []
}

gps_file_b = {
	'gps_list' : []
}

basic_google_maps_string = "<!DOCTYPE html><html><head> <meta name='viewport' content='initial-scale=1.0, user-scalable=no'> <meta charset='utf-8'> <title>GPS compare</title> <style>html, body, #map-canvas{{height: 100%; margin: 0px; padding: 0px}}</style> <script src='https://maps.googleapis.com/maps/api/js?v=3.exp'></script> <script>function initialize(){{var gpsCoords=[ {0} ]; var mapOptions={{zoom: 16, center: gpsCoords[0], mapTypeId: google.maps.MapTypeId.SATELLITE}}; var map=new google.maps.Map(document.getElementById('map-canvas'), mapOptions); gpsCoords.forEach(function(gpsPoint){{new google.maps.Circle({{strokeColor: '#ff0000', strokeOpacity: 0.9, strokeWeight: 2, fillColor: '#ff0000', fillOpacity: 0.4, map: map, center: gpsPoint, radius: 1.5}});}});}}google.maps.event.addDomListener(window, 'load', initialize); </script></head><body> <div id='map-canvas'></div></body></html>"
double_google_maps_string = "<!DOCTYPE html><html><head> <meta name='viewport' content='initial-scale=1.0, user-scalable=no'> <meta charset='utf-8'> <title>Simple Polylines</title> <style>html, body, #map-canvas{{height: 100%; margin: 0px; padding: 0px}}div.wrap{{float: left; width: 50%;}}div.wrap2{{float: right; width: 50%;}}</style> <script src='https://maps.googleapis.com/maps/api/js?v=3.exp'></script> <script>function initialize(){{var gpsCoords=[ {0} ]; var gpsCoords2=[ {1} ]; var mapOptions={{zoom: 17, center: gpsCoords[0], mapTypeId: google.maps.MapTypeId.SATELLITE}}; var map=new google.maps.Map(document.getElementById('map-canvas'), mapOptions); var map2=new google.maps.Map(document.getElementById('map-canvas2'), mapOptions); gpsCoords.forEach(function(gpsPoint){{new google.maps.Circle({{strokeColor: '#ff0000', strokeOpacity: 0.9, strokeWeight: 2, fillColor: '#ff0000', fillOpacity: 0.4, map: map, center: gpsPoint, radius: 0.6}});}}); gpsCoords2.forEach(function(gpsPoint){{new google.maps.Circle({{strokeColor: '#ff0000', strokeOpacity: 0.9, strokeWeight: 2, fillColor: '#ff0000', fillOpacity: 0.4, map: map2, center: gpsPoint, radius: 0.6}});}}); google.maps.event.addListener(map, 'bounds_changed', (function(){{map2.setCenter(map.getCenter());}})); map.addListener('zoom_changed', function(){{map2.setZoom(map.getZoom());}});}}google.maps.event.addDomListener(window, 'load', initialize); </script></head><body> <div style='width:100%; height:100%;'> <div id='map-canvas' class='wrap' style='width:50%; height:100%; background-color:orange;'>Loading map A ...</div><div id='map-canvas2' class='wrap2' style='width:50%; height:100%; background-color:blue;'>Loading map B ...</div></div></body></html>"

def import_gps_points(filename_a, filename_b):

	global gps_file_a
	global gps_file_b

	gps_file_a['gps_list'] = get_gps_from_file(filename_a)
	gps_file_b['gps_list'] = get_gps_from_file(filename_b)

	pass

def get_gps_from_file(filename):
	gps_list = []

	if(file_len(filename) > 4000):
		lines_list = open(filename).readlines()
		for i in xrange(4000):
			random_line = random.choice(lines_list)
			points = random_line.split()
			gps_list.append( (points[0], points[1], points[2]) )
		pass
	else:
		with open(filename, "r") as ins:
			for line in ins:
				points = line.split()
				gps_list.append( (points[0], points[1], points[2]) )
				pass

	return gps_list

def get_gps_formatted_from_list(gps_list):
	gps_template = 'new google.maps.LatLng({0}, {1}),'
	gps_formatted_string = ''

	for gps in gps_list:
		gps_formatted_string  += gps_template.format(gps[0] , gps[1])
		pass

	gps_formatted_string = gps_formatted_string[:-1]
	return gps_formatted_string

def random_line(filename):
	afile = open(filename)

	line = next(afile)
	for num, aline in enumerate(afile):
		if random.randrange(num + 2): continue
		line = aline
	return line

def file_len(filename):
	with open(filename) as f:
		for i, l in enumerate(f):
			pass
	return i + 1

def main(filename_a, filename_b, out_filename):
	in_filename_a = filename_a
	in_filename_b = filename_b

	if not out_filename.endswith(('.html')):
		out_filename += '.html'

	import_gps_points(in_filename_a, in_filename_b)

	gps_formatted_a = get_gps_formatted_from_list(gps_file_a['gps_list'])
	gps_formatted_b = get_gps_formatted_from_list(gps_file_b['gps_list'])

	html = double_google_maps_string.format(gps_formatted_a, gps_formatted_b)
	
	with open(out_filename, "w") as text_file:
		text_file.write(html)

	pass

if __name__ == "__main__":
	parser = argparse.ArgumentParser(description='Script to compare 2 gps point clouds')
	parser.add_argument('-a', '--filea', type=str, help='Filepath for the first GPS file', required=True)
	parser.add_argument('-b', '--fileb', type=str, help='Filepath for the second GPS file', required=True)
	parser.add_argument('-o', '--output', type=str, help='Output file', required=False, default='output.html')

	args = parser.parse_args()

	main(args.filea, args.fileb, args.output)