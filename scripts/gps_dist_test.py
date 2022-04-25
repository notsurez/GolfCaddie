#!/usr/bin/env python


from geographiclib.geodesic import Geodesic

lat1 = 43.142035
lat2 = 43.143304
long1 = -75.227344
long2 = -75.226676

def GPS_bearing():

	fwd_azimuth = Geodesic.WGS84.Inverse(lat1, long1, lat2, long2)['azi1']
	distance = Geodesic.WGS84.Inverse(lat1, long1, lat2, long2)['s12']

	if fwd_azimuth < 0:
		fwd_azimuth = fwd_azimuth + 360
		#if back_azimuth < 0:
		#    back_azimuth = back_azimuth + 360

	print("forward azimuth = " + str(fwd_azimuth))
	#print("back azimuth = " + str(back_azimuth))
	print("distance in meters = " + str(distance))

GPS_bearing()

