// GPSMath.h

#ifndef _GPSMATH_h
#define _GPSMATH_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

const float pi = 4.0 * atan(1.0);   //Pi - 3.1415...
const float re = 6367e3;            //mean Radius of Earth in meters
const float deg2rad = pi / 180.0;   //degree to radians

bool GPSDist(float fromLatdeg, float fromLondeg, float toLatdeg, float toLondeg, long int *dist, long int *azim)
{
	float xh, yh, zh, xt, yt, zt;
	float lat = 0;
	float lon = 0;

	//decompose coord 1 (FROM)
	lat = fromLatdeg * deg2rad;
	lon = fromLondeg * deg2rad;
	xh = cos(lon) * cos(lat);
	yh = sin(lon) * cos(lat);
	zh = sin(lat);

	//decompose coord 2 (TO)
	lat = toLatdeg * deg2rad;
	lon = toLondeg * deg2rad;
	xt = cos(lon) * cos(lat);
	yt = sin(lon) * cos(lat);
	zt = sin(lat);

	// calculate distance
	float distance = re * re * ((xh - xt) * (xh - xt) + (yh - yt) * (yh - yt) + (zh - zt) * (zh - zt));
	distance = sqrt(distance);
	float Dvar = distance / 2.0;
	float Evar = sqrt((re * re) - (Dvar * Dvar));
	float angle = atan(Dvar / Evar);
	distance = angle * re * 2.0; // (the 2.0 is because we computed the half distance)
	long azimaj, azimin; long dismaj, dismin;
	azimaj = 0;
	dismaj = 0; dismin = 0;

	if (distance == 0)
		*azim = 0;
	else
	{
		float xn = yh * zt - zh * yt;
		float yn = zh * xt - xh * zt;
		float zn = xh * yt - yh * xt;
		float rn = sqrt(xn * xn + yn * yn + zn * zn);
		xn = xn / rn;
		yn = yn / rn;
		zn = zn / rn;
		float xm = -yh;
		float ym = +xh;
		float zm = 0.0;
		float rm = sqrt(xm * xm + ym * ym + zm * zm);
		xm = xm / rm; ym = ym / rm; zm = zm / rm;
		float azimuth = xm * xn + ym * yn + zm * zn;

		azimuth = acos(azimuth) / deg2rad;		// convert rad to deg

		if (toLondeg > fromLondeg) azimuth = 180 - azimuth; else azimuth = 180 + azimuth;
		
		*azim = long(azimuth);
	}
	if (distance >= 10000.0)
	{
		*dist = distance / 1000;
		return true;
	}
	else
	{
		*dist = distance;
		return false;
	}
	
}

#endif