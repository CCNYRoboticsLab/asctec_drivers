#include "math.h"
#include "gpsmath.h"

struct GPS_DATA GPS_Data;
struct GPS_DATA gps_data_temp;

unsigned int gpsDataOkTrigger=0;

void xy2latlon(double lat0, double lon0, double X, double Y, double *lat, double *lon)	//X: East, Y: North in m; lat0,lon0: Reference coordinates; lat,lon: current GPS measurement
{
        *lat=lat0+Y/MEAN_EARTH_DIAMETER*360./M_PI;
        *lon=lon0+X/MEAN_EARTH_DIAMETER*360./M_PI/cos(lat0*UMR);
}


