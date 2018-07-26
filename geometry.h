#ifndef GEOMETRY_H
#define GEOMETRY_H
#include <arm_neon.h>
#include <arm_acle.h>
// geometry 

#include "config.h"

void Find_Nearest_Waypoint(float cur_pos_lat, float cur_pos_lon, float * distance, float * bearing, 
			   char  * * name);

typedef struct {
	float Lat;
	float SinLat;
	float CosLat;
	float Lon;
	char Name[24];
} PT_T;

extern float p2SinLat[168],p2CosLat[168],p2Lon[168];

#endif
