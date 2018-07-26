#include "geometry.h"
#include <math.h>
#include <string.h>
#include <arm_neon.h>
#include <arm_acle.h>
#include "proto.h"
#include "neon_sincos.h"
#define PI 3.14159265f
#define PI_OVER_180 (0.017453293f) // (3.1415927/180.0)

extern const PT_T waypoints[];

#define VER 12  

#define OPT 2

/*
10: radians in table, precalc'd sin, cos
11: Calc_Closeness
12: Don't do bearing
*/

// Table holds precalculated sin/cos for p2. Table Lat/Lon values are in radians

#if VER==10
float Calc_Distance( PT_T * p1,  const PT_T * p2) { 
  // calculates distance in kilometers between locations (represented in radians)

  return acosf(p1->SinLat * p2->SinLat + 
	       p1->CosLat * p2->CosLat*
	       cosf(p2->Lon - p1->Lon))* 6371;
}
#endif

float Calc_Bearing( PT_T * p1,  const PT_T * p2){
  // calculates bearing in degrees between locations (represented in radians)	
  float term1, term2;
  float angle;
	
  term1 = sinf(p1->Lon - p2->Lon)*p2->CosLat;
  term2 = p1->CosLat*p2->SinLat - 
    p1->SinLat*p2->CosLat*cosf(p1->Lon - p2->Lon);
  angle = atan2f(term1, term2) * (180/PI);
  if (angle < 0.0)
    angle += 360;
  return angle;
}

#if (VER==11) || (VER==12)
inline float Calc_Closeness( PT_T * p1,  const PT_T * p2) { 
  // calculates closeness (decreases as distance increases) of location

  return p1->SinLat * p2->SinLat + 
    p1->CosLat * p2->CosLat*
    cosf(p2->Lon - p1->Lon);
}


#endif


void Find_Nearest_Waypoint(float cur_pos_lat, float cur_pos_lon, float * distance, float * bearing, 
			   char  * * name) {
  // cur_pos_lat and cur_pos_lon are in degrees
  // distance is in kilometers
  // bearing is in degrees
		
  int i=0,closest_i=0;
  PT_T ref;
  float d, b,c, max_c=0,closest_d = 1E10;

  #if OPT==1 || OPT==2
    int j = 0;

    float p1SinLat; 
    float p1CosLat;
    float p1Lon;

    float32x4_t v4_p1SinLat;
    float32x4_t v4_p1CosLat;
    float32x4_t v4_p2SinLat;
    float32x4_t v4_p2CosLat;
    float32x4_t v4_p1Lon;
    float32x4_t v4_p2Lon;
  #endif

  *distance = 0.0f;
  *bearing = 0;
  *name = '\0';

  ref.Lat = cur_pos_lat*PI/180;
  ref.Lon = cur_pos_lon*PI/180;
  ref.SinLat = sinf(ref.Lat);
  ref.CosLat = cosf(ref.Lat);
		
  strcpy(ref.Name, "Reference");

  #if OPT==0
    while (strcmp(waypoints[i].Name, "END")) {
      #if VER==10
        d = Calc_Distance(&ref, &(waypoints[i]) );
      // if we found a closer waypoint, remember it and display it
        if (d<closest_d) {
          closest_d = d;
          closest_i = i;
        }
      #else
        c = Calc_Closeness(&ref, &(waypoints[i]) );
        if (c>max_c) {
          max_c = c;
          closest_i = i;
        }
      #endif

      #if (VER==10) || (VER==11)
        b = Calc_Bearing(&ref, &(waypoints[i]) ); // Deletable!
      #endif

      i++;
    }
    //printf("%f\n",max_c);
  #endif

  #if OPT==1 || OPT==2
    p1SinLat = ref.SinLat;
    p1CosLat = ref.CosLat;
    p1Lon = ref.Lon;
    v4_p1SinLat = vld1q_dup_f32(&p1SinLat);
    v4_p1CosLat = vld1q_dup_f32(&p1CosLat);
    v4_p1Lon = vld1q_dup_f32(&p1Lon);
    int n = 1;
    int z = 0;
    int q = 0;
    float max_temp[168] = {0.0f};
    float32x4_t c4_max_val;
    float max_val = 0.0f;
    c4_max_val = vld1q_dup_f32(&max_val);


    ///////////
    float f[4] = {0.0f};
    float ftemp;
    float f_add[4] = {0.0f,1.0f,2.0f,3.0f};
    float32x4_t v_f;
    v_f = vld1q_f32(f);
    float32x4_t v_fadd;
    v_fadd = vld1q_f32(f_add);
    uint32x4_t res;
    float32x4_t res1 =  vdupq_n_f32(0.0); 
    float32x4_t res2 =  vdupq_n_f32(-1.0); 
    float32x4_t temp3;
    ///////////
    for(j=0;j<168;j+=4){


      v4_p2SinLat = vld1q_f32(&p2SinLat[j]);
      v4_p2CosLat = vld1q_f32(&p2CosLat[j]);
      v4_p2Lon = vld1q_f32(&p2Lon[j]);
       

      ////////logic for calculating closeness//////////////
      float32x4_t SinLatMul = vmulq_f32(v4_p1SinLat,v4_p2SinLat);
      float32x4_t CosLatMul = vmulq_f32(v4_p1CosLat,v4_p2CosLat);
      float32x4_t LonSub = vsubq_f32(v4_p2Lon,v4_p1Lon);
      #if OPT==1
      float cosVal[4] = {cosf(vgetq_lane_f32(LonSub,0)),cosf(vgetq_lane_f32(LonSub,1)),cosf(vgetq_lane_f32(LonSub,2)),cosf(vgetq_lane_f32(LonSub,3))};
      float32x4_t cosVal1 = vld1q_f32(cosVal);
      #else
      float32x4_t cosVal1=cos_neon(LonSub);
      #endif
      float32x4_t temp1 = vmulq_f32(CosLatMul,cosVal1);
      float32x4_t temp2 = vaddq_f32(SinLatMul,temp1);
      c4_max_val = vmaxq_f32(c4_max_val,temp2);

      //////////logic for getting the index of the 4 max values////////
      ftemp = (float)j;
      v_f = vdupq_n_f32(ftemp);
      v_f = vaddq_f32(v_f,v_fadd);
      res = vceqq_f32(c4_max_val,temp2);
      res1 = vbslq_f32(res,v_f,res1);
      //////////

      n=n+1;
      if(n==42){
        //////////logic for reduction of max value///////////////
        float32x2_t v2_u, v2_l;
        float32x2_t v2_zero = vdup_n_f32(0.0); 
        v2_u = vget_high_f32(c4_max_val);
        v2_l = vget_low_f32(c4_max_val);
        v2_u = vpmax_f32(v2_u, v2_l);
        v2_u = vpmax_f32(v2_u, v2_zero);
        max_c =  vget_lane_f32(v2_u, 0);

        //////////logic for reduction of index///////////////

        temp3 = vdupq_n_f32(max_c);
        res = vceqq_f32(c4_max_val,temp3);
        res2 = vbslq_f32(res,res1,res2);
        v2_u = vget_high_f32(res2);
        v2_l = vget_low_f32(res2);
        v2_u = vpmax_f32(v2_u, v2_l);
        v2_u = vpmax_f32(v2_u, v2_zero);
        closest_i = (int) vget_lane_f32(v2_u, 0);
      }

    }


    //printf("%f\n",max_c);

  #endif  

  // Finish calcuations for the closest point

  #if VER==10
    d = closest_d; 
  #else
    d = acosf(max_c)*6371; // finish distance calcuation
  #endif	

  #if VER==12
    b = Calc_Bearing(&ref, &(waypoints[closest_i]) );
  #endif

  // return information to calling function about closest waypoint 
  *distance = d;
  *bearing = b;
  *name = (char * ) (waypoints[closest_i].Name);
}
