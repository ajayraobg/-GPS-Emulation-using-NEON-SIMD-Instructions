/*----------------------------------------------------------------------------
 *----------------------------------------------------------------------------*/
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include "geometry.h"
#include <string.h>

//#define TEST1_LAT (45.0)
//#define TEST1_LON (79.0)

#define N_TESTS (1000)

#define validate 0
#define base_code 0
extern const PT_T waypoints[];

float p2SinLat[168],p2CosLat[168],p2Lon[168];

float randGen(float lower, float upper){
	float num = fmod(rand(),(upper-lower)) + lower;
	return num;
}

/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main (void) {

	#if validate==0
	float dist, bearing, cur_pos_lat, cur_pos_lon;
	char * name;
	struct timespec start, end;
	unsigned long diff, total=0, min=1234567890;
	int n=0;
	FILE *fp1 ,*fp2;
	if (base_code == 1){
			fp1 = fopen("op_base_log.txt","w+");
	}
	else{
			fp2 = fopen("op_log.txt","w+");
	}

	
	//cur_pos_lat = TEST1_LAT;
	//cur_pos_lon = TEST1_LON;

	/*	printf("Current location is %f deg N, %f deg W\n", cur_pos_lat,
	       cur_pos_lon);
	*/
	for(n=0;n<168;n++){
		if(n<165){
			p2SinLat[n] = waypoints[n].SinLat;
			p2CosLat[n] = waypoints[n].CosLat;
			p2Lon[n] = waypoints[n].Lon;
		}
		else{
			p2SinLat[n] = 0.0f;
			p2CosLat[n] = 0.0f;
			p2Lon[n] = 0.0f;
		}
	}
	for (n=0; n<N_TESTS; n++) {
	  cur_pos_lat = randGen(-90.0,90.0);
	  cur_pos_lon = randGen(0,360.0);
	  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);
	  Find_Nearest_Waypoint(cur_pos_lat, cur_pos_lon,
				&dist, &bearing, &name);
	  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &end);
	  
	  diff = 1000000000 * (end.tv_sec - start.tv_sec) +
	    end.tv_nsec - start.tv_nsec;
	  //	  printf("%2d: %8lu ns\n", n, diff);
	  total += diff;
	  if (diff < min)
	    min = diff;

	  if(base_code == 1){
	  		fprintf(fp1,"%f\n%f\n%s\n",dist, bearing, name);
	  }
	  else{
	  		fprintf(fp2,"%f\n%f\n%s\n",dist, bearing, name);
	  }
	}
	/*printf("Closest waypoint is %s. %f km away at bearing %f degrees\n",
	       name, dist, bearing);*/

	printf("Average %10.3f us\n", total/(1000.0*N_TESTS));
	printf("Minimum %10.3f us\n",  min/1000.0);

	if(base_code == 1){		
		fclose (fp1);
	}
	else{
		fclose (fp2);
	}

	#endif


	#if validate==1
		int  i=0,j=0,temp=1;
    	double numbers[4000];
   		double numbers_1[4000];
   		char name[4000][200];
   		char name_1[4000][200];
    	double atof ( const char * str );
    	char line[8000];  /* declare a char array */
    	char line_1[8000];

    	FILE *file, *file1;  /* declare a FILE pointer  */
    	file = fopen("op_base_log.txt", "r");  /* open a text file for reading */
    	file1 = fopen("op_log.txt", "r");

    	while(fgets(line, sizeof line, file)!=NULL) {       /* keep looping until NULL pointer... */
        	if (temp%3 == 0){
        		strcpy(name[i],line);
        	}
        	else{
        		numbers[i]=atof(line);  /* convert string to double float*/
        	}
        	i++;
		temp++;
    	}
		temp =1;
    	while(fgets(line_1, sizeof line_1, file1)!=NULL) {       /* keep looping until NULL pointer... */
			if(temp%3==0){
				strcpy(name_1[j],line_1);
			}
			else{
				numbers_1[j]=atof(line_1);  /* convert string to double float*/
			}
        	j++;
			temp++;
    	}
		temp=1;

    	for (i=0 ; i<3000 ; i++) {
    		if(temp%3==0){
    			if(strcmp(name[i],name_1[i]) != 0){
    				printf("Error waypoint name not same %s  %s!!!\n",name[i],name_1[i]);
    			}
    		}
    		else{
        		if(fabs(numbers[i])>fabs(numbers_1[i])){
            		if((((fabs(numbers[i]) - fabs(numbers_1[i]))/fabs(numbers[i]))*100) > 0.001){
                		printf("Error greater than 0.01 for %f %f \n",numbers[i],numbers_1[i]);
          			}
          		}
        		else{
                	if((((fabs(numbers_1[i]) - fabs(numbers[i]))/fabs(numbers_1[i]))*100) > 0.001){
                    	printf("Error greater than 0.01 for %f %f \n",numbers[i],numbers_1[i]);
                	}
            	}
        	}
		temp++;
		}
    	fclose(file);
    	fclose(file1);


	#endif
	exit(0);

}

