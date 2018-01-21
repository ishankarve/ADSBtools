/* This program receives ADS-B data packets in SBS1 basestation format.
 * and calculates bearing, slant range and elevation to the aircraft from 
 * a stationary location.
 * Coded by Ishan Anant Karve <ishan.karve@gmail.com>, Jun 2016
 * "gcc polarplot.c -o polarplot -lm"
 */
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <netdb.h>
#include <string.h>
#include <syslog.h>
#include <netdb.h>
#define BUFFER_SIZE (256 * 1024)  /* 256 KB */
#define URL_SIZE    256
#define MAXLEN 		80
#define d2r (M_PI / 180.0)
#define r2d (180 / M_PI)

//--------------------------------------------------------------------------------
struct aircraft_parameters
{
  //all are char to prevent segfault due to null values
  char *modeScode;
  char *upddate;
  char *updtime;
  char *flightcode;
  char *latitude;
  char *longitude;
  char *msg_type;
  char *sog;
  char *cog;
  char *altitude;
  char *vertrate;
  char *squawk;
  char *callsign;
  char *alert;
  char *emerg;
  char *spi;
  char *onground;
};
struct aircraft_parameters process_messages(char ** msg)
{
	struct aircraft_parameters proc_params;
	char *raw_token;
	char *msg_buffer[512];
	char *s[20];
	int i=0;
	//printf("%s\r\n",*msg);
	////tokenise message string 
	raw_token = strsep (msg, ",");//get first token
	while (raw_token != NULL)
	{ 
	
	raw_token = strsep (msg, ",");
	
	if (raw_token == NULL)
		raw_token=0;
	
	msg_buffer[i]=raw_token;
	i++;
	}
	
	proc_params.msg_type=msg_buffer[0];
	proc_params.modeScode=msg_buffer[3];
	proc_params.upddate=msg_buffer[5];
	proc_params.updtime=msg_buffer[6];
	proc_params.callsign=msg_buffer[9];
	proc_params.flightcode=msg_buffer[9];
	proc_params.altitude=msg_buffer[10];
	proc_params.sog=msg_buffer[11];
	proc_params.cog=msg_buffer[12];
	proc_params.latitude=msg_buffer[13];
	proc_params.longitude=msg_buffer[14];
	proc_params.vertrate=msg_buffer[15];
	proc_params.squawk=msg_buffer[16];
	proc_params.alert=msg_buffer[17];
	proc_params.emerg=msg_buffer[18];
	proc_params.spi=msg_buffer[19];
	proc_params.onground=msg_buffer[21];
	// for some unknown reasons (for me ) the buffer points is always i-1			
	//printf("%s\r\n",*msg);
	return proc_params;
}
int readline(int fd, char ** out)
{
	/* Keep reading till header "MSG" is received
	 * store it in buffer
	 * keep reading till newline is encountered
	 * exit function
	 */
	int buf_size = 1024;
	int bytesloaded = 0;
	int ret;
	char buf;
	char * buffer = malloc(buf_size);
	if (NULL == buffer)
		return -1;
	//loop till header is received
	while (1)
	{
		bytesloaded=0;
		bzero(buffer,1024);
		ret = read(fd, &buf, 1);
		if (ret < 1)
		{
			free(buffer);
			return -1;
		}
		if (buf=='M')  	//Got M; now wait for S
		{
			bytesloaded=0;
			buffer[bytesloaded] = buf;
			bytesloaded++;
			ret = read(fd, &buf, 1);
			if (ret < 1)
			{
				free(buffer);
				return -1;
			}
			if (buf=='S')  	//Got S; now wait for G
			{
				buffer[bytesloaded] = buf;
				bytesloaded++;
				ret = read(fd, &buf, 1);
				if (ret < 1)
				{
					free(buffer);
					return -1;
				}
				if (buf=='G')  	//Got G; now wait for CR-LF
				{
					buffer[bytesloaded] = buf;
					bytesloaded++;
					while(1)
					{
						ret = read(fd, &buf, 1);
						if (ret < 1)
						{
							free(buffer);
							return -1;
						}
						if (buf=='\r')
							break;
						buffer[bytesloaded] = buf;
						bytesloaded++;
					}
					buffer[bytesloaded] = '\0';
					*out = buffer; // complete line
					return bytesloaded;
				}
			}
		}
	}
}

//calculate haversine distance for linear distance
double get_distance(double lat1, double long1, double lat2, double long2)
{
    double dlong = (long2 - long1) * d2r;
    double dlat = (lat2 - lat1) * d2r;
    double a = pow(sin(dlat/2.0), 2) + cos(lat1*d2r) * cos(lat2*d2r) * pow(sin(dlong/2.0), 2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    double d = 6367 * c;

    return d; //distance in Kms
}

double get_bearing(double lat1, double long1, double lat2, double long2)
{
	double y = sin(long2-long1) * cos(lat2);
	double x = cos(lat1)* sin(lat2) - sin(lat1)*cos(lat2)*cos(long2-long1);
	double brng = 360-(atan2(y, x)*r2d);
	return fmod(brng,360);
	
}
double slant_range(double range,double altitude)
{
	return sqrt((range*range)+(altitude*altitude));
	
}

double get_elevation(double range,double altitude)
{
	return atan((altitude)/range)*r2d;
	
}


int main(int argc, char *argv[])
{
	int sockfd, portno;
	struct sockaddr_in serv_addr;
	struct hostent *server;
	struct aircraft_parameters air_params;
	double a_lat,a_lon,a_height_ft,a_height_mtrs,distance,bearing,elevation,s_range;
	char* tbuf;
	int ret;
	int i,j;
	syslog(LOG_INFO,"%s","Execution Started.\n");
	fprintf(stderr, "\nCoded and conceptualised by Ishan Anant Karve <ishan.karve@gmail.com>.\nProgram check connectivity with local ADSB server.\n\n");
	/* Create a socket point */
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0)
	{
		perror("ERROR opening socket");
		exit(1);
	}
	bzero(&(serv_addr.sin_zero),8);
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(30003);
	//serv_addr.sin_addr = *((struct in_addr *)server->h_addr);
	serv_addr.sin_addr.s_addr = inet_addr("192.168.2.107");  //adress of ADS-B Server
	/* Now connect to the server */
	if (connect(sockfd,(struct sockaddr *)&serv_addr,sizeof(struct sockaddr)) == -1)
	{
		perror("ERROR connecting");
		exit(1);
	}
	/* Now read server response */
	while (1)
	{
		ret = readline(sockfd, &tbuf);
		if (ret < 0)
		{
			perror("ERROR reading from socket");
			exit(1);
		}
		//printf ("%d\n",ret);
		//extract fields from message
		air_params=process_messages(&tbuf);
		//message type 3 has the lat, lon and height information
		if (strcmp(air_params.msg_type,"3")==0)
			{
			a_lat=strtod(air_params.latitude, NULL);
			a_lon=strtod(air_params.longitude, NULL);
			a_height_ft=strtod(air_params.altitude, NULL);
			a_height_mtrs=a_height_ft*0.3048;
			distance=get_distance(a_lat, a_lon, 28.5290937, 77.1351067);
			bearing=get_bearing(28.5290937, 77.1351067,a_lat, a_lon);
			elevation=get_elevation(distance*1000,a_height_mtrs);
			s_range=slant_range(distance*1000,a_height_mtrs)/1000;
			printf ("%s|height=%f Mtrs|distance=%f|bearing=%f|elevation=%f|Delta=%f\n",air_params.modeScode,a_height_mtrs,distance,bearing,elevation,distance-s_range);
			}
		}
	return 0;
}
