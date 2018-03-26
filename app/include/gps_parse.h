/*
 * @File  gps_parse.h
 * @Brief 
 * 
 * @Author: Neucrack 
 * @Date: 2017-12-13 10:57:17 
 * @Last Modified by: Neucrack
 * @Last Modified time: 2017-12-13 15:54:21
 */

#ifndef __GPS_PARSE_H
#define __GPS_PARSE_H

#ifdef __cplusplus
extern "C"{
#endif

#include "stdint.h"
#include "stdbool.h"

typedef enum{
    GPS_FIX_NO = 1  ,  //no fix
    GPS_FIX_2D      ,
    GPS_FIX_3D      ,
    GPS_FIX_MAX
}GPS_Fix_t;


//TODO: optimize, not elegent
typedef struct _GPS_Information{
	unsigned char Real_Locate;  	    //Real Locate Valid
	unsigned char Located_Status;       //Locate Valid ��Ч��λ����Ч��λ
	unsigned char fixModeGPS;           //Locate Mode A=�Զ��ֶ�2D/3D��M=�ֶ�2D/3D
    unsigned char fixModeBDS;           //Locate Mode A=�Զ��ֶ�2D/3D��M=�ֶ�2D/3D
	GPS_Fix_t fixGPS;	          	    //1=δ��λ��2=2D��λ��3=3D��λ
    GPS_Fix_t fixBDS;                   //BDS fix status
	char UTC_Time[7];                   //UTC TIme  hhmmss.sss(ʱ����.����)��ʽ
	char UTC_Date[7];                   //UTC Date
	char latitude[10];                  //Latitude  ddmm.mmmm(�ȷ�)��ʽ(ǰ���0Ҳ��������)
	char NS_Indicator;                  //NS
	char longitude[11];                 //Longtitude
	char EW_Indicator;                  //EW
	double Knot_Speed;                  //Knot_Speed ��������(000.0~999.9�ڣ�ǰ���0Ҳ��������)��
	double Speed;						//��������  (0000.0~1851.8����/Сʱ��ǰ���0Ҳ��������)
	double Course;                      //Course  ���汱Ϊ�ο���׼����˳ʱ�뷽��������ĽǶȡ�
	double Magnetic_Course;				//Magnetic Course �Դű�Ϊ�ο���׼����˳ʱ�뷽��������ĽǶȡ�
	
	double PDOP;                        //PDOP�ۺ�λ�þ������ӣ�0.5 - 99.9��
	double HDOP;                        //HDOPˮƽ�������ӣ�0.5 - 99.9��
	double VDOP;                        //VDOP��ֱ�������ӣ�0.5 - 99.9��
	
	double MSL_Altitude;                //High ��ƽ��߶�
	double Geoid_Separation;            //Geoid Separation���ˮ׼�����
	unsigned char Use_EPH_Sum;       	//Use_EPH_Sum
	unsigned char User_EPH[12];         //User_EPH   contain:Satellite Used1
	unsigned short EPH_State[12][4]; 	//EPH_State  contain:Satellite ID , Elevation , Azimuth , SNR (C/No)
}GPS_Information_t;


bool GPS_ParseOneNMEA(uint8_t* oneNmeaData);
GPS_Information_t* Gps_GetInfo();
bool GPS_Parse(uint8_t* nmeas);

#ifdef __cplusplus
}
#endif

#endif

