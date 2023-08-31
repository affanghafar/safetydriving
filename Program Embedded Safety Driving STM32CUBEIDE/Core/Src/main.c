/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include <math.h>
#include "usbd_cdc_if.h"
#include <malloc.h>
#include "string.h"
#include <stdio.h>
#include <stdlib.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */


uint8_t fsampling=250;
#define filter_length 750 //fs*waktu
#define th 37
int kondisi_sebelum, kondisi_sekarang;
float lpf[filter_length],hpf[filter_length],derivativ[filter_length],squaring[filter_length],moving[filter_length],moving1[filter_length],threshold[filter_length],threshold1[filter_length],threshold2[filter_length],qrs[filter_length],threshold3[filter_length],threshold4[filter_length];
uint16_t Acc1x=0,ecg,gyro1=0, Acc1z=0,Acc1xa,Acc1xb,Acc1za,Acc1zb, gyro1a,gyro1b;
uint8_t rx_data1,rx_data;
uint8_t  myData[24];
uint32_t on=0, off=0, timerCount=0, fes=0, fes2=0, tmr=0;
uint32_t count3=0,count4=0,n=0,count5=0,count2=0,count1=0,windowing=0,p=0,count_rr=0,k=0,z1=0,q1=0,count6=0, count7=0;
uint8_t Cmd_End[3] = {0xFF,0xFF,0xFF};
float d,e,f,g,h,l,hr,ecg_vol[filter_length],durasi,o,t,v;
float a11, b11, c11, d11,e11,f11,g11,h11,l11;
float a22, b22, c22, d22,e22,f22,g22,h22,l22;
int m;
float max_value,ecg_lpf,ecg_hpf,deriv,squar,mav, mav1,thr,bpm1;


//------------------------Const Sinyal ECG----------------------------//


//------------------------Const Untuk Postural----------------------------//
//#define FrekSampling	20			// dalam Hz
//#define dt 				0.05	// dalam detik
#define FrekSampling	250			// dalam Hz
#define dt 				0.004		// dalam detik
#define pi 				3.141593
#define d90 			90
#define Rad2Deg 		57.29578
//0g acc (kalibrasi)
#define UparmAc_X_0g	  (2457+1642)/2//(2537+1393)/2 //(2412+1592)/2//1861.36
#define UparmAc_Z_0g	  (2431+1633)/2//(2472+1345)/2 //(2432+1627)/2
//--------sensitivitas acc (kalibrasi)
#define UparmAc_X_Sens	 (2457-1642)/2//(2537-1393)/2 //(2412-1592)/2
#define UparmAc_Z_Sens	 (2431-1633)/2//(2472-1345)/2 //(2432-1627)/2
//Koefisien Regresi kalibrasi
#define aUparm 	0.9971
#define bUparm 	0.0585
//--------sensitivitas gyro (sesuai data sheet)(4095*0.00067/3.3)
#define GyroSenstvtS 	1.16 //2.7
//--------gyro offset (kalibrasi lsb kondisi diam)
#define Gyro_offset	1671


/* Private function prototypes -----------------------------------------------*/

//ADC_ChannelConfTypeDef sConfig = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);

ADC_ChannelConfTypeDef sConfig = {0};
/* USER CODE END PV */
//================== DIGITAL Butterworth LPF ======================//
static const float a1 =  1.6474599811; static const float a2 = 0.7008967812; //4Hz
static const float b0 =  0.0133592000; static const float b1 = 0.0267184001;
static const float b2 =  0.0133592000;
//================== KALMAN FILTER Uparm
static const float QangleS = 0.001;  //tingkat kepercayaan Akselero Uparm
static const float QgyroS  = 0.003;  //tingkat kepercayaan Gyro Uparm
static const float RangleS = 0.04;	 //kemungkinan jiter Akselero Uparm (rad)

float UparmLajuGr, TiltUparmGr, UparmAc_X_Vctr,UparmAc_Z_Vctr,UparmAcclX_LPF, UparmAcclZ_LPF,UparmGyroY_LPF, t=0,miuk ;
float TiltAc=0.0,Tilttemp, TiltKal,GyroUparm, VUparmx,VUparmz,VgyroUparm,sudut[filter_length],velocity[filter_length],signal1[filter_length];
int cntr, Uparmac_a,Uparmac_b,Uparmkal_a,Uparmkal_b,ri;
int gyroUparma,gyroUparmb,gyroUparmc;
int VUparmx_a,VUparmx_b,VUparmz_a,VUparmz_b, VgyroUparm_a,VgyroUparm_b,Vecg_a,Vecg_b;
float ErrEstmt, K0, K1, ErrTilt,error1,tarr,heart_rate;
float Bias_T, P1T, P2T, P3T, P4T,Bias_S, P1S, P2S, P3S, P4S,Bias_F, P1F, P2F, P3F, P4F;
int sinyal[14],sinyal_1[14],sinyal_2[14],Ylpf[14],Ylpf_1[14],Ylpf_2[14];
int sinyal_1_acc1x[14],sinyal_2_acc1x[14],Ylpf_1_acc1x[14],Ylpf_2_acc1x[14];
int sinyal_1_acc1z[14],sinyal_2_acc1z[14],Ylpf_1_acc1z[14],Ylpf_2_acc1z[14];
int sinyal_1_gyro1y[14],sinyal_2_gyro1y[14],Ylpf_1_gyro1y[14],Ylpf_2_gyro1y[14];
int gyroUparma,gyroUparmb,gyroUparmc;

int flag,Program_state, flagkirim, map_ecg,i,Elpf_a,Elpf_b,Ehpf_a,Ehpf_b,panjang_bpm[filter_length];
int deriv_a,deriv_b,squar_a,squar_b, mav_a,mav_b,window,mav_a1,mav_b1,thr_a,thr_b,rr_a,rr_b,rr1_a,rr1_b,bpm_a,bpm_b;
char ecg_kirim[5], ecg_data[20], hasil_data[5],map_hasil;
char Rx_data[1];
float Vecg,Vecg1,count,tho,AN,AP,rr_temp[filter_length],rr1[0][filter_length],nilairr[filter_length];
int count_k[filter_length];
int qrs_hasil;
float ecg_vol1[filter_length], off_ecg_temp,sum,sum_data, kecepatan;
float hasil1[filter_length], hasil2[filter_length], hasil3[filter_length];
float hasil_QRS[filter_length];
float ZrTiltUparmAc, zeroposition;
float signal[filter_length];
float rr_index[filter_length],rr[filter_length],bpm[filter_length];
uint32_t rr_index_length=0,rr_length=0;
//fuzzy
int lop,j,m,q,stmltime,stmltime_1,stmltime2,stmltime_2,stmltime3,stmltime_3,cntrTA;
float a,b,c,jfloat, jfloat1, jfloat2, jfloat3, jfloat4, in1,in2,in3,in4,in5[filter_length],in6[filter_length], in7[filter_length], in8[filter_length], min[24],imfhr[3][3],imfkc[3][3],imfdr[4][4],imfps[3][3],omfstbd[4][4],miu[8],miuerr[8],miudr[8],miumaxstbd[101],maxplantar,maxflexion,maxflexion1;
float maxx,fuzzy_mks[10],ttk[10],num,denum,centroid,AnkKal_Save,AnkKal_Save2,ShoulderKal_Save;
int in1a,in1b,in2a,in2b,in3a,in3b,in4a,in4b,in5a,in5b,centa,centb,centc,centd,cente,centf,gyroShouldera,gyroShoulderb,gyroShoulderc,gyroUparma,gyroUparmb,gyroUparmc,gyroLowarma,gyroLowarmb,gyroLowarmc;
float miu_low[filter_length], miu_vhigh1[filter_length],miu_high[filter_length],miu_low1[filter_length], miu_medium1[filter_length],miu_high1[filter_length],miu_low2[filter_length],miu_high2[filter_length],miu_low3[filter_length],miu_medium3[filter_length],miu_high3[filter_length];
float miuk_low, miuk_medium, miuk_high, miuk_low1, miuk_medium1, miuk_high1,miuk_vhigh1;
float miul_hr, mium_hr, miuh_hr, miul_pos, miuh_pos, miul_kec, miuh_kec,mius_dur,mium_dur,miul_dur,miuvl_dur;
float rule[27], sum_rr = 0.0;
float value_a, value_w1, value_w2, value_w3, outf;
float low_miu, medium_miu, high_miu,max1, max2,max3,max4;
float value_aw, value_dr1, value_dr2,value_sl, value_rmssd;
float miu_low3[filter_length], miu_medium3[filter_length], miu_high3[filter_length],miu_low4[filter_length], miu_medium4[filter_length], miu_high4[filter_length],miu_vhigh4[filter_length];
/* USER CODE BEGIN PFP */



void Kalman(void)
{
	sinyal[1]=Acc1x;
	sinyal[2]=Acc1z;
	sinyal[3]=gyro1;

	//LPF filter
	for (i = 1; i < 4; i++)
	{
		if (i==1){
			Ylpf_2[i] = Ylpf_2_acc1x[i];
			Ylpf_1[i] = Ylpf_1_acc1x[i];
			sinyal_2[i] = sinyal_2_acc1x[i];
			sinyal_1[i] = sinyal_1_acc1x[i];
		}
		else if (i==2){
			Ylpf_2[i] = Ylpf_2_acc1z[i];
			Ylpf_1[i] = Ylpf_1_acc1z[i];
			sinyal_2[i] = sinyal_2_acc1z[i];
			sinyal_1[i] = sinyal_1_acc1z[i];
		}
		else if (i==3){
			Ylpf_2[i] = Ylpf_2_gyro1y[i];
			Ylpf_1[i] = Ylpf_1_gyro1y[i];
			sinyal_2[i] = sinyal_2_gyro1y[i];
			sinyal_1[i] = sinyal_1_gyro1y[i];
		}

		Ylpf[i] = (uint16_t)(a1*((float)Ylpf_1[i])-a2*((float)Ylpf_2[i])+b0*((float)sinyal[i])+b1*((float)sinyal_1[i])+b2*((float)sinyal_2[i]));
		if (Ylpf[i]<0){Ylpf[i]=0;}

		if (i==1){
			Ylpf_2_acc1x[i] = Ylpf_1[i];
			Ylpf_1_acc1x[i] = Ylpf[i];
			sinyal_2_acc1x[i] = sinyal_1[i];
			sinyal_1_acc1x[i] = sinyal[i];
		}
		else if (i==2){
			Ylpf_2_acc1z[i] = Ylpf_1[i];
			Ylpf_1_acc1z[i] = Ylpf[i];
			sinyal_2_acc1z[i] = sinyal_1[i];
			sinyal_1_acc1z[i] = sinyal[i];
		}
		else if (i==3){
			Ylpf_2_gyro1y[i] = Ylpf_1[i];
			Ylpf_1_gyro1y[i] = Ylpf[i];
			sinyal_2_gyro1y[i] = sinyal_1[i];
			sinyal_1_gyro1y[i] = sinyal[i];
		}
	}

	//using lpf
	UparmAcclX_LPF = Ylpf[1];
	UparmAcclZ_LPF = Ylpf[2];
	UparmGyroY_LPF = Ylpf[3];

	UparmAc_X_Vctr = (float)((UparmAcclX_LPF - UparmAc_X_0g)/ UparmAc_X_Sens);
	UparmAc_Z_Vctr = (float)((UparmAcclZ_LPF - UparmAc_Z_0g)/ UparmAc_Z_Sens);


	TiltAc	=  atan2f(UparmAc_Z_Vctr,UparmAc_X_Vctr);


	TiltAc	= TiltAc*Rad2Deg;

//	if (zeroposition==1.0)
//	    	{
//	    		ZrTiltUparmAc	= TiltAc;
////	    		ZeroHip = ZrTiltThighAc - ZrTiltTrunkAc;
//	    		TiltKal = TiltAc;
//	    	}

//---------------------------------KALMAN FILTER------------------------------------//

    //--------------------- Time Update / State Update: ( “Prediksi ? )
    //------ prediksi state
    UparmLajuGr = (float)((UparmGyroY_LPF-Gyro_offset)/GyroSenstvtS);
    TiltKal += (UparmLajuGr - Bias_S)*dt; //TiltUparm = nilai Prediksi

//    GyroUparm += (UparmLajuGr * dt); //TiltShoulder = nilai Prediksi

//    TiltKal = TiltAc - TiltKal;
    // += (UparmLajuGr * dt); //TiltShoulder = nilai Prediksi
    //GyroUparm *= Rad2Deg;

    //------ prediksi matrik Error Covariance
    //??
    P4S += QgyroS * dt;
    P2S -= P4S * dt;
    P3S -= P4S * dt;
    P1S += (QangleS + P4S*dt - P3S - P2S )*dt;

    //--------------------- Measurement Update / Kalman Update (  “Koreksi ? )
    //------ hitung Error Estimate
    //??
    ErrEstmt = P1S + RangleS;

    //------ hitung Kalman filter Gain
    K0 = P1S / ErrEstmt;
    K1 = P3S / ErrEstmt;
    //pembaharuan
    //------ hitung tilt error, selisih nilai measurement - nilai Prediksi state
    ErrTilt  = TiltAc - TiltKal;  //TiltAc = nilai Measurement

    //------ Koreksi nilai estimasi state
    TiltKal += K0 * ErrTilt;  //estimasi sudut tilt Uparm terkoreksi
    Bias_S    += K1 * ErrTilt;  //estimasi bias gyro terkoreksi, perb output dan real

    //------ Koreksi matrik Error Covariance ->utk iterasi berikutnya
    P1S -= K0 * P1S;
    P2S -= K0 * P2S;
    P3S -= K1 * P1S;
    P4S -= K1 * P2S;

//    TiltKal = TiltAc - TiltKal;
}

void vel(void){



 if(count3==0){
	 sudut[count2]=TiltKal;

	 if(count2==0) velocity[count2]=0.0;
	 else velocity[count2]=(sudut[count2]-sudut[count2-1])/dt;
 }
 else{
	 sudut[count2]=TiltKal;
	 if(count2==0)velocity[count2]=(sudut[count2]-sudut[filter_length+count2-1])/dt;
	 else velocity[count2]= (sudut[count2]-sudut[count2-1])/dt;
 }


 kecepatan=fabs(velocity[count2]);
 count2++;

 if(count2==filter_length){
	 count3=1;
	 count2=0;
 }
}

void time(void){

 if(TiltKal>43.0){
 count6 +=1;
	}
else{
count6 =0;
}
durasi = count6/250.0;
}

//void time(void){
//
// if(TiltKal<=41.0){
//	 kondisi_sekarang=0;
//	 }
// else{
//	 kondisi_sekarang=1;
// }
// if(kondisi_sekarang!=kondisi_sebelum){
//   count6 = 0;
//   kondisi_sebelum=kondisi_sekarang;
// }
// else{
//	 count6 += 1;
// }
//durasi = count6/250.0;
//}

float get_index(float *array, int n) {
    if(n < 0) {
        return array[0];
    }
    return array[n];
}

float get_rotating_index(float *array, int max, int n) {
    if(n < 0) {
        return array[max + (n)];
    }
    return array[n];
}

void PanTompkins(void)
{
	count1 = k % filter_length;
	//count_rr = q % filter_length;
//	if(count2==0){
		ecg_vol[count1]=(float)ecg*3.3/4095;

//
                      sum_data = sum_data + ecg_vol[count1];
                      ecg_vol[count1] = ecg_vol[count1] - (sum_data/(z1 + 1));

                      //Max Value
//                      if(count1==0){
//      	                 max_value=ecg_vol[count1];
//                      }
//                      if(ecg_vol[count1]>max_value){
//      	                 max_value=ecg_vol[count1];
//                      }


                    //  w2fb1[count1] = qj1[0] * signal[count1] +  qj1[1] * get_rotating_index(signal, filter_length,count1-1);
                      lpf[count1] = 2*get_rotating_index(lpf, filter_length,count1-1)-get_rotating_index(lpf, filter_length,count1-2)
                    		                  +ecg_vol[count1]-2*get_rotating_index(ecg_vol, filter_length,count1-6)+get_rotating_index(ecg_vol, filter_length,count1-12);
                      hpf[count1] = get_rotating_index(lpf, filter_length,count1-16)-get_rotating_index(lpf, filter_length,count1-1)/32.0-lpf[count1]/32.0
                    		                  +get_rotating_index(lpf, filter_length,count1-32)/32.0;
                      derivativ[count1] =(1.0/8.0)*get_rotating_index(hpf, filter_length,count1-1)+2*hpf[count1]-get_rotating_index(hpf, filter_length,count1-3)-2*get_rotating_index(hpf, filter_length,count1-4);
                      squaring[count1] = pow(derivativ[count1],2);
                      //Moving Average
					 window=50;
					 sum=0.0;
					 for (j = 0; j <= window; j++)
					 {
						 sum=sum+get_rotating_index(squaring, filter_length,count1-(window-j));
					 }
					 moving[count1] =((1.0/window)*sum);
//
//	                 //Thresholding
	                 if(moving[count1]>get_rotating_index(moving1, filter_length,count1-1)){
	                	 moving1[count1] = moving[count1];
	                 }
	                 else{
	                	 moving1[count1]=get_rotating_index(moving1, filter_length,count1-1);
	                	 if(moving[count1]<=get_rotating_index(moving1, filter_length,count1-1)/2.0){
	                	 moving1[count1] = moving[count1];
	                	 }
	                 }
//
//	                 //Pembanding Thresholding
////	                 tho=0.25;
////	                 AN=0.875;
////	                 AP=0.875;
////
	                 threshold[count1]=get_rotating_index(threshold1, filter_length,count1-1)+0.25*abs((get_rotating_index(threshold2, filter_length,count1-1)-get_rotating_index(threshold1, filter_length,count1-1)));
	                 if(moving1[count1]>threshold[count1]){
	                	 qrs[count1]=1.0;
	                	 threshold2[count1]=(0.875*get_rotating_index(threshold2, filter_length,count1-1))+((1.0-0.875)*moving1[count1]);
	                	 threshold1[count1]=get_rotating_index(threshold1, filter_length,count1-1); //NPKI  running estimate of the noise peak
	                	 threshold3[count1]=threshold1[count1];
	                	 threshold4[count1]=threshold2[count1];
	                 }
	                 else{
	                	 qrs[count1]=0.0;
	                	 threshold1[count1]=(0.875*get_rotating_index(threshold1, filter_length,count1-1))+((1.0-0.875)*moving1[count1]);
	                	 threshold2[count1]=get_rotating_index(threshold2, filter_length,count1-1); //spki running estimate of the signal peak
	                	 threshold3[count1]=threshold1[count1];
	                	 threshold4[count1]=threshold2[count1];

	                 }

	                 if (qrs[count1-1] == 0.0 && qrs[count1] == 1.0) {
	                 	         rr_index[rr_index_length] = k;

	                 	        if (rr_index_length >= 2) {
	                 	            float t = 1.0 / 250.0;
	                 	            rr[rr_length] =  fabs((rr_index[rr_index_length] * t) - (rr_index[rr_index_length - 1] * t));
	                 	            bpm[rr_length]  = 60.0/ rr[rr_length];
	                 	            hr= bpm[rr_length];
//	                 	            sum_rr += fabs((rr[rr_length] - rr[rr_length - 1])) *fabs((rr[rr_length] - rr[rr_length - 1]));
//	                 	          	value_rmssd = sqrt(sum_rr / (rr_length-1));
	                 	            rr_length++;
	                 	        }


	                 	        rr_index_length++;

	                 	}


	                 //BPM
	              //   count=0;
//	                 windowing=0;
//	                 p=0;
//	                 if((count1 % 250*60*5)==0){
//	                	 panjang_bpm[windowing]=p;
//	                	 windowing=windowing+1;
//	                	 p=0;
//	                 }
//	                 if(count1>0){
//	                	 if(get_rotating_index(qrs, filter_length,count1-1)==1.0){
//	                		 if(qrs[count1]==0){
//	                			 rr_temp[count_rr]=count1/250.0;
//	                			 if(count_rr>0){
//	                				 if(p>0){
//	                					 rr1[p]=fabs(rr_temp[count_rr]-rr_temp[count_rr-1]);
//	                					 bpm[p]=60.0/rr1[p];
//	                					 heart_rate= bpm[p];
//
//	                				 }
//	                			 }
//	                			 p++;
//	                			 count_rr++;
//	                		 }
//	                	 }
//	                 }

    Vecg=ecg_vol[count1];
    ecg_lpf=lpf[count1];
    ecg_hpf=hpf[count1];
    deriv=derivativ[count1];
    squar=squaring[count1];
    mav=moving[count1];
    mav1=moving1[count1];
    thr=threshold[count1];
    ri=qrs[count1];
//    ri=rr1[windowing][j];
//    bpm1=bpm[windowing][j];


    k++;
    z1++;
    if(count1==filter_length){
    	//count1=0;
    	sum_data=0;
    	//count2=1;
    }
}



float persgaris1(float first, float second, float third, float in){
	low_miu = 0;
	if (in <= second && in >= 0.0){
		low_miu = 1.0;
	}
	else if (in >= second && in <= third){
		low_miu = (third-in)/(third-second);
	}
	else if (in >= third){
		low_miu = 0.0;
	}
	return low_miu;
}

float persgaris2(float first, float second, float third, float in){
	medium_miu = 0;
	if (in <= first){
		medium_miu = 0.0;
	}
	else if (in >first && in <=second ){
		medium_miu = (in - first)/(second - first);
	}
	else if (in > second && in <=third){
		medium_miu = ((second - in)/(third - second)) + 1;
	}
	else if (in > third){
		medium_miu = 0.0;
	}
	return medium_miu;
}

float persgaris3(float first, float second, float third, float in){
	high_miu = 0;
	if (in <= first ){
		high_miu = 0.0;
	}
	else if (in >= first && in < second){
		high_miu = 1-((second-in)/(second-first));
		}
	else if (in >= second){
		high_miu = 1.0;
	}
	return high_miu;
}

//float carimin(float* a, float* b) {
//	//minn = min[0]
//    for (i = 0; i < n; i++) {
//    	min = (a[i] < b[i]) ? a[i] : b[i];
//    }
//}

float carimin(float a, float b) {
    return (a < b) ? a : b; // Mengembalikan nilai minimum antara a dan b
}

float carimax(float a, float b) {
    return (a > b) ? a : b; // Mengembalikan nilai maksimum antara a dan b
}




void center(void){
	num = 0.0;
	denum = 0.0;
	if(value_a > 0 && value_w1 > 0 && value_w2 > 0 && value_w3 > 0){
			for (lop = 0;lop < 30 ; lop++){
				num += value_aw*lop;
				denum += value_aw;
			}
			for (lop = 30;lop < 36 ; lop++){
				jfloat3 = (float)lop;
				a = omfstbd[0][0];	b = omfstbd[0][1];	c = omfstbd[0][2];
				miu_low4[lop] = persgaris1(a, b, c, jfloat3);
				miumaxstbd[lop] = miu_low4[lop];
				if (miumaxstbd[lop] > value_aw){
					miumaxstbd[lop] = value_aw;
				}
				num += miumaxstbd[lop]*jfloat3;
				denum += miumaxstbd[lop];
			}
			for (lop = 36;lop < 55 ; lop++){
				jfloat3 = (float)lop;
				d = omfstbd[1][0];	    e = omfstbd[1][1];	f = omfstbd[1][2];
				miu_medium4[lop] = persgaris2(d, e, f, jfloat3);;
				miumaxstbd[lop] = miu_medium4[lop];
				if (miumaxstbd[lop] > value_dr1){miumaxstbd[lop] = value_dr1;}
				num += miumaxstbd[lop]*jfloat3;
				denum += miumaxstbd[lop];
			}
			for (lop = 55;lop < 74 ; lop++){
				jfloat3 = (float)lop;
				g = omfstbd[2][0];	    h = omfstbd[2][1];	l = omfstbd[2][2];
				miu_high4[lop] = persgaris2(g, h, l, jfloat3);
				miumaxstbd[lop] = miu_high4[lop];
				if (miumaxstbd[lop] > value_dr2){miumaxstbd[lop] = value_dr2;}
				num += miumaxstbd[lop]*jfloat3;
				denum += miumaxstbd[lop];
				}
			for (lop = 74;lop < 80 ; lop++){
				jfloat3 = (float)lop;
				o = omfstbd[3][0];  	t = omfstbd[3][1];	v = omfstbd[3][2];
				miu_vhigh4[lop] = persgaris3(o, t, v, jfloat3);
				miumaxstbd[lop] = miu_vhigh4[lop];
				if (miumaxstbd[lop] > value_sl){miumaxstbd[lop] = value_sl;}
				num += miumaxstbd[lop]*jfloat3;
				denum += miumaxstbd[lop];
				}
			for (lop = 80;lop <= 100 ; lop++){
					num += value_sl*lop;
					denum += value_sl;
				}
		  }

		  else if(value_a > 0 && value_w1 > 0 && value_w2 > 0 && value_w3 == 0){
				for (lop = 0;lop < 30 ; lop++){
				num += value_aw*lop;
				denum += value_aw;
			}
			for (lop = 30;lop < 36 ; lop++){
				jfloat3 = (float)lop;
				a = omfstbd[0][0];	b = omfstbd[0][1];	c = omfstbd[0][2];
				miu_low4[lop] = persgaris1(a, b, c, jfloat3);
				miumaxstbd[lop] = miu_low4[lop];
				if (miumaxstbd[lop] > value_aw){
					miumaxstbd[lop] = value_aw;
				}
				num += miumaxstbd[lop]*jfloat3;
				denum += miumaxstbd[lop];
			}
			for (lop = 36;lop < 55 ; lop++){
				jfloat3 = (float)lop;
				d = omfstbd[1][0];	    e = omfstbd[1][1];	f = omfstbd[1][2];
				miu_medium4[lop] = persgaris2(d, e, f, jfloat3);;
				miumaxstbd[lop] = miu_medium4[lop];
				if (miumaxstbd[lop] > value_dr1){miumaxstbd[lop] = value_dr1;}
				num += miumaxstbd[lop]*jfloat3;
				denum += miumaxstbd[lop];
			}
			for (lop = 55;lop < 80 ; lop++){
				jfloat3 = (float)lop;
				g = omfstbd[2][0];	    h = omfstbd[2][1];	l = omfstbd[2][2];
				miu_high4[lop] = persgaris2(g, h, l, jfloat3);
				miumaxstbd[lop] = miu_high4[lop];
				if (miumaxstbd[lop] > value_dr2){miumaxstbd[lop] = value_dr2;}
				num += miumaxstbd[lop]*jfloat3;
				denum += miumaxstbd[lop];
				}
		  }

		else if((value_a > 0 && value_w1 > 0 && value_w2 == 0 && value_w3 > 0)|| (value_a > 0 && value_w1 > 0 && value_w2 == 0 && value_w3 == 0)){
			for (lop = 0;lop < 30 ; lop++){
				num += value_aw*lop;
				denum += value_aw;
			}
			for (lop = 30;lop < 36 ; lop++){
				jfloat3 = (float)lop;
				a = omfstbd[0][0];	b = omfstbd[0][1];	c = omfstbd[0][2];
				miu_low4[lop] = persgaris1(a, b, c, jfloat3);
				miumaxstbd[lop] = miu_low4[lop];
				if (miumaxstbd[lop] > value_aw){
					miumaxstbd[lop] = value_aw;
				}
				num += miumaxstbd[lop]*jfloat3;
				denum += miumaxstbd[lop];
			}
			for (lop = 36;lop < 60 ; lop++){
				jfloat3 = (float)lop;
				d = omfstbd[1][0];	    e = omfstbd[1][1];	f = omfstbd[1][2];
				miu_medium4[lop] = persgaris2(d, e, f, jfloat3);;
				miumaxstbd[lop] = miu_medium4[lop];
				if (miumaxstbd[lop] > value_dr1){miumaxstbd[lop] = value_dr1;}
				num += miumaxstbd[lop]*jfloat3;
				denum += miumaxstbd[lop];
			}
			for (lop = 70;lop < 80 ; lop++){
				jfloat3 = (float)lop;
				o = omfstbd[3][0];  	t = omfstbd[3][1];	v = omfstbd[3][2];
				miu_vhigh4[lop] = persgaris3(o, t, v, jfloat3);
				miumaxstbd[lop] = miu_vhigh4[lop];
				if (miumaxstbd[lop] > value_sl){miumaxstbd[lop] = value_sl;}
				num += miumaxstbd[lop]*jfloat3;
				denum += miumaxstbd[lop];
				}
			for (lop = 80;lop <= 100 ; lop++){
					num += value_sl*lop;
					denum += value_sl;
				}
		  }

	else if((value_a > 0 && value_w1==0 && value_w2 > 0 && value_w3 > 0)|| (value_a == 0 && value_w1==0 && value_w2 > 0 && value_w3 > 0)){
			for (lop = 0;lop < 30 ; lop++){
				num += value_aw*lop;
				denum += value_aw;
			}
			for (lop = 30;lop < 40 ; lop++){
				jfloat3 = (float)lop;
				a = omfstbd[0][0];	b = omfstbd[0][1];	c = omfstbd[0][2];
				miu_low4[lop] = persgaris1(a, b, c, jfloat3);
				miumaxstbd[lop] = miu_low4[lop];
				if (miumaxstbd[lop] > value_aw){
					miumaxstbd[lop] = value_aw;
				}
				num += miumaxstbd[lop]*jfloat3;
				denum += miumaxstbd[lop];
			}
			for (lop = 50;lop < 74 ; lop++){
				jfloat3 = (float)lop;
				g = omfstbd[2][0];	    h = omfstbd[2][1];	l = omfstbd[2][2];
				miu_high4[lop] = persgaris2(g, h, l, jfloat3);
				miumaxstbd[lop] = miu_high4[lop];
				if (miumaxstbd[lop] > value_dr2){miumaxstbd[lop] = value_dr2;}
				num += miumaxstbd[lop]*jfloat3;
				denum += miumaxstbd[lop];
				}
			for (lop = 74;lop < 80 ; lop++){
				jfloat3 = (float)lop;
				o = omfstbd[3][0];  	t = omfstbd[3][1];	v = omfstbd[3][2];
				miu_vhigh4[lop] = persgaris3(o, t, v, jfloat3);
				miumaxstbd[lop] = miu_vhigh4[lop];
				if (miumaxstbd[lop] > value_sl){miumaxstbd[lop] = value_sl;}
				num += miumaxstbd[lop]*jfloat3;
				denum += miumaxstbd[lop];
				}
			for (lop = 80;lop <= 100 ; lop++){
					num += value_sl*lop;
					denum += value_sl;
				}
		  }
	else if((value_a > 0 && value_w1 == 0 && value_w2 > 0 && value_w3 == 0)|| (value_a > 0 && value_w1 == 0 && value_w2 == 0 && value_w3 > 0)|| (value_a > 0 && value_w1 == 0 && value_w2 == 0 && value_w3 == 0)|| (value_a == 0 && value_w1 > 0 && value_w2 == 0 && value_w3 > 0)|| (value_a == 0 && value_w1 > 0 && value_w2 == 0 && value_w3 == 0)|| (value_a == 0 && value_w1 == 0 && value_w2 > 0 && value_w3 == 0)|| (value_a == 0 && value_w1 == 0 && value_w2 == 0 && value_w3 > 0)){
			for (lop = 0;lop < 30 ; lop++){
				num += value_aw*lop;
				denum += value_aw;
			}
			for (lop = 30;lop < 40 ; lop++){
				jfloat3 = (float)lop;
				a = omfstbd[0][0];	b = omfstbd[0][1];	c = omfstbd[0][2];
				miu_low4[lop] = persgaris1(a, b, c, jfloat3);
				miumaxstbd[lop] = miu_low4[lop];
				if (miumaxstbd[lop] > value_aw){
					miumaxstbd[lop] = value_aw;
				}
				num += miumaxstbd[lop]*jfloat3;
				denum += miumaxstbd[lop];
			}
			for (lop = 30;lop < 60 ; lop++){
				jfloat3 = (float)lop;
				d = omfstbd[1][0];	    e = omfstbd[1][1];	f = omfstbd[1][2];
				miu_medium4[lop] = persgaris2(d, e, f, jfloat3);;
				miumaxstbd[lop] = miu_medium4[lop];
				if (miumaxstbd[lop] > value_dr1){miumaxstbd[lop] = value_dr1;}
				num += miumaxstbd[lop]*jfloat3;
				denum += miumaxstbd[lop];
			}
			for (lop = 50;lop < 80 ; lop++){
				jfloat3 = (float)lop;
				g = omfstbd[2][0];	    h = omfstbd[2][1];	l = omfstbd[2][2];
				miu_high4[lop] = persgaris2(g, h, l, jfloat3);
				miumaxstbd[lop] = miu_high4[lop];
				if (miumaxstbd[lop] > value_dr2){miumaxstbd[lop] = value_dr2;}
				num += miumaxstbd[lop]*jfloat3;
				denum += miumaxstbd[lop];
				}
			for (lop = 70;lop < 80 ; lop++){
				jfloat3 = (float)lop;
				o = omfstbd[3][0];  	t = omfstbd[3][1];	v = omfstbd[3][2];
				miu_vhigh4[lop] = persgaris3(o, t, v, jfloat3);
				miumaxstbd[lop] = miu_vhigh4[lop];
				if (miumaxstbd[lop] > value_sl){miumaxstbd[lop] = value_sl;}
				num += miumaxstbd[lop]*jfloat3;
				denum += miumaxstbd[lop];
				}
			for (lop = 80;lop <= 100 ; lop++){
					num += value_sl*lop;
					denum += value_sl;
				}
		  }
	else if(value_a == 0 && value_w1 > 0 && value_w2 > 0 && value_w3 > 0){
			for (lop = 30;lop < 55 ; lop++){
				jfloat3 = (float)lop;
				d = omfstbd[1][0];	    e = omfstbd[1][1];	f = omfstbd[1][2];
				miu_medium4[lop] = persgaris2(d, e, f, jfloat3);;
				miumaxstbd[lop] = miu_medium4[lop];
				if (miumaxstbd[lop] > value_dr1){miumaxstbd[lop] = value_dr1;}
				num += miumaxstbd[lop]*jfloat3;
				denum += miumaxstbd[lop];
			}
			for (lop = 55;lop < 74 ; lop++){
				jfloat3 = (float)lop;
				g = omfstbd[2][0];	    h = omfstbd[2][1];	l = omfstbd[2][2];
				miu_high4[lop] = persgaris2(g, h, l, jfloat3);
				miumaxstbd[lop] = miu_high4[lop];
				if (miumaxstbd[lop] > value_dr2){miumaxstbd[lop] = value_dr2;}
				num += miumaxstbd[lop]*jfloat3;
				denum += miumaxstbd[lop];
				}
			for (lop = 74;lop < 80 ; lop++){
				jfloat3 = (float)lop;
				o = omfstbd[3][0];  	t = omfstbd[3][1];	v = omfstbd[3][2];
				miu_vhigh4[lop] = persgaris3(o, t, v, jfloat3);
				miumaxstbd[lop] = miu_vhigh4[lop];
				if (miumaxstbd[lop] > value_sl){miumaxstbd[lop] = value_sl;}
				num += miumaxstbd[lop]*jfloat3;
				denum += miumaxstbd[lop];
				}
			for (lop = 80;lop <= 100 ; lop++){
					num += value_sl*lop;
					denum += value_sl;
				}
		  }
	else if(value_a == 0 && value_w1 > 0 && value_w2 > 0 && value_w3 == 0){
			for (lop = 30;lop < 55 ; lop++){
				jfloat3 = (float)lop;
				d = omfstbd[1][0];	    e = omfstbd[1][1];	f = omfstbd[1][2];
				miu_medium4[lop] = persgaris2(d, e, f, jfloat3);;
				miumaxstbd[lop] = miu_medium4[lop];
				if (miumaxstbd[lop] > value_dr1){miumaxstbd[lop] = value_dr1;}
				num += miumaxstbd[lop]*jfloat3;
				denum += miumaxstbd[lop];
			}
			for (lop = 55;lop < 80 ; lop++){
				jfloat3 = (float)lop;
				g = omfstbd[2][0];	    h = omfstbd[2][1];	l = omfstbd[2][2];
				miu_high4[lop] = persgaris2(g, h, l, jfloat3);
				miumaxstbd[lop] = miu_high4[lop];
				if (miumaxstbd[lop] > value_dr2){miumaxstbd[lop] = value_dr2;}
				num += miumaxstbd[lop]*jfloat3;
				denum += miumaxstbd[lop];
				}

		  }



	centroid = num / denum;
		if (centroid < 0)
							      {
				centroid =0;
							      }
}

//float carimax(float max[8]){
//	maxx = max[0];
//	for (i = 1 ; i < 7 ; i++){
//		if (maxx < max[i]){maxx= max[i];}
//	}
//	return maxx;
//}

void Fuzzy(void){
	m = n % filter_length;

	in1 = abs(TiltAc);//98.9;
	in2 = durasi*10.0;
	in3 = kecepatan;

	jfloat = in1;

	a = imfps[0][0];	b = imfps[0][1];	c = imfps[0][2];
	d = imfps[1][0];	e = imfps[1][1];	f = imfps[1][2];

	miu_low[m] = persgaris1(a, b, c, jfloat);
	miu_high[m] = persgaris3(d, e, f, jfloat);

//	miuk_low=miu_low[m];
//	miuk_high=miu_high[m];

	jfloat1 = in2;

	a = imfdr[0][0];	b = imfdr[0][1];	c = imfdr[0][2];
	d = imfdr[1][0];	e = imfdr[1][1];	f = imfdr[1][2];
	g = imfdr[2][0];	h = imfdr[2][1];	l = imfdr[2][2];
	o = imfdr[3][0];	t = imfdr[3][1];	v = imfdr[3][2];

	miu_low1[m]=persgaris1(a, b, c, jfloat1);
	miu_medium1[m]=persgaris2(d, e, f, jfloat1);
	miu_high1[m]=persgaris2(g, h, l, jfloat1);
	miu_vhigh1[m]=persgaris3(o, t, v, jfloat1);

//	miuk_low=miu_low[m];
//	miuk_high=miu_high[m];

	jfloat2 = in3;

	a = imfkc[0][0];	b = imfkc[0][1];	c = imfkc[0][2];
	d = imfkc[1][0];	e = imfkc[1][1];	f = imfkc[1][2];

	miu_low2[m]=persgaris1(a, b, c, jfloat2);
	miu_high2[m]=persgaris3(d, e, f, jfloat2);

//	miuk_low2=miu_low[m];
//	miuk_high2=miu_high[m];



	rule[0] = carimin(carimin(miu_low[m],   miu_low1[m]),    miu_low2[m] );//active
	rule[1] = carimin(carimin(miu_low[m],   miu_medium1[m]), miu_low2[m] );//active
	rule[2] = carimin(carimin(miu_low[m],   miu_high1[m]),   miu_low2[m] );//active
	rule[3] = carimin(carimin(miu_low[m],   miu_vhigh1[m]),  miu_low2[m] );//active
	rule[4] = carimin(carimin(miu_low[m],   miu_low1[m]),    miu_high2[m] );//active
	rule[5] = carimin(carimin(miu_low[m],   miu_medium1[m]), miu_high2[m] );//active
	rule[6] = carimin(carimin(miu_low[m],   miu_high1[m]),   miu_high2[m] );//active
	rule[7] = carimin(carimin(miu_low[m],   miu_vhigh1[m]),  miu_high2[m] );//active
	rule[8] = carimin(carimin(miu_high[m],  miu_low1[m]),    miu_low2[m] );//active
	rule[9] = carimin(carimin(miu_high[m],  miu_medium1[m]), miu_low2[m] );//weak1
	rule[10] = carimin(carimin(miu_high[m], miu_high1[m]),   miu_low2[m] );//weak2
	rule[11] = carimin(carimin(miu_high[m], miu_vhigh1[m]),  miu_low2[m] );//weak3
	rule[12] = carimin(carimin(miu_high[m], miu_low1[m]),    miu_high2[m] );//active
	rule[13] = carimin(carimin(miu_high[m], miu_medium1[m]), miu_high2[m] );//weak1
	rule[14] = carimin(carimin(miu_high[m], miu_high1[m]),   miu_high2[m] );//weak2
	rule[15] = carimin(carimin(miu_high[m], miu_vhigh1[m]),  miu_high2[m] );//weak3


	value_a = carimax(carimax(carimax(carimax(carimax(carimax(carimax(carimax(carimax(rule[0], rule[1]), rule[2]), rule[3]), rule[4]),rule[5]),rule[6]),rule[7]),rule[8]),rule[12]);
	value_w1 = carimax(rule[9], rule[13]);
	value_w2  = carimax(rule[10], rule[14]);
	value_w3  = carimax(rule[11], rule[15]);

//	center();
	miul_pos=miu_low[m];
	miuh_pos=miu_high[m];

	mius_dur=miu_low1[m];
	mium_dur=miu_medium1[m];
	miul_dur=miu_high1[m];
	miuvl_dur=miu_vhigh1[m];

	miul_kec=miu_low2[m];
	miuh_kec=miu_high2[m];

	Fuzzy1();


	outf=centroid;



	n++;
//

////	//cari maksimal
//	fuzzy_mks[0] = carimax(pl);
//	fuzzy_mks[1] = carimax(pm);
//	fuzzy_mks[2] = carimax(ps);
//
//
//	//centroid
//	centr();
//
//	stmltime = stmltime_1+centroid;
//

}

void Fuzzy1(void){
	m = n % filter_length;

	in4 = hr;
	in5[m]= value_a;
	in6[m]= value_w1;
	in7[m]= value_w2;
	in8[m]= value_w3;

	jfloat4 = in4;

	a = imfhr[0][0];	b = imfhr[0][1];	c = imfhr[0][2];
	d = imfhr[1][0];	e = imfhr[1][1];	f = imfhr[1][2];
	g = imfhr[2][0];	h = imfhr[2][1];	l = imfhr[2][2];

	miu_low3[m] = persgaris1(a, b, c, jfloat4);
	miu_medium3[m] = persgaris2(d, e, f, jfloat4);
	miu_high3[m] = persgaris3(g, h, l, jfloat4);

	rule[0]  = carimin(miu_low3[m], in5[m]);//Aw
	rule[1]  = carimin(miu_low3[m], in6[m]);//w1
	rule[2]  = carimin(miu_low3[m], in7[m]);//w2
	rule[3]  = carimin(miu_low3[m], in8[m]);//sl
	rule[4]  = carimin(miu_medium3[m], in5[m]);//aw
	rule[5]  = carimin(miu_medium3[m], in6[m]);//aw
	rule[6]  = carimin(miu_medium3[m], in7[m]);//w1
	rule[7]  = carimin(miu_medium3[m], in8[m]);//w2
	rule[8]  = carimin(miu_high3[m], in5[m]);//aw
	rule[9]  = carimin(miu_high3[m], in6[m]);//aw
	rule[10] = carimin(miu_high3[m], in7[m]);//w1
	rule[11] = carimin(miu_high3[m], in8[m]);//w1

	value_aw  = carimax(carimax(carimax(carimax(rule[0], rule[4]), rule[5]), rule[8]), rule[9]);
	value_dr1 = carimax(carimax(carimax(rule[1], rule[6]), rule[10]), rule[11]);
	value_dr2 = carimax(rule[2], rule[7]);
    value_sl  = rule[3];

    max1=value_aw;
    max2=value_dr1;
    max3=value_dr2;
    max4=value_sl;
	center();
//	miuk_low=miu_medium[m];
//    miuk_medium=miu_medium1[m];
//    miuk_high=miu_medium2[m];
//	miul_hr=miu_low[m];
//	//mium_hr=miu_medium[m];
//	miuh_hr=miu_high[m];
//	miul_pos=miu_low1[m];
//	mium_pos=miu_medium1[m];
//	miuh_pos=miu_high1[m];
//	miul_kec=miu_low2[m];
//	mium_kec=miu_medium2[m];
//	miuh_kec=miu_high2[m];




	n++;
//

////	//cari maksimal
//	fuzzy_mks[0] = carimax(pl);
//	fuzzy_mks[1] = carimax(pm);
//	fuzzy_mks[2] = carimax(ps);
//
//
//	//centroid
//	centr();
//
//	stmltime = stmltime_1+centroid;
//

}


/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//uint8_t Rx_data;  //  creating a buffer of 10 bytes
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
		if(htim->Instance == TIM2){
			flag=1;
		}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_IT(&huart2, Rx_data, 1);
	if(Rx_data[0]=='a'){

		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, RESET);
		memset(sinyal, 0, sizeof(sinyal));
		TiltAc=0.0;
		HAL_TIM_Base_Start_IT(&htim2);
		//t=0;
		Program_state=1;
		//count1=0;
		flagkirim=1;
		zeroposition=1.0;
		kondisi_sebelum=1;

		//CDC_Transmit_FS(Rx_data, 1);

		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, RESET);
	}
	else if(Rx_data[0]=='b'){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, SET);
		HAL_TIM_Base_Stop_IT(&htim2);
		//t=0;
		flagkirim=0;
		Program_state=0;
		zeroposition=0.0;
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,0);
		//count2=0;
		//CDC_Transmit_FS(Rx_data, 1);
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, SET);
	}


}

void ADC_Select_CH8 (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = ADC_CHANNEL_8;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void ADC_Select_CH1 (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = ADC_CHANNEL_1;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void ADC_Select_CH4 (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = ADC_CHANNEL_4;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void ADC_Select_CH5 (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = ADC_CHANNEL_5;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}


//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(flagkirim==1){
//		if(flagkirim2==1){
//			flagkirim2=0;
//			HAL_UART_Transmit(&huart1, Cmd_End, 3,10);
////			HAL_UART_Transmit(&huart1, ecg_data, strlen(ecg_data),10);
//			HAL_UART_Transmit_IT(&huart1, ecg_data, strlen(ecg_data));
////			HAL_UART_Transmit_IT(&huart1, Cmd_End, 3);
//
//		}
//	}
//}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	Bias_T = 0; Bias_S = 0; Bias_F = 0;
	P1T = 0; P2T = 0; P3T = 0; P4T = 0;
	P1S = 0; P2S = 0; P3S = 0; P4S = 0;
	P1F = 0; P2F = 0; P3F = 0; P4F = 0;

	ZrTiltUparmAc = 0.0;
	zeroposition=0.0;


	 //----------------------INPUT MEMBERSHIP HEART RATE-----------------------//
			imfhr[0][0] = 0.0;	imfhr[0][1] = 60.0;		imfhr[0][2] = 80.0;
			imfhr[1][0] = 63.4;	imfhr[1][1] = 80.0;		imfhr[1][2] = 95.6;
			imfhr[2][0] = 90.0;	imfhr[2][1] = 100.0;	imfhr[2][2] = 140.0;

			//------------------INPUT MEMBERSHIP FUNCTION DATA POSTURAL------------------//
			imfps[0][0] = 0.0;	imfps[0][1] = 43.0;		imfps[0][2] = 53.0;
			imfps[1][0] = 43.0;	imfps[1][1] = 53.0;		imfps[1][2] = 135.0;

			//------------------INPUT MEMBERSHIP FUNCTION DATA KECEPATAN SUDUT------------------//
			imfkc[0][0] = 0.0;	    imfkc[0][1] = 625.0;		imfkc[0][2] = 898.0;
			imfkc[1][0] = 625.0;	imfkc[1][1] = 898.0;		imfkc[1][2] = 3500.0;

			//------------------INPUT MEMBERSHIP FUNCTION DURASI------------------//
			imfdr[0][0] = 0.0;	imfdr[0][1] = 10;		imfdr[0][2] = 15;
			imfdr[1][0] = 10;	imfdr[1][1] = 18 ;		imfdr[1][2] = 26;
			imfdr[2][0] = 22;	imfdr[2][1] = 30;		imfdr[2][2] = 38;
			imfdr[3][0] = 33;	imfdr[3][1] = 40;		imfdr[3][2] = 70;

//			//----------------------OUTPUT MEMBERSHIP FUNCTION------------------------//
			omfstbd[0][0] = 0.0;  	omfstbd[0][1] = 30.0;	omfstbd[0][2] = 40.0;
			omfstbd[1][0] = 30.0;	omfstbd[1][1] = 45.0;	omfstbd[1][2] = 60.0;
			omfstbd[2][0] = 50.0;  	omfstbd[2][1] = 65.0;	omfstbd[2][2] = 80.0;
			omfstbd[3][0] = 70.0;  	omfstbd[3][1] = 80.0;	omfstbd[3][2] = 100.0;

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  TIM1->CCR1=6;
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,0);
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_UART_Receive_IT (&huart1, Rx_data, sizeof(Rx_data));
  HAL_UART_Receive_IT(&huart2,Rx_data,1);
//  HAL_UART_Transmit_IT(&huart1, Cmd_End, 3);
//  HAL_UART_Transmit_IT(&huart1, ecg_data, strlen(ecg_data));
//


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(Program_state==1){
	  			  if(flag==1){
	  //
	  			  			  flag=0;
	  		  			      ADC_Select_CH1();
	  		  			  	  HAL_ADC_Start(&hadc1);
	  		  			 	  HAL_ADC_PollForConversion(&hadc1, 1000);
	  		  			 	  ecg = HAL_ADC_GetValue(&hadc1);;
	  		  			 	  HAL_ADC_Stop(&hadc1);

	  		  			 	  ADC_Select_CH5();
	  		  			 	  HAL_ADC_Start(&hadc1);
	  		  			 	  HAL_ADC_PollForConversion(&hadc1, 1000);
	  		  			 	  Acc1x = HAL_ADC_GetValue(&hadc1);;
	  		  			 	  HAL_ADC_Stop(&hadc1);

	  		  			 	  ADC_Select_CH8();
	  		  			 	  HAL_ADC_Start(&hadc1);
	  		  			 	  HAL_ADC_PollForConversion(&hadc1, 1000);
	  		  			 	  Acc1z = HAL_ADC_GetValue(&hadc1);;
	  		  			 	  HAL_ADC_Stop(&hadc1);

	  		  			 	  ADC_Select_CH4();
	  		  			 	  HAL_ADC_Start(&hadc1);
	  		  			 	  HAL_ADC_PollForConversion(&hadc1, 1000);
	  		  			 	  gyro1 = HAL_ADC_GetValue(&hadc1);;
	  		  			 	  HAL_ADC_Stop(&hadc1);

	  		  	//		 	  map_ecg = round((float)ecg/4096.0*180.0);
	  		  	//		 	  map_hasil = (ecg_lpf*150);
	  		  	//		 	  sprintf(ecg_data, "add 1,0,%d",map_ecg);
	  		  	//		 	  sprintf(hasil_data, "add 5,0,%d",map_hasil);
	  		  	//		 	  //map_hasil = round((float)ecg[0]/4096.0*200.0);
	  		  	////		      sprintf(ecg_data, "add 5,0,%d",map hasil);
	  		  	//		 	  HAL_UART_Transmit(&huart1, ecg_data, strlen(ecg_data),10);
	  		  	//		 	  HAL_UART_Transmit(&huart1, Cmd_End, 3, 10);
	  		  	//		 	  HAL_UART_Transmit(&huart1, hasil_data, strlen(hasil_data), 10);
	  		  	//		 	  HAL_UART_Transmit(&huart1, Cmd_End, 3, 10);
	  		  			 	  Kalman();
	  		  			 	  PanTompkins();
	  		  			 	  vel();
	  		  			 	  Fuzzy();
	  		  			 	  time();

	  		  			 	  if(outf>=65.0){
	  		  			 	   	 // HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,1);
	  		  			 		  tmr=1;
	  		  			 		  fes=1;
	  		  			 		  HAL_UART_Transmit_IT(&huart2, myData, 0);
	  		  			 	  }
	  		  			 	 if(fes==1){
								  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,1);
								  on++;
								  off=0;
								  if(on==750){
									fes2=1;
									fes=0;
								  }

							  }
	  		  			 	 if(fes2==1){
	  		  			 		 off++;
	  		  			 		 on=0;
	  		  			 		 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,0);
	  		  			 		 if(off==750){
	  		  			 			 fes=1;
	  		  			 			 fes2=0;
	  		  			 		 }
	  		  			 	 }

	  		  			 	  if (tmr==1){
	  		  			 		timerCount++;
	  		  			 	  }
	  		  			 	  if (timerCount==45000){
	  		  			 		fes=0;
	  		  			 		fes2=0;
	  		  			 		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,0);
	  		  			     	//HAL_UART_Transmit_IT(&huart2, myData, 1);
	  		  			 	  }


	  		  			 			Acc1xa=((UparmAcclX_LPF+360)*10)/100;
	  		  			 			Acc1xb=((UparmAcclX_LPF+360)*10)-(Acc1xa*100);
	  		  			 			Acc1za=((UparmAcclZ_LPF+360)*10)/100;
	  		  			 			Acc1zb=((UparmAcclZ_LPF+360)*10)-(Acc1za*100);


	  		  			 			gyro1a=((UparmGyroY_LPF+360)*10)/100;
	  		  			 			gyro1b=((UparmGyroY_LPF+360)*10)-(gyro1a*100);

	  		  			 			Uparmac_a=((TiltAc+360)*10)/100;
	  		  			 			Uparmac_b=((TiltAc+360)*10)-(Uparmac_a*100);


	  		  			 			Uparmkal_a=((TiltKal+360)*10)/100;
	  		  			 			Uparmkal_b=((TiltKal+360)*10)-(Uparmkal_a*100);

	  		  			 			gyroUparma=((GyroUparm+6000)*10)/10000;
	  		  			 			gyroUparmb=(((GyroUparm+6000)*10)/100)-(gyroUparma*100);
	  		  			 			gyroUparmc=((GyroUparm+6000)*10)-((gyroUparma*10000)+(gyroUparmb*100));

	  		  			 		    VUparmx = (Acc1x*3.3)/4095;
	  		  			 		    VUparmz = (Acc1z*3.3)/4095;
	  		  			 		   	VgyroUparm = (gyro1*3.3)/4095;


	  		  			 			VUparmx_a = (VUparmx*100)/100;
	  		  			 			VUparmx_b = (VUparmx*100)-(VUparmx_a*100);
	  		  			 			VUparmz_a = (VUparmz*100)/100;
	  		  			 			VUparmz_b = (VUparmz*100)-(VUparmz_a*100);

	  		  			 			VgyroUparm_a = (VgyroUparm*100)/100;
	  		  			 			VgyroUparm_b = (VgyroUparm*100)-(VgyroUparm_a*100);

	  		  			 			Vecg_a = ((Vecg+50)*100)/100;
	  		  			 			Vecg_b = ((Vecg+50)*100)-(Vecg_a*100);


	  		  			 			myData[0]='e';

	  		  			 			myData[1]=UparmAcclX_LPF/100;
	  		  			 			myData[2]=UparmAcclX_LPF-(myData[1]*100);
	  		  			 			myData[3]=UparmAcclZ_LPF/100;
	  		  			 		    myData[4]=UparmAcclZ_LPF-(myData[3]*100);
	  		  			 			myData[5]=UparmGyroY_LPF/100;
	  		  			 			myData[6]=UparmGyroY_LPF-(myData[5]*100);
	  		  			 			myData[7]=ecg/100;
	  		  			 			myData[8]=ecg-(myData[7]*100);
	  		  		//
	  		  			 			myData[9]=Uparmac_a;
	  		  			 			myData[10]=Uparmac_b;
	  		  			 			myData[11]=Uparmkal_a;
	  		  			 		    myData[12]=Uparmkal_b;
	  		  	//		 		    myData[13]=Uparmkal_a;
	  		  	//		 		   	myData[14]=Uparmkal_b;
	  		  			    		myData[13]=(kecepatan+1000)/100;
	  		  			 		   	myData[14]=(kecepatan+1000)-(myData[13]*100);

	  		  			 			myData[15]=ri;

	  		  			 			myData[16]=Vecg_a;
	  		  			 			myData[17]=Vecg_b;

	  		  			 			myData[18]=(hr*100)/100;
	  		  			 			myData[19]=(hr*100)-(myData[18]*100);

	  		  			 			myData[20]=((durasi+20)*100)/100;
	  		  			 			myData[21]=((durasi+20)*100)-(myData[20]*100);

	  		  			 			myData[22]=((outf+200)*10)/100;
	  		  			 			myData[23]=((outf+200)*10)-(myData[22]*100);

//	  		  			 		    myData[24]=((value_rmssd+20)*100)/100;
//	  		  			 			myData[25]=((value_rmssd+20)*100)-(myData[24]*100);



//	  		  			 			myData[20]=((ecg_lpf+100)*10)/100;;
//	  		  			 			myData[21]=((ecg_lpf+100)*10)-(myData[20]*100);
//
//	  		  			 			myData[22]=((ecg_hpf+50)*100)/100;
//	  		  			 		    myData[23]=((ecg_hpf+50)*100)-(myData[22]*100);
//
//	  		  			 		    myData[24]=((deriv+200)*10)/100;
//	  		  			 		    myData[25]=((deriv+200)*10)-(myData[24]*100);
//
//	  		  			 		    myData[26]=(squar+1000)/100;
//	  		  			 		    myData[27]=(squar+1000)-(myData[26]*100);
//
//	  		  			 			myData[28]=((mav+100)*10)/100;
//	  		  			 			myData[29]=((mav+100)*10)-(myData[28]*100);
//
//	  		  			 			myData[30]=((mav1+100)*10)/100;
//	  		  			 			myData[31]=((mav1+100)*10)-(myData[30]*100);
//
//	  		  			 		    myData[32]=((thr+200)*10)/100;
//	  		  			 		    myData[33]=((thr+200)*10)-(myData[32]*100);

//	  		  			 		    myData[34]=((durasi+20)*100)/100;
//									myData[35]=((durasi+20)*100)-(myData[34]*100);
//
//									myData[36]=((outf+200)*10)/100;
//									myData[37]=((outf+200)*10)-(myData[36]*100);
	  //	//
	  //		  			 		    myData[34]=((miul_hr+50)*100)/100;
	  //		  			 		    myData[35]=((miul_hr+50)*100)-(myData[34]*100);
	  //
	  //		  			 		    myData[36]=((mium_hr+50)*100)/100;
	  //		  						myData[37]=((mium_hr+50)*100)-(myData[36]*100);
	  //
	  //		  						myData[38]=((miuh_hr+50)*100)/100;
	  //		  						myData[39]=((miuh_hr+50)*100)-(myData[38]*100);
	  //
	  //		  						myData[40]=((miul_pos+50)*100)/100;
	  //								myData[41]=((miul_pos+50)*100)-(myData[40]*100);
	  //
	  //								myData[42]=((mium_pos+50)*100)/100;
	  //								myData[43]=((mium_pos+50)*100)-(myData[42]*100);
	  //
	  //								myData[44]=((miuh_pos+50)*100)/100;
	  //								myData[45]=((miuh_pos+50)*100)-(myData[44]*100);
	  //
	  //								myData[46]=((miul_kec+50)*100)/100;
	  //								myData[47]=((miul_kec+50)*100)-(myData[46]*100);
	  //
	  //								myData[48]=((mium_kec+50)*100)/100;
	  //								myData[49]=((mium_kec+50)*100)-(myData[48]*100);
	  //
	  //								myData[50]=((miuh_kec+50)*100)/100;
	  //								myData[51]=((miuh_kec+50)*100)-(myData[50]*100);
	  //
	  //								myData[52]=((outf+200)*10)/100;
	  //						        myData[53]=((outf+200)*10)-(myData[52]*100);
	  //
	  //						        myData[54]=((durasi+20)*100)/100;
	  //						        myData[55]=((durasi+20)*100)-(myData[54]*100);

//	  		  			 		    myData[22]=((miul_pos+50)*100)/100;
//	  		  			 		    myData[23]=((miul_pos+50)*100)-(myData[22]*100);
//
//	  		  			 		    myData[24]=((miuh_pos+50)*100)/100;
//	  		  						myData[25]=((miuh_pos+50)*100)-(myData[24]*100);
//
//	  		  						myData[26]=((mius_dur+50)*100)/100;
//	  		  						myData[27]=((mius_dur+50)*100)-(myData[26]*100);
//
//	  		  						myData[28]=((mium_dur+50)*100)/100;
//	  								myData[29]=((mium_dur+50)*100)-(myData[28]*100);
//
//	  								myData[30]=((miul_dur+50)*100)/100;
//	  								myData[31]=((miul_dur+50)*100)-(myData[30]*100);
//
//	  								myData[32]=((miuvl_dur+50)*100)/100;
//	  								myData[33]=((miuvl_dur+50)*100)-(myData[32]*100);
//
//	  								myData[34]=((miul_kec+50)*100)/100;
//	  								myData[35]=((miul_kec+50)*100)-(myData[34]*100);
//
//	  								myData[36]=((miuh_kec+50)*100)/100;
//	  								myData[37]=((miuh_kec+50)*100)-(myData[36]*100);
//
//	  								myData[38]=((max1+50)*100)/100;
//									myData[39]=((max1+50)*100)-(myData[38]*100);
//
//									myData[40]=((max2+50)*100)/100;
//									myData[41]=((max2+50)*100)-(myData[40]*100);
//
//									myData[42]=((max3+50)*100)/100;
//									myData[43]=((max3+50)*100)-(myData[42]*100);
//
//									myData[44]=((max4+50)*100)/100;
//									myData[45]=((max4+50)*100)-(myData[44]*100);
//
//									myData[46]=((outf+200)*10)/100;
//								    myData[47]=((outf+200)*10)-(myData[46]*100);

	  //								myData[50]=((miuh_kec+50)*100)/100;
	  //								myData[51]=((miuh_kec+50)*100)-(myData[50]*100);
	  //
	  //								myData[52]=((outf+200)*10)/100;
	  //						        myData[53]=((outf+200)*10)-(myData[52]*100);
	  //
	  //						        myData[54]=((durasi+20)*100)/100;
	  //						        myData[55]=((durasi+20)*100)-(myData[54]*100);


	  		  			 		//	myData[55] = 'f';
	  		  		//
	  		  			 			HAL_UART_Transmit_IT(&huart2, myData, sizeof(myData));


	  		  		//	 			char myString1[30] = "";  //																    10										20									    30										40										50									   60									    70

	  		  		//	 		    sprintf(myString1,"E%04u\n",
	  		  		//	 			sprintf(myString1,"E%04u%04u%04u%02u%02u%02u%02u%02u%02u%02u\n",
	  		  		//	 				   (uint16_t)Acc1x,(uint16_t)Acc1z, (uint16_t)gyro1, (uint8_t)Uparmac_a,
	  		  		//	 				   (uint8_t)Uparmac_b,(uint8_t)Uparmkal_a,(uint8_t)Uparmkal_b,
	  		  		//	 			       (uint8_t)gyroUparma,(uint8_t)gyroUparmb,(uint8_t)gyroUparmc);
	  		  		//	 			HAL_UART_Transmit_IT(&huart2,(uint8_t *) myString1,strlen((char const*)myString1));
	  		  		//	 		    HAL_Delay(100);

	  		  //			 			sprintf(ecg_kirim,"E%04d\n",(uint16_t)ecg);
	  		  //			 			HAL_UART_Transmit(&huart2, ecg_kirim, strlen(ecg_kirim), 100);
	  		  			 		//	cntr+=1;

	  		  		//	 	        sprintf(ecg_kirim,"%04d\n",ecg);
	  		  		//	            CDC_Transmit_FS(ecg_kirim, strlen(ecg_kirim));

	  		  	//		 		    if(flagkirim==1){


	  		  	//		 		  	map_ecg = round((float)ecg/4096.0*180.0);
	  		  		//	 		    map_gyro = round((float)gyro/4096.0*200.0);
	  		  	//		 		    sprintf(ecg_data, "add 1,0,%d",map_ecg);
	  		  	//	 			    HAL_UART_Transmit_IT(&huart1, ecg_data, strlen(ecg_data));
	  		  	//		 		    HAL_UART_Transmit_IT(&huart1, Cmd_End, 3);
	  		  		//	 		    HAL_UART_Transmit(&huart1, gyro_data, strlen(gyro_data), 100);
	  		  		//	 		    HAL_UART_Transmit(&huart1, Cmd_End, 3, 10);
	  		  			 		    }


	  		  //			 		  		  	 }



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  		}


    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 36;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 200;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7200-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 40-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 3600;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 997;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
