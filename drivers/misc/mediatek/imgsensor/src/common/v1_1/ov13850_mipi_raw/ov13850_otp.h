#ifndef OV13850_OTP_H
#define OV13850_OTP_H

//#define OV13850_OTP_SUPPORT

#if 0//def OV13850_OTP_SUPPORT
/*file discription : otp*/
#define OTP_DRV_LSC_SIZE 186

struct otp_struct {
int flag;//bit[7]:info,bit[6]:wb,bit[5]:vcm,bit[4]:lenc
int module_integrator_id;
int lens_id;
int production_year;
int production_month;
int production_day;
int rg_ratio;
int bg_ratio;
//int light_rg;
//int light_bg;
//int typical_rg_ratio;
//int typical_bg_ratio;
unsigned char lenc[OTP_DRV_LSC_SIZE];
int checksumLSC;
int checksumOTP;
int checksumTotal;
int VCM_start;
int VCM_end;
int VCM_dir;
};

void otp_cali(unsigned char writeid);

#endif

#endif






