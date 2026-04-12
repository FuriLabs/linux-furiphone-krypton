#ifndef OV8856OTP
#define OV8856OTP
//#ifdef CONFIG_RLK_CAM_OTP_ON_OR_OFF
//extern int cqcq_disable_otp;
//#endif
#include "ov8856mipiraw_Sensor.h"
struct otp_struct {
int flag; // bit[7]: info, bit[6]:wb, bit[5]:vcm, bit[4]:lenc
int module_integrator_id;
int lens_id;
int production_year;
int production_month;
int production_day;
int rg_ratio;
int bg_ratio;
int lenc[240];
int checksum;
int VCM_start;
int VCM_end;
int VCM_dir;
};

// return value:
// bit[7]: 0 no otp info, 1 valid otp info
// bit[6]: 0 no otp wb, 1 valib otp wb
// bit[5]: 0 no otp vcm, 1 valid otp vcm
// bit[4]: 0 no otp lenc/invalid otp lenc, 1 valid otp lenc

int flag_awbotpread = 0;
int flag_lscotpread = 0;
struct otp_struct otpdate;
int read_otp(void)
{
	int otp_flag, addr, temp, i;
	//set 0x5002[3] to
	int temp1;
	int checksum2=0;
/*
#ifdef CONFIG_RLK_CAM_OTP_ON_OR_OFF
	if(cqcq_disable_otp==1)return otpdate.flag;//quan.chang debug
	else if(cqcq_disable_otp==2){
		flag_awbotpread=0;
		flag_lscotpread=0;
	}
#endif
*/
	if(flag_awbotpread!=1)
	{
		write_cmos_sensor(0x0100, 0x01);
		temp1 = read_cmos_sensor(0x5001);
		write_cmos_sensor(0x5001, (0x00 & 0x08) | (temp1 & (~0x08)));
		// read OTP into buffer
		write_cmos_sensor(0x3d84, 0xC0);
		write_cmos_sensor(0x3d88, 0x70); // OTP start address
		write_cmos_sensor(0x3d89, 0x10);
		write_cmos_sensor(0x3d8A, 0x72); // OTP end address
		write_cmos_sensor(0x3d8B, 0x0a);
		write_cmos_sensor(0x3d81, 0x01); // load otp into buffer
		mdelay(10);
		// OTP base information and WB calibration data
		otp_flag = read_cmos_sensor(0x7010);
		addr = 0;
		if((otp_flag & 0xc0) == 0x40) {
			addr = 0x7011; // base address of info group 1
		}
		else if((otp_flag & 0x30) == 0x10) {
			addr = 0x7019; // base address of info group 2
		}
		if(addr != 0) {
			otpdate.flag = 0xC0; // valid info and AWB in OTP
			otpdate.module_integrator_id = read_cmos_sensor(addr);
			 LOG_INF("module_integrator_id : 0x%x,addr : %x\n",otpdate.module_integrator_id,addr);	

			otpdate.lens_id = read_cmos_sensor( addr + 1);
			otpdate.production_year = read_cmos_sensor( addr + 2);
			otpdate.production_month = read_cmos_sensor( addr + 3);
			otpdate.production_day = read_cmos_sensor(addr + 4);
			temp = read_cmos_sensor(addr + 7);
			otpdate.rg_ratio = (read_cmos_sensor(addr + 5)<<2) + ((temp>>6) & 0x03);
			otpdate.bg_ratio = (read_cmos_sensor(addr + 6)<<2) + ((temp>>4) & 0x03);
			LOG_INF("ww_lens_id=%x,addr=%x ,rg_h=%x,addr=%x,bg_h=%x,addr=%x,temp=%x,otpdate.rg_ratio=%x,otpdate.bg_ratio=%x\n",otpdate.lens_id,( addr + 1),read_cmos_sensor(addr + 5),(addr + 5),read_cmos_sensor(addr + 6),(addr + 6),temp,otpdate.rg_ratio,otpdate.bg_ratio);//add by wei.wang2 for otp check
		}
		else {
			otpdate.flag = 0x00; // not info and AWB in OTP
			otpdate.module_integrator_id = 0;
			otpdate.lens_id = 0;
			otpdate.production_year = 0;
			otpdate.production_month = 0;
			otpdate.production_day = 0;
			otpdate.rg_ratio = 0;
			otpdate.bg_ratio = 0;
		}
		flag_awbotpread = 1;
		 LOG_INF("module_integrator_id : 0x%x,year = %d,month=%d,day =%d\n",otpdate.module_integrator_id,otpdate.production_year,otpdate.production_month,otpdate.production_day);	
	}
	#if 0
	// OTP VCM Calibration
	otp_flag = read_cmos_sensor(0x7021);
	addr = 0;
	if((otp_flag & 0xc0) == 0x40) {
		addr = 0x7022; // base address of VCM Calibration group 1
	}
	else if((otp_flag & 0x30) == 0x10) {
		addr = 0x7025; // base address of VCM Calibration group 2
	}
	if(addr != 0) {
		otpdate.flag |= 0x20;
		temp = read_cmos_sensor(addr + 2);
		otpdate.VCM_start = (read_cmos_sensor(addr)<<2) | ((temp>>6) & 0x03);
		otpdate.VCM_end = (read_cmos_sensor(addr + 1) << 2) | ((temp>>4) & 0x03);
		otpdate.VCM_dir = (temp>>2) & 0x03;
	}
	else {
		otpdate.VCM_start = 0;
		otpdate.VCM_end = 0;
		otpdate.VCM_dir = 0;
	}
	#endif
	// OTP Lenc Calibration
	if(flag_lscotpread!=1)
	{
		otp_flag = read_cmos_sensor(0x7028);
		addr = 0;
		if((otp_flag & 0xc0) == 0x40) {
			addr = 0x7029; // base address of Lenc Calibration group 1
		}
		else if((otp_flag & 0x30) == 0x10) {
			addr = 0x711a; // base address of Lenc Calibration group 2
		}
		if(addr != 0) {
			for(i=0;i<240;i++) {
				otpdate.lenc[i]=read_cmos_sensor(addr + i);
				LOG_INF("ww_ov8856 addr = %x,data = %x\n",(addr+i),otpdate.lenc[i]);//add by wei.wang2 for otp check
				checksum2 += otpdate.lenc[i];
			}
			checksum2 = (checksum2)%256;
			otpdate.checksum = read_cmos_sensor(addr + 240);
			if(otpdate.checksum == checksum2){
				LOG_INF("ov8856 LSC checksum pass !\n");
				otpdate.flag |= 0x10;	
			}
			else{
				LOG_INF("ov8856 LSC checksum fail !set_checksum =%d,read_checksum=%d\n",otpdate.checksum,checksum2);
			}
		}
		else {
			for(i=0;i<240;i++) {
				otpdate.lenc[i]=0;
			}
		}
		for(i=0x7010;i<=0x720a;i++) {
			write_cmos_sensor(i,0); // clear OTP buffer, recommended use continuous write to accelarate
		}
		//set 0x5002[3] to
		temp1 = read_cmos_sensor(0x5001);
		write_cmos_sensor(0x5001, (0x08 & 0x08) | (temp1 & (~0x08)));
		flag_lscotpread =1;
	}
	return otpdate.flag;
}
// return value:
// bit[7]: 0 no otp info, 1 valid otp info
// bit[6]: 0 no otp wb, 1 valib otp wb
// bit[5]: 0 no otp vcm, 1 valid otp vcm
// bit[4]: 0 no otp lenc, 1 valid otp lenc

int apply_otp(void)
{
//RG_Ratio_Typical and BG_Ratio_Typical get form O-Film  hejun
	int RG_Ratio_Typical = 0x119/*0x12f*/, BG_Ratio_Typical = 0x144/*0x171*/; //quan.chang modify according code supplied from SHENGTAI
	int rg, bg, R_gain, G_gain, B_gain, Base_gain, temp, i;
/*
#ifdef CONFIG_RLK_CAM_OTP_ON_OR_OFF
	if(cqcq_disable_otp==1)return otpdate.flag;//quan.chang debug
#endif
*/
	// apply OTP WB Calibration
	if (otpdate.flag & 0x40) {
		rg = otpdate.rg_ratio;
		bg = otpdate.bg_ratio;
		LOG_INF("ww_rg = %x,bg = %x\n",rg,bg);
		//calculate G gain
		R_gain = (RG_Ratio_Typical*1000) / rg;
		B_gain = (BG_Ratio_Typical*1000) / bg;
		LOG_INF("ww_R_gain = %x|%d,B_gain = %x|%d\n",R_gain,R_gain,B_gain,B_gain);
		G_gain = 1000;
		if (R_gain < 1000 || B_gain < 1000)
		{
			if (R_gain < B_gain)
				Base_gain = R_gain;
			else
				Base_gain = B_gain;
		}
		else
		{
			Base_gain = G_gain;
		}
		R_gain = 0x400 * R_gain / (Base_gain);
		B_gain = 0x400 * B_gain / (Base_gain);
		G_gain = 0x400 * G_gain / (Base_gain);

		LOG_INF("R_gain = %d, B_gain = %d, G_gain =%d, \n", R_gain, B_gain, G_gain);
		// update sensor WB gain
		if (R_gain>0x400) {
			write_cmos_sensor(0x5019, R_gain>>8);
			write_cmos_sensor(0x501a, R_gain & 0x00ff);
		}
		if (G_gain>0x400) {
			write_cmos_sensor(0x501b, G_gain>>8);
			write_cmos_sensor(0x501c, G_gain & 0x00ff);
		}
		if (B_gain>0x400) {
			write_cmos_sensor(0x501d, B_gain>>8);
			write_cmos_sensor(0x501e, B_gain & 0x00ff);
		}
	}
	// apply OTP Lenc Calibration
	if (otpdate.flag & 0x10) {
		temp = read_cmos_sensor(0x5000);
		temp = 0x20 | temp;
		write_cmos_sensor(0x5000, temp);
		for(i=0;i<240;i++) {
			write_cmos_sensor(0x5900 + i, otpdate.lenc[i]);
		}
	}
	return otpdate.flag;
}

#endif
