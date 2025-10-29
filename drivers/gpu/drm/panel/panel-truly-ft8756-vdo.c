/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
*/

#include <drm/drmP.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <linux/backlight.h>
#include <linux/delay.h>

#include <linux/gpio/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>
#include <linux/of_graph.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mtk_drm_graphics_base.h"
#include "../mediatek/mtk_log.h"
#include "../mediatek/mtk_panel_ext.h"
#endif

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "../mediatek/mtk_corner_pattern/mtk_data_hw_roundedpattern.h"

#endif
#include "../i2c/lcm_i2c.h"

struct jdi {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *bias_pos;
	struct gpio_desc *bias_neg;
	bool prepared;
	bool enabled;

	int error;
};

#define jdi_dcs_write_seq(ctx, seq...)                                         \
	({                                                                     \
		const u8 d[] = { seq };                                        \
		BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 64,                           \
				 "DCS sequence too big for stack");            \
		jdi_dcs_write(ctx, d, ARRAY_SIZE(d));                          \
	})

#define jdi_dcs_write_seq_static(ctx, seq...)                                  \
	({                                                                     \
		static const u8 d[] = { seq };                                 \
		jdi_dcs_write(ctx, d, ARRAY_SIZE(d));                          \
	})

static inline struct jdi *panel_to_jdi(struct drm_panel *panel)
{
	return container_of(panel, struct jdi, panel);
}

#ifdef PANEL_SUPPORT_READBACK
static int jdi_dcs_read(struct jdi *ctx, u8 cmd, void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;

	if (ctx->error < 0)
		return 0;

	ret = mipi_dsi_dcs_read(dsi, cmd, data, len);
	if (ret < 0) {
		dev_info(ctx->dev, "error %d reading dcs seq:(%#x)\n", ret,
			 cmd);
		ctx->error = ret;
	}

	return ret;
}

static void jdi_panel_get_data(struct jdi *ctx)
{
	u8 buffer[3] = { 0 };
	static int ret;

	pr_debug("%s+\n", __func__);

	if (ret == 0) {
		ret = jdi_dcs_read(ctx, 0x0A, buffer, 1);
		pr_debug("%s  0x%08x\n", __func__, buffer[0] | (buffer[1] << 8));
		dev_info(ctx->dev, "return %d data(0x%08x) to dsi engine\n",
			 ret, buffer[0] | (buffer[1] << 8));
	}
}
#endif

static void jdi_dcs_write(struct jdi *ctx, const void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;
	char *addr;

	if (ctx->error < 0)
		return;

	addr = (char *)data;
	if ((int)*addr < 0xB0)
		ret = mipi_dsi_dcs_write_buffer(dsi, data, len);
	else
		ret = mipi_dsi_generic_write(dsi, data, len);
	if (ret < 0) {
		dev_info(ctx->dev, "error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}

static void jdi_panel_init(struct jdi *ctx)
{
	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return;
	}
	gpiod_set_value(ctx->reset_gpio, 0);
	udelay(15 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
	udelay(1 * 1000);
	gpiod_set_value(ctx->reset_gpio, 0);
	udelay(10 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
	udelay(10 * 1000);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	//----------------------LCD initial code Start----------------------//
	//--------------CMD WR enable--------------//
	jdi_dcs_write_seq_static(ctx, 0x00,0x00);
	jdi_dcs_write_seq_static(ctx, 0xFF,0x80,0x57,0x01); 

	jdi_dcs_write_seq_static(ctx, 0x00,0x80);
	jdi_dcs_write_seq_static(ctx, 0xFF,0x80,0x57);

	//--------------Panel Resolution --------------//
	jdi_dcs_write_seq_static(ctx, 0x00,0xA3);  //Y=1600
	jdi_dcs_write_seq_static(ctx, 0xB3,0x06,0x40,0x00,0x18);

	//--------------Voltage set--------------//
	jdi_dcs_write_seq_static(ctx, 0x00,0x93);//VGH_N 16V
	jdi_dcs_write_seq_static(ctx, 0xC5,0x61);

	jdi_dcs_write_seq_static(ctx, 0x00,0x97);//VGH_I 16V
	jdi_dcs_write_seq_static(ctx, 0xC5,0x61);

	jdi_dcs_write_seq_static(ctx, 0x00,0x9A);//VGL_N -12V 
	jdi_dcs_write_seq_static(ctx, 0xC5,0xC1);

	jdi_dcs_write_seq_static(ctx, 0x00,0x9C);//VGL_I -12V 
	jdi_dcs_write_seq_static(ctx, 0xC5,0xC1);

	jdi_dcs_write_seq_static(ctx, 0x00,0xB6); //VGHO1_N_I=15v   
	jdi_dcs_write_seq_static(ctx, 0xC5,0x4D,0x4D);

	jdi_dcs_write_seq_static(ctx, 0x00,0xB8); //VGLO1_N_I -11V
	jdi_dcs_write_seq_static(ctx, 0xC5,0x37,0x37); 

	jdi_dcs_write_seq_static(ctx, 0x00,0x00); 
	jdi_dcs_write_seq_static(ctx, 0xD8,0x37,0x37);  //GVDDP/N  5.6V

	jdi_dcs_write_seq_static(ctx, 0x00,0x00);
	jdi_dcs_write_seq_static(ctx, 0xD9,0x00,0xAF,0xAF,0xAF);

	jdi_dcs_write_seq_static(ctx, 0x00,0x07);
	jdi_dcs_write_seq_static(ctx, 0xD9,0x00,0xAF,0xAF,0xAF);

	jdi_dcs_write_seq_static(ctx, 0x00,0x82); 
	jdi_dcs_write_seq_static(ctx, 0xC5,0x55);  //LVD

	jdi_dcs_write_seq_static(ctx, 0x00,0x83); 
	jdi_dcs_write_seq_static(ctx, 0xC5,0x07);  //LVD Enable

	//Vgh_s_sel>>vgh ¥[±j
	jdi_dcs_write_seq_static(ctx, 0x00,0x96);
	jdi_dcs_write_seq_static(ctx, 0xf5,0x19);
	 
	//Vgl_s_sel>>vgl ¥[±j
	jdi_dcs_write_seq_static(ctx, 0x00,0x86);
	jdi_dcs_write_seq_static(ctx, 0xf5,0x19);
	 
	// VGH CLK Line Rate(1 Line)
	jdi_dcs_write_seq_static(ctx, 0x00,0x94);
	jdi_dcs_write_seq_static(ctx, 0xC5,0x15);
	 
	// VGL CLK Line Rate(1 Line)
	jdi_dcs_write_seq_static(ctx, 0x00,0x9B);
	jdi_dcs_write_seq_static(ctx, 0xC5,0x51);

	jdi_dcs_write_seq_static(ctx, 0x00,0xA3);  //GVDD_EN
	jdi_dcs_write_seq_static(ctx, 0xA5,0x04); 

	jdi_dcs_write_seq_static(ctx, 0x00,0x99);  
	jdi_dcs_write_seq_static(ctx, 0xCF,0x56); 

	jdi_dcs_write_seq_static(ctx, 0x00,0x86);  
	jdi_dcs_write_seq_static(ctx, 0xB7,0x80); //I2C EN

	jdi_dcs_write_seq_static(ctx, 0x00,0xA5);  
	jdi_dcs_write_seq_static(ctx, 0xB0,0x2D); //

	jdi_dcs_write_seq_static(ctx, 0x00,0x90); //MIPI 4lane 
	jdi_dcs_write_seq_static(ctx, 0xFF,0x00);

	//--------------Gamma setting--------------//
	jdi_dcs_write_seq_static(ctx, 0x00,0x00);
	jdi_dcs_write_seq_static(ctx, 0xE1,0x0C,0x17,0x19,0x28,0x31,0x3B,0x4C,0x5A,0x5C,0x69,0x6D,0x83,0x7E,0x69,0x69,0x5E);
	jdi_dcs_write_seq_static(ctx, 0x00,0x10);
	jdi_dcs_write_seq_static(ctx, 0xE1,0x56,0x4A,0x3B,0x31,0x29,0x1A,0x18,0x12);

	jdi_dcs_write_seq_static(ctx, 0x00,0x00);
	jdi_dcs_write_seq_static(ctx, 0xE2,0x0C,0x17,0x19,0x28,0x31,0x3B,0x4C,0x5A,0x5C,0x69,0x6D,0x83,0x7E,0x69,0x69,0x5E);
	jdi_dcs_write_seq_static(ctx, 0x00,0x10);
	jdi_dcs_write_seq_static(ctx, 0xE2,0x56,0x4A,0x3B,0x31,0x29,0x1A,0x18,0x12);

	//--------------TCON setting--------------//
	//longH term off
	jdi_dcs_write_seq_static(ctx, 0x00, 0x80);
	jdi_dcs_write_seq_static(ctx, 0xCE, 0x00);
	// VB term EN
	jdi_dcs_write_seq_static(ctx, 0x00, 0xD0);
	jdi_dcs_write_seq_static(ctx, 0xCE, 0x01);
	//RTN  Compensation off
	jdi_dcs_write_seq_static(ctx, 0x00, 0xE0);
	jdi_dcs_write_seq_static(ctx, 0xCE, 0x00);
	//follow MIPI video VS/HS
	jdi_dcs_write_seq_static(ctx, 0x00, 0xA1);
	jdi_dcs_write_seq_static(ctx, 0xC1, 0xCC);
	//fifo mode Vsync shift
	jdi_dcs_write_seq_static(ctx, 0x00, 0xA5);
	jdi_dcs_write_seq_static(ctx, 0xC1, 0x00 ,0x10);

	//VFP=180
	jdi_dcs_write_seq_static(ctx, 0x00, 0x70);
	jdi_dcs_write_seq_static(ctx, 0xC0, 0x00 ,0xA0 ,0x01 ,0x09 ,0x00 ,0x24);

	//normal mode VB term2£º 1.4ms
	jdi_dcs_write_seq_static(ctx, 0x00, 0xD1);
	jdi_dcs_write_seq_static(ctx, 0xCE, 0x00 ,0x14 ,0x01 ,0x01 ,0x00 ,0xF0 ,0x01);
	//LPF/FIFO mode VB term2£º 1.4ms
	jdi_dcs_write_seq_static(ctx, 0x00, 0xE8);
	jdi_dcs_write_seq_static(ctx, 0xCE, 0x00 ,0xF0 ,0x00 ,0xF0);

	jdi_dcs_write_seq_static(ctx, 0x00, 0xB0);
	jdi_dcs_write_seq_static(ctx, 0xC0, 0x00 ,0xA0 ,0x01 ,0x0A ,0x24);

	jdi_dcs_write_seq_static(ctx, 0x00, 0x80);
	jdi_dcs_write_seq_static(ctx, 0xC1, 0x00 ,0x00 );

	jdi_dcs_write_seq_static(ctx, 0x00, 0x90);
	jdi_dcs_write_seq_static(ctx, 0xC1, 0x01);

	jdi_dcs_write_seq_static(ctx, 0x00, 0xF1);
	jdi_dcs_write_seq_static(ctx, 0xCF, 0x5A);

	jdi_dcs_write_seq_static(ctx, 0x00, 0xF5);
	jdi_dcs_write_seq_static(ctx, 0xCF, 0x01);

	jdi_dcs_write_seq_static(ctx, 0x00, 0xF6);
	jdi_dcs_write_seq_static(ctx, 0xCF, 0x5A);

	//Mode switch by CMD2
	jdi_dcs_write_seq_static(ctx, 0x00, 0xF7);
	jdi_dcs_write_seq_static(ctx, 0xCF, 0x11);
	// fifo mode Vstart&Vmid int
	jdi_dcs_write_seq_static(ctx, 0x00, 0x70);
	jdi_dcs_write_seq_static(ctx, 0xCF, 0x06 ,0x06 ,0x68 ,0x6C ,0x02 ,0x02 ,0x3E ,0x42);
	//fifo mode Vend int
	jdi_dcs_write_seq_static(ctx, 0x00, 0xC0);
	jdi_dcs_write_seq_static(ctx, 0xCF, 0x04 ,0x04 ,0x16 ,0x1A);
	// fifo mode Vblank int
	jdi_dcs_write_seq_static(ctx, 0x00, 0xC5);
	jdi_dcs_write_seq_static(ctx, 0xCF, 0x06 ,0x06 ,0x4A ,0x4E);


	////////////////////////////////////////////////////////																
	//panel mapping setting
	//u2d_L
	jdi_dcs_write_seq_static(ctx, 0x00,0x80);
	jdi_dcs_write_seq_static(ctx, 0xCC,0x01,0x00,0x00,0x00,0x01,0x07,0x09,0x0B,0x0D,0x0F,0x11,0x03,0x05,0x23,0x00,0x00);

	jdi_dcs_write_seq_static(ctx, 0x00,0x90);
	jdi_dcs_write_seq_static(ctx, 0xCC,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);

	//d2u_L
	jdi_dcs_write_seq_static(ctx, 0x00,0xA0);
	jdi_dcs_write_seq_static(ctx, 0xCC,0x00,0x25,0x01,0x26,0x05,0x03,0x0D,0x0B,0x09,0x07,0x26,0x1C,0x00,0x00,0x00,0x00);

	jdi_dcs_write_seq_static(ctx, 0x00,0xB0);
	jdi_dcs_write_seq_static(ctx, 0xCC,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);

	//u2d_R
	jdi_dcs_write_seq_static(ctx, 0x00,0x80);
	jdi_dcs_write_seq_static(ctx, 0xCD,0x01,0x00,0x00,0x00,0x01,0x06,0x08,0x0A,0x0C,0x0E,0x10,0x02,0x04,0x22,0x00,0x00);

	jdi_dcs_write_seq_static(ctx, 0x00,0x90);
	jdi_dcs_write_seq_static(ctx, 0xCD,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);

	//d2u_R
	jdi_dcs_write_seq_static(ctx, 0x00,0xA0);
	jdi_dcs_write_seq_static(ctx, 0xCD,0x00,0x25,0x01,0x26,0x04,0x02,0x0C,0x0A,0x08,0x06,0x26,0x1C,0x00,0x00,0x00,0x00);

	jdi_dcs_write_seq_static(ctx, 0x00,0xB0);
	jdi_dcs_write_seq_static(ctx, 0xCD,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);


	//power off enmode setting											
	jdi_dcs_write_seq_static(ctx, 0x00,0x80);
	jdi_dcs_write_seq_static(ctx, 0xCB,0xC1,0xC1,0xC1,0xC1,0xC1,0xC1,0xC1,0xC1,0xC1,0xC1,0xC1,0xC1,0xC1,0xC1,0xC1,0xC1);
	//power on enmode setting											
	jdi_dcs_write_seq_static(ctx, 0x00,0x90);
	jdi_dcs_write_seq_static(ctx, 0xCB,0x54,0x54,0x54,0x54,0x54,0x54,0x54,0x54,0x54,0x54,0x54,0x54,0x54,0x54,0x54,0x54);
	jdi_dcs_write_seq_static(ctx, 0x00,0xEE);											
	jdi_dcs_write_seq_static(ctx, 0xCB,0x00);											
											
	jdi_dcs_write_seq_static(ctx, 0x00,0x90);											
	jdi_dcs_write_seq_static(ctx, 0xC3,0x00);											
											
	//skip & powr on1 enmode setting											
	jdi_dcs_write_seq_static(ctx, 0x00,0xA0);
	jdi_dcs_write_seq_static(ctx, 0xCB,0x00,0x00,0x00,0x00,0xC0,0x00,0x00,0x00);											
											
	//power off blank enmode setting											
	jdi_dcs_write_seq_static(ctx, 0x00,0xB0);											
	jdi_dcs_write_seq_static(ctx, 0xCB,0x55,0x55,0x55,0x55);											
											
	//power on blank enmode setting											
	jdi_dcs_write_seq_static(ctx, 0x00,0xC0);											
	jdi_dcs_write_seq_static(ctx, 0xCB,0x55,0x55,0x55,0x55);											
											
	//power on blank enmode setting											
	jdi_dcs_write_seq_static(ctx, 0x00,0xD2);
	jdi_dcs_write_seq_static(ctx, 0xCB,0x81,0x00,0x81,0x81,0x00,0x81,0x81,0x00,0x81,0x81,0x00);										
											
	jdi_dcs_write_seq_static(ctx, 0x00,0xE0);
	jdi_dcs_write_seq_static(ctx, 0xCB,0x81,0x81,0x00,0x81,0x81,0x00,0x81,0x81,0x00,0x81,0x81,0x00,0x81);
																				
	jdi_dcs_write_seq_static(ctx, 0x00,0xFA);											
	jdi_dcs_write_seq_static(ctx, 0xCB,0x83,0x00);											
											
	jdi_dcs_write_seq_static(ctx, 0x00,0xEF);											
	jdi_dcs_write_seq_static(ctx, 0xCB,0x05);											
											

	//STV1 
	jdi_dcs_write_seq_static(ctx, 0x00,0x68);
	jdi_dcs_write_seq_static(ctx, 0xC2,0x8E,0x05,0x00,0x00);
	//STV2
	jdi_dcs_write_seq_static(ctx, 0x00,0x6C);
	jdi_dcs_write_seq_static(ctx, 0xC2,0x8D,0x05,0x00,0x00);
	//STV3
	jdi_dcs_write_seq_static(ctx, 0x00,0x70);
	jdi_dcs_write_seq_static(ctx, 0xC2,0x8C,0x05,0x00,0x00);
	//STV4
	jdi_dcs_write_seq_static(ctx, 0x00,0x74);
	jdi_dcs_write_seq_static(ctx, 0xC2,0x8B,0x05,0x00,0x00);

	jdi_dcs_write_seq_static(ctx, 0x00,0xE8);
	jdi_dcs_write_seq_static(ctx, 0xC3,0x00,0x00,0x00,0x00);

	jdi_dcs_write_seq_static(ctx, 0x00,0xEC);
	jdi_dcs_write_seq_static(ctx, 0xC3,0x00,0x00,0x00,0x00);

	jdi_dcs_write_seq_static(ctx, 0x00,0xF9); //GOFF3 add VST4
	jdi_dcs_write_seq_static(ctx, 0xCB,0x20);

	jdi_dcs_write_seq_static(ctx, 0x00,0xFE); //GOFF4 add VST4
	jdi_dcs_write_seq_static(ctx, 0xCB,0x08);

	jdi_dcs_write_seq_static(ctx, 0x00,0xF8); //GOFF3/GOFF4 add VST4 shift & width
	jdi_dcs_write_seq_static(ctx, 0xCD,0x8A,0x55,0x89);

	//RST
	jdi_dcs_write_seq_static(ctx, 0x00,0xEA);
	jdi_dcs_write_seq_static(ctx, 0xC2,0x12,0x00,0x10,0x07,0x00,0x00);


	//CKV1
	jdi_dcs_write_seq_static(ctx, 0x00,0x8C);
	jdi_dcs_write_seq_static(ctx, 0xC2,0x89,0x04,0x04,0x00,0x00);

	//CKV2
	jdi_dcs_write_seq_static(ctx, 0x00,0x91);
	jdi_dcs_write_seq_static(ctx, 0xC2,0x88,0x05,0x04,0x00,0x00);

	//CKV3
	jdi_dcs_write_seq_static(ctx, 0x00,0x96);
	jdi_dcs_write_seq_static(ctx, 0xC2,0x87,0x06,0x04,0x00,0x00);

	//CKV4
	jdi_dcs_write_seq_static(ctx, 0x00,0x9B);
	jdi_dcs_write_seq_static(ctx, 0xC2,0x86,0x07,0x04,0x00,0x00);

	//CKV5
	jdi_dcs_write_seq_static(ctx, 0x00,0xA0);
	jdi_dcs_write_seq_static(ctx, 0xC2,0x85,0x00,0x04,0x00,0x00);

	//CKV6
	jdi_dcs_write_seq_static(ctx, 0x00,0xA5);
	jdi_dcs_write_seq_static(ctx, 0xC2,0x84,0x00,0x04,0x00,0x00);

	//CKV7
	jdi_dcs_write_seq_static(ctx, 0x00,0xAA);
	jdi_dcs_write_seq_static(ctx, 0xC2,0x83,0x00,0x04,0x00,0x00);

	//CKV8
	jdi_dcs_write_seq_static(ctx, 0x00,0xAF);
	jdi_dcs_write_seq_static(ctx, 0xC2,0x82,0x00,0x04,0x00,0x00);

	//CKV9
	jdi_dcs_write_seq_static(ctx, 0x00,0xB4);
	jdi_dcs_write_seq_static(ctx, 0xC2,0x81,0x00,0x04,0x00,0x00);

	//CKV10
	jdi_dcs_write_seq_static(ctx, 0x00,0xB9);
	jdi_dcs_write_seq_static(ctx, 0xC2,0x80,0x01,0x04,0x00,0x00);

	//CKV11
	jdi_dcs_write_seq_static(ctx, 0x00,0xBE);
	jdi_dcs_write_seq_static(ctx, 0xC2,0x01,0x02,0x04,0x00,0x00);

	//CKV12
	jdi_dcs_write_seq_static(ctx, 0x00,0xC3);
	jdi_dcs_write_seq_static(ctx, 0xC2,0x02,0x03,0x04,0x00,0x00);

	//CKV width setting
	jdi_dcs_write_seq_static(ctx, 0x00,0xdc);
	jdi_dcs_write_seq_static(ctx, 0xC2,0xbb,0xbb,0xbb,0xbb,0xbb,0xbb,0x00,0x00);

	//--------------Source setting--------------//
	jdi_dcs_write_seq_static(ctx, 0x00,0x98);
	jdi_dcs_write_seq_static(ctx, 0xC4,0x08);

	jdi_dcs_write_seq_static(ctx, 0x00,0x90);
	jdi_dcs_write_seq_static(ctx, 0xE9,0x10,0xFF,0xFF,0xFF,0x00);

	jdi_dcs_write_seq_static(ctx, 0x00,0x85);//Posdmy pch
	jdi_dcs_write_seq_static(ctx, 0xC4,0x80);

	jdi_dcs_write_seq_static(ctx, 0x00,0x81);
	jdi_dcs_write_seq_static(ctx, 0xA4,0x73);
	jdi_dcs_write_seq_static(ctx, 0x00,0x86);//4VB pch
	jdi_dcs_write_seq_static(ctx, 0xA4,0xB6);

	jdi_dcs_write_seq_static(ctx, 0x00,0x95);//VB pch data
	jdi_dcs_write_seq_static(ctx, 0xC4,0x80);

	jdi_dcs_write_seq_static(ctx, 0x00,0x93); 
	jdi_dcs_write_seq_static(ctx, 0xC4,0x8F);

	jdi_dcs_write_seq_static(ctx, 0x00,0x80); 
	jdi_dcs_write_seq_static(ctx, 0xB3,0x17);

	jdi_dcs_write_seq_static(ctx, 0x00,0xA5);  
	jdi_dcs_write_seq_static(ctx, 0xB0,0x2D); //RC delay 95ns

	//--------------Power on&off--------------//
	jdi_dcs_write_seq_static(ctx, 0x00,0xCA);//Power on 3 
	jdi_dcs_write_seq_static(ctx, 0xC0,0x90,0x11);

	jdi_dcs_write_seq_static(ctx, 0x00,0xB7);//sd_en_sdpl_sel
	jdi_dcs_write_seq_static(ctx, 0xF5,0x1D);

	jdi_dcs_write_seq_static(ctx, 0x00,0x90);
	jdi_dcs_write_seq_static(ctx, 0xC3,0x00);

	jdi_dcs_write_seq_static(ctx, 0x00,0xB1);//VCOM
	jdi_dcs_write_seq_static(ctx, 0xF5,0x11);

	//--------------sleep in allgateon--------------//
	jdi_dcs_write_seq_static(ctx, 0x00,0xB0); 
	jdi_dcs_write_seq_static(ctx, 0xC5,0x00);

	jdi_dcs_write_seq_static(ctx, 0x00,0xB3);
	jdi_dcs_write_seq_static(ctx, 0xC5,0x00);

	jdi_dcs_write_seq_static(ctx, 0x00,0xB2);
	jdi_dcs_write_seq_static(ctx, 0xC5,0x0D);

	jdi_dcs_write_seq_static(ctx, 0x00,0xB5);
	jdi_dcs_write_seq_static(ctx, 0xC5,0x02);

	jdi_dcs_write_seq_static(ctx, 0x00,0xC2);
	jdi_dcs_write_seq_static(ctx, 0xF5,0x42);

	jdi_dcs_write_seq_static(ctx, 0x00,0x96);
	jdi_dcs_write_seq_static(ctx, 0xF5,0x00);

	jdi_dcs_write_seq_static(ctx, 0x00,0x80); 
	jdi_dcs_write_seq_static(ctx, 0xA4,0x4E); //SD bias adjust.

	jdi_dcs_write_seq_static(ctx, 0x00,0x93);
	jdi_dcs_write_seq_static(ctx, 0xE9,0x00,0x00); //SR_SET_CAP_Value Setting

	//CKV Rising¿ªEQ
	jdi_dcs_write_seq_static(ctx, 0x00,0x86);
	jdi_dcs_write_seq_static(ctx, 0xC3,0x0F,0x00,0x00);

	jdi_dcs_write_seq_static(ctx, 0x00,0x94);
	jdi_dcs_write_seq_static(ctx, 0xC3,0x0F,0x00,0x00);
	 	
	//--------------CMD WR Disable--------------//
	//jdi_dcs_write_seq_static(ctx, 0x00,0x00);
	//jdi_dcs_write_seq_static(ctx, 0xFF,0x00,0x00,0x00); 

	//jdi_dcs_write_seq_static(ctx, 0x00,0x80);
	//jdi_dcs_write_seq_static(ctx, 0xFF,0x00,0x00);
	//----------------------LCD initial code End----------------------//
	//SLPOUT and DISPON
	jdi_dcs_write_seq_static(ctx, 0x11);
	msleep(120);
	jdi_dcs_write_seq_static(ctx, 0x29);
	jdi_dcs_write_seq_static(ctx, 0x35,0x00);
	msleep(300);	

	pr_debug("%s-\n", __func__);
}

static int jdi_disable(struct drm_panel *panel)
{
	struct jdi *ctx = panel_to_jdi(panel);

	if (!ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = false;

	return 0;
}

static int jdi_unprepare(struct drm_panel *panel)
{

	struct jdi *ctx = panel_to_jdi(panel);

	pr_debug("%s\n", __func__);

	if (!ctx->prepared)
		return 0;

	if (ctx->backlight)
		backlight_device_set_brightness(ctx->backlight, 0);

	jdi_dcs_write_seq_static(ctx, 0x28);
	msleep(25);
	jdi_dcs_write_seq_static(ctx, 0x10);
	msleep(75);

	ctx->error = 0;
	ctx->prepared = false;
	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);


	ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
		"bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(ctx->dev, "%s: cannot get bias-neg 1 %ld\n",
			__func__, PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
	gpiod_set_value(ctx->bias_neg, 0);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);

	udelay(500);

	ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
		"bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_err(ctx->dev, "%s: cannot get bias-pos 0 %ld\n",
			__func__, PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	gpiod_set_value(ctx->bias_pos, 0);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);


	return 0;
}

static int jdi_prepare(struct drm_panel *panel)
{
	struct jdi *ctx = panel_to_jdi(panel);
	int ret;
	LCM_DATA_T2 i2c_data;
	pr_debug("%s+\n", __func__);
	if (ctx->prepared)
		return 0;
#if 0
	// lcd reset H -> L -> L
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset-gpios %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(10000, 10001);
	gpiod_set_value(ctx->reset_gpio, 0);
	msleep(20);
	gpiod_set_value(ctx->reset_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
#endif
	// end
	ctx->bias_pos =
	    devm_gpiod_get_index(ctx->dev, "bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_err(ctx->dev, "%s: cannot get bias-pos 0 %ld\n",
			__func__, PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	gpiod_set_value(ctx->bias_pos, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);

	usleep_range(2000, 2001);
	ctx->bias_neg =
	    devm_gpiod_get_index(ctx->dev, "bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(ctx->dev, "%s: cannot get bias-neg 1 %ld\n",
			__func__, PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
	gpiod_set_value(ctx->bias_neg, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);
#if 1
	i2c_data.cmd = 0x00;  //+5V
	i2c_data.data = 0x10; //5.8
	lcm_i2c_set_data(1, &i2c_data);
	i2c_data.cmd = 0x01;  //-5V
	i2c_data.data = 0x10; //-5.8
	lcm_i2c_set_data(1, &i2c_data);
	mdelay(15);
#endif
	jdi_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0)
		jdi_unprepare(panel);

	ctx->prepared = true;

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_rst(panel);
#endif
#ifdef PANEL_SUPPORT_READBACK
	jdi_panel_get_data(ctx);
#endif

	pr_debug("%s-\n", __func__);
	return ret;
}

static int jdi_enable(struct drm_panel *panel)
{
	struct jdi *ctx = panel_to_jdi(panel);
	printk("%s\n", __func__);
	if (ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = true;

	return 0;
}

#define HFP (16)
#define HSA (4)
#define HBP (14)
#define VFP (266)
#define VSA (6)
#define VBP (30)
#define VAC (1600)
#define HAC (720)
static u32 fake_heigh = VAC;
static u32 fake_width = HAC;
static bool need_fake_resolution;

static struct drm_display_mode default_mode = {
	.clock = 129034,
	.hdisplay = HAC,
	.hsync_start = HAC + HFP,
	.hsync_end = HAC + HFP + HSA,
	.htotal = HAC + HFP + HSA + HBP,
	.vdisplay = VAC,
	.vsync_start = VAC + VFP,
	.vsync_end = VAC + VFP + VSA,
	.vtotal = VAC + VFP + VSA + VBP,
	.vrefresh = 90,
};

#if defined(CONFIG_MTK_PANEL_EXT)

#if 1
static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct jdi *ctx = panel_to_jdi(panel);

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, on);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	return 0;
}
#endif

static int panel_ata_check(struct drm_panel *panel)
{
	struct jdi *ctx = panel_to_jdi(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	unsigned char data[3] = {0x00, 0x00, 0x00};
	unsigned char id[3] = {0x00, 0x00, 0x00};
	ssize_t ret;

	ret = mipi_dsi_dcs_read(dsi, 0x4, data, 3);
	if (ret < 0) {
		pr_err("%s error\n", __func__);
		return 0;
	}

	DDPINFO("ATA read data %x %x %x\n", data[0], data[1], data[2]);

	if (data[0] == id[0] &&
			data[1] == id[1] &&
			data[2] == id[2])
		return 1;

	DDPINFO("ATA expect read data is %x %x %x\n",
			id[0], id[1], id[2]);

	return 0;
}

static int jdi_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle,
				 unsigned int level)
{
	char bl_tb0[] = {0x51, 0xFF};

	bl_tb0[1] = level;

	if (!cb)
		return -1;

	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

	return 0;
}

static int jdi_get_virtual_heigh(void)
{
	return VAC;
}

static int jdi_get_virtual_width(void)
{
	return HAC;
}

static struct mtk_panel_params ext_params = {
	.pll_clk = 425,
	.vfp_low_power = VFP,
	.cust_esd_check = 0,
	.esd_check_enable = 0,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
	},
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.dsc_params = {
		.enable = 0,
		.ver = 17,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 34,
		.rct_on = 1,
		.bit_per_channel = 8,
		.dsc_line_buf_depth = 9,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 1600,
		.pic_width = 720,
		.slice_height = 8,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 170,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 43,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 3511,
		.slice_bpg_offset = 3255,
		.initial_offset = 6144,
		.final_offset = 7072,
		.flatness_minqp = 3,
		.flatness_maxqp = 12,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 11,
		.rc_quant_incr_limit1 = 11,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
		},
	.data_rate = 850,
	.dyn_fps = {
		.switch_en = 1, .vact_timing_fps = 90,
	},
};

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = jdi_setbacklight_cmdq,
	.ata_check = panel_ata_check,
	.get_virtual_heigh = jdi_get_virtual_heigh,
	.get_virtual_width = jdi_get_virtual_width,
};
#endif

struct panel_desc {
	const struct drm_display_mode *modes;
	unsigned int num_modes;

	unsigned int bpc;

	struct {
		unsigned int width;
		unsigned int height;
	} size;

	/**
	 * @prepare: the time (in milliseconds) that it takes for the panel to
	 *	   become ready and start receiving video data
	 * @enable: the time (in milliseconds) that it takes for the panel to
	 *	  display the first valid frame after starting to receive
	 *	  video data
	 * @disable: the time (in milliseconds) that it takes for the panel to
	 *	   turn the display off (no content is visible)
	 * @unprepare: the time (in milliseconds) that it takes for the panel
	 *		 to power itself down completely
	 */
	struct {
		unsigned int prepare;
		unsigned int enable;
		unsigned int disable;
		unsigned int unprepare;
	} delay;
};

static void change_drm_disp_mode_params(struct drm_display_mode *mode)
{
	if (fake_heigh > 0 && fake_heigh < VAC) {
		mode->vdisplay = fake_heigh;
		mode->vsync_start = fake_heigh + VFP;
		mode->vsync_end = fake_heigh + VFP + VSA;
		mode->vtotal = fake_heigh + VFP + VSA + VBP;
	}
	if (fake_width > 0 && fake_width < HAC) {
		mode->hdisplay = fake_width;
		mode->hsync_start = fake_width + HFP;
		mode->hsync_end = fake_width + HFP + HSA;
		mode->htotal = fake_width + HFP + HSA + HBP;
	}
}

static int jdi_get_modes(struct drm_panel *panel)
{
	struct drm_display_mode *mode;

	if (need_fake_resolution)
		change_drm_disp_mode_params(&default_mode);
	mode = drm_mode_duplicate(panel->drm, &default_mode);
	if (!mode) {
		dev_err(panel->drm->dev, "failed to add mode %ux%ux@%u\n",
			default_mode.hdisplay, default_mode.vdisplay,
			default_mode.vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(panel->connector, mode);

	panel->connector->display_info.width_mm = 64;
	panel->connector->display_info.height_mm = 129;

	return 1;
}

static const struct drm_panel_funcs jdi_drm_funcs = {
	.disable = jdi_disable,
	.unprepare = jdi_unprepare,
	.prepare = jdi_prepare,
	.enable = jdi_enable,
	.get_modes = jdi_get_modes,
};

static void check_is_need_fake_resolution(struct device *dev)
{
	unsigned int ret = 0;

	ret = of_property_read_u32(dev->of_node, "fake_heigh", &fake_heigh);
	if (ret)
		need_fake_resolution = false;
	ret = of_property_read_u32(dev->of_node, "fake_width", &fake_width);
	if (ret)
		need_fake_resolution = false;
	if (fake_heigh > 0 && fake_heigh < VAC)
		need_fake_resolution = true;
	if (fake_width > 0 && fake_width < HAC)
		need_fake_resolution = true;
}

static int jdi_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct jdi *ctx;
	struct device_node *backlight;
	int ret;
	struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;
	printk("%s\n", __func__);
	dsi_node = of_get_parent(dev->of_node);
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);
		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			if (!remote_node) {
				pr_debug("No panel connected,skip probe lcm\n");
				return -ENODEV;
			}
			pr_debug("device node name:%s\n", remote_node->name);
		}
	}
	if (remote_node != dev->of_node) {
		pr_debug("%s+ skip probe due to not current lcm\n", __func__);
		return -ENODEV;
	}

	pr_debug("%s+\n", __func__);
	ctx = devm_kzalloc(dev, sizeof(struct jdi), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			  MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_EOT_PACKET |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS;

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(dev, "%s: cannot get reset-gpios %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	devm_gpiod_put(dev, ctx->reset_gpio);
	usleep_range(2000, 2001);
	printk("%s 22\n", __func__);
	ctx->bias_pos = devm_gpiod_get_index(dev, "bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_err(dev, "%s: cannot get bias-pos 0 %ld\n",
			__func__, PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	devm_gpiod_put(dev, ctx->bias_pos);

	ctx->bias_neg = devm_gpiod_get_index(dev, "bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(dev, "%s: cannot get bias-neg 1 %ld\n",
			__func__, PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
	devm_gpiod_put(dev, ctx->bias_neg);

	ctx->prepared = true;
	ctx->enabled = true;
	printk("%s 33\n", __func__);
	drm_panel_init(&ctx->panel);
	ctx->panel.dev = dev;
	ctx->panel.funcs = &jdi_drm_funcs;

	ret = drm_panel_add(&ctx->panel);
	if (ret < 0)
		return ret;


	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ext_params, &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;
#endif
	check_is_need_fake_resolution(dev);
	pr_debug("%s-\n", __func__);

	return ret;
}

static int jdi_remove(struct mipi_dsi_device *dsi)
{
	struct jdi *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);

	return 0;
}

static const struct of_device_id jdi_of_match[] = {
	{
	    .compatible = "truly,ft8756,vdo",
	},
	{ }
};

MODULE_DEVICE_TABLE(of, jdi_of_match);

static struct mipi_dsi_driver jdi_driver = {
	.probe = jdi_probe,
	.remove = jdi_remove,
	.driver = {
		.name = "panel-truly-ft8756-vdo",
		.owner = THIS_MODULE,
		.of_match_table = jdi_of_match,
	},
};


module_mipi_dsi_driver(jdi_driver);

MODULE_AUTHOR("Tai-Hua Tseng <tai-hua.tseng@mediatek.com>");
MODULE_DESCRIPTION("truly ft8756 VDO LCD Panel Driver");
MODULE_LICENSE("GPL v2");
