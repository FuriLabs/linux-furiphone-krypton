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
	struct gpio_desc *tp_reset_gpio;
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

	pr_info("%s+\n", __func__);

	if (ret == 0) {
		ret = jdi_dcs_read(ctx, 0x0A, buffer, 1);
		pr_info("%s  0x%08x\n", __func__, buffer[0] | (buffer[1] << 8));
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
	udelay(10 * 1000);
	gpiod_set_value(ctx->reset_gpio, 0);
	udelay(10 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
	udelay(10 * 1000);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	jdi_dcs_write_seq_static(ctx,0xDF,0x93, 0x86, 0x19);
	jdi_dcs_write_seq_static(ctx,0xDE,0x00);
	jdi_dcs_write_seq_static(ctx,0xB2,0x01, 0x23, 0x60, 0x55, 0x88, 0xDB);
	jdi_dcs_write_seq_static(ctx,0xB6,0x08, 0x00, 0x00, 0x00, 0x30, 0x00);
	jdi_dcs_write_seq_static(ctx,0xBB,0x00, 0x42, 0x00, 0x83, 0x46, 0x33, 0x33, 0x33, 0x00, 0x5A);
	jdi_dcs_write_seq_static(ctx,0xBD,0x00, 0x37);
	jdi_dcs_write_seq_static(ctx,0xBF,0x53, 0xE7, 0x30, 0x00, 0x63, 0x03, 0x8D, 0x00);
	jdi_dcs_write_seq_static(ctx,0xC0,0x11, 0x22, 0x01, 0x22, 0x01);
	jdi_dcs_write_seq_static(ctx,0xC3,0x03, 0x01, 0x42, 0x07, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x08, 0x01, 0x01, 0x01, 0x08, 0xA0, 0x71, 0x71, 0x71, 0x71, 0x01, 0xA0, 0x71, 0x71, 0x71, 0x01, 0xA0, 0x71, 0x00, 0x01, 0x06, 0x01, 0x06, 0x01, 0x06, 0x08, 0x0E, 0x0A, 0x10, 0x0A, 0x10, 0x04, 0x57);
	jdi_dcs_write_seq_static(ctx,0xC4,0x01, 0x02, 0x00, 0x00, 0x10, 0x01, 0x01, 0x01, 0x02, 0x08, 0x01, 0x01, 0x01, 0x08, 0xA0, 0x71, 0x01, 0xA0, 0x01, 0xA0, 0x01, 0x06, 0x01, 0x06, 0x01, 0x06, 0x08, 0x0E, 0x01, 0x06, 0x01, 0x06, 0x04, 0x57);
	jdi_dcs_write_seq_static(ctx,0xC6,0x01, 0x58, 0x00, 0xDC, 0x00, 0x28, 0x16, 0x82, 0x01, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x01);
	jdi_dcs_write_seq_static(ctx,0xC8,0x3B, 0x80);
	jdi_dcs_write_seq_static(ctx,0xC9,0x0B, 0x00);
	jdi_dcs_write_seq_static(ctx,0xCE,0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05);
	jdi_dcs_write_seq_static(ctx,0xCF,0x00, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	jdi_dcs_write_seq_static(ctx,0xD0,0x00, 0x96, 0x94, 0xAA, 0xAB, 0xAB, 0x9E, 0x9F, 0x2B, 0x84, 0x86, 0x88, 0x8A, 0x8C, 0x8E, 0x90, 0x92, 0x82, 0x80, 0x2B, 0x2B, 0x2B, 0x2B);
	jdi_dcs_write_seq_static(ctx,0xD1,0x00, 0x97, 0x95, 0xAA, 0xAB, 0xAB, 0x9E, 0x9F, 0x2B, 0x85, 0x87, 0x89, 0x8B, 0x8D, 0x8F, 0x91, 0x93, 0x83, 0x81, 0x2B, 0x2B, 0x2B, 0x2B);
	jdi_dcs_write_seq_static(ctx,0xD2,0x00, 0x01, 0x03, 0x2B, 0x2A, 0x2B, 0x1E, 0x1F, 0x2B, 0x13, 0x11, 0x0F, 0x0D, 0x0B, 0x09, 0x07, 0x05, 0x15, 0x17, 0x2B, 0x2B, 0x2B, 0x2B);
	jdi_dcs_write_seq_static(ctx,0xD3,0x00, 0x00, 0x02, 0x2B, 0x2A, 0x2B, 0x1E, 0x1F, 0x2B, 0x12, 0x10, 0x0E, 0x0C, 0x0A, 0x08, 0x06, 0x04, 0x14, 0x16, 0x2B, 0x2B, 0x2B, 0x2B);
	jdi_dcs_write_seq_static(ctx,0xD4,0x30, 0x00, 0x00, 0x00, 0x18, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x99, 0x00, 0xD0, 0x00, 0x77, 0xAE, 0xB0, 0x00, 0x01, 0x04, 0x04, 0x04);
	jdi_dcs_write_seq_static(ctx,0xD5,0x03, 0x00, 0x30, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0xE0, 0x00, 0x00, 0x00, 0x07, 0x32, 0x5A, 0x10, 0x0E, 0x01, 0x00, 0x03, 0x00, 0x01, 0xF6, 0x00, 0x1C, 0x10, 0x00, 0x00, 0x00, 0x00, 0x71, 0x00, 0x04, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x08, 0x02, 0x02, 0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	jdi_dcs_write_seq_static(ctx,0xD7,0x00, 0x98, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0xA0, 0x80, 0xA0, 0x80);
	jdi_dcs_write_seq_static(ctx,0xDE,0x01);
	jdi_dcs_write_seq_static(ctx,0xC8,0x04, 0x01, 0x04);
	jdi_dcs_write_seq_static(ctx,0xB9,0x01);
	jdi_dcs_write_seq_static(ctx,0xC2,0x00, 0x00, 0x05, 0x55, 0x55, 0x6A, 0xAA, 0xBF, 0xFF, 0xF0, 0x20, 0x20, 0x28, 0x41, 0x73, 0x9A, 0xBB, 0xD7, 0xEF, 0x04, 0x18, 0x39, 0x55, 0x6D, 0x83, 0x96, 0xA8, 0xB9, 0xC8, 0xE7, 0x05, 0x24, 0x46, 0x6C, 0x97, 0xAE, 0xC9, 0xE7, 0x0A, 0x33, 0x65, 0x84, 0xA7, 0xBC, 0xD3, 0xEE, 0xFC);
	jdi_dcs_write_seq_static(ctx,0xC3,0x00, 0x00, 0x05, 0x55, 0x55, 0x6A, 0xAA, 0xBF, 0xFF, 0xF0, 0x20, 0x20, 0x28, 0x41, 0x73, 0x9A, 0xBB, 0xD7, 0xEF, 0x04, 0x18, 0x39, 0x55, 0x6D, 0x83, 0x96, 0xA8, 0xB9, 0xC8, 0xE7, 0x05, 0x24, 0x46, 0x6C, 0x97, 0xAE, 0xC9, 0xE7, 0x0A, 0x33, 0x65, 0x84, 0xA7, 0xBC, 0xD3, 0xEE, 0xFC);
	jdi_dcs_write_seq_static(ctx,0xC4,0x00, 0x00, 0x05, 0x55, 0x55, 0x6A, 0xAA, 0xBF, 0xFF, 0xF0, 0x20, 0x20, 0x28, 0x41, 0x73, 0x9A, 0xBB, 0xD7, 0xEF, 0x04, 0x18, 0x39, 0x55, 0x6D, 0x83, 0x96, 0xA8, 0xB9, 0xC8, 0xE7, 0x05, 0x24, 0x46, 0x6C, 0x97, 0xAE, 0xC9, 0xE7, 0x0A, 0x33, 0x65, 0x84, 0xA7, 0xBC, 0xD3, 0xEE, 0xFC);
	jdi_dcs_write_seq_static(ctx,0xC5,0x00, 0x00, 0x05, 0x55, 0x55, 0x6A, 0xAA, 0xBF, 0xFF, 0xF0, 0x20, 0x20, 0x28, 0x41, 0x73, 0x9A, 0xBB, 0xD7, 0xEF, 0x04, 0x18, 0x39, 0x55, 0x6D, 0x83, 0x96, 0xA8, 0xB9, 0xC8, 0xE7, 0x05, 0x24, 0x46, 0x6C, 0x97, 0xAE, 0xC9, 0xE7, 0x0A, 0x33, 0x65, 0x84, 0xA7, 0xBC, 0xD3, 0xEE, 0xFC);
	jdi_dcs_write_seq_static(ctx,0xC6,0x00, 0x00, 0x05, 0x55, 0x55, 0x6A, 0xAA, 0xBF, 0xFF, 0xF0, 0x20, 0x20, 0x28, 0x41, 0x73, 0x9A, 0xBB, 0xD7, 0xEF, 0x04, 0x18, 0x39, 0x55, 0x6D, 0x83, 0x96, 0xA8, 0xB9, 0xC8, 0xE7, 0x05, 0x24, 0x46, 0x6C, 0x97, 0xAE, 0xC9, 0xE7, 0x0A, 0x33, 0x65, 0x84, 0xA7, 0xBC, 0xD3, 0xEE, 0xFC);
	jdi_dcs_write_seq_static(ctx,0xC7,0x00, 0x00, 0x05, 0x55, 0x55, 0x6A, 0xAA, 0xBF, 0xFF, 0xF0, 0x20, 0x20, 0x28, 0x41, 0x73, 0x9A, 0xBB, 0xD7, 0xEF, 0x04, 0x18, 0x39, 0x55, 0x6D, 0x83, 0x96, 0xA8, 0xB9, 0xC8, 0xE7, 0x05, 0x24, 0x46, 0x6C, 0x97, 0xAE, 0xC9, 0xE7, 0x0A, 0x33, 0x65, 0x84, 0xA7, 0xBC, 0xD3, 0xEE, 0xFC);
	jdi_dcs_write_seq_static(ctx,0xC9,0x05, 0x7F, 0x00, 0x55, 0x55, 0x55, 0x99, 0x99, 0x99, 0x00, 0x00, 0x40, 0x00);
	jdi_dcs_write_seq_static(ctx,0xDE,0x02);
	jdi_dcs_write_seq_static(ctx,0xC2,0x02, 0x42, 0xD0, 0x02, 0x00, 0xC4, 0x61, 0x73, 0xFB);
	jdi_dcs_write_seq_static(ctx,0xC4,0x02, 0x11, 0x07, 0x00, 0x1A);
	jdi_dcs_write_seq_static(ctx,0xC5,0x07, 0xFF, 0xFF, 0x07, 0xFF, 0xFF);
	jdi_dcs_write_seq_static(ctx,0xC6,0x40, 0x06);
	jdi_dcs_write_seq_static(ctx,0xD6,0x03, 0x40, 0x01, 0x01, 0x0D, 0x40, 0x10, 0x64, 0x70);
	jdi_dcs_write_seq_static(ctx,0xE6,0x10, 0x0A, 0xC8);
	jdi_dcs_write_seq_static(ctx,0xEE,0x10, 0x01, 0x10, 0x00);
	jdi_dcs_write_seq_static(ctx,0xDE,0x03);
	jdi_dcs_write_seq_static(ctx,0xD1,0x05, 0x10);
	jdi_dcs_write_seq_static(ctx,0xDE,0x04);
	jdi_dcs_write_seq_static(ctx,0xCD,0x20, 0x04, 0x21, 0x10, 0x33);
	jdi_dcs_write_seq_static(ctx,0xCE,0x00, 0x28, 0x50, 0x0A, 0x5A, 0x00, 0x00);
	jdi_dcs_write_seq_static(ctx,0xDE,0x07);
	jdi_dcs_write_seq_static(ctx,0xDE,0x00);
	jdi_dcs_write_seq_static(ctx,0xB3,0x01, 0x00, 0x00, 0x88);
    
	jdi_dcs_write_seq_static(ctx,0x11,0x00);
	msleep(120);

	jdi_dcs_write_seq_static(ctx,0x29,0x00);
	//msleep(2);
	//jdi_dcs_write_seq_static(ctx,0xc2,0x60);
	msleep(20);
	pr_info("%s-\n", __func__);
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

	pr_info("%s\n", __func__);

	if (!ctx->prepared)
		return 0;

	msleep(20);
	jdi_dcs_write_seq_static(ctx, 0x28);
	msleep(50);
	jdi_dcs_write_seq_static(ctx, 0x10);
	msleep(120);
	jdi_dcs_write_seq_static(ctx, 0x4F,0x01);

	ctx->error = 0;
	ctx->prepared = false;

#if 0
	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
#endif

	ctx->tp_reset_gpio = devm_gpiod_get(ctx->dev,
		"tp-reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->tp_reset_gpio)) {
		dev_warn(ctx->dev, "%s: cannot get tp-reset %ld\n",
			__func__, PTR_ERR(ctx->tp_reset_gpio));
		return PTR_ERR(ctx->tp_reset_gpio);
	}
	printk("%s tp_reset_gpio 0\n", __func__);
	gpiod_set_value(ctx->tp_reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->tp_reset_gpio);

	ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
		"bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(ctx->dev, "%s: cannot get bias-neg 1 %ld\n",
			__func__, PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
	gpiod_set_value(ctx->bias_neg, 0);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);

	udelay(1000);

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
	pr_info("%s+\n", __func__);
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
	i2c_data.data = 0x14; //5.8
	lcm_i2c_set_data(1, &i2c_data);
	i2c_data.cmd = 0x01;  //-5V
	i2c_data.data = 0x14; //-5.8
	lcm_i2c_set_data(1, &i2c_data);
	mdelay(15);
#endif
	jdi_panel_init(ctx);
	mdelay(12);

	ctx->tp_reset_gpio = devm_gpiod_get(ctx->dev,
		"tp-reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->tp_reset_gpio)) {
		dev_warn(ctx->dev, "%s: cannot get tp-reset %ld\n",
			__func__, PTR_ERR(ctx->tp_reset_gpio));
		return PTR_ERR(ctx->tp_reset_gpio);
	}
	printk("%s tp_reset_gpio 1\n", __func__);
	gpiod_set_value(ctx->tp_reset_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->tp_reset_gpio);

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

	pr_info("%s-\n", __func__);
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
#if 0
JDEVB_VS(4);
JDEVB_VBP(32); 
JDEVB_VFP(220); 
JDEVB_HS(16); 
JDEVB_HBP(16);
JDEVB_HFP(40);
#endif

#define HFP (40)
#define HSA (16)
#define HBP (16)
#define VFP (220)
#define VSA (4)
#define VBP (32)
#define VAC (1920)
#define HAC (1200)
static u32 fake_heigh = VAC;
static u32 fake_width = HAC;
static bool need_fake_resolution;

static struct drm_display_mode default_mode = {
	.clock = 166072,
	.hdisplay = HAC,
	.hsync_start = HAC + HFP,
	.hsync_end = HAC + HFP + HSA,
	.htotal = HAC + HFP + HSA + HBP,
	.vdisplay = VAC,
	.vsync_start = VAC + VFP,
	.vsync_end = VAC + VFP + VSA,
	.vtotal = VAC + VFP + VSA + VBP,
	.vrefresh = 60,
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
	.pll_clk = 498,
	//.vfp_low_power = 750,
	.cust_esd_check = 0,
	.esd_check_enable = 0,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0a,
		.count = 1,
		.para_list[0] = 0x9c,
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
	printk("%s jd9366ts \n", __func__);
	dsi_node = of_get_parent(dev->of_node);
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);
		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			if (!remote_node) {
				pr_info("No panel connected,skip probe lcm\n");
				return -ENODEV;
			}
			pr_info("device node name:%s dev->of_node->name:%s\n", remote_node->name,dev->of_node->name);
		}
	}
	if (remote_node != dev->of_node) {
		pr_info("%s+ jd9366ts skip probe due to not current lcm\n", __func__);
		return -ENODEV;
	}

	pr_info("%s+ jd9366ts \n", __func__);
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

	ctx->tp_reset_gpio = devm_gpiod_get(ctx->dev,
		"tp-reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->tp_reset_gpio)) {
		dev_warn(ctx->dev, "%s: cannot get tp-reset %ld\n",
			__func__, PTR_ERR(ctx->tp_reset_gpio));
		return PTR_ERR(ctx->tp_reset_gpio);
	}
	printk("%s tp_reset_gpio 1\n", __func__);
	devm_gpiod_put(ctx->dev, ctx->tp_reset_gpio);

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
	pr_info("%s-\n", __func__);

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
	    .compatible = "truly,jd9366ts,vdo",
	},
	{ }
};

MODULE_DEVICE_TABLE(of, jdi_of_match);

static struct mipi_dsi_driver jdi_driver = {
	.probe = jdi_probe,
	.remove = jdi_remove,
	.driver = {
		.name = "panel-truly-jd9366ts-vdo",
		.owner = THIS_MODULE,
		.of_match_table = jdi_of_match,
	},
};


module_mipi_dsi_driver(jdi_driver);

MODULE_AUTHOR("Tai-Hua Tseng <tai-hua.tseng@mediatek.com>");
MODULE_DESCRIPTION("truly jd9366ts VDO LCD Panel Driver");
MODULE_LICENSE("GPL v2");
