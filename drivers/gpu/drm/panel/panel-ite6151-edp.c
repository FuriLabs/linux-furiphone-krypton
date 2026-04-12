// SPDX-License-Identifier: GPL-2.0
/*
 * IT6151 MIPI-to-eDP bridge panel driver
 *
 * Copyright (c) 2024 MediaTek Inc.
 *
 * This driver is ported from LK (Little Kernel) format to Linux DRM panel
 * driver for kernel-4.19 on MTK6853 Android 12 platform.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/fb.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>
#include <stdbool.h>

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mtk_panel_ext.h"
#include "../mediatek/mtk_log.h"
#include "../mediatek/mtk_drm_graphics_base.h"
#endif

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "../mediatek/mtk_corner_pattern/mtk_data_hw_roundedpattern.h"
#endif

/* IT6151 I2C addresses */
#define IT6151_DP_I2C_ADDR 0x5C
#define IT6151_MIPI_I2C_ADDR 0x6C

/* IT6151 chip identification */
#define IT6151_VENDOR_ID_0 0x54
#define IT6151_VENDOR_ID_1 0x49
#define IT6151_DEVICE_ID_DP_0 0x51
#define IT6151_DEVICE_ID_DP_1 0x61
#define IT6151_DEVICE_ID_MIPI_0 0x21
#define IT6151_DEVICE_ID_MIPI_1 0x61
#define IT6151_REVISION_A1 0xA1

/* IT6151 chip detection result */
enum it6151_chip_type {
	IT6151_CHIP_UNKNOWN = -1,
	IT6151_CHIP_DP_INTERFACE = 0,
	IT6151_CHIP_MIPI_INTERFACE = 1,
};

/* IT6151 configuration constants from LK driver */
#define PANEL_RESOLUTION_1920x1200p60RB  

/* MIPI lane configuration */
#define MIPI_4_LANE (3)
#define MIPI_3_LANE (2)
#define MIPI_2_LANE (1)
#define MIPI_1_LANE (0)

/* MIPI Packed Pixel Stream formats */
#define RGB_24b (0x3E)
#define RGB_30b (0x0D)
#define RGB_36b (0x1D)
#define RGB_18b_P (0x1E)
#define RGB_18b_L (0x2E)
#define YCbCr_16b (0x2C)
#define YCbCr_20b (0x0C)
#define YCbCr_24b (0x1C)

/* DPTX configuration */
#define B_DPTXIN_6Bpp (0)
#define B_DPTXIN_8Bpp (1)
#define B_DPTXIN_10Bpp (2)
#define B_DPTXIN_12Bpp (3)

#define B_LBR (1)
#define B_HBR (0)

#define B_4_LANE (3)
#define B_2_LANE (1)
#define B_1_LANE (0)

#define B_SSC_ENABLE (1)
#define B_SSC_DISABLE (0)

/* Configuration settings for 1920x1200p60RB */
#ifdef PANEL_RESOLUTION_1920x1200p60RB
#define PANEL_WIDTH 1920
#define VIC 0 /* non-Zero value for CEA setting */
#define MP_HPOL 1
#define MP_VPOL 0
#define DPTX_LANE_COUNT B_2_LANE
#define MIPI_LANE_COUNT MIPI_4_LANE
#define EN_UFO 0
#define MIPI_PACKED_FMT RGB_24b
#define MP_H_RESYNC 1
#define MP_V_RESYNC 1
#endif

/* Training and configuration settings */
#define TRAINING_BITRATE (B_HBR)
#define DPTX_SSC_SETTING (B_SSC_DISABLE)
#define MP_MCLK_INV (0)
#define MP_CONTINUOUS_CLK (1)
#define MP_LANE_DESKEW (1)
#define MP_PCLK_DIV (2)
#define MP_LANE_SWAP (0)
#define MP_PN_SWAP (0)

#define DP_PN_SWAP (0)
#define DP_AUX_PN_SWAP (0)
#define DP_LANE_SWAP (0)

#define LVDS_LANE_SWAP (0)
#define LVDS_PN_SWAP (0)
#define LVDS_DC_BALANCE (0)

#define LVDS_6BIT (0) /* '0' for 8 bit, '1' for 6 bit */
#define VESA_MAP (1) /* '0' for JEIDA , '1' for VESA MAP */

#define INT_MASK (0)
#define MIPI_INT_MASK (0)
#define TIMER_CNT (0x0A)

/* OCP2131 I2C address */
#define OCP2131_I2C_ADDR 0x3E

/* GPIO definitions from LK driver */
#define GPIO_IT6151_RST_EN "it6151_rst-gpio"
#define GPIO_IT6151_STB_EN "it6151_stb-gpio"
#define GPIO_IT6151_ENPSR_EN "it6151_enpsr-gpio"
#define GPIO_IT6151_INT "it6151_int-gpio"
#define GPIO_LCD_PWR_EN "lcd_pwr-gpio"
#define GPIO_LCD_BIAS_EN1 "lcd_bias1-gpio"
#define GPIO_LCD_BIAS_EN2 "lcd_bias2-gpio"
#define GPIO_LCD_BL_EN "lcd_bl-gpio"
#define GPIO_LCD_TEST_EN "lcd_test-gpio"

/**
 * struct it6151_panel - IT6151 panel driver private structure
 * @dev: device handle
 * @panel: DRM panel structure
 * @dp_client: I2C client for DisplayPort interface (0x5C)
 * @mipi_client: I2C client for MIPI interface (0x6C)
 * @it6151_reset_gpio: GPIO descriptor for reset control (GPIO64)
 * @enpsr_gpio: GPIO descriptor for ENPSR control (GPIO17)
 * @stb_en_gpio: GPIO descriptor for STB enable control (GPIO93)
 * @ldo_en_gpio: GPIO descriptor for LDO power enable control
 * @chip_type: Detected chip interface type (DP or MIPI)
 * @revision: Chip revision ID
 * @prepared: Panel prepare state
 * @enabled: Panel enable state
 * @error: Error state for error handling
 */
struct it6151_panel {
	struct device *dev;
	struct drm_panel panel;

	struct i2c_client *dp_client;
	struct i2c_client *mipi_client;

	struct gpio_desc *it6151_reset_gpio;
	struct gpio_desc *it6151_stb_gpio;
	struct gpio_desc *it6151_enpsr_gpio;
	struct gpio_desc *it6151_int_gpio;
	struct gpio_desc *lcd_pwr_gpio;
	struct gpio_desc *lcd_bias1_gpio;
	struct gpio_desc *lcd_bias2_gpio;
	struct gpio_desc *lcd_test_gpio;
	struct gpio_desc *reset_gpio;

	struct i2c_client *ocp2131_client;
    
	enum it6151_chip_type chip_type;
	u8 revision;

	bool prepared;
	bool enabled;
	int error;
};

static inline struct it6151_panel *to_it6151_panel(struct drm_panel *panel)
{
	return container_of(panel, struct it6151_panel, panel);
}

/**
 * it6151_handle_error - Centralized error handling for IT6151 operations
 * @panel: IT6151 panel structure
 * @error: Error code to handle
 * @operation: Description of the operation that failed
 * @reg: Register address (if applicable, use 0xFF if not applicable)
 * @val: Register value (if applicable, use 0xFF if not applicable)
 *
 * This function provides centralized error handling with consistent logging
 * and error state management following kernel standards.
 *
 * Returns: The original error code
 */
static int it6151_handle_error(struct it6151_panel *panel, int error,
			       const char *operation, u8 reg, u8 val)
{
	if (error >= 0)
		return error;

	/* Set panel error state for subsequent operations */
	panel->error = error;

	/* Log error with appropriate detail level based on error type */
	switch (error) {
	case -ENODEV:
		dev_err(panel->dev,
			"IT6151 %s failed: device not found (reg=0x%02x)\n",
			operation, reg);
		break;
	case -ETIMEDOUT:
		dev_err(panel->dev,
			"IT6151 %s failed: timeout (reg=0x%02x, val=0x%02x)\n",
			operation, reg, val);
		break;
	case -EIO:
		dev_err(panel->dev,
			"IT6151 %s failed: I/O error (reg=0x%02x, val=0x%02x)\n",
			operation, reg, val);
		break;
	case -ENXIO:
		dev_err(panel->dev,
			"IT6151 %s failed: no such device or address (reg=0x%02x)\n",
			operation, reg);
		break;
	case -EBUSY:
		dev_warn(
			panel->dev,
			"IT6151 %s failed: device busy, will retry (reg=0x%02x)\n",
			operation, reg);
		break;
	default:
		dev_err(panel->dev,
			"IT6151 %s failed: error %d (reg=0x%02x, val=0x%02x)\n",
			operation, error, reg, val);
		break;
	}

	return error;
}

/* I2C communication functions with enhanced retry mechanism and logging */
static int it6151_i2c_write_retry(struct i2c_client *client, u8 reg, u8 val,
				  int retries)
{
	int ret;
	int retry = retries;
	int original_retries = retries;

	if (!client) {
		pr_err("IT6151: I2C client is NULL for write operation\n");
		return -ENODEV;
	}

	while (retry--) {
		ret = i2c_smbus_write_byte_data(client, reg, val);
		if (ret >= 0) {
			/* Log successful operation only if retries were needed */
			if (retry < (original_retries - 1)) {
				dev_dbg(&client->dev,
					"I2C write succeeded after %d retries: addr=0x%02x, reg=0x%02x, val=0x%02x\n",
					original_retries - retry - 1,
					client->addr, reg, val);
			}
			break;
		}

		/* Log retry attempts for debugging */
		if (retry > 0) {
			dev_dbg(&client->dev,
				"I2C write retry %d/%d failed: addr=0x%02x, reg=0x%02x, val=0x%02x, error=%d\n",
				original_retries - retry, original_retries,
				client->addr, reg, val, ret);
			msleep(1);
		} else {
			/* Final failure - log as error */
			dev_err(&client->dev,
				"I2C write failed after %d retries: addr=0x%02x, reg=0x%02x, val=0x%02x, error=%d\n",
				original_retries, client->addr, reg, val, ret);
		}
	}

	return ret;
}

static int it6151_i2c_read_retry(struct i2c_client *client, u8 reg, int retries)
{
	int ret;
	int retry = retries;
	int original_retries = retries;

	if (!client) {
		pr_err("IT6151: I2C client is NULL for read operation\n");
		return -ENODEV;
	}

	while (retry--) {
		ret = i2c_smbus_read_byte_data(client, reg);
		if (ret >= 0) {
			/* Log successful operation only if retries were needed */
			if (retry < (original_retries - 1)) {
				dev_dbg(&client->dev,
					"I2C read succeeded after %d retries: addr=0x%02x, reg=0x%02x, val=0x%02x\n",
					original_retries - retry - 1,
					client->addr, reg, ret);
			}
			break;
		}

		/* Log retry attempts for debugging */
		if (retry > 0) {
			dev_dbg(&client->dev,
				"I2C read retry %d/%d failed: addr=0x%02x, reg=0x%02x, error=%d\n",
				original_retries - retry, original_retries,
				client->addr, reg, ret);
			msleep(1);
		} else {
			/* Final failure - log as error */
			dev_err(&client->dev,
				"I2C read failed after %d retries: addr=0x%02x, reg=0x%02x, error=%d\n",
				original_retries, client->addr, reg, ret);
		}
	}

	return ret;
}

/**
 * it6151_dp_write - Write to DisplayPort I2C interface (0x5C)
 * @panel: IT6151 panel structure
 * @reg: Register address
 * @val: Value to write
 *
 * Returns: 0 on success, negative error code on failure
 */
static int it6151_dp_write(struct it6151_panel *panel, u8 reg, u8 val)
{
	int ret;

	if (panel->error < 0)
		return panel->error;

	if (!panel->dp_client) {
		return it6151_handle_error(panel, -ENODEV,
					   "DP I2C client not initialized", reg,
					   val);
	}

	ret = it6151_i2c_write_retry(panel->dp_client, reg, val, 3);
	if (ret < 0) {
		return it6151_handle_error(panel, ret, "DP I2C write", reg,
					   val);
	}

	dev_dbg(panel->dev, "DP I2C write: reg=0x%02x, val=0x%02x\n", reg, val);
	return ret;
}

/**
 * it6151_dp_read - Read from DisplayPort I2C interface (0x5C)
 * @panel: IT6151 panel structure
 * @reg: Register address
 * @val: Pointer to store read value
 *
 * Returns: 0 on success, negative error code on failure
 */
static int it6151_dp_read(struct it6151_panel *panel, u8 reg, u8 *val)
{
	int ret;

	if (panel->error < 0)
		return panel->error;

	if (!panel->dp_client) {
		return it6151_handle_error(panel, -ENODEV,
					   "DP I2C client not initialized", reg,
					   0xFF);
	}

	ret = it6151_i2c_read_retry(panel->dp_client, reg, 3);
	if (ret < 0) {
		return it6151_handle_error(panel, ret, "DP I2C read", reg,
					   0xFF);
	}

	*val = ret;
	dev_dbg(panel->dev, "DP I2C read: reg=0x%02x, val=0x%02x\n", reg, *val);
	return 0;
}

/**
 * it6151_mipi_write - Write to MIPI I2C interface (0x6C)
 * @panel: IT6151 panel structure
 * @reg: Register address
 * @val: Value to write
 *
 * Returns: 0 on success, negative error code on failure
 */
static int it6151_mipi_write(struct it6151_panel *panel, u8 reg, u8 val)
{
	int ret;

	if (!panel->mipi_client) {
		return it6151_handle_error(panel, -ENODEV, "MIPI I2C client not initialized", reg, val);
	}

	ret = it6151_i2c_write_retry(panel->mipi_client, reg, val, 3);
	if (ret < 0) {
		return it6151_handle_error(panel, ret, "MIPI I2C write", reg, val);
	}

	dev_dbg(panel->dev, "MIPI I2C write: reg=0x%02x, val=0x%02x\n", reg, val);
	return ret;
}

/**
 * it6151_mipi_read - Read from MIPI I2C interface (0x6C)
 * @panel: IT6151 panel structure
 * @reg: Register address
 * @val: Pointer to store read value
 *
 * Returns: 0 on success, negative error code on failure
 */
static int it6151_mipi_read(struct it6151_panel *panel, u8 reg, u8 *val)
{
	int ret;

	if (!panel->mipi_client) {
		return it6151_handle_error(panel, -ENODEV, "MIPI I2C client not initialized", reg, 0xFF);
	}

	ret = it6151_i2c_read_retry(panel->mipi_client, reg, 3);
	if (ret < 0) {
		return it6151_handle_error(panel, ret, "MIPI I2C read", reg, 0xFF);
	}

	*val = ret;
	dev_dbg(panel->dev, "MIPI I2C read: reg=0x%02x, val=0x%02x\n", reg, *val);
	return 0;
}

/**
 * it6151_dptx_init - Initialize IT6151 DisplayPort TX interface
 * @panel: IT6151 panel structure
 *
 * This function initializes the IT6151 DisplayPort TX interface following
 * the same sequence as the original LK driver IT6151_DPTX_init() function.
 * All register configurations and timing are preserved from the LK code.
 *
 * Returns: 0 on success, negative error code on failure
 */
static int it6151_dptx_init(struct it6151_panel *panel)
{
	dev_info(panel->dev, "Initializing IT6151 DPTX interface\n");

	/* Reset error state for initialization */
	panel->error = 0;

	/* DP Transmitter Software Reset */
	it6151_dp_write(panel, 0x05, 0x29);
	mdelay(1);
	it6151_dp_write(panel, 0x05, 0x00);
	mdelay(10); /* Wait for reset to complete */

	/* Configure Interrupt Mask */
	it6151_dp_write( panel, 0x09, INT_MASK); /* Enable HPD_IRQ, HPD_CHG, VIDSTABLE */
	it6151_dp_write(panel, 0x0A, 0x00);
	it6151_dp_write(panel, 0x0B, 0x00);

	/* Configure DP Transmitter Basic Parameters */
	it6151_dp_write(panel, 0xC5, 0xC1);
	it6151_dp_write(panel, 0xB5, 0x00);
	it6151_dp_write(panel, 0xB7, 0x80);
	it6151_dp_write(panel, 0xC4, 0xF0);

	/* Clear All Interrupts */
	it6151_dp_write(panel, 0x06, 0xFF);
	it6151_dp_write(panel, 0x07, 0xFF);
	it6151_dp_write(panel, 0x08, 0xFF);

	/* Configure DP Transmitter Parameters */
	it6151_dp_write(panel, 0x05, 0x00);
	it6151_dp_write(panel, 0x0c, 0x08);
	it6151_dp_write(panel, 0x21, 0x05);
	it6151_dp_write(panel, 0x3a, 0x04);
	it6151_dp_write(panel, 0x5f, 0x06);

	/* Configure PLL Parameters - Optimized for NV133WUM-N61 */
	it6151_dp_write(panel, 0xc9, 0xf5);
	it6151_dp_write(panel, 0xca, 0x4c);
	it6151_dp_write(panel, 0xcb, 0x37);
	it6151_dp_write(panel, 0xd3, 0x03);
	it6151_dp_write(panel, 0xd4, 0x60);
	it6151_dp_write(panel, 0xe8, 0x11);
	it6151_dp_write(panel, 0xec, VIC);

	mdelay(10); /* Wait for PLL to stabilize */

	/* Configure Link Training Parameters */
	dev_info(panel->dev, "Configuring DP link training\n");
	it6151_dp_write(panel, 0x23, 0x42);
	it6151_dp_write(panel, 0x24, 0x07);
	it6151_dp_write(panel, 0x25, 0x01);
	it6151_dp_write(panel, 0x26, 0x00);
	it6151_dp_write(panel, 0x27, 0x10);
	it6151_dp_write(panel, 0x2B, 0x05);
	it6151_dp_write(panel, 0x23, 0x40);

	/* Configure DP PHY Parameters */
	it6151_dp_write(panel, 0x22, (DP_AUX_PN_SWAP << 3) | (DP_PN_SWAP << 2) | 0x03);
    it6151_dp_write(panel, 0x16, (DPTX_SSC_SETTING << 4) | (DP_LANE_SWAP << 3) | (DPTX_LANE_COUNT << 1) | TRAINING_BITRATE);

	/* Configure Drive Strength - Optimized for NV133WUM-N61 */
	it6151_dp_write(panel, 0x0f, 0x01);
	it6151_dp_write(panel, 0x76, 0xa7);
	it6151_dp_write(panel, 0x77, 0xaf);
	it6151_dp_write(panel, 0x7e, 0x8f);
	it6151_dp_write(panel, 0x7f, 0x07);
	it6151_dp_write(panel, 0x80, 0xef);
	it6151_dp_write(panel, 0x81, 0x5f);
	it6151_dp_write(panel, 0x82, 0xef);
	it6151_dp_write(panel, 0x83, 0x07);
	it6151_dp_write(panel, 0x88, 0x38);
	it6151_dp_write(panel, 0x89, 0x1f);
	it6151_dp_write(panel, 0x8a, 0x48);
	it6151_dp_write(panel, 0x0f, 0x00);

	/* Start Link Training */
	it6151_dp_write(panel, 0x5c, 0xf3);
	it6151_dp_write(panel, 0x17, 0x04);
	mdelay(1);
	it6151_dp_write(panel, 0x17, 0x01);
	mdelay(20); /* Wait for link training to complete */

	dev_info(panel->dev, "IT6151 DPTX initialization completed\n");

	return 0;
}

/**
 * it6151_mipi_init_dp_interface - Initialize MIPI interface for DP-detected chip
 * @panel: IT6151 panel structure
 *
 * This function initializes the MIPI interface when the chip is detected
 * via the DP interface. It follows the initialization sequence from the
 * original LK driver vit6151_init() function for DP interface detection.
 *
 * Returns: 0 on success, negative error code on failure
 */
static int it6151_mipi_init_dp_interface(struct it6151_panel *panel)
{
	dev_info(panel->dev, "Initializing MIPI interface (DP-detected chip)\n");

	/* DP SW Reset */
	it6151_dp_write(panel, 0x05, 0x04);
	mdelay(10); /* Wait for reset to complete */

	/* Set MIPI I2C address mapping */
	it6151_dp_write(panel, 0xfd, (IT6151_MIPI_I2C_ADDR << 1) | 1);

	/* MIPI interface configuration - enhanced */
	it6151_mipi_write(panel, 0x05, 0x00);
	mdelay(5);

	/* Configure MIPI lane count and format */
	it6151_mipi_write(panel, 0x0c, (MP_LANE_SWAP << 7) | (MP_PN_SWAP << 6) | (MIPI_LANE_COUNT << 4) | EN_UFO);
    it6151_mipi_write(panel, 0x11, MP_MCLK_INV);

	/* Enhanced MIPI receiver stability configuration */
	it6151_mipi_write(panel, 0x10, 0x00); /* Clear error flags */
	it6151_mipi_write(panel, 0x12, 0x03); /* Set MIPI receiver parameters */

	/* Revision-specific clock configuration */
	if (panel->revision == IT6151_REVISION_A1) {
		dev_info(panel->dev, "IT6151A1 detected\n");
		it6151_mipi_write(panel, 0x19, MP_LANE_DESKEW);
	} else {
		dev_info(panel->dev, "IT6151A0 detected\n");
		it6151_mipi_write( panel, 0x19, (MP_CONTINUOUS_CLK << 1) | MP_LANE_DESKEW);
	}

	/* Configure MIPI data format and width */
	it6151_mipi_write(panel, 0x27, MIPI_PACKED_FMT);
	it6151_mipi_write(panel, 0x28, ((PANEL_WIDTH / 4 - 1) >> 2) & 0xC0);
	it6151_mipi_write(panel, 0x29, (PANEL_WIDTH / 4 - 1) & 0xFF);

	/* Configure MIPI timing parameters */
	it6151_mipi_write(panel, 0x2e, 0x34);
	it6151_mipi_write(panel, 0x2f, 0x01);

	/* Configure sync signal polarity */
	it6151_mipi_write(panel, 0x4e, (MP_V_RESYNC << 3) | (MP_H_RESYNC << 2) | (MP_VPOL << 1) | (MP_HPOL));
	it6151_mipi_write(panel, 0x80, (EN_UFO << 5) | MP_PCLK_DIV);
	it6151_mipi_write(panel, 0x84, 0x8f);

	/* Configure interrupt and timer */
	it6151_mipi_write(panel, 0x09, MIPI_INT_MASK);
	it6151_mipi_write(panel, 0x92, TIMER_CNT);

	dev_info(panel->dev,
		 "MIPI interface initialization (DP-detected) completed\n");

	return 0;
}


/**
 * it6151_mipi_init_mipi_interface - Initialize MIPI interface for MIPI-detected chip
 * @panel: IT6151 panel structure
 *
 * This function initializes the MIPI interface when the chip is detected
 * via the MIPI interface. It follows the initialization sequence from the
 * original LK driver vit6151_init() function for MIPI interface detection.
 *
 * Returns: 0 on success, negative error code on failure
 */
static int it6151_mipi_init_mipi_interface(struct it6151_panel *panel)
{
	int ret;

	dev_info(panel->dev,
		 "Initializing MIPI interface (MIPI-detected chip)\n");

	/* MIPI reset sequence */
	ret = it6151_mipi_write(panel, 0x05, 0x33);
	if (ret < 0)
		return ret;

	ret = it6151_mipi_write(panel, 0x05, 0x40);
	if (ret < 0)
		return ret;

	ret = it6151_mipi_write(panel, 0x05, 0x00);
	if (ret < 0)
		return ret;

	/* MIPI lane and format configuration */
	ret = it6151_mipi_write(panel, 0x0c,
				(MP_LANE_SWAP << 7) | (MP_PN_SWAP << 6) |
					(MIPI_LANE_COUNT << 4));
	if (ret < 0)
		return ret;

	ret = it6151_mipi_write(panel, 0x11, MP_MCLK_INV);
	if (ret < 0)
		return ret;

	ret = it6151_mipi_write(panel, 0x19,
				(MP_CONTINUOUS_CLK << 1) | MP_LANE_DESKEW);
	if (ret < 0)
		return ret;

	ret = it6151_mipi_write(panel, 0x4E,
				(MP_V_RESYNC << 3) | (MP_H_RESYNC << 2) |
					(MP_VPOL << 1) | (MP_HPOL));
	if (ret < 0)
		return ret;

	ret = it6151_mipi_write(panel, 0x72, 0x01);
	if (ret < 0)
		return ret;

	ret = it6151_mipi_write(panel, 0x73, 0x03);
	if (ret < 0)
		return ret;

	ret = it6151_mipi_write(panel, 0x80, MP_PCLK_DIV);
	if (ret < 0)
		return ret;

	/* LVDS configuration registers */
	ret = it6151_mipi_write(panel, 0xC0, 0x13);
	if (ret < 0)
		return ret;

	ret = it6151_mipi_write(panel, 0xC1, 0x01);
	if (ret < 0)
		return ret;

	ret = it6151_mipi_write(panel, 0xC2, 0x47);
	if (ret < 0)
		return ret;

	ret = it6151_mipi_write(panel, 0xC3, 0x67);
	if (ret < 0)
		return ret;

	ret = it6151_mipi_write(panel, 0xC4, 0x04);
	if (ret < 0)
		return ret;

	ret = it6151_mipi_write(panel, 0xCB,
				(LVDS_PN_SWAP << 5) | (LVDS_LANE_SWAP << 4) |
					(LVDS_6BIT << 2) |
					(LVDS_DC_BALANCE << 1) | VESA_MAP);
	if (ret < 0)
		return ret;

	dev_info(
		panel->dev,
		"MIPI interface initialization (MIPI-detected) completed successfully\n");

	return 0;
}

/**
 * it6151_chip_init - Initialize IT6151 chip based on detected interface
 * @panel: IT6151 panel structure
 *
 * This function performs the complete IT6151 chip initialization following
 * the same logic as the original LK driver vit6151_init() function. It
 * initializes both MIPI and DP interfaces based on the detected chip type.
 *
 * Returns: 0 on success, negative error code on failure
 */
static int it6151_chip_init(struct it6151_panel *panel)
{
	int ret;

	dev_info(panel->dev, "Starting IT6151 chip initialization\n");

	/* Initialize based on detected chip interface type */
	if (panel->chip_type == IT6151_CHIP_DP_INTERFACE) {
		/* Initialize MIPI interface first for DP-detected chip */
		ret = it6151_mipi_init_dp_interface(panel);
		if (ret < 0) {
			dev_err(panel->dev,
				"MIPI interface initialization failed: %d\n",
				ret);
			return ret;
		}

		/* Then initialize DP TX interface */
		ret = it6151_dptx_init(panel);
		if (ret < 0) {
			dev_err(panel->dev, "DPTX initialization failed: %d\n",
				ret);
			return ret;
		}

		dev_info(
			panel->dev,
			"IT6151 chip initialization completed (DP interface detected)\n");
		return 0;

	} else if (panel->chip_type == IT6151_CHIP_MIPI_INTERFACE) {
		/* Initialize MIPI interface for MIPI-detected chip */
		ret = it6151_mipi_init_mipi_interface(panel);
		if (ret < 0) {
			dev_err(panel->dev,
				"MIPI interface initialization failed: %d\n",
				ret);
			return ret;
		}

		dev_info(
			panel->dev,
			"IT6151 chip initialization completed (MIPI interface detected)\n");
		return 1; /* Return 1 as in original LK code for MIPI interface */

	} else {
		dev_err(panel->dev, "Cannot initialize - unknown chip type\n");
		return -ENODEV;
	}
}

/**
 * it6151_power_on - Power on the IT6151 panel
 * @panel: IT6151 panel structure
 *
 * This function powers on the IT6151 panel following the same sequence
 * as the original LK driver lcm_init() function. It handles regulator
 * control, GPIO sequencing, and proper timing delays.
 *
 * Returns: 0 on success, negative error code on failure
 */
static int ocp2131_i2c_write_byte(struct it6151_panel *panel, u8 addr, u8 value)
{
	int ret;

	if (!panel->ocp2131_client) {
		dev_err(panel->dev, "OCP2131 I2C client not initialized\n");
		return -ENODEV;
	}

	ret = i2c_smbus_write_byte_data(panel->ocp2131_client, addr, value);
	if (ret < 0) {
		dev_err(panel->dev, "OCP2131 I2C write failed: ret=%d\n", ret);
		return ret;
	}

	return 0;
}

static int it6151_power_on(struct it6151_panel *panel)
{
	dev_info(panel->dev, "Starting IT6151 panel power-on sequence\n");

	/* 1. Ensure all control signals are low */
	gpiod_set_value_cansleep(panel->lcd_test_gpio, 0);
	gpiod_set_value_cansleep(panel->it6151_reset_gpio, 0);
	gpiod_set_value_cansleep(panel->it6151_stb_gpio, 0);
	gpiod_set_value_cansleep(panel->it6151_enpsr_gpio, 0);
	gpiod_set_value_cansleep(panel->it6151_int_gpio, 0);

	/* 2. Enable 1.2V power */
	dev_info(panel->dev, "Enable 1.2V power\n");
	gpiod_set_value_cansleep(panel->lcd_pwr_gpio, 1);
	mdelay(50); /* Increased delay for power stability */

	/* 3. Enable bias voltage */
	dev_info(panel->dev, "Enable bias voltage\n");
	gpiod_set_value_cansleep(panel->lcd_bias1_gpio, 1);
	mdelay(5);
	gpiod_set_value_cansleep(panel->lcd_bias2_gpio, 1);
	mdelay(10);

	/* 4. Configure OCP2131 bias chip */
	dev_info(panel->dev, "Configure OCP2131\n");
	ocp2131_i2c_write_byte(panel, 0x00, 0x10);
	mdelay(2);
	ocp2131_i2c_write_byte(panel, 0x01, 0x10);
	mdelay(10); /* Wait for bias voltage to stabilize */

	/* 5. IT6151 reset sequence */
	dev_info(panel->dev, "IT6151 reset sequence\n");
	gpiod_set_value_cansleep(panel->it6151_reset_gpio, 0);
	mdelay(20);
	gpiod_set_value_cansleep(panel->it6151_reset_gpio, 1);
	mdelay(100); /* Wait for chip to boot */

	/* 6. DSI reset sequence */
	/* This is handled by the DSI host driver, no need to do it here */

	/* 7. Initialize IT6151 */
	it6151_chip_init(panel);

	dev_info(panel->dev, "IT6151 panel power-on sequence completed\n");

	return 0;
}


/**
 * it6151_power_off - Power off the IT6151 panel
 * @panel: IT6151 panel structure
 *
 * This function powers off the IT6151 panel following the same sequence
 * as the original LK driver lcm_suspend() function. It handles GPIO
 * control and regulator shutdown in the proper order.
 *
 * Returns: 0 on success, negative error code on failure
 */
static int it6151_power_off(struct it6151_panel *panel)
{
	dev_info(panel->dev, "Starting IT6151 panel power-off sequence\n");

	/* 1. Put IT6151 chip in reset state */
	dev_info(panel->dev, "Putting IT6151 into reset state\n");
	gpiod_set_value_cansleep(panel->it6151_reset_gpio, 0);
	mdelay(10);

	/* 2. Disable OCP2131 bias chip */
	dev_info(panel->dev, "Disabling OCP2131 bias chip\n");
	ocp2131_i2c_write_byte(panel, 0x00, 0x00);
	mdelay(1);
	ocp2131_i2c_write_byte(panel, 0x01, 0x00);
	mdelay(5);

	/* 3. Disable bias voltage */
	dev_info(panel->dev, "Disabling bias voltage\n");
	gpiod_set_value_cansleep(panel->lcd_bias2_gpio, 0);
	mdelay(5);
	gpiod_set_value_cansleep(panel->lcd_bias1_gpio, 0);
	mdelay(10);

	/* 4. Disable 1.2V power */
	dev_info(panel->dev, "Disabling 1.2V power\n");
	gpiod_set_value_cansleep(panel->lcd_pwr_gpio, 0);
	mdelay(20);

	/* 5. Set all control GPIOs to low */
	dev_info(panel->dev, "Setting all control GPIOs to low\n");
	gpiod_set_value_cansleep(panel->it6151_stb_gpio, 1);
	gpiod_set_value_cansleep(panel->it6151_enpsr_gpio, 1);
	gpiod_set_value_cansleep(panel->lcd_test_gpio, 0);

	dev_info(panel->dev, "IT6151 panel power-off sequence completed\n");

	return 0;
}


/**
 * it6151_esd_recover - ESD recovery function
 * @panel: IT6151 panel structure
 *
 * This function performs ESD recovery by re-initializing the DPTX interface.
 * It follows the same logic as the original LK driver IT6151_ESD_Recover() function.
 *
 * Returns: 0 on success, negative error code on failure
 */
static int it6151_esd_recover(struct it6151_panel *panel)
{
	u8 reg_val;
	int ret;

	dev_info(panel->dev, "Performing IT6151 ESD recovery\n");

	/* Read status register as in original LK code */
	ret = it6151_dp_read(panel, 0x06, &reg_val);
	if (ret < 0) {
		dev_err(panel->dev,
			"Failed to read status register during ESD recovery: %d\n",
			ret);
		/* Continue with recovery even if read fails */
	} else {
		dev_dbg(panel->dev,
			"ESD recovery status register 0x06: 0x%02x\n", reg_val);
	}

	/* Re-initialize DPTX interface */
	ret = it6151_dptx_init(panel);
	if (ret < 0) {
		dev_err(panel->dev,
			"ESD recovery DPTX initialization failed: %d\n", ret);
		return ret;
	}

	dev_info(panel->dev, "IT6151 ESD recovery completed successfully\n");

	return 0;
}

/* DRM panel interface functions */
static int it6151_disable(struct drm_panel *panel)
{
	struct it6151_panel *it6151 = to_it6151_panel(panel);
	int ret;

	if (!it6151->enabled)
		return 0;

	dev_info(it6151->dev, "Disabling panel\n");
    
	it6151->enabled = false;
    return 0;
    
	/* Disable video output by stopping DPTX interface */
	if (it6151->chip_type == IT6151_CHIP_DP_INTERFACE) {
		/* Disable DPTX video output */
		ret = it6151_dp_write(it6151, 0x17, 0x00);
		if (ret < 0) {
			dev_err(it6151->dev,
				"Failed to disable DPTX video output: %d\n",
				ret);
			/* Continue with disable even if this fails */
		}

		/* Clear video stable interrupt */
		ret = it6151_dp_write(it6151, 0x06, 0xFF);
		if (ret < 0) {
			dev_err(it6151->dev,
				"Failed to clear interrupts during disable: %d\n",
				ret);
		}
	}

	/* Disable MIPI interface video processing */
	ret = it6151_mipi_write(it6151, 0x05, 0x33);
	if (ret < 0) {
		dev_err(it6151->dev, "Failed to disable MIPI interface: %d\n",
			ret);
		/* Continue with disable even if this fails */
	}

	it6151->enabled = false;

	dev_info(it6151->dev, "Panel disabled successfully\n");

	return 0;
}

static int it6151_unprepare(struct drm_panel *panel)
{
	struct it6151_panel *it6151 = to_it6151_panel(panel);
	int ret;

	if (!it6151->prepared)
		return 0;

	dev_info(it6151->dev, "Unpreparing panel\n");
    
	it6151->prepared = false;
	return 0;
    
	/* Power off the panel following LK driver lcm_suspend() logic */
	ret = it6151_power_off(it6151);
	if (ret < 0) {
		dev_err(it6151->dev, "Failed to power off panel: %d\n", ret);
		/* Continue with unprepare even if power off fails */
	}

	it6151->prepared = false;

	dev_info(it6151->dev, "Panel unprepared successfully\n");

	return 0;
}

static int it6151_prepare(struct drm_panel *panel)
{
	struct it6151_panel *it6151 = to_it6151_panel(panel);
	int ret;

	if (it6151->prepared)
		return 0;

	dev_info(it6151->dev, "Preparing panel\n");

	it6151->prepared = true;
	return 0;
    
	/* Power on the panel following LK driver lcm_init() logic */
	ret = it6151_power_on(it6151);
	if (ret < 0) {
		dev_err(it6151->dev, "Failed to power on panel: %d\n", ret);
		return ret;
	}

	/* Initialize IT6151 chip with converted initialization sequences */
	ret = it6151_chip_init(it6151);
	if (ret < 0) {
		dev_err(it6151->dev, "IT6151 chip initialization failed: %d\n",
			ret);
		/* Power off on initialization failure */
		it6151_power_off(it6151);
		return ret;
	}

	it6151->prepared = true;

	dev_info(it6151->dev, "Panel prepared successfully\n");

	return 0;
}

static int it6151_enable(struct drm_panel *panel)
{
	struct it6151_panel *it6151 = to_it6151_panel(panel);
	int ret;

	if (it6151->enabled)
		return 0;

	dev_info(it6151->dev, "Enabling panel\n");

	/* Enable video output by starting DPTX interface */
	if (it6151->chip_type == IT6151_CHIP_DP_INTERFACE) {
		/* Enable DPTX video output */
		ret = it6151_dp_write(it6151, 0x17, 0x01);
		if (ret < 0) {
			dev_err(it6151->dev,
				"Failed to enable DPTX video output: %d\n",
				ret);
			return ret;
		}

		/* Wait for video stable as in original LK code */
		mdelay(5);

		/* Enable video stable interrupt */
		ret = it6151_dp_write(it6151, 0x09, INT_MASK);
		if (ret < 0) {
			dev_err(it6151->dev,
				"Failed to enable video stable interrupt: %d\n",
				ret);
			/* Continue with enable even if this fails */
		}
	}

	/* Enable MIPI interface video processing */
	ret = it6151_mipi_write(it6151, 0x05, 0x00);
	if (ret < 0) {
		dev_err(it6151->dev, "Failed to enable MIPI interface: %d\n",
			ret);
		return ret;
	}

	/* Additional MIPI enable configuration for video output */
	ret = it6151_mipi_write(it6151, 0x84, 0x8f);
	if (ret < 0) {
		dev_err(it6151->dev,
			"Failed to configure MIPI video output: %d\n", ret);
		/* Continue with enable even if this fails */
	}

	it6151->enabled = true;

	dev_info(it6151->dev, "Panel enabled successfully\n");

	return 0;
}


#define HFP (78)
#define HSA (50)
#define HBP (88)
#define VFP (36)
#define VSA (25)
#define VBP (44)
#define VAC (1200)
#define HAC (1920)
static u32 fake_heigh = 1200;
static u32 fake_width = 1920;
static bool need_fake_resolution;

static struct drm_display_mode default_mode = {
	.clock = 145500,
	.hdisplay = HAC,
	.hsync_start = HAC + HFP,
	.hsync_end = HAC + HFP + HSA,
	.htotal = HAC + HFP + HSA + HBP,
	.vdisplay = VAC,
	.vsync_start = VAC + VFP,
	.vsync_end = VAC + VFP + VSA,
	.vtotal = VAC + VFP + VSA + VBP,
	.vrefresh = 50,
};

static int it6151_get_modes(struct drm_panel *panel)
{
	struct it6151_panel *it6151 = to_it6151_panel(panel);
	struct drm_connector *connector = panel->connector;
	struct drm_device *drm = panel->drm;
	struct drm_display_mode *mode;

	dev_dbg(it6151->dev, "Getting display modes\n");

	/* Create 1920x1080 display mode */
#if 0
	mode = drm_mode_create(drm);
	if (!mode) {
		dev_err(it6151->dev, "Failed to create display mode\n");
		return 0;
	}
	/* Set timing parameters from LK driver lcm_get_params() function
	 * Resolution: 1920x1080@60Hz
	 * HFP=148, HSA=44, HBP=88, VFP=36, VSA=5, VBP=4
	 * Horizontal polarity: positive (MP_HPOL=1)
	 * Vertical polarity: negative (MP_VPOL=0)
	 */
	mode->clock = 148500; /* Pixel clock in kHz */
	mode->hdisplay = 1920;
	mode->hsync_start = mode->hdisplay + 148; /* HFP = 148 */
	mode->hsync_end = mode->hsync_start + 44; /* HSA = 44 */
	mode->htotal = mode->hsync_end + 88; /* HBP = 88 */
	mode->vdisplay = 1200;
	mode->vsync_start = mode->vdisplay + 36; /* VFP = 36 */
	mode->vsync_end = mode->vsync_start + 5; /* VSA = 5 */
	mode->vtotal = mode->vsync_end + 4; /* VBP = 4 */
	mode->vrefresh = 60;
#else
	mode = drm_mode_duplicate(panel->drm, &default_mode);
	if (!mode) {
		dev_err(panel->drm->dev, "failed to add mode %ux%ux@%u\n",
			default_mode.hdisplay, default_mode.vdisplay,
			default_mode.vrefresh);
		return -ENOMEM;
	}
#endif
	/* Set sync polarity flags to match LK driver configuration
	 * MP_HPOL=1 -> positive horizontal sync
	 * MP_VPOL=0 -> negative vertical sync
	 */
	mode->flags = DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_NVSYNC;

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_set_name(mode);
	drm_mode_probed_add(connector, mode);
	panel->connector->display_info.width_mm = 286;
	panel->connector->display_info.height_mm = 179;
	return 1;
}

static const struct drm_panel_funcs it6151_panel_funcs = {
	.disable = it6151_disable,
	.unprepare = it6151_unprepare,
	.prepare = it6151_prepare,
	.enable = it6151_enable,
	.get_modes = it6151_get_modes,
};

#if defined(CONFIG_MTK_PANEL_EXT)
/**
 * it6151_panel_ext_reset - MTK panel extension reset function
 * @panel: DRM panel structure
 * @on: Reset state (1 = reset active, 0 = reset inactive)
 *
 * This function provides MTK-specific reset control for the IT6151 panel.
 * It follows the same pattern as other MTK panel drivers.
 *
 * Returns: 0 on success, negative error code on failure
 */
static int it6151_panel_ext_reset(struct drm_panel *panel, int on)
{
	struct it6151_panel *it6151 = to_it6151_panel(panel);

	if (!it6151->reset_gpio) {
		dev_warn(it6151->dev,
			 "Reset GPIO not available for MTK reset\n");
		return 0;
	}
    
	return 0;
    
	gpiod_set_value_cansleep(it6151->reset_gpio, on);
	dev_dbg(it6151->dev, "MTK panel reset: %s\n",
		on ? "active" : "inactive");

	return 0;
}

/**
 * it6151_panel_ata_check - MTK panel ATA (Automated Test Application) check
 * @panel: DRM panel structure
 *
 * This function performs ATA check for the IT6151 panel by reading chip ID
 * registers and verifying the expected values.
 *
 * Returns: 1 if ATA check passes, 0 if it fails
 */
static int it6151_panel_ata_check(struct drm_panel *panel)
{
	struct it6151_panel *it6151 = to_it6151_panel(panel);
	u8 ven_id[2], dev_id[2];
	int ret;

	dev_info(it6151->dev, "Performing MTK ATA check\n");

	/* Read vendor ID from DP interface */
	ret = it6151_dp_read(it6151, 0x00, &ven_id[0]);
	if (ret < 0) {
		dev_err(it6151->dev,
			"ATA check: Failed to read vendor ID[0]\n");
		return 0;
	}

	ret = it6151_dp_read(it6151, 0x01, &ven_id[1]);
	if (ret < 0) {
		dev_err(it6151->dev,
			"ATA check: Failed to read vendor ID[1]\n");
		return 0;
	}

	/* Read device ID from DP interface */
	ret = it6151_dp_read(it6151, 0x02, &dev_id[0]);
	if (ret < 0) {
		dev_err(it6151->dev,
			"ATA check: Failed to read device ID[0]\n");
		return 0;
	}

	ret = it6151_dp_read(it6151, 0x03, &dev_id[1]);
	if (ret < 0) {
		dev_err(it6151->dev,
			"ATA check: Failed to read device ID[1]\n");
		return 0;
	}

	DDPINFO("IT6151 ATA read data: VenID=0x%02x,0x%02x DevID=0x%02x,0x%02x\n",
		ven_id[0], ven_id[1], dev_id[0], dev_id[1]);

	/* Check if this matches IT6151 DP interface signature */
	if (ven_id[0] == IT6151_VENDOR_ID_0 &&
	    ven_id[1] == IT6151_VENDOR_ID_1 &&
	    dev_id[0] == IT6151_DEVICE_ID_DP_0 &&
	    dev_id[1] == IT6151_DEVICE_ID_DP_1) {
		dev_info(it6151->dev, "MTK ATA check passed\n");
		return 1;
	}

	DDPINFO("IT6151 ATA expect: VenID=0x%02x,0x%02x DevID=0x%02x,0x%02x\n",
		IT6151_VENDOR_ID_0, IT6151_VENDOR_ID_1, IT6151_DEVICE_ID_DP_0,
		IT6151_DEVICE_ID_DP_1);

	dev_err(it6151->dev, "MTK ATA check failed\n");
	return 0;
}

/**
 * it6151_setbacklight_cmdq - MTK backlight control function
 * @dsi: DSI device pointer
 * @cb: Callback function for command queue
 * @handle: Command queue handle
 * @level: Backlight level (0-255)
 *
 * This function provides MTK-specific backlight control for the IT6151 panel.
 * Since IT6151 is a bridge chip, backlight control is typically handled by
 * the connected eDP panel or external backlight controller.
 *
 * Returns: 0 on success, negative error code on failure
 */
static int it6151_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle,
				    unsigned int level)
{
	/* IT6151 is a MIPI-to-eDP bridge, backlight control is typically
	 * handled by the connected eDP panel or external backlight controller.
	 * This function is provided for MTK framework compatibility.
	 */
	pr_info("IT6151: MTK backlight control level=%u (bridge chip - no direct control)\n",
		level);

	/* Return success as backlight control is not applicable for bridge chip */
	return 0;
}

/**
 * MTK panel extension parameters for IT6151
 * These parameters configure MTK-specific display pipeline settings
 */
static struct mtk_panel_params it6151_ext_params = {
	.pll_clk = 475,  /* PLL clock in MHz, matching 148.5MHz pixel clock */
	.vfp_low_power = 136,  /* VFP in low power mode */
	.cust_esd_check = 0,  /* Enable custom ESD check */
	.esd_check_enable = 0,  /* Enable ESD check */
	.ssc_disable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x00,  /* Read vendor ID register */
		.count = 1,
		.para_list[0] = IT6151_VENDOR_ID_0,  /* Expected vendor ID */
	},
	#if  0
	.lcm_esd_check_table[1] = {
		.cmd = 0x01,  /* Read vendor ID register */
		.count = 1,
		.para_list[0] = IT6151_VENDOR_ID_1,  /* Expected vendor ID */
	},
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.dsc_params = {
		.enable = 0,  /* DSC disabled for IT6151 bridge */
	},
	.data_rate = 1188,  /* Data rate in Mbps (148.5 * 4 lanes * 2) */
	.dyn_fps = {
		.switch_en = 0,  /* Dynamic FPS switching disabled */
		.vact_timing_fps = 60,  /* Vertical active timing FPS */
	},
	#endif /* #if 0 */
};

static int lcm_get_virtual_heigh(void)
{
	return VAC;
}

static int lcm_get_virtual_width(void)
{
	return HAC;
}

/**
 * MTK panel extension functions for IT6151
 * These functions provide MTK-specific panel control interfaces
 */
static struct mtk_panel_funcs it6151_ext_funcs = {
	.reset = it6151_panel_ext_reset,
	.set_backlight_cmdq = it6151_setbacklight_cmdq,
	.ata_check = it6151_panel_ata_check,
	.get_virtual_heigh = lcm_get_virtual_heigh,
	.get_virtual_width = lcm_get_virtual_width,
};
#endif

/**
 * it6151_setup_i2c_clients - Setup dual I2C clients for DP and MIPI interfaces
 * @panel: IT6151 panel structure
 *
 * Returns: 0 on success, negative error code on failure
 */
static int it6151_setup_i2c_clients(struct it6151_panel *panel)
{
	struct device *dev = panel->dev;
	struct i2c_adapter *adapter;
	struct i2c_board_info dp_info = {
		.type = "it6151-dp",
		.addr = IT6151_DP_I2C_ADDR,
	};
	struct i2c_board_info mipi_info = {
		.type = "it6151-mipi",
		.addr = IT6151_MIPI_I2C_ADDR,
	};
	struct i2c_board_info ocp2131_info = {
		.type = "ocp2131",
		.addr = OCP2131_I2C_ADDR,
	};

	/* Get I2C adapter - typically from device tree or platform data */
	adapter = i2c_get_adapter(6); /* IT6151_BUSNUM from original LK code */
	if (!adapter) {
		dev_err(dev, "Failed to get I2C adapter\n");
		return -ENODEV;
	}

	/* Create DP I2C client (0x5C) */
	panel->dp_client = i2c_new_device(adapter, &dp_info);
	if (!panel->dp_client) {
		dev_err(dev, "Failed to create DP I2C client\n");
		i2c_put_adapter(adapter);
		return -ENODEV;
	}

	/* Create MIPI I2C client (0x6C) */
	panel->mipi_client = i2c_new_device(adapter, &mipi_info);
	if (!panel->dp_client) {
		dev_err(dev, "Failed to create MIPI I2C client\n");
		i2c_put_adapter(adapter);
		return -ENODEV;
	}

	/* Create OCP2131 I2C client (0x3E) */
	panel->ocp2131_client = i2c_new_device(adapter, &ocp2131_info);
	if (!panel->ocp2131_client) {
		dev_err(dev, "Failed to create OCP2131 I2C client\n");
		i2c_unregister_device(panel->dp_client);
		i2c_put_adapter(adapter);
		return -ENODEV;
	}

	i2c_put_adapter(adapter);

	dev_info(dev, "I2C clients setup: DP=0x%02x, OCP2131=0x%02x\n",
		 panel->dp_client->addr, panel->ocp2131_client->addr);

	return 0;
}


/**
 * it6151_cleanup_i2c_clients - Cleanup I2C clients
 * @panel: IT6151 panel structure
 */
static void it6151_cleanup_i2c_clients(struct it6151_panel *panel)
{
	if (panel->ocp2131_client) {
		i2c_unregister_device(panel->ocp2131_client);
		panel->ocp2131_client = NULL;
	}

	if (panel->mipi_client) {
		i2c_unregister_device(panel->mipi_client);
		panel->mipi_client = NULL;
	}

	if (panel->dp_client) {
		i2c_unregister_device(panel->dp_client);
		panel->dp_client = NULL;
	}
}

/**
 * it6151_detect_chip - Detect and identify IT6151 chip
 * @panel: IT6151 panel structure
 *
 * This function detects the IT6151 chip by reading vendor ID, device ID,
 * and revision ID from both DP and MIPI I2C interfaces. It follows the
 * same detection logic as the original LK driver.
 *
 * The IT6151 chip has two I2C interfaces:
 * - DP interface (0x5C): VenID=0x54,0x49, DevID=0x51,0x61
 * - MIPI interface (0x6C): VenID=0x54,0x49, DevID=0x21,0x61
 *
 * Returns: IT6151_CHIP_DP_INTERFACE, IT6151_CHIP_MIPI_INTERFACE, or IT6151_CHIP_UNKNOWN
 */
static int it6151_detect_chip(struct it6151_panel *panel)
{
	u8 ven_id[2], dev_id[2], rev_id;
	int ret;

	dev_info(panel->dev, "Detecting IT6151 chip...\n");

	/* Reset error state for detection */
	panel->error = 0;

	/* First, try to detect chip via DP interface (0x5C) */
	dev_dbg(panel->dev, "Trying DP interface detection (0x%02x)\n", IT6151_DP_I2C_ADDR);

	/* Read vendor ID from DP interface */
	ret = it6151_dp_read(panel, 0x00, &ven_id[0]);
	if (ret < 0) {
		dev_dbg(panel->dev, "DP interface vendor ID[0] read failed: %d\n", ret);
		goto try_mipi_interface;
	}

	ret = it6151_dp_read(panel, 0x01, &ven_id[1]);
	if (ret < 0) {
		dev_dbg(panel->dev, "DP interface vendor ID[1] read failed: %d\n", ret);
		goto try_mipi_interface;
	}

	/* Read device ID from DP interface */
	ret = it6151_dp_read(panel, 0x02, &dev_id[0]);
	if (ret < 0) {
		dev_dbg(panel->dev, "DP interface device ID[0] read failed: %d\n", ret);
		goto try_mipi_interface;
	}

	ret = it6151_dp_read(panel, 0x03, &dev_id[1]);
	if (ret < 0) {
		dev_dbg(panel->dev, "DP interface device ID[1] read failed: %d\n", ret);
		goto try_mipi_interface;
	}

	/* Read revision ID from DP interface */
	ret = it6151_dp_read(panel, 0x04, &rev_id);
	if (ret < 0) {
		dev_dbg(panel->dev, "DP interface revision ID read failed: %d\n", ret);
		goto try_mipi_interface;
	}

	dev_info(
		panel->dev,
		"DP interface IDs: VenID=0x%02x,0x%02x DevID=0x%02x,0x%02x RevID=0x%02x\n",
		ven_id[0], ven_id[1], dev_id[0], dev_id[1], rev_id);

	/* Check if this matches IT6151 DP interface signature */
	if (ven_id[0] == IT6151_VENDOR_ID_0 &&
	    ven_id[1] == IT6151_VENDOR_ID_1 &&
	    dev_id[0] == IT6151_DEVICE_ID_DP_0 &&
	    dev_id[1] == IT6151_DEVICE_ID_DP_1) {
		dev_info( panel->dev, "IT6151 chip detected via DP interface (RevID=0x%02x)\n", rev_id);

		/* Store revision for later use in initialization */
		if (rev_id == IT6151_REVISION_A1) {
			dev_info(panel->dev, "IT6151 revision A1 detected\n");
		} else {
			dev_info(panel->dev,
				 "IT6151 revision 0x%02x detected\n", rev_id);
		}

		return IT6151_CHIP_DP_INTERFACE;
	}

try_mipi_interface:
	/* Reset error state for MIPI interface detection */
	panel->error = 0;

	/* Try to detect chip via MIPI interface (0x6C) */
	dev_dbg(panel->dev, "Trying MIPI interface detection (0x%02x)\n", IT6151_MIPI_I2C_ADDR);

	/* Read vendor ID from MIPI interface */
	ret = it6151_mipi_read(panel, 0x00, &ven_id[0]);
	if (ret < 0) {
		dev_dbg(panel->dev,
			"MIPI interface vendor ID[0] read failed: %d\n", ret);
		goto detection_failed;
	}

	ret = it6151_mipi_read(panel, 0x01, &ven_id[1]);
	if (ret < 0) {
		dev_dbg(panel->dev,
			"MIPI interface vendor ID[1] read failed: %d\n", ret);
		goto detection_failed;
	}

	/* Read device ID from MIPI interface */
	ret = it6151_mipi_read(panel, 0x02, &dev_id[0]);
	if (ret < 0) {
		dev_dbg(panel->dev,
			"MIPI interface device ID[0] read failed: %d\n", ret);
		goto detection_failed;
	}

	ret = it6151_mipi_read(panel, 0x03, &dev_id[1]);
	if (ret < 0) {
		dev_dbg(panel->dev,
			"MIPI interface device ID[1] read failed: %d\n", ret);
		goto detection_failed;
	}

	/* Read revision ID from MIPI interface */
	ret = it6151_mipi_read(panel, 0x04, &rev_id);
	if (ret < 0) {
		dev_dbg(panel->dev,
			"MIPI interface revision ID read failed: %d\n", ret);
		goto detection_failed;
	}

	dev_info(
		panel->dev,
		"MIPI interface IDs: VenID=0x%02x,0x%02x DevID=0x%02x,0x%02x RevID=0x%02x\n",
		ven_id[0], ven_id[1], dev_id[0], dev_id[1], rev_id);

	/* Check if this matches IT6151 MIPI interface signature */
	if (ven_id[0] == IT6151_VENDOR_ID_0 &&
	    ven_id[1] == IT6151_VENDOR_ID_1 &&
	    dev_id[0] == IT6151_DEVICE_ID_MIPI_0 &&
	    dev_id[1] == IT6151_DEVICE_ID_MIPI_1) {
		dev_info(
			panel->dev,
			"IT6151 chip detected via MIPI interface (RevID=0x%02x)\n",
			rev_id);

		/* Store revision for later use in initialization */
		if (rev_id == IT6151_REVISION_A1) {
			dev_info(panel->dev, "IT6151 revision A1 detected\n");
		} else {
			dev_info(panel->dev,
				 "IT6151 revision 0x%02x detected\n", rev_id);
		}

		return IT6151_CHIP_MIPI_INTERFACE;
	}

detection_failed:
	dev_err(panel->dev, "IT6151 chip detection failed\n");
	dev_err(panel->dev, "Expected: VenID=0x%02x,0x%02x\n",
		IT6151_VENDOR_ID_0, IT6151_VENDOR_ID_1);
	dev_err(panel->dev,
		"Expected DP DevID=0x%02x,0x%02x or MIPI DevID=0x%02x,0x%02x\n",
		IT6151_DEVICE_ID_DP_0, IT6151_DEVICE_ID_DP_1,
		IT6151_DEVICE_ID_MIPI_0, IT6151_DEVICE_ID_MIPI_1);

	/* Reset error state */
	panel->error = 0;

	return IT6151_CHIP_UNKNOWN;
}

/**
 * it6151_get_gpio_resources - Get GPIO resources from device tree
 * @panel: IT6151 panel structure
 *
 * This function obtains GPIO descriptors for reset, LED enable, and STB enable
 * controls from the device tree. It follows the GPIO naming convention from
 * the LK driver: GPIO64 (reset), GPIO17 (LED enable), GPIO93 (STB enable).
 *
 * Returns: 0 on success, negative error code on failure
 */
static int it6151_get_gpio_resources(struct it6151_panel *panel)
{
	struct device *dev = panel->dev;

	dev_dbg(dev, "Getting GPIO resources from device tree\n");

	panel->it6151_reset_gpio = devm_gpiod_get_optional(dev, GPIO_IT6151_RST_EN, GPIOD_OUT_HIGH);
	panel->it6151_stb_gpio = devm_gpiod_get_optional(dev, GPIO_IT6151_STB_EN, GPIOD_OUT_LOW);
	panel->it6151_enpsr_gpio = devm_gpiod_get_optional(dev, GPIO_IT6151_ENPSR_EN, GPIOD_OUT_LOW);
	panel->it6151_int_gpio = devm_gpiod_get_optional(dev, GPIO_IT6151_INT, GPIOD_IN);
	panel->lcd_pwr_gpio = devm_gpiod_get_optional(dev, GPIO_LCD_PWR_EN, GPIOD_OUT_HIGH);
	panel->lcd_bias1_gpio = devm_gpiod_get_optional(dev, GPIO_LCD_BIAS_EN1, GPIOD_OUT_HIGH);
	panel->lcd_bias2_gpio = devm_gpiod_get_optional(dev, GPIO_LCD_BIAS_EN2, GPIOD_OUT_HIGH);
	panel->lcd_test_gpio = devm_gpiod_get_optional(dev, GPIO_LCD_TEST_EN, GPIOD_OUT_LOW);

	return 0;
}

/**
 * it6151_get_regulator_resources - Get regulator resources from device tree
 * @panel: IT6151 panel structure
 *
 * This function obtains the VGP3 1.2V regulator from the device tree.
 * The regulator is used to power the IT6151 chip as per the LK driver logic.
 *
 * Returns: 0 on success, negative error code on failure
 */
static int it6151_get_regulator_resources(struct it6151_panel *panel)
{
	struct device *dev = panel->dev;

	/* VGP3 1.2V regulator support DISABLED for debugging */
	dev_info(dev, "Regulator resources DISABLED for debugging\n");
	return 0;
}

/**
 * it6151_test_i2c_communication - Test I2C communication with both interfaces
 * @panel: IT6151 panel structure
 *
 * This function performs basic I2C communication tests to verify that both
 * DP and MIPI I2C clients are working correctly. It attempts to read from
 * known registers that should be accessible and performs a write/read test.
 *
 * Returns: 0 on success, negative error code on failure
 */
static int it6151_test_i2c_communication(struct it6151_panel *panel)
{
	u8 dp_test_val, mipi_test_val;
	u8 dp_readback, mipi_readback;
	int ret;

	dev_info(panel->dev, "Testing I2C communication...\n");

	/* Test DP I2C interface - try to read vendor ID registers */
	ret = it6151_dp_read(panel, 0x00, &dp_test_val);
	if (ret < 0) {
		dev_err(panel->dev, "DP I2C read test failed: %d\n", ret);
		return ret;
	}

	/* Test MIPI I2C interface - try to read vendor ID registers */
	ret = it6151_mipi_read(panel, 0x00, &mipi_test_val);
	if (ret < 0) {
		dev_err(panel->dev, "MIPI I2C read test failed: %d\n", ret);
		return ret;
	}

	/* Test DP I2C write/read cycle with a safe register (0x09 - interrupt mask) */
	ret = it6151_dp_write(panel, 0x09, 0xFF);
	if (ret < 0) {
		dev_err(panel->dev, "DP I2C write test failed: %d\n", ret);
		return ret;
	}

	ret = it6151_dp_read(panel, 0x09, &dp_readback);
	if (ret < 0) {
		dev_err(panel->dev, "DP I2C write/read test failed: %d\n", ret);
		return ret;
	}

	/* Test MIPI I2C write/read cycle with a safe register */
	ret = it6151_mipi_write(panel, 0x05, 0x00);
	if (ret < 0) {
		dev_err(panel->dev, "MIPI I2C write test failed: %d\n", ret);
		return ret;
	}

	ret = it6151_mipi_read(panel, 0x05, &mipi_readback);
	if (ret < 0) {
		dev_err(panel->dev, "MIPI I2C write/read test failed: %d\n",
			ret);
		return ret;
	}

	dev_info(panel->dev, "  DP (0x%02x): reg[0x00]=0x%02x, reg[0x09]=0x%02x\n",
		 IT6151_DP_I2C_ADDR, dp_test_val, dp_readback);
	dev_info(panel->dev, "  MIPI (0x%02x): reg[0x00]=0x%02x, reg[0x05]=0x%02x\n",
		 IT6151_MIPI_I2C_ADDR, mipi_test_val, mipi_readback);

	/* Reset error state after successful test */
	panel->error = 0;

	return 0;
}

/* MIPI DSI driver functions */
static int it6151_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct it6151_panel *panel;
	int ret;
    
	struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;

	dsi_node = of_get_parent(dev->of_node);
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);
		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			if (!remote_node) {
				pr_info("No panel connected,skip probe lcm\n");
				return -ENODEV;
			}
			pr_info("device node name:%s\n", remote_node->name);
		}
	}
	if (remote_node != dev->of_node) {
		pr_info("%s+ skip probe due to not current lcm\n", __func__);
		return -ENODEV;
	}
    
	dev_info(dev, "IT6151 panel probe starting\n");

	/* Allocate panel structure */
	panel = devm_kzalloc(dev, sizeof(*panel), GFP_KERNEL);
	if (!panel) {
		dev_err(dev, "Failed to allocate panel structure\n");
		return -ENOMEM;
	}

	panel->dev = dev;
	panel->error = 0;
	panel->prepared = false;
	panel->enabled = false;
	panel->chip_type = IT6151_CHIP_UNKNOWN;
	panel->revision = 0;

	dev_dbg(dev, "Panel structure allocated and initialized\n");

	/* Get GPIO resources from device tree first */
	dev_dbg(dev, "Getting GPIO resources...\n");
	ret = it6151_get_gpio_resources(panel);
	if (ret < 0) {
		dev_err(dev, "Failed to get GPIO resources: %d\n", ret);
		return ret;
	}
	dev_info(dev, "GPIO resources obtained successfully\n");

	/* Initialize dual I2C clients for DP (0x5C) and MIPI (0x6C) interfaces */
	dev_dbg(dev, "Setting up I2C clients...\n");
	ret = it6151_setup_i2c_clients(panel);
	if (ret < 0) {
		dev_err(dev, "Failed to setup I2C clients: %d\n", ret);
		return ret;
	}
	dev_info(dev, "I2C clients setup completed successfully\n");

	/* Test I2C communication to verify both interfaces are working */
	dev_dbg(dev, "Testing I2C communication...\n");
	ret = it6151_test_i2c_communication(panel);
	if (ret < 0) {
		dev_warn( dev, "I2C communication test failed, but continuing probe: %d\n", ret);
		/* Reset error state to allow probe to continue */
		panel->error = 0;
	} else {
		dev_info(dev, "I2C communication test passed\n");
	}

	/* Detect and identify IT6151 chip */
	dev_dbg(dev, "Detecting IT6151 chip...\n");
	panel->chip_type = it6151_detect_chip(panel);
	if (panel->chip_type == IT6151_CHIP_UNKNOWN) {
		dev_err(dev, "IT6151 chip detection failed - unsupported or missing chip\n");
		ret = -ENODEV;
		goto cleanup_i2c;
	}

	/* Store revision ID for later use in initialization */
	dev_dbg(dev, "Reading chip revision...\n");
	if (panel->chip_type == IT6151_CHIP_DP_INTERFACE) {
		ret = it6151_dp_read(panel, 0x04, &panel->revision);
		if (ret < 0) {
			dev_warn(
				dev,
				"Failed to read revision from DP interface: %d\n",
				ret);
			panel->revision = 0;
			panel->error = 0; /* Reset error state */
		}
	} else if (panel->chip_type == IT6151_CHIP_MIPI_INTERFACE) {
		ret = it6151_mipi_read(panel, 0x04, &panel->revision);
		if (ret < 0) {
			dev_warn(
				dev,
				"Failed to read revision from MIPI interface: %d\n",
				ret);
			panel->revision = 0;
			panel->error = 0; /* Reset error state */
		}
	}

	dev_info(
		dev,
		"IT6151 chip successfully detected: type=%s, revision=0x%02x\n",
		panel->chip_type == IT6151_CHIP_DP_INTERFACE ? "DP" : "MIPI",
		panel->revision);

	/* Get regulator resources from device tree */
	dev_dbg(dev, "Getting regulator resources...\n");
	ret = it6151_get_regulator_resources(panel);
	if (ret < 0) {
		dev_err(dev, "Failed to get regulator resources: %d\n", ret);
		goto cleanup_i2c;
	}
	dev_info(dev, "Regulator resources obtained successfully\n");

	/* Initialize DRM panel structure */
	dev_dbg(dev, "Initializing DRM panel...\n");
	drm_panel_init(&panel->panel);
	panel->panel.dev = dev;
	panel->panel.funcs = &it6151_panel_funcs;

	ret = drm_panel_add(&panel->panel);
	if (ret < 0) {
		dev_err(dev, "Failed to add DRM panel: %d\n", ret);
		goto cleanup_i2c;
	}
	dev_info(dev, "DRM panel added successfully\n");

	mipi_dsi_set_drvdata(dsi, panel);

	/* Configure DSI parameters */
	dev_dbg(dev, "Configuring DSI parameters...\n");
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE
			 | MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_EOT_PACKET;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to attach to DSI host: %d\n", ret);
		goto cleanup_panel;
	}
	dev_info(dev, "DSI attachment completed successfully\n");

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_handle_reg(&panel->panel);
	/* Create MTK panel extension for MediaTek-specific features */
	dev_dbg(dev, "Creating MTK panel extension...\n");
    ret = mtk_panel_ext_create(dev, &it6151_ext_params, &it6151_ext_funcs, &panel->panel);
	if (ret < 0) {
		dev_err(dev, "Failed to create MTK panel extension: %d\n", ret);
		goto cleanup_dsi;
	}
	dev_info(dev, "MTK panel extension created successfully\n");
#endif

	dev_info(dev, "IT6151 panel probe completed successfully\n");

	return 0;

#if defined(CONFIG_MTK_PANEL_EXT)
cleanup_dsi:
	mipi_dsi_detach(dsi);
#endif

cleanup_panel:
	drm_panel_remove(&panel->panel);
cleanup_i2c:
	it6151_cleanup_i2c_clients(panel);
	dev_err(dev, "IT6151 panel probe failed with error: %d\n", ret);
	return ret;
}

static int it6151_remove(struct mipi_dsi_device *dsi)
{
	struct it6151_panel *panel = mipi_dsi_get_drvdata(dsi);

	dev_info(&dsi->dev, "IT6151 panel remove\n");

	mipi_dsi_detach(dsi);
	drm_panel_remove(&panel->panel);
	it6151_cleanup_i2c_clients(panel);

	return 0;
}

static const struct of_device_id it6151_of_match[] = {
	{
		.compatible = "boe,it6151_wuxga_edp_dsi_video",
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, it6151_of_match);

static struct mipi_dsi_driver it6151_driver = {
	.probe = it6151_probe,
	.remove = it6151_remove,
	.driver = {
		.name = "panel-ite6151-edp",
		.of_match_table = it6151_of_match,
	},
};
module_mipi_dsi_driver(it6151_driver);

MODULE_AUTHOR("Eli(oywj321@gmail.com)");
MODULE_DESCRIPTION("IT6151 MIPI-to-eDP bridge panel driver");
MODULE_LICENSE("GPL v2");
