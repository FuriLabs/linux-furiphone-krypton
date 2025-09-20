# Makefile for the focaltech touchscreen drivers.


ccflags-$(CONFIG_DRM_MEDIATEK_V2) += -I$(srctree)/drivers/gpu/drm/mediatek/mediatek/

obj-$(CONFIG_TOUCHSCREEN_FTS_FT8057S) += focaltech_tp.o
focaltech_tp-y := focaltech_core.o \
		focaltech_ex_fun.o \
		focaltech_ex_mode.o \
		focaltech_gesture.o \
		focaltech_esdcheck.o \
		focaltech_point_report_check.o \
		focaltech_proximity.o


focaltech_tp-y += focaltech_flash_spi.o


focaltech_tp-y += focaltech_spi_v2.o


focaltech_tp-y += focaltech_debug.o

focaltech_tp-y += focaltech_hp.o


