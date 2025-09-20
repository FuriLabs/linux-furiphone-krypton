#
# Makefile for TMS devices
#

MODULE_NAME := tms_device_modules
obj-$(CONFIG_TMS_NFC_DEVICE) += $(MODULE_NAME).o
$(MODULE_NAME)-objs := tms_common.o

ccflags-y += \
    -DCONFIG_TMS_NFC_DEVICE \
    -DCONFIG_TMS_GUIDE_DEVICE \
    -DCONFIG_TMS_ESE_DEVICE

# TMS NFC driver
ifeq ($(CONFIG_TMS_NFC_DEVICE),y)
$(MODULE_NAME)-objs += nfc/nfc_common.o \
                       nfc/nfc_driver.o
endif
                       
# Guidev driver suport
ifeq ($(CONFIG_TMS_GUIDE_DEVICE),y)
$(MODULE_NAME)-objs += guidev/guide_driver.o
endif

# TMS eSE driver
ifeq ($(CONFIG_TMS_ESE_DEVICE),y)
$(MODULE_NAME)-objs += ese/ese_common.o \
                       ese/ese_driver.o
endif