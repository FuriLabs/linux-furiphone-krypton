#
# Makefile for nfc devices
#

obj-y    += pn553_i2c.o
pn553_i2c-objs                  := cold_reset.o pn553.o

ccflags-$(CONFIG_NFC_PN553_DEVICES) := -DDEBUG

