# SPDX-License-Identifier: GPL-2.0
# Copyright (c) 2021 Mediatek Inc.

CFG_FM_CHIP := mt6635

define build_kernel_modules
    $(MAKE) -C $(KERNEL_SRC) M=$(M)/Build/$(1) modules $(KBUILD_OPTIONS) CFG_FM_PLAT=$(1) CFG_BUILD_CONNAC2=$(2) CFG_FM_CHIP_ID=$(3) CFG_FM_CHIP=$(4) KBUILD_EXTRA_SYMBOLS="$(5)"
endef

define install_kernel_modules
   $(MAKE) M=$(M)/Build/$(1) -C $(KERNEL_SRC) modules_install
endef

define clean_kernel_modules
   $(MAKE) -C $(KERNEL_SRC) M=$(M)/Build/$(1) clean
endef

all:
ifdef CFG_FM_CHIP_ID
	$(call build_kernel_modules,$(CFG_FM_PLAT),$(CFG_BUILD_CONNAC2),$(CFG_FM_CHIP_ID),$(CFG_FM_CHIP))
else
	$(call build_kernel_modules,connac2x,true,,mt6635)
	$(call build_kernel_modules,mt6631_6635,false)
	$(call build_kernel_modules,mt6631,false,,mt6631)
	$(call build_kernel_modules,mt6635,false,,mt6635)
	# legacy chip
	$(call build_kernel_modules,soc,false,,mt6580)
	$(call build_kernel_modules,mt6625,false,,mt6625)
	$(call build_kernel_modules,mt6627,false,,mt6627)
	$(call build_kernel_modules,mt6630,false,,mt6630)
endif

modules_install:
ifdef CFG_FM_CHIP_ID
	$(call install_kernel_modules,$(CFG_FM_PLAT),$(CFG_BUILD_CONNAC2),$(CFG_FM_CHIP_ID),$(CFG_FM_CHIP))
else
	$(call install_kernel_modules,connac2x,true,,mt6635)
	$(call install_kernel_modules,mt6631_6635,false)
	$(call install_kernel_modules,mt6631,false,,mt6631)
	$(call install_kernel_modules,mt6635,false,,mt6635)
	# legacy chip
	$(call install_kernel_modules,soc,false,,mt6580)
	$(call install_kernel_modules,mt6625,false,,mt6625)
	$(call install_kernel_modules,mt6627,false,,mt6627)
	$(call install_kernel_modules,mt6630,false,,mt6630)
endif

clean:
ifdef CFG_FM_CHIP_ID
	$(call clean_kernel_modules,$(CFG_FM_PLAT),$(CFG_BUILD_CONNAC2),$(CFG_FM_CHIP_ID),$(CFG_FM_CHIP))
else
	$(call clean_kernel_modules,connac2x,true,,mt6635)
	$(call clean_kernel_modules,mt6631_6635,false)
	$(call clean_kernel_modules,mt6631,false,,mt6631)
	$(call clean_kernel_modules,mt6635,false,,mt6635)
	# legacy chip
	$(call clean_kernel_modules,soc,false,,mt6580)
	$(call clean_kernel_modules,mt6625,false,,mt6625)
	$(call clean_kernel_modules,mt6627,false,,mt6627)
	$(call clean_kernel_modules,mt6630,false,,mt6630)
endif
