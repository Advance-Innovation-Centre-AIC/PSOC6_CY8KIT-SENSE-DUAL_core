#
# This file is generated by ModusToolbox during the 'make getlibs' operation
# Any edits to this file will be lost the next time the library manager is run or
# the next time 'make getlibs' is run.
#
# List of local libraries


# Path to the current BSP
SEARCH_TARGET_APP_CY8CKIT-062S2-43012=../bsps/TARGET_APP_CY8CKIT-062S2-43012

# The search paths for the included middleware
SEARCH_sensor-xensiv-dps3xx=../../mtb_shared/sensor-xensiv-dps3xx/release-v1.0.0
SEARCH_cat1cm0p=../../mtb_shared/cat1cm0p/release-v1.0.0
SEARCH_cmsis=../../mtb_shared/cmsis/release-v5.8.0
SEARCH_core-lib=../../mtb_shared/core-lib/release-v1.3.1
SEARCH_core-make=../../mtb_shared/core-make/release-v3.0.3
SEARCH_mtb-hal-cat1=../../mtb_shared/mtb-hal-cat1/release-v2.3.0
SEARCH_mtb-pdl-cat1=../../mtb_shared/mtb-pdl-cat1/release-v3.3.0
SEARCH_recipe-make-cat1a=../../mtb_shared/recipe-make-cat1a/release-v2.0.0

# Search libraries added to build
SEARCH_MTB_MK+=$(SEARCH_sensor-xensiv-dps3xx)
SEARCH_MTB_MK+=$(SEARCH_cat1cm0p)
SEARCH_MTB_MK+=$(SEARCH_cmsis)
SEARCH_MTB_MK+=$(SEARCH_core-lib)
SEARCH_MTB_MK+=$(SEARCH_core-make)
SEARCH_MTB_MK+=$(SEARCH_mtb-hal-cat1)
SEARCH_MTB_MK+=$(SEARCH_mtb-pdl-cat1)
SEARCH_MTB_MK+=$(SEARCH_recipe-make-cat1a)

-include $(CY_INTERNAL_APP_PATH)/importedbsp.mk
COMPONENTS += MW_SENSOR_XENSIV_DPS3XX
COMPONENTS += MW_CAT1CM0P
COMPONENTS += MW_CMSIS
COMPONENTS += MW_CORE_LIB
COMPONENTS += MW_CORE_MAKE
COMPONENTS += MW_MTB_HAL_CAT1
COMPONENTS += MW_MTB_PDL_CAT1
COMPONENTS += MW_RECIPE_MAKE_CAT1A

# Register map file
DEVICE_CY8C624ABZI-S2D44_SVD=$(SEARCH_mtb-pdl-cat1)/devices/COMPONENT_CAT1A/svd/psoc6_02.svd


#
# generate make targets for the graphical editors that are specific to this project
#

capsense-configurator:
	$(CY_TOOL_mtblaunch_EXE_ABS) --project . --short-name capsense-configurator
.PHONY: capsense-configurator

capsense-tuner:
	$(CY_TOOL_mtblaunch_EXE_ABS) --project . --short-name capsense-tuner
.PHONY: capsense-tuner

config_usbdev:
	$(CY_TOOL_mtblaunch_EXE_ABS) --project . --short-name usbdev-configurator
.PHONY: config_usbdev

usbdev-configurator:
	$(CY_TOOL_mtblaunch_EXE_ABS) --project . --short-name usbdev-configurator
.PHONY: usbdev-configurator

smartio-configurator:
	$(CY_TOOL_mtblaunch_EXE_ABS) --project . --short-name smartio-configurator
.PHONY: smartio-configurator

modlibs:
	$(CY_TOOL_mtblaunch_EXE_ABS) --project . --short-name library-manager
.PHONY: modlibs

library-manager:
	$(CY_TOOL_mtblaunch_EXE_ABS) --project . --short-name library-manager
.PHONY: library-manager

qspi-configurator:
	$(CY_TOOL_mtblaunch_EXE_ABS) --project . --short-name qspi-configurator
.PHONY: qspi-configurator

config_bt:
	$(CY_TOOL_mtblaunch_EXE_ABS) --project . --short-name bt-configurator
.PHONY: config_bt

bt-configurator:
	$(CY_TOOL_mtblaunch_EXE_ABS) --project . --short-name bt-configurator
.PHONY: bt-configurator

seglcd-configurator:
	$(CY_TOOL_mtblaunch_EXE_ABS) --project . --short-name seglcd-configurator
.PHONY: seglcd-configurator

config:
	$(CY_TOOL_mtblaunch_EXE_ABS) --project . --short-name device-configurator
.PHONY: config

device-configurator:
	$(CY_TOOL_mtblaunch_EXE_ABS) --project . --short-name device-configurator
.PHONY: device-configurator

bsp-assistant:
	$(CY_TOOL_mtblaunch_EXE_ABS) --project . --short-name bsp-assistant
.PHONY: bsp-assistant

config_lin:
	$(CY_TOOL_mtblaunch_EXE_ABS) --project . --short-name lin-configurator
.PHONY: config_lin

lin-configurator:
	$(CY_TOOL_mtblaunch_EXE_ABS) --project . --short-name lin-configurator
.PHONY: lin-configurator

