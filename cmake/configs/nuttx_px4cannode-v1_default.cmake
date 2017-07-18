include(nuttx/px4_impl_nuttx)

add_definitions(
	-DPARAM_NO_ORB
	-DPARAM_NO_AUTOSAVE
	)

px4_nuttx_configure(HWCLASS m3 CONFIG nsh)

#
# UAVCAN boot loadable Module ID

set(uavcanblid_sw_version_major 0)
set(uavcanblid_sw_version_minor 1)

#
# Bring in common uavcan hardware identity definitions
#

include(configs/uavcan_board_ident/px4cannode-v1)

px4_nuttx_make_uavcan_bootloadable(BOARD ${BOARD}
 BIN ${CMAKE_CURRENT_BINARY_DIR}/src/firmware/nuttx/firmware_nuttx.bin
 HWNAME ${uavcanblid_name}
 HW_MAJOR ${uavcanblid_hw_version_major}
 HW_MINOR ${uavcanblid_hw_version_minor}
 SW_MAJOR ${uavcanblid_sw_version_major}
 SW_MINOR ${uavcanblid_sw_version_minor})

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/toolchains/Toolchain-arm-none-eabi.cmake)

set(config_module_list

	#
	# Board support modules
	#

	drivers/stm32
	drivers/led
	drivers/boards/px4cannode-v1

	#
	# System commands
	#
	systemcmds/reboot
	systemcmds/top
	systemcmds/config
	systemcmds/ver

	#
	# General system control
	#
	modules/uavcannode

	#
	# Library modules
	#
	modules/param
	modules/systemlib
	modules/uORB
	lib/version

	#
	# Libraries
	#
	# had to add for cmake, not sure why wasn't in original config
	platforms/nuttx
	platforms/common
	platforms/nuttx/px4_layer


)

set(config_extra_libs
	uavcan
	uavcan_stm32_driver
	)
