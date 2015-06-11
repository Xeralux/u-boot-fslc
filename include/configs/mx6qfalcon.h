/*
 * Copyright (C) 2013, 2015 Sensity Systems, Inc
 *
 * Configuration settings for the Sensity Systems Falcon board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __MX6QFALCON_CONFIG_H
#define __MX6QFALCON_CONFIG_H

#include <asm/arch/imx-regs.h>
#include <asm/imx-common/gpio.h>
#include <linux/sizes.h>
#include "mx6_common.h"

#define CONFIG_MX6
#define CONFIG_MACH_TYPE	3981
#define CONFIG_MXC_UART_BASE	UART1_BASE
#define CONFIG_CONSOLE_DEV		"ttymxc0"

#define CONFIG_MMCROOT_DEV		"/dev/mmcblk"
#define CONFIG_MMCROOT_DEVNUM		"0"
#define CONFIG_MMCROOT_PART		"2"
#define CONFIG_MMCROOT			CONFIG_MMCROOT_DEV CONFIG_MMCROOT_DEVNUM "p" CONFIG_MMCROOT_PART

#define CONFIG_SECURE_BOOT

#if defined(CONFIG_MX6Q)
#define CONFIG_DEFAULT_FDT_FILE	"imx6q-sensity_falcon.dtb"
#else
#error Unknown configuration for Sensity Falcon board
#endif

#define PHYS_SDRAM_SIZE		(2u * 1024 * 1024 * 1024)

#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG

#define CONFIG_SYS_GENERIC_BOARD

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(10 * SZ_1M)

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_BOARD_LATE_INIT
#define CONFIG_MXC_GPIO

#define CONFIG_MXC_UART

/* MMC Configs */
#define CONFIG_FSL_ESDHC
#define CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_ESDHC_ADDR      0

#define CONFIG_MMC
#define CONFIG_CMD_MMC
#define CONFIG_GENERIC_MMC
#define CONFIG_BOUNCE_BUFFER
#define CONFIG_CMD_EXT2
#define CONFIG_CMD_FAT
#define CONFIG_DOS_PARTITION

#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_MII
#define CONFIG_CMD_NET
#define CONFIG_FEC_MXC
#define CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RGMII
#define CONFIG_ETHPRIME			"FEC"
#define CONFIG_FEC_MXC_PHYADDR		3

#define CONFIG_PHYLIB
#define CONFIG_PHY_MICREL

/*  One-Time Programmable fuse support.  */
#define	CONFIG_MXC_OCOTP
#define	CONFIG_CMD_FUSE

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX              1
#define CONFIG_BAUDRATE                        115200

/* I2C */
#define CONFIG_CMD_I2C

/* PMIC */
#define CONFIG_POWER
#define CONFIG_POWER_I2C
#define CONFIG_POWER_LTC3676
#define CONFIG_POWER_LTC3676_I2C_ADDR 0x3c
#define CONFIG_POWER_LTC3676_I2C_BUS  0

/* Command definition */
#include <config_cmd_default.h>

#define CONFIG_CMD_BMODE
#define CONFIG_CMD_BOOTZ
#define CONFIG_CMD_SETEXPR
#undef CONFIG_CMD_IMLS

#ifdef CONFIG_MANUFACTURING
#define CONFIG_BOOTDELAY               5
#else
#define CONFIG_BOOTDELAY               0
#endif

#ifdef CONFIG_MANUFACTURING
#define CONFIG_BOOT_RETRY_TIME        -1
#define bootfailcheck__ "no"
#else
/* Command timeout, 5 minutes by default, min 10 sec if set */
#define CONFIG_BOOT_RETRY_TIME         300
#define bootfailcheck__ "yes"
#endif
#define CONFIG_BOOT_RETRY_TIME_MIN     10
#define CONFIG_RESET_TO_RETRY

#define CONFIG_LOADADDR                        0x12000000
#define CONFIG_SYS_TEXT_BASE           0x17800000

#define mkstring_(_x) mkstring__(_x)
#define mkstring__(_x) #_x

#ifdef CONFIG_DDRTEST
#define DDRTESTDEFS \
	"loadtest=fatload mmc 0:1 0x907000 ddr-stress-test-mx6dq.bin\0" \
	"ddrtest=echo Loading DDR stress test...; if run loadtest; then go 0x907000; fi\0"
#else
#define DDRTESTDEFS ""
#endif

#define CONFIG_EXTRA_ENV_SETTINGS \
	"bootfailcheck=" bootfailcheck__ "\0" \
	"bootfailcount=0\0" \
	"bootfailmax=3\0" \
	"bootdelay=" mkstring_(CONFIG_BOOTDELAY) "\0" \
	"bootretry=" mkstring_(CONFIG_BOOT_RETRY_TIME) "\0" \
	"autoboot=yes\0" \
	"lastbootfailed=no\0" \
	"upgradeinprogress=no\0" \
	"uimage=uImage\0" \
	"fdt_file=" CONFIG_DEFAULT_FDT_FILE "\0" \
	"fdt_addr=0x18000000\0" \
	"console=" CONFIG_CONSOLE_DEV "\0" \
	"fdt_high=0xffffffff\0"	  \
	"initrd_high=0xffffffff\0" \
	"mmcdev=0\0" \
	"mmcpart=2\0" \
	"mmcroot=" CONFIG_MMCROOT " rootwait ro\0" \
	"mmcroot_eval=setenv mmcroot " CONFIG_MMCROOT_DEV "${mmcdev}p${mmcpart} rootwait ro\0" \
	"mmcargs=test -n ${mmcroot_eval}  &&  run mmcroot_eval; " \
		"setenv bootargs console=${console},${baudrate} " \
		"loglevel=3 " \
		"root=${mmcroot};" \
		"echo Boot args: ${bootargs}\0" \
	"loaduimage=ext2load mmc ${mmcdev}:${mmcpart} ${loadaddr} /boot/${uimage}\0" \
	"loadfdt=ext2load mmc ${mmcdev}:${mmcpart} ${fdt_addr} /boot/${fdt_file}\0" \
	"defaultfdt=ext2load mmc ${mmcdev}:${mmcpart} ${fdt_addr} /boot/" CONFIG_DEFAULT_FDT_FILE "\0" \
	"bootauto=if test ${bootfailcheck} = yes -a ${lastbootfailed} = yes; then " \
			"setexpr bootfailcount ${bootfailcount} + 1;" \
			"if test ${mmcpart} -eq 2; then setenv mmcpart 3; else setenv mmcpart 2; fi;" \
			"echo \"Last boot failed (count ${bootfailcount} of ${bootfailmax}), trying partition ${mmcpart}\";" \
		"fi;" \
		"if test ${bootfailcheck} != yes -o ${bootfailcount} -lt ${bootfailmax}; then " \
			"if test ${bootfailcheck} = yes; then setenv lastbootfailed yes; saveenv; fi;" \
			"if mmc rescan; then " \
				"run mmcargs;" \
				"if run loaduimage; then " \
					"run mmcboot;" \
				"fi;" \
			"fi;" \
		"else " \
			"echo \"FAIL: too many boot failures\";" \
		"fi;" \
		"echo \"Auto-boot FAILED\";\0" \
	"mmcboot=echo Booting from mmc ...; " \
		"if run loadfdt; then " \
			"bootm ${loadaddr} - ${fdt_addr}; " \
		"else if run defaultfdt; then " \
		       "bootm ${loadaddr} - ${fdt_addr}; " \
		"else " \
			"echo FAIL: could not load FDT; " \
		"fi; fi;\0" \
	DDRTESTDEFS

#ifdef CONFIG_DDRTEST
#define CONFIG_BOOTCOMMAND "run ddrtest"
#else
#define CONFIG_BOOTCOMMAND \
	"mmc dev ${mmcdev};" \
	"if env print ethaddr; then " \
		"echo Ethernet address: ${ethaddr}; " \
	"else " \
		"echo No Ethernet address set, disabling auto-boot.;" \
		"setenv autoboot no; saveenv;" \
	"fi;" \
	"if mmc rescan; then " \
		"if test \"${autoboot}\" != \"no\"; then " \
			"echo \"Auto-booting...\";" \
			"run bootauto;" \
		"else " \
			"echo \"Auto-boot disabled. run bootauto to boot.\";" \
		"fi;" \
	"else " \
		"echo \"mmc rescan failed, skipping auto-boot.\";" \
	"fi"
#endif

#define CONFIG_ARP_TIMEOUT     200UL

/* Miscellaneous configurable options */
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT_HUSH_PS2     "> "
#define CONFIG_SYS_PROMPT              "U-Boot > "
#define CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_CBSIZE              256

/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_MAXARGS             16
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE

#define CONFIG_SYS_LOAD_ADDR           CONFIG_LOADADDR
#define CONFIG_SYS_HZ                  1000

#define CONFIG_CMDLINE_EDITING
#define CONFIG_STACKSIZE               (128 * 1024)

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS           1
#define PHYS_SDRAM                     MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE          PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR       IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE       IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* FLASH and environment organization */
#define CONFIG_SYS_NO_FLASH

#define CONFIG_ENV_SIZE			(8 * 1024)

#ifdef CONFIG_MANUFACTURING
#define CONFIG_ENV_IS_NOWHERE
#else
#define CONFIG_ENV_IS_IN_MMC
#endif

#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_ENV_OFFSET		(6 * 64 * 1024)
#define CONFIG_ENV_OFFSET_REDUND	(8 * 64 * 1024)
#endif

#define CONFIG_OF_LIBFDT

#ifndef CONFIG_SYS_DCACHE_OFF
#define CONFIG_CMD_CACHE
#endif

#define CONFIG_SYS_FSL_USDHC_NUM	2
#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_SYS_MMC_ENV_DEV		1	/* SDHC4 */
#endif

/* Framebuffer */
#define CONFIG_SYS_CONSOLE_IS_IN_ENV

/*watchdog - disable for DDR tests*/
#ifndef CONFIG_DDRTEST
#define CONFIG_HW_WATCHDOG
#define CONFIG_IMX_WATCHDOG
#endif

/*i2c*/
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_SPEED		100000

#define CONFIG_CMD_REGINFO
#define CONFIG_CMD_MEMTEST
#define CONFIG_SYS_ALT_MEMTEST
#define CONFIG_SYS_MEMTEST_START 0x10000000
#define CONFIG_SYS_MEMTEST_END 0x4F500000
#define CONFIG_SYS_MEMTEST_SCRATCH (CONFIG_SYS_MEMTEST_END + sizeof(vu_long))

#define CONFIG_TEMPERATURE

#endif                         /* __MX6QFALCON_CONFIG_H */
