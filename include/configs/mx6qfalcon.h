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

#define CONFIG_MACH_TYPE	3981
#define CONFIG_MXC_UART_BASE	UART1_BASE
#define CONFIG_CONSOLE_DEV		"ttymxc0"

#define CONFIG_MMCROOT_DEV		"/dev/mmcblk"
#ifdef CONFIG_MANUFACTURING
#define CONFIG_MMCROOT_DEVNUM		"0"
#else
#define CONFIG_MMCROOT_DEVNUM		"1"
#endif
#define CONFIG_MMCROOT_PART		"2"
#define CONFIG_MMCROOT			CONFIG_MMCROOT_DEV CONFIG_MMCROOT_DEVNUM "p" CONFIG_MMCROOT_PART

#define CONFIG_SECURE_BOOT

#define PHYS_SDRAM_SIZE		(2u * 1024 * 1024 * 1024)

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(10 * SZ_1M)

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_BOARD_LATE_INIT

#define CONFIG_MXC_UART

#define CONFIG_SYS_FSL_ESDHC_ADDR      0

#define CONFIG_FEC_MXC
#define CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RGMII
#define CONFIG_ETHPRIME			"FEC"
#define CONFIG_FEC_MXC_PHYADDR		3

#define CONFIG_PHYLIB
#define CONFIG_PHY_MICREL

/* PMIC */
#define CONFIG_POWER
#define CONFIG_POWER_I2C
#define CONFIG_POWER_LTC3676
#define CONFIG_POWER_LTC3676_I2C_ADDR 0x3c
#define CONFIG_POWER_LTC3676_I2C_BUS  0

/* Command definition */

#define CONFIG_CMD_BMODE

#ifdef CONFIG_MANUFACTURING
#define CONFIG_BOOT_RETRY_TIME        -1
#else
/* Command timeout, 5 minutes by default, min 10 sec if set */
#define CONFIG_BOOT_RETRY_TIME         300
#define CONFIG_BOOTCOUNT_ENV
#define CONFIG_BOOTCOUNT_LIMIT
#endif
#define CONFIG_BOOT_RETRY_TIME_MIN     10
#define CONFIG_RESET_TO_RETRY

#ifdef CONFIG_DDRTEST
#define DDRTESTDEFS \
	"loadtest=fatload mmc 0:1 0x907000 ddr-stress-test-mx6dq.bin\0" \
	"ddrtest=echo Loading DDR stress test...; if run loadtest; then go 0x907000; fi\0"
#else
#define DDRTESTDEFS ""
#endif

#define CONFIG_EXTRA_ENV_SETTINGS \
	"bootcount=0\0" \
	"upgrade_available=1\0" \
	"bootlimit=3\0" \
	"bootretry=" __stringify(CONFIG_BOOT_RETRY_TIME) "\0" \
	"image=zImage\0" \
	"fdtfile=" CONFIG_DEFAULT_FDT_FILE "\0" \
	"fdtaddr=0x18000000\0" \
	"console=" CONFIG_CONSOLE_DEV "\0" \
	"fdt_high=0xffffffff\0"	  \
	"initrd_high=0xffffffff\0" \
	"initrdfile=initrd\0" \
	"ramdisk_addr_r=0x19000000\0" \
	"extra_bootargs=loglevel=3\0" \
	"mmcdev=" CONFIG_MMCROOT_DEVNUM "\0" \
	"mmcpart=" CONFIG_MMCROOT_PART "\0" \
	"mmcroot=" CONFIG_MMCROOT " rootwait ro\0" \
	"mmcroot_eval=setenv mmcroot " CONFIG_MMCROOT_DEV "${mmcdev}p${mmcpart} rootwait ro\0" \
	"mmcpart_swap=if test ${mmcpart} -eq 2; then setenv mmcpart 3; else setenv mmcpart 2; fi; setenv bootcount 0; saveenv\0" \
	"mmcargs=test -n ${mmcroot_eval}  &&  run mmcroot_eval; " \
		"setenv bootargs console=${console},${baudrate} " \
		"root=${mmcroot} ${extra_bootargs}; " \
		"echo Boot args: ${bootargs}\0" \
	"loadimage=ext2load mmc ${mmcdev}:${mmcpart} ${loadaddr} /boot/${image}\0" \
	"loadfdt=ext2load mmc ${mmcdev}:${mmcpart} ${fdtaddr} /boot/${fdtfile}\0" \
	"loadinitrd=echo Loading initrd at ${ramdisk_addr_r}; if ext2load mmc ${mmcdev}:${mmcpart} ${ramdisk_addr_r} /boot/${initrdfile}; then " \
			"setenv initrd_addr ${ramdisk_addr_r}; " \
		"else " \
			"echo No initrd present - skipping; " \
			"setenv initrd_addr -; " \
		"fi;\0" \
	"defaultfdt=ext2load mmc ${mmcdev}:${mmcpart} ${fdtaddr} /boot/" CONFIG_DEFAULT_FDT_FILE "\0" \
	"mmcboot=run loadinitrd; echo Booting from mmc ...; " \
		"if run loadfdt; then " \
			"bootz ${loadaddr} ${initrd_addr} ${fdtaddr}; " \
		"else if run defaultfdt; then " \
		       "bootz ${loadaddr} ${initrd_addr} ${fdtaddr}; " \
		"else " \
			"echo FAIL: could not load FDT; " \
		"fi; fi;\0" \
	"altbootcmd=run mmcpart_swap; run bootcmd\0" \
	DDRTESTDEFS

#ifdef CONFIG_DDRTEST
#define CONFIG_BOOTCOMMAND "run ddrtest"
#else
#define CONFIG_BOOTCOMMAND "mmc dev ${mmcdev};" \
		"if mmc rescan; then " \
			"run mmcargs;" \
			"if run loadimage; then " \
				"run mmcboot;" \
			"else " \
				"echo FAIL: could not find Linux kernel; " \
			"fi;" \
		"else " \
			"echo FAIL: mmc rescan failed;" \
		"fi"
#endif

#define CONFIG_ARP_TIMEOUT     200UL

/* Miscellaneous configurable options */
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_PROMPT_HUSH_PS2     "> "
#undef  CONFIG_SYS_PROMPT
#define CONFIG_SYS_PROMPT              "U-Boot > "
#define CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_CBSIZE              512

/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE		(CONFIG_SYS_CBSIZE*2)
#define CONFIG_SYS_BARGSIZE		CONFIG_SYS_CBSIZE

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

#define CONFIG_SYS_FSL_USDHC_NUM	2
#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_SYS_MMC_ENV_DEV		1	/* SDHC4 */
#endif

/*watchdog - disable for DDR tests*/
#ifndef CONFIG_DDRTEST
#define CONFIG_HW_WATCHDOG
#define CONFIG_IMX_WATCHDOG
#endif

/*i2c*/
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_MXC_I2C1
#define CONFIG_SYS_I2C_MXC_I2C2
#define CONFIG_SYS_I2C_SPEED		100000

#define CONFIG_CMD_REGINFO
#define CONFIG_CMD_MEMTEST
#define CONFIG_SYS_ALT_MEMTEST
#define CONFIG_SYS_MEMTEST_START 0x10000000
#define CONFIG_SYS_MEMTEST_END 0x4F500000
#define CONFIG_SYS_MEMTEST_SCRATCH (CONFIG_SYS_MEMTEST_END + sizeof(vu_long))

#define CONFIG_TEMPERATURE

#define CONFIG_SYS_VSNPRINTF

#define CONFIG_CMD_DATE
#define CONFIG_RTC_ISL1208
#define CONFIG_SYS_I2C_RTC_ADDR 0x6F

#endif                         /* __MX6QFALCON_CONFIG_H */
