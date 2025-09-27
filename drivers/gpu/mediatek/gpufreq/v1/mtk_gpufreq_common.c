/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

/**
 * @file    mtk_gpufreq_common.c
 * @brief   Driver for GPU-DVFS
 */

/**
 * ===============================================
 * SECTION : Include files
 * ===============================================
 */
#include <linux/slab.h>
#include <mt-plat/aee.h>
#include "mtk_gpufreq_common.h"

static void dump_except(enum g_exception_enum except_type, char *except_str)
{
	(void)except_type;
	(void)except_str;
}

void gpu_assert(bool cond, enum g_exception_enum except_type,
	const char *except_str, ...)
{
	va_list args;
	int cx;
	char tmp_string[1024];

	if (unlikely(!(cond))) {
		va_start(args, except_str);
		cx = vsnprintf(tmp_string, sizeof(tmp_string),
			except_str, args);
		va_end(args);

		pr_info("[GPU/DVFS] assert:%s", tmp_string);
		if (cx >= 0)
			dump_except(except_type, tmp_string);
	}
}

/* check if there have pending info
 * return: the count of pending info
 */
void check_pending_info(void)
{
}

