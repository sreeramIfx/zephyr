/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "cy_pdl.h"
#include "cyhal.h"

#define CY_CORTEX_M7_0_APPL_ADDR 0x10080000
#define CY_CORTEX_M7_1_APPL_ADDR 0x10280000
#define CM7_DUAL                 1

int main(void)
{
	cy_rslt_t result;

	Cy_SysEnableCM7(CORE_CM7_0, CY_CORTEX_M7_0_APPL_ADDR);
#if CM7_DUAL
	Cy_SysEnableCM7(CORE_CM7_1, CY_CORTEX_M7_1_APPL_ADDR);
#endif

	for (;;) {
		Cy_SysPm_CpuEnterDeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
	}
}
