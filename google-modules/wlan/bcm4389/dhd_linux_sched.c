/*
 * Expose some of the kernel scheduler routines
 *
 * Copyright (C) 2025, Broadcom.
 *
 *      Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2 (the "GPL"),
 * available at http://www.broadcom.com/licenses/GPLv2.php, with the
 * following added to such license:
 *
 *      As a special exception, the copyright holders of this software give you
 * permission to link this software with independent modules, and to copy and
 * distribute the resulting executable under terms of your choice, provided that
 * you also meet, for each linked independent module, the terms and conditions of
 * the license of that module.  An independent module is a module which is not
 * derived from this software.  The special exception does not apply to any
 * modifications of the software.
 *
 *
 * <<Broadcom-WL-IPTag/Open:>>
 *
 * $Id$
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <typedefs.h>
#include <linuxver.h>

int setScheduler(struct task_struct *p, int policy, struct sched_param *param)
{
	int rc = 0;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 9, 0))
	switch (policy) {
		case SCHED_FIFO:
			sched_set_fifo(p);
			break;
		case SCHED_NORMAL:
			sched_set_normal(p, param->sched_priority);
			break;
		default:
			printk("%s: invalid policy:%d\n", __func__, policy);
			rc = -1;
			break;
	}
#else
	rc = sched_setscheduler(p, policy, param);
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(5, 9, 0) */
	return rc;
}

int get_scheduler_policy(struct task_struct *p)
{
	int rc = SCHED_NORMAL;
	rc = p->policy;
	return rc;
}
