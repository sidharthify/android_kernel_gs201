/*
 * Linux-specific abstractions to gain some independence from linux kernel versions.
 * Pave over some 2.2 versus 2.4 versus 2.6 kernel differences.
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
 * <<Broadcom-WL-IPTag/Dual:>>
 */

#ifndef _linuxver_h_
#define _linuxver_h_

/*
 * The below pragmas are added as workaround for errors caused by update
 * of gcc version to 4.8.2. GCC 4.6 adds -Wunused-but-set-variable and
 * -Wunused-but-set-parameter to -Wall, for some configurations those
 * warnings are produced in linux kernel. So for now the below pragmas
 * disable the offending warnings. Permanent solution is to use -isystem
 * but there is a performance problem with this change on RHEL5 servers
 *
 */
#if __GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 6)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wunused-but-set-parameter"
#endif

#include <typedefs.h>
#include <linux/version.h>
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 0))
#include <linux/config.h>
#else
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 33))
#include <generated/autoconf.h>
#else
#include <linux/autoconf.h>
#endif
#endif /* (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 0)) */

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 1, 0))
#include <linux/kconfig.h>
#endif
#include <linux/module.h>

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 3, 0))
/* __NO_VERSION__ must be defined for all linkables except one in 2.2 */
#ifdef __UNDEF_NO_VERSION__
#undef __NO_VERSION__
#else
#define __NO_VERSION__
#endif
#endif	/* LINUX_VERSION_CODE < KERNEL_VERSION(2, 3, 0) */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 5, 0)
#define module_param(_name_, _type_, _perm_)	MODULE_PARM(_name_, "i")
#define module_param_string(_name_, _string_, _size_, _perm_) \
		MODULE_PARM(_string_, "c" __MODULE_STRING(_size_))
#endif

/* linux/malloc.h is deprecated, use linux/slab.h instead. */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 4, 9))
#include <linux/malloc.h>
#else
#include <linux/slab.h>
#endif

#include <linux/types.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/string.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/netdevice.h>
#include <linux/time.h>
#include <linux/rtc.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27))
#include <linux/semaphore.h>
#else
#include <asm/semaphore.h>
#endif /* (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)) */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 28))
#undef IP_TOS
#endif /* (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 28)) */
#include <asm/io.h>

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2, 5, 41))
#include <linux/workqueue.h>
#else
#include <linux/tqueue.h>
#ifndef work_struct
#define work_struct tq_struct
#endif
#ifndef INIT_WORK
#define INIT_WORK(_work, _func, _data) INIT_TQUEUE((_work), (_func), (_data))
#endif
#ifndef schedule_work
#define schedule_work(_work) schedule_task((_work))
#endif
#ifndef flush_scheduled_work
#define flush_scheduled_work() flush_scheduled_tasks()
#endif
#endif	/* LINUX_VERSION_CODE > KERNEL_VERSION(2, 5, 41) */

/*
 * TODO:
 * daemonize() API is deprecated from kernel-3.8 onwards. More debugging
 *      has to be done whether this can cause any issue in case, if driver is
 *      loaded as a module from userspace.
 */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0))
#define DAEMONIZE(a)	do { \
		allow_signal(SIGKILL);	\
		allow_signal(SIGTERM);	\
	} while (0)
#elif ((LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0)) && \
	(LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 0)))
#define DAEMONIZE(a) daemonize(a); \
	allow_signal(SIGKILL); \
	allow_signal(SIGTERM);
#else /* Linux 2.4 (w/o preemption patch) */
#define RAISE_RX_SOFTIRQ() \
	cpu_raise_softirq(smp_processor_id(), NET_RX_SOFTIRQ)
#define DAEMONIZE(a) daemonize(); \
	do { if (a) \
		strncpy(current->comm, a, MIN(sizeof(current->comm), (strlen(a)))); \
	} while (0);
#endif /* LINUX_VERSION_CODE  */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 19)
#define	MY_INIT_WORK(_work, _func)	INIT_WORK(_work, _func)
#else
#define	MY_INIT_WORK(_work, _func)	INIT_WORK(_work, _func, _work)
#if !(LINUX_VERSION_CODE == KERNEL_VERSION(2, 6, 18) && defined(RHEL_MAJOR) && \
	(RHEL_MAJOR == 5))
/* Exclude RHEL 5 */
typedef void (*work_func_t)(void *work);
#endif
#endif	/* >= 2.6.20 */

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 0))
/* Some distributions have their own 2.6.x compatibility layers */
#ifndef IRQ_NONE
typedef void irqreturn_t;
#define IRQ_NONE
#define IRQ_HANDLED
#define IRQ_RETVAL(x)
#endif
#else
typedef irqreturn_t(*FN_ISR) (int irq, void *dev_id, struct pt_regs *ptregs);
#endif	/* LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 0) */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 18)
#define IRQF_SHARED	SA_SHIRQ
#endif /* < 2.6.18 */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 17)
#ifdef	CONFIG_NET_RADIO
#endif
#endif	/* < 2.6.17 */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 5, 67)
#define MOD_INC_USE_COUNT
#define MOD_DEC_USE_COUNT
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(2, 5, 67) */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
#include <linux/sched.h>
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32) */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
#include <linux/sched/rt.h>
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0) */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 11, 0)
#include <uapi/linux/sched/types.h>
#endif /* LINUX_VERS >= 4.11.0 */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 29)
#include <net/lib80211.h>
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 29)
#include <linux/ieee80211.h>
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 14)
#include <net/ieee80211.h>
#endif
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 30) */

#ifndef __exit
#define __exit
#endif
#ifndef __devexit
#define __devexit
#endif
#ifndef __devinit
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0))
	#define __devinit	__init
#else
/* All devices are hotpluggable since linux 3.8.0 */
	#define __devinit
#endif
#endif /* !__devinit */
#ifndef __devinitdata
#define __devinitdata
#endif
#ifndef __devexit_p
#define __devexit_p(x)	x
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 4, 0))

#define pci_get_drvdata(dev)		(dev)->sysdata
#define pci_set_drvdata(dev, value)	(dev)->sysdata = (value)

/*
 * New-style (2.4.x) PCI/hot-pluggable PCI/CardBus registration
 */

struct pci_device_id {
	unsigned int vendor, device;		/* Vendor and device ID or PCI_ANY_ID */
	unsigned int subvendor, subdevice;	/* Subsystem ID's or PCI_ANY_ID */
	unsigned int class, class_mask;		/* (class,subclass,prog-if) triplet */
	unsigned long driver_data;		/* Data private to the driver */
};

struct pci_driver {
	struct list_head node;
	char *name;
	const struct pci_device_id *id_table;	/* NULL if wants all devices */
	int (*probe)(struct pci_dev *dev,
	             const struct pci_device_id *id); /* New device inserted */
	void (*remove)(struct pci_dev *dev);	/* Device removed (NULL if not a hot-plug
						 * capable driver)
						 */
	void (*suspend)(struct pci_dev *dev);	/* Device suspended */
	void (*resume)(struct pci_dev *dev);	/* Device woken up */
};

#define MODULE_DEVICE_TABLE(type, name)
#define PCI_ANY_ID (~0)

/* compatpci.c */
#define pci_module_init pci_register_driver
extern int pci_register_driver(struct pci_driver *drv);
extern void pci_unregister_driver(struct pci_driver *drv);

#endif /* PCI registration */

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 18))
#define pci_module_init pci_register_driver
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 2, 18))
#ifdef MODULE
#define module_init(x) int init_module(void) { return x(); }
#define module_exit(x) void cleanup_module(void) { x(); }
#else
#define module_init(x)	__initcall(x);
#define module_exit(x)	__exitcall(x);
#endif
#endif	/* LINUX_VERSION_CODE < KERNEL_VERSION(2, 2, 18) */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 31)
#define WL_USE_NETDEV_OPS
#else
#undef WL_USE_NETDEV_OPS
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 31)) && defined(CONFIG_RFKILL)
#define WL_CONFIG_RFKILL
#else
#undef WL_CONFIG_RFKILL
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 3, 48))
#define list_for_each(pos, head) \
	for (pos = (head)->next; pos != (head); pos = pos->next)
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 3, 13))
#define pci_resource_start(dev, bar)	((dev)->base_address[(bar)])
#elif (LINUX_VERSION_CODE < KERNEL_VERSION(2, 3, 44))
#define pci_resource_start(dev, bar)	((dev)->resource[(bar)].start)
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 3, 23))
#define pci_enable_device(dev) do { } while (0)
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 3, 14))
#define net_device device
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 3, 42))

/*
 * DMA mapping
 *
 * See linux/Documentation/DMA-mapping.txt
 */

#ifndef PCI_DMA_TODEVICE
#define	PCI_DMA_TODEVICE	1
#define	PCI_DMA_FROMDEVICE	2
#endif

typedef u32 dma_addr_t;

/* Pure 2^n version of get_order */
static inline int get_order(unsigned long size)
{
	int order;

	size = (size-1) >> (PAGE_SHIFT-1);
	order = -1;
	do {
		size >>= 1;
		order++;
	} while (size);
	return order;
}

static inline void *pci_alloc_consistent(struct pci_dev *hwdev, size_t size,
                                         dma_addr_t *dma_handle)
{
	void *ret;
	int gfp = GFP_ATOMIC | GFP_DMA;

	ret = (void *)__get_free_pages(gfp, get_order(size));

	if (ret != NULL) {
		bzero(ret, size);
		*dma_handle = virt_to_bus(ret);
	}
	return ret;
}

static inline void pci_free_consistent(struct pci_dev *hwdev, size_t size,
                                       void *vaddr, dma_addr_t dma_handle)
{
	free_pages((unsigned long)vaddr, get_order(size));
}

#ifdef ILSIM
extern uint pci_map_single(void *dev, void *va, uint size, int direction);
extern void pci_unmap_single(void *dev, uint pa, uint size, int direction);
#else
#define pci_map_single(cookie, address, size, dir)	virt_to_bus(address)
#define pci_unmap_single(cookie, address, size, dir)
#endif

#endif /* DMA mapping */

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0)

typedef struct timer_list timer_list_compat_t;

#define init_timer_compat(timer_compat, cb, priv) \
	init_timer(timer_compat); \
	(timer_compat)->data = (ulong)priv; \
	(timer_compat)->function = cb
#define timer_set_private(timer_compat, priv) (timer_compat)->data = (ulong)priv
#define timer_expires(timer_compat) (timer_compat)->expires

#else /* LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0) */

typedef struct timer_list_compat {
	struct timer_list timer;
	void *arg;
	void (*callback)(ulong arg);
} timer_list_compat_t;

extern void timer_cb_compat(struct timer_list *tl);

#define init_timer_compat(timer_compat, cb, priv) \
	(timer_compat)->arg = priv; \
	(timer_compat)->callback = cb; \
	timer_setup(&(timer_compat)->timer, timer_cb_compat, 0);
#define timer_set_private(timer_compat, priv) (timer_compat)->arg = priv
#define timer_expires(timer_compat) (timer_compat)->timer.expires

#define del_timer(t) del_timer(&((t)->timer))
#define del_timer_sync(t) del_timer_sync(&((t)->timer))
#define timer_pending(t) timer_pending(&((t)->timer))
#define add_timer(t) add_timer(&((t)->timer))
#define mod_timer(t, j) mod_timer(&((t)->timer), j)
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0) */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 19, 0)
#define rtc_time_to_tm(a, b) rtc_time64_to_tm(a, b)
#else
#define rtc_time_to_tm(a, b) rtc_time_to_tm(a, b)
#endif /* LINUX_VER >= 3.19.0 */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 20, 0)
#define time_to_tm(a, b, c) time64_to_tm(a, b, c)
#else
#define time_to_tm(a, b, c) time_to_tm(a, b, c)
#endif /* LINUX_VER >= 4.20.0 */

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 3, 43))

#define dev_kfree_skb_any(a)		dev_kfree_skb(a)
#define netif_down(dev)			do { (dev)->start = 0; } while (0)

/* pcmcia-cs provides its own netdevice compatibility layer */
#ifndef _COMPAT_NETDEVICE_H

/*
 * SoftNet
 *
 * For pre-softnet kernels we need to tell the upper layer not to
 * re-enter start_xmit() while we are in there. However softnet
 * guarantees not to enter while we are in there so there is no need
 * to do the netif_stop_queue() dance unless the transmit queue really
 * gets stuck. This should also improve performance according to tests
 * done by Aman Singla.
 */

#define dev_kfree_skb_irq(a)	dev_kfree_skb(a)
#define netif_wake_queue(dev) \
		do { clear_bit(0, &(dev)->tbusy); mark_bh(NET_BH); } while (0)
#define netif_stop_queue(dev)	set_bit(0, &(dev)->tbusy)

static inline void netif_start_queue(struct net_device *dev)
{
	dev->tbusy = 0;
	dev->interrupt = 0;
	dev->start = 1;
}

#define netif_queue_stopped(dev)	(dev)->tbusy
#define netif_running(dev)		(dev)->start

#endif /* _COMPAT_NETDEVICE_H */

#define netif_device_attach(dev)	netif_start_queue(dev)
#define netif_device_detach(dev)	netif_stop_queue(dev)

/* 2.4.x renamed bottom halves to tasklets */
#define tasklet_struct				tq_struct
static inline void tasklet_schedule(struct tasklet_struct *tasklet)
{
	queue_task(tasklet, &tq_immediate);
	mark_bh(IMMEDIATE_BH);
}

static inline void tasklet_init(struct tasklet_struct *tasklet,
                                void (*func)(unsigned long),
                                unsigned long data)
{
	tasklet->next = NULL;
	tasklet->sync = 0;
	tasklet->routine = (void (*)(void *))func;
	tasklet->data = (void *)data;
}

#define tasklet_kill(tasklet)	{ do {} while (0); }

/* 2.4.x introduced del_timer_sync() */
#define del_timer_sync(timer) del_timer(timer)

#else

#define netif_down(dev)

#endif /* SoftNet */

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 4, 3))

/*
 * Emit code to initialise a tq_struct's routine and data pointers
 */
#define PREPARE_TQUEUE(_tq, _routine, _data)			\
	do {							\
		(_tq)->routine = _routine;			\
		(_tq)->data = _data;				\
	} while (0)

/*
 * Emit code to initialise all of a tq_struct
 */
#define INIT_TQUEUE(_tq, _routine, _data)			\
	do {							\
		INIT_LIST_HEAD(&(_tq)->list);			\
		(_tq)->sync = 0;				\
		PREPARE_TQUEUE((_tq), (_routine), (_data));	\
	} while (0)

#endif	/* LINUX_VERSION_CODE < KERNEL_VERSION(2, 4, 3) */

/* Power management related macro & routines */
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 9)
#define	PCI_SAVE_STATE(a, b)	pci_save_state(a)
#define	PCI_RESTORE_STATE(a, b)	pci_restore_state(a)
#else
#define	PCI_SAVE_STATE(a, b)	pci_save_state(a, b)
#define	PCI_RESTORE_STATE(a, b)	pci_restore_state(a, b)
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 4, 6))
static inline int
pci_save_state(struct pci_dev *dev, u32 *buffer)
{
	int i;
	if (buffer) {
		/* 100% dword access ok here? */
		for (i = 0; i < 16; i++)
			pci_read_config_dword(dev, i * 4, &buffer[i]);
	}
	return 0;
}

static inline int
pci_restore_state(struct pci_dev *dev, u32 *buffer)
{
	int i;

	if (buffer) {
		for (i = 0; i < 16; i++)
			pci_write_config_dword(dev, i * 4, buffer[i]);
	}
	/*
	 * otherwise, write the context information we know from bootup.
	 * This works around a problem where warm-booting from Windows
	 * combined with a D3(hot)->D0 transition causes PCI config
	 * header data to be forgotten.
	 */
	else {
		for (i = 0; i < 6; i ++)
			pci_write_config_dword(dev,
			                       PCI_BASE_ADDRESS_0 + (i * 4),
			                       pci_resource_start(dev, i));
		pci_write_config_byte(dev, PCI_INTERRUPT_LINE, dev->irq);
	}
	return 0;
}
#endif /* PCI power management */

/* Old cp0 access macros deprecated in 2.4.19 */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 4, 19))
#define read_c0_count() read_32bit_cp0_register(CP0_COUNT)
#endif

/* Module refcount handled internally in 2.6.x */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 24))
#ifndef SET_MODULE_OWNER
#define SET_MODULE_OWNER(dev)		do {} while (0)
#define OLD_MOD_INC_USE_COUNT		MOD_INC_USE_COUNT
#define OLD_MOD_DEC_USE_COUNT		MOD_DEC_USE_COUNT
#else
#define OLD_MOD_INC_USE_COUNT		do {} while (0)
#define OLD_MOD_DEC_USE_COUNT		do {} while (0)
#endif
#else /* LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 24) */
#ifndef SET_MODULE_OWNER
#define SET_MODULE_OWNER(dev)		do {} while (0)
#endif
#ifndef MOD_INC_USE_COUNT
#define MOD_INC_USE_COUNT			do {} while (0)
#endif
#ifndef MOD_DEC_USE_COUNT
#define MOD_DEC_USE_COUNT			do {} while (0)
#endif
#define OLD_MOD_INC_USE_COUNT		MOD_INC_USE_COUNT
#define OLD_MOD_DEC_USE_COUNT		MOD_DEC_USE_COUNT
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 24) */

#ifndef SET_NETDEV_DEV
#define SET_NETDEV_DEV(net, pdev)	do {} while (0)
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 1, 0))
#ifndef HAVE_FREE_NETDEV
#define free_netdev(dev)		kfree(dev)
#endif
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(3, 1, 0) */

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 0))
/* struct packet_type redefined in 2.6.x */
#define af_packet_priv			data
#endif

/* suspend args */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 11)
#define DRV_SUSPEND_STATE_TYPE pm_message_t
#else
#define DRV_SUSPEND_STATE_TYPE uint32
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 19)
#define CHECKSUM_HW	CHECKSUM_PARTIAL
#endif

typedef struct {
	void	*parent;  /* some external entity that the thread supposed to work for */
	char	*proc_name;
	struct	task_struct *p_task;
	long	thr_pid;
	int		prio; /* priority */
	struct	semaphore sema;
	int	terminated;
	struct	completion completed;
	int	flush_ind;
	struct	completion flushed;
	spinlock_t	spinlock;
	int		up_cnt;
} tsk_ctl_t;

/* ANDREY: new MACROs to start stop threads(OLD kthread API STYLE) */
/* requires  tsk_ctl_t tsk  argument, the caller's priv data is passed in owner ptr */
/* note this macro assumes there may be only one context waiting on thread's completion */
#ifdef DHD_DEBUG
#ifndef CUSTOM_PREFIX
#define DBG_THR(x) printk x
#else
extern char* osl_get_rtctime(void);
#define DBG_THR_PREFIX "[%s]"CUSTOM_PREFIX, osl_get_rtctime()
#define DBG_THR(x)	\
do {	\
	pr_cont(DBG_THR_PREFIX);	\
	pr_cont x;			\
} while (0)
#endif /* !CUSTOM_PREFIX */
#else
#define DBG_THR(x)
#endif /* DHD_DEBUG */

extern unsigned long osl_spin_lock(void *lock);
extern void osl_spin_unlock(void *lock, unsigned long flags);

#define TSK_LOCK(lock, flags)	(flags) = osl_spin_lock(lock)
#define TSK_UNLOCK(lock, flags)	osl_spin_unlock((lock), (flags))

static inline bool binary_sema_down(tsk_ctl_t *tsk)
{
	if (down_interruptible(&tsk->sema) == 0) {
		unsigned long flags = 0;
		TSK_LOCK(&tsk->spinlock, flags);
		if (tsk->up_cnt == 1)
			tsk->up_cnt--;
		else {
			DBG_THR(("dhd_dpc_thread: Unexpected up_cnt %d\n", tsk->up_cnt));
		}
		TSK_UNLOCK(&tsk->spinlock, flags);
		return false;
	} else
		return true;
}

static inline bool binary_sema_up(tsk_ctl_t *tsk)
{
	bool sem_up = false;
	unsigned long flags = 0;

	TSK_LOCK(&tsk->spinlock, flags);
	if (tsk->up_cnt == 0) {
		tsk->up_cnt++;
		sem_up = true;
	} else if (tsk->up_cnt == 1) {
		/* dhd_sched_dpc: dpc is alread up! */
	} else
		DBG_THR(("dhd_sched_dpc: unexpected up cnt %d!\n", tsk->up_cnt));

	TSK_UNLOCK(&tsk->spinlock, flags);

	if (sem_up)
		up(&tsk->sema);

	return sem_up;
}

#if  (LINUX_VERSION_CODE > KERNEL_VERSION(5, 6, 0))
#define SMP_RD_BARRIER_DEPENDS(x)
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 0))
#define SMP_RD_BARRIER_DEPENDS(x) smp_read_barrier_depends(x)
#else
#define SMP_RD_BARRIER_DEPENDS(x) smp_rmb(x)
#endif

#define PROC_START(thread_func, owner, tsk_ctl, flags, name) \
{ \
	sema_init(&((tsk_ctl)->sema), 0); \
	init_completion(&((tsk_ctl)->completed)); \
	init_completion(&((tsk_ctl)->flushed)); \
	(tsk_ctl)->parent = owner; \
	(tsk_ctl)->proc_name = name;  \
	(tsk_ctl)->terminated = FALSE; \
	(tsk_ctl)->flush_ind = FALSE; \
	(tsk_ctl)->up_cnt = 0; \
	(tsk_ctl)->p_task  = kthread_run(thread_func, tsk_ctl, (char*)name); \
	if (IS_ERR((tsk_ctl)->p_task)) { \
		(tsk_ctl)->thr_pid = -1; \
		DBG_THR(("%s(): thread:%s create failed\n", __FUNCTION__, \
			(tsk_ctl)->proc_name)); \
	} else { \
		(tsk_ctl)->thr_pid = (tsk_ctl)->p_task->pid; \
		spin_lock_init(&((tsk_ctl)->spinlock)); \
		DBG_THR(("%s(): thread:%s:%lx started\n", __FUNCTION__, \
			(tsk_ctl)->proc_name, (tsk_ctl)->thr_pid)); \
	}; \
}

#define PROC_WAIT_TIMEOUT_MSEC	5000 /* 5 seconds */

#define PROC_STOP(tsk_ctl) \
{ \
	uint timeout = (uint)msecs_to_jiffies(PROC_WAIT_TIMEOUT_MSEC); \
	(tsk_ctl)->terminated = TRUE; \
	smp_wmb(); \
	up(&((tsk_ctl)->sema));	\
	DBG_THR(("%s(): thread:%s:%lx wait for terminate\n", __FUNCTION__, \
			 (tsk_ctl)->proc_name, (tsk_ctl)->thr_pid)); \
	timeout = (uint)wait_for_completion_timeout(&((tsk_ctl)->completed), timeout); \
	if (timeout == 0) \
		DBG_THR(("%s(): thread:%s:%lx terminate timeout\n", __FUNCTION__, \
			 (tsk_ctl)->proc_name, (tsk_ctl)->thr_pid)); \
	else \
		DBG_THR(("%s(): thread:%s:%lx terminated OK\n", __FUNCTION__, \
			 (tsk_ctl)->proc_name, (tsk_ctl)->thr_pid)); \
	(tsk_ctl)->parent = NULL; \
	(tsk_ctl)->proc_name = NULL;  \
	(tsk_ctl)->thr_pid = -1; \
	(tsk_ctl)->up_cnt = 0; \
}

#define PROC_STOP_USING_BINARY_SEMA(tsk_ctl) \
{ \
	uint timeout = (uint)msecs_to_jiffies(PROC_WAIT_TIMEOUT_MSEC); \
	(tsk_ctl)->terminated = TRUE; \
	smp_wmb(); \
	binary_sema_up(tsk_ctl);	\
	DBG_THR(("%s(): thread:%s:%lx wait for terminate\n", __FUNCTION__, \
			 (tsk_ctl)->proc_name, (tsk_ctl)->thr_pid)); \
	timeout = (uint)wait_for_completion_timeout(&((tsk_ctl)->completed), timeout); \
	if (timeout == 0) \
		DBG_THR(("%s(): thread:%s:%lx terminate timeout\n", __FUNCTION__, \
			 (tsk_ctl)->proc_name, (tsk_ctl)->thr_pid)); \
	else \
		DBG_THR(("%s(): thread:%s:%lx terminated OK\n", __FUNCTION__, \
			 (tsk_ctl)->proc_name, (tsk_ctl)->thr_pid)); \
	(tsk_ctl)->parent = NULL; \
	(tsk_ctl)->proc_name = NULL;  \
	(tsk_ctl)->thr_pid = -1; \
}

/*
* Flush is non-rentrant, so callers must make sure
* there is no race condition.
* For safer exit, added wait_for_completion_timeout
* with 1 sec timeout.
*/
#define PROC_FLUSH_USING_BINARY_SEMA(tsk_ctl) \
{ \
	uint timeout = (uint)msecs_to_jiffies(PROC_WAIT_TIMEOUT_MSEC); \
	(tsk_ctl)->flush_ind = TRUE; \
	smp_wmb(); \
	binary_sema_up(tsk_ctl);	\
	DBG_THR(("%s(): thread:%s:%lx wait for flush\n", __FUNCTION__, \
			 (tsk_ctl)->proc_name, (tsk_ctl)->thr_pid)); \
	timeout = (uint)wait_for_completion_timeout(&((tsk_ctl)->flushed), timeout); \
	if (timeout == 0) \
		DBG_THR(("%s(): thread:%s:%lx flush timeout\n", __FUNCTION__, \
			 (tsk_ctl)->proc_name, (tsk_ctl)->thr_pid)); \
	else \
		DBG_THR(("%s(): thread:%s:%lx flushed OK\n", __FUNCTION__, \
			 (tsk_ctl)->proc_name, (tsk_ctl)->thr_pid)); \
}

/*  ----------------------- */

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 31))
#define KILL_PROC(nr, sig) \
{ \
struct task_struct *tsk; \
struct pid *pid;    \
pid = find_get_pid((pid_t)nr);    \
tsk = pid_task(pid, PIDTYPE_PID);    \
if (tsk) send_sig(sig, tsk, 1); \
}
#else
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)) && (LINUX_VERSION_CODE <= \
	KERNEL_VERSION(2, 6, 30))
#define KILL_PROC(pid, sig) \
{ \
	struct task_struct *tsk; \
	tsk = find_task_by_vpid(pid); \
	if (tsk) send_sig(sig, tsk, 1); \
}
#else
#define KILL_PROC(pid, sig) \
{ \
	kill_proc(pid, sig, 1); \
}
#endif
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 31) */

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 0))
#include <linux/time.h>
#include <linux/wait.h>
#else
#include <linux/sched.h>

#define __wait_event_interruptible_timeout(wq, condition, ret)		\
do {									\
	wait_queue_t __wait;						\
	init_waitqueue_entry(&__wait, current);				\
									\
	add_wait_queue(&wq, &__wait);					\
	for (;;) {							\
		set_current_state(TASK_INTERRUPTIBLE);			\
		if (condition)						\
			break;						\
		if (!signal_pending(current)) {				\
			ret = schedule_timeout(ret);			\
			if (!ret)					\
				break;					\
			continue;					\
		}							\
		ret = -ERESTARTSYS;					\
		break;							\
	}								\
	current->state = TASK_RUNNING;					\
	remove_wait_queue(&wq, &__wait);				\
} while (0)

#define wait_event_interruptible_timeout(wq, condition, timeout)	\
({									\
	long __ret = timeout;						\
	if (!(condition))						\
		__wait_event_interruptible_timeout(wq, condition, __ret); \
	__ret;								\
})

#endif /* (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 0)) */

/*
For < 2.6.24, wl creates its own netdev but doesn't
align the priv area like the genuine alloc_netdev().
Since netdev_priv() always gives us the aligned address, it will
not match our unaligned address for < 2.6.24
*/
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 24))
#define DEV_PRIV(dev)	(dev->priv)
#else
#define DEV_PRIV(dev)	netdev_priv(dev)
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 20)
#define WL_ISR(i, d, p)         wl_isr((i), (d))
#else
#define WL_ISR(i, d, p)         wl_isr((i), (d), (p))
#endif  /* < 2.6.20 */

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 0))
#define netdev_priv(dev) dev->priv
#endif /* (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 0)) */

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 25))
#define CAN_SLEEP()	((!in_atomic() && !irqs_disabled()))
#else
#define CAN_SLEEP()	(FALSE)
#endif

#define KMALLOC_FLAG (CAN_SLEEP() ? GFP_KERNEL: GFP_ATOMIC)

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 170))
#define RANDOM32	get_random_u32
#define RANDOM_BYTES    get_random_bytes
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0))
#define RANDOM32	prandom_u32
#define RANDOM_BYTES    prandom_bytes
#else
#define RANDOM32	random32
#define RANDOM_BYTES    get_random_bytes
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0) */

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0))
#define SRANDOM32(entropy)	prandom_seed(entropy)
#else
#define SRANDOM32(entropy)	srandom32(entropy)
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0) */

/*
 * Overide latest kfifo functions with
 * older version to work on older kernels
 */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 33)) && !defined(WL_COMPAT_WIRELESS)
#define kfifo_in_spinlocked(a, b, c, d)		kfifo_put(a, (u8 *)b, c)
#define kfifo_out_spinlocked(a, b, c, d)	kfifo_get(a, (u8 *)b, c)
#define kfifo_esize(a)				1
#elif (LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 32)) && \
	(LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 36)) &&	!defined(WL_COMPAT_WIRELESS)
#define kfifo_in_spinlocked(a, b, c, d)		kfifo_in_locked(a, b, c, d)
#define kfifo_out_spinlocked(a, b, c, d)	kfifo_out_locked(a, b, c, d)
#define kfifo_esize(a)				1
#endif /* (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 33)) */

#if __GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 6)
#pragma GCC diagnostic pop
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 9, 0))
static inline struct inode *file_inode(const struct file *f)
{
	return f->f_dentry->d_inode;
}
#endif /* (LINUX_VERSION_CODE < KERNEL_VERSION(3, 9, 0)) */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0) */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
#define vfs_write(fp, buf, len, pos) kernel_write(fp, buf, len, pos)
#define vfs_read(fp, buf, len, pos) kernel_read(fp, buf, len, pos)
int kernel_read_compat(struct file *file, loff_t offset, char *addr, unsigned long count);
#else /* LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0) */
#define kernel_read_compat(file, offset, addr, count) kernel_read(file, offset, addr, count)
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0) */

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 17, 0)
#define timespec64 timespec
#define ktime_get_real_ts64(timespec) ktime_get_real_ts(timespec)
#define ktime_to_timespec64(timespec) ktime_to_timespec(timespec)
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(3, 17, 0) */

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 6, 0)) && (LINUX_VERSION_CODE >= \
	KERNEL_VERSION(4, 20, 0))
static inline void get_monotonic_boottime(struct timespec *ts)
{
	*ts = ktime_to_timespec(ktime_get_boottime());
}
#endif /* LINUX_VER >= 4.20 */

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 6, 0)) && (LINUX_VERSION_CODE >= \
	KERNEL_VERSION(5, 0, 0))
static inline void do_gettimeofday(struct timeval *tv)
{
	struct timespec64 now;

	ktime_get_real_ts64(&now);
	tv->tv_sec = now.tv_sec;
	tv->tv_usec = now.tv_nsec/1000;
}
#endif /* LINUX_VER >= 5.0 */

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
#define GETFS_AND_SETFS_TO_KERNEL_DS(fs) \
{ \
	fs = get_fs(); \
	set_fs(KERNEL_DS); \
}

#define SETFS(fs) set_fs(fs)
#define MM_SEGMENT_T mm_segment_t
#else
/* From 5.10 kernel get/set_fs are obsolete and direct kernel_read/write operations can be used */
#define GETFS_AND_SETFS_TO_KERNEL_DS(fs) BCM_REFERENCE(fs)
#define SETFS(fs) BCM_REFERENCE(fs)
#define MM_SEGMENT_T void *
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0) */

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 17, 0))
#define NETDEV_ADDR_SET(net, dst_len, addr, src_len) \
	__dev_addr_set(net, addr, dst_len)
#else
#define NETDEV_ADDR_SET(net, dst_len, addr, src_len) \
	(void)memcpy_s(net->dev_addr, dst_len, addr, src_len)
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(5, 17, 0) */

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 17, 0))
#define KTHREAD_COMPLETE_AND_EXIT(comp, code) kthread_complete_and_exit(comp, code)
#else
#define KTHREAD_COMPLETE_AND_EXIT(comp, code) complete_and_exit(comp, code)
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(5, 17, 0) */

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0))
#define DHD_DMA_FREE_COHERENT(pdev, size, va, paddr) \
	dma_free_coherent(&((struct pci_dev *)pdev)->dev, size, va, paddr)
#define DHD_DMA_SET_MASK(pdev, mask) \
	dma_set_mask(&((struct pci_dev *)pdev)->dev, mask)
#define DHD_DMA_SET_COHERENT_MASK(pdev, mask) \
	dma_set_coherent_mask(&((struct pci_dev *)pdev)->dev, mask)
#define DHD_DMA_MAPPING_ERROR(pdev, addr) \
	dma_mapping_error(&((struct pci_dev *)pdev)->dev, addr)
#define DHD_DMA_MAP_SINGLE(pdev, size, m_addr, dir) \
	dma_map_single(&((struct pci_dev *)pdev)->dev, size, m_addr, dir)
#define DHD_DMA_UNMAP_SINGLE(pdev, size, m_addr, dir) \
	dma_unmap_single(&((struct pci_dev *)pdev)->dev, size, m_addr, dir)
#else
#define DHD_DMA_FREE_COHERENT(pdev, size, va, paddr)	pci_free_consistent(pdev, size, va, paddr)
#define DHD_DMA_SET_MASK(pdev, mask)			pci_set_dma_mask(pdev, mask)
#define DHD_DMA_SET_COHERENT_MASK(pdev, mask)		pci_set_consistent_dma_mask(pdev, mask)
#define DHD_DMA_MAPPING_ERROR(pdev, addr)		pci_dma_mapping_error(pdev, addr)
#define DHD_DMA_MAP_SINGLE(pdev, size, m_addr, dir)	pci_map_single(pdev, size, m_addr, dir)
#define DHD_DMA_UNMAP_SINGLE(pdev, size, m_addr, dir)	pci_unmap_single(pdev, size, m_addr, dir)
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0) */

#endif /* _linuxver_h_ */
