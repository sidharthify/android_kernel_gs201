// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2018-2024 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 *
 */

#include <linux/kthread.h>

#include <mali_kbase.h>
#include "mali_kbase_config_defaults.h"
#include <mali_kbase_ctx_sched.h>
#include <mali_kbase_reset_gpu.h>
#include <mali_kbase_as_fault_debugfs.h>
#include "mali_kbase_csf.h"
#include <tl/mali_kbase_tracepoints.h>
#include <backend/gpu/mali_kbase_pm_internal.h>
#include <linux/export.h>
#include <linux/delay.h>
#include <csf/mali_kbase_csf_registers.h>
#include <uapi/gpu/arm/midgard/mali_base_kernel.h>
#include <mali_kbase_hwaccess_time.h>
#include <trace/events/power.h>
#include "mali_kbase_csf_tiler_heap.h"
#include "mali_kbase_csf_tiler_heap_reclaim.h"
#include "mali_kbase_csf_mcu_shared_reg.h"
#include <linux/version_compat_defs.h>
#include <hwcnt/mali_kbase_hwcnt_context.h>
#if IS_ENABLED(CONFIG_MALI_TRACE_POWER_GPU_WORK_PERIOD)
#include <mali_kbase_gpu_metrics.h>
#include <csf/mali_kbase_csf_trace_buffer.h>
#endif /* CONFIG_MALI_TRACE_POWER_GPU_WORK_PERIOD */
#include <uapi/gpu/arm/midgard/platform/pixel/pixel_memory_group_manager.h>



/* Value to indicate that a queue group is not groups_to_schedule list */
#define KBASEP_GROUP_PREPARED_SEQ_NUM_INVALID (U32_MAX)

/* This decides the upper limit on the waiting time for the Scheduler
 * to exit the sleep state. Usually the value of autosuspend_delay is
 * expected to be around 100 milli seconds.
 */
#define MAX_AUTO_SUSPEND_DELAY_MS (5000)

/* Maximum number of endpoints which may run tiler jobs. */
#define CSG_TILER_MAX ((u8)1)

/* Maximum dynamic CSG slot priority value */
#define MAX_CSG_SLOT_PRIORITY ((u8)15)

/* CSF scheduler time slice value */
#define CSF_SCHEDULER_TIME_TICK_MS (100) /* 100 milliseconds */

/* CSG_REQ:STATUS_UPDATE timeout */
#define CSG_STATUS_UPDATE_REQ_TIMEOUT_MS (250) /* 250 milliseconds */

/* A GPU address space slot is reserved for MCU. */
#define NUM_RESERVED_AS_SLOTS (1)

/* Time to wait for completion of PING req before considering MCU as hung */
#define FW_PING_AFTER_ERROR_TIMEOUT_MS (10)

/* Time to wait for completion of PING request before considering MCU as hung
 * when GPU reset is triggered during protected mode.
 */
#define FW_PING_ON_GPU_RESET_IN_PMODE_TIMEOUT_MS (500)


/* Explicitly defining this blocked_reason code as SB_WAIT for clarity */
#define CS_STATUS_BLOCKED_ON_SB_WAIT CS_STATUS_BLOCKED_REASON_REASON_WAIT

static int scheduler_group_schedule(struct kbase_queue_group *group);
static void remove_group_from_idle_wait(struct kbase_queue_group *const group);
static void insert_group_to_runnable(struct kbase_csf_scheduler *const scheduler,
				     struct kbase_queue_group *const group,
				     enum kbase_csf_group_state run_state);
static struct kbase_queue_group *
scheduler_get_protm_enter_async_group(struct kbase_device *const kbdev,
				      struct kbase_queue_group *const group);
static struct kbase_queue_group *get_tock_top_group(struct kbase_csf_scheduler *const scheduler);
static void scheduler_enable_tick_timer_nolock(struct kbase_device *kbdev);
static int suspend_active_queue_groups(struct kbase_device *kbdev, unsigned long *slot_mask,
				       bool reset);
static int suspend_active_groups_on_powerdown(struct kbase_device *kbdev, bool system_suspend);
static void schedule_in_cycle(struct kbase_queue_group *group, bool force);
static bool queue_group_scheduled_locked(struct kbase_queue_group *group);

#define kctx_as_enabled(kctx) (!kbase_ctx_flag(kctx, KCTX_AS_DISABLED_ON_FAULT))

bool is_gpu_level_suspend_supported(struct kbase_device *const kbdev)
{
	return false;
}

#if IS_ENABLED(CONFIG_MALI_TRACE_POWER_GPU_WORK_PERIOD)
/**
 * gpu_metrics_ctx_init() - Take a reference on GPU metrics context if it exists,
 *                          otherwise allocate and initialise one.
 *
 * @kctx: Pointer to the Kbase context.
 *
 * The GPU metrics context represents an "Application" for the purposes of GPU metrics
 * reporting. There may be multiple kbase_contexts contributing data to a single GPU
 * metrics context.
 * This function takes a reference on GPU metrics context if it already exists
 * corresponding to the Application that is creating the Kbase context, otherwise
 * memory is allocated for it and initialised.
 *
 * Return: 0 on success, or negative on failure.
 */
static inline int gpu_metrics_ctx_init(struct kbase_context *kctx)
{
	struct kbase_gpu_metrics_ctx *gpu_metrics_ctx;
	struct kbase_device *kbdev = kctx->kbdev;
	int ret = 0;

	const struct cred *cred = get_current_cred();
	const unsigned int aid = cred->euid.val;

	put_cred(cred);

	/* Return early if this is not a Userspace created context */
	if (unlikely(!kctx->filp))
		return 0;

	/* Serialize against the other threads trying to create/destroy Kbase contexts. */
	mutex_lock(&kbdev->kctx_list_lock);
	spin_lock_bh(&kbdev->csf.scheduler.gpu_metrics_lock);
	gpu_metrics_ctx = kbase_gpu_metrics_ctx_get(kbdev, aid);
	spin_unlock_bh(&kbdev->csf.scheduler.gpu_metrics_lock);

	if (!gpu_metrics_ctx) {
		gpu_metrics_ctx = kmalloc(sizeof(*gpu_metrics_ctx), GFP_KERNEL);

		if (gpu_metrics_ctx) {
			spin_lock_bh(&kbdev->csf.scheduler.gpu_metrics_lock);
			kbase_gpu_metrics_ctx_init(kbdev, gpu_metrics_ctx, aid);
			spin_unlock_bh(&kbdev->csf.scheduler.gpu_metrics_lock);
		} else {
			dev_err(kbdev->dev, "Allocation for gpu_metrics_ctx failed");
			ret = -ENOMEM;
		}
	}

	kctx->gpu_metrics_ctx = gpu_metrics_ctx;
	mutex_unlock(&kbdev->kctx_list_lock);

	return ret;
}

/**
 * gpu_metrics_ctx_term() - Drop a reference on a GPU metrics context and free it
 *                          if the refcount becomes 0.
 *
 * @kctx: Pointer to the Kbase context.
 */
static inline void gpu_metrics_ctx_term(struct kbase_context *kctx)
{
	/* Return early if this is not a Userspace created context */
	if (unlikely(!kctx->filp))
		return;

	/* Serialize against the other threads trying to create/destroy Kbase contexts. */
	mutex_lock(&kctx->kbdev->kctx_list_lock);
	spin_lock_bh(&kctx->kbdev->csf.scheduler.gpu_metrics_lock);
	kbase_gpu_metrics_ctx_put(kctx->kbdev, kctx->gpu_metrics_ctx);
	spin_unlock_bh(&kctx->kbdev->csf.scheduler.gpu_metrics_lock);
	mutex_unlock(&kctx->kbdev->kctx_list_lock);
}

/**
 * struct gpu_metrics_event - A GPU metrics event recorded in trace buffer.
 *
 * @csg_slot_act:  The 32bit data consisting of a GPU metrics event.
 *                 5 bits[4:0] represents CSG slot number.
 *                 1 bit [5]  represents the transition of the CSG group on the slot.
 *                            '1' means idle->active whilst '0' does active->idle.
 * @timestamp:     64bit timestamp consisting of a GPU metrics event.
 *
 * Note: It's packed and word-aligned as agreed layout with firmware.
 */
struct gpu_metrics_event {
	u32 csg_slot_act;
	u64 timestamp;
} __packed __aligned(4);
#define GPU_METRICS_EVENT_SIZE sizeof(struct gpu_metrics_event)

#define GPU_METRICS_ACT_SHIFT 5
#define GPU_METRICS_ACT_MASK (0x1 << GPU_METRICS_ACT_SHIFT)
#define GPU_METRICS_ACT_GET(val) (((val)&GPU_METRICS_ACT_MASK) >> GPU_METRICS_ACT_SHIFT)

#define GPU_METRICS_CSG_MASK 0x1f
#define GPU_METRICS_CSG_GET(val) ((val)&GPU_METRICS_CSG_MASK)

/**
 * gpu_metrics_read_event() - Read a GPU metrics trace from trace buffer
 *
 * @kbdev:    Pointer to the device
 * @kctx:     Kcontext that is derived from CSG slot field of a GPU metrics.
 * @prev_act: Previous CSG activity transition in a GPU metrics.
 * @cur_act:  Current CSG activity transition in a GPU metrics.
 * @ts:       CSG activity transition timestamp in a GPU metrics.
 *
 * This function reads firmware trace buffer, named 'gpu_metrics' and
 * parse one 12-byte data packet into following information.
 * - The number of CSG slot on which CSG was transitioned to active or idle.
 * - Activity transition (1: idle->active, 0: active->idle).
 * - Timestamp in nanoseconds when the transition occurred.
 *
 * Return: true on success.
 */
static bool gpu_metrics_read_event(struct kbase_device *kbdev, struct kbase_context **kctx,
				   bool *prev_act, bool *cur_act, uint64_t *ts)
{
	struct firmware_trace_buffer *tb = kbdev->csf.scheduler.gpu_metrics_tb;
	struct gpu_metrics_event e;

	if (kbase_csf_firmware_trace_buffer_read_data(tb, (u8 *)&e, GPU_METRICS_EVENT_SIZE) ==
	    GPU_METRICS_EVENT_SIZE) {
		const u8 slot = GPU_METRICS_CSG_GET(e.csg_slot_act);
		struct kbase_queue_group *group;

		if (WARN_ON_ONCE(slot >= kbdev->csf.global_iface.group_num)) {
			dev_err(kbdev->dev, "invalid CSG slot (%u)", slot);
			pixel_gpu_uevent_kmd_error_send(kbdev, GPU_UEVENT_INFO_TRACE_BUF_INVALID_SLOT);
			return false;
		}

		group = kbdev->csf.scheduler.csg_slots[slot].resident_group;

		if (unlikely(!group)) {
			dev_err(kbdev->dev, "failed to find CSG group from CSG slot (%u)", slot);
			return false;
		}

		*cur_act = GPU_METRICS_ACT_GET(e.csg_slot_act);
		*ts = kbase_backend_time_convert_gpu_to_cpu(kbdev, e.timestamp);
		*kctx = group->kctx;

		*prev_act = group->prev_act;
		group->prev_act = *cur_act;

		return true;
	}

	dev_err(kbdev->dev, "failed to read a GPU metrics from trace buffer");

	return false;
}

/**
 * drain_gpu_metrics_trace_buffer() - Drain the "gpu_metrics" trace buffer
 *
 * @kbdev: Pointer to the device
 *
 * This function is called to drain the "gpu_metrics" trace buffer. As per the events
 * read from trace buffer, the start and end of GPU activity for different GPU metrics
 * context is reported to the frontend.
 *
 * Return: Timestamp in nanoseconds to be provided to the frontend for emitting the
 *         tracepoint.
 */
static u64 drain_gpu_metrics_trace_buffer(struct kbase_device *kbdev)
{
	u64 system_time = 0;
	u64 ts_before_drain;
	u64 ts = 0;

	lockdep_assert_held(&kbdev->csf.scheduler.gpu_metrics_lock);

	kbase_backend_get_gpu_time_norequest(kbdev, NULL, &system_time, NULL);
	/* CPU time value that was used to derive the parameters for time conversion,
	 * was retrieved from ktime_get_raw_ts64(). But the tracing subsystem would use
	 * ktime_get_raw_fast_ns() to assign timestamp to the gpu metrics tracepoints
	 * if MONOTONIC_RAW clock source is used. This can potentially cause 'end_time_ns'
	 * value emitted for a tracepoint to be greater than the tracepoint emission time.
	 * As per the kernel doc ktime_get_raw_fast_ns() isn't supposed to be called by
	 * the drivers. So it would be used only if really needed.
	 */
	ts_before_drain = kbase_backend_time_convert_gpu_to_cpu(kbdev, system_time);

	while (!kbase_csf_firmware_trace_buffer_is_empty(kbdev->csf.scheduler.gpu_metrics_tb)) {
		struct kbase_context *kctx;
		bool prev_act;
		bool cur_act;

		if (gpu_metrics_read_event(kbdev, &kctx, &prev_act, &cur_act, &ts)) {
			if (prev_act == cur_act) {
				/* Error handling
				 *
				 * In case of active CSG, Kbase will try to recover the
				 * lost event by ending previously active event and
				 * starting a new one.
				 *
				 * In case of inactive CSG, the event is drop as Kbase
				 * cannot recover.
				 */
				dev_err(kbdev->dev,
					"Invalid activity state transition. (prev_act = %u, cur_act = %u)",
					prev_act, cur_act);
				if (cur_act) {
					kbase_gpu_metrics_ctx_end_activity(kctx, ts);
					kbase_gpu_metrics_ctx_start_activity(kctx, ts);
				}
			} else {
				/* Normal handling */
				if (cur_act)
					kbase_gpu_metrics_ctx_start_activity(kctx, ts);
				else
					kbase_gpu_metrics_ctx_end_activity(kctx, ts);
			}
		} else
			break;
	}

	return (ts >= ts_before_drain ? ts + 1 : ts_before_drain);
}

/**
 * emit_gpu_metrics_to_frontend() - Emit GPU metrics events to the frontend.
 *
 * @kbdev: Pointer to the device
 *
 * This function must be called only from the timer callback function to emit GPU metrics
 * data to the frontend.
 */
static void emit_gpu_metrics_to_frontend(struct kbase_device *kbdev)
{
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;
	u64 ts;

#ifdef CONFIG_MALI_DEBUG
	WARN_ON_ONCE(!in_serving_softirq());
#endif

#if IS_ENABLED(CONFIG_MALI_NO_MALI)
	return;
#endif

	spin_lock(&scheduler->gpu_metrics_lock);
	ts = drain_gpu_metrics_trace_buffer(kbdev);
	kbase_gpu_metrics_emit_tracepoint(kbdev, ts);
	spin_unlock(&scheduler->gpu_metrics_lock);
}

/**
 * emit_gpu_metrics_to_frontend_for_off_slot_group() - Emit GPU metrics events to the frontend
 *                                                     after a group went off the CSG slot.
 *
 * @group: Pointer to the queue group that went off slot.
 *
 * This function must be called after the CSG suspend/terminate request has completed
 * and before the mapping between the queue group and CSG slot is removed.
 */
static void emit_gpu_metrics_to_frontend_for_off_slot_group(struct kbase_queue_group *group)
{
	struct kbase_device *kbdev = group->kctx->kbdev;
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;
	u64 ts;

	lockdep_assert_held(&scheduler->lock);
	lockdep_assert_held(&scheduler->gpu_metrics_lock);

#if IS_ENABLED(CONFIG_MALI_NO_MALI)
	return;
#endif

	ts = drain_gpu_metrics_trace_buffer(kbdev);
	/* If the group is marked as active even after going off the slot, then it implies
	 * that the CSG suspend/terminate request didn't complete.
	 */
	if (unlikely(group->prev_act)) {
		kbase_gpu_metrics_ctx_end_activity(group->kctx, ts);
		group->prev_act = 0;
	}
	kbase_gpu_metrics_emit_tracepoint(kbdev, ts);
}

/**
 * gpu_metrics_timer_callback() - Callback function for the GPU metrics hrtimer
 *
 * @timer: Pointer to the GPU metrics hrtimer
 *
 * This function will emit power/gpu_work_period tracepoint for all the active
 * GPU metrics contexts. The timer will be restarted if needed.
 *
 * Return: enum value to indicate that timer should not be restarted.
 */
static enum hrtimer_restart gpu_metrics_timer_callback(struct hrtimer *timer)
{
	struct kbase_device *kbdev =
		container_of(timer, struct kbase_device, csf.scheduler.gpu_metrics_timer);

	emit_gpu_metrics_to_frontend(kbdev);
	hrtimer_start(&kbdev->csf.scheduler.gpu_metrics_timer,
		      HR_TIMER_DELAY_NSEC(kbase_gpu_metrics_get_tp_emit_interval()),
		      HRTIMER_MODE_REL_SOFT);
	return HRTIMER_NORESTART;
}
#endif /* CONFIG_MALI_TRACE_POWER_GPU_WORK_PERIOD */

/**
 * wait_for_dump_complete_on_group_deschedule() - Wait for dump on fault and
 *              scheduling tick/tock to complete before the group deschedule.
 *
 * @group: Pointer to the group that is being descheduled.
 *
 * This function blocks the descheduling of the group until the dump on fault is
 * completed and scheduling tick/tock has completed.
 * To deschedule an on slot group CSG termination request would be sent and that
 * might time out if the fault had occurred and also potentially affect the state
 * being dumped. Moreover the scheduler lock would be held, so the access to debugfs
 * files would get blocked.
 * Scheduler lock and 'kctx->csf.lock' are released before this function starts
 * to wait. When a request sent by the Scheduler to the FW times out, Scheduler
 * would also wait for the dumping to complete and release the Scheduler lock
 * before the wait. Meanwhile Userspace can try to delete the group, this function
 * would ensure that the group doesn't exit the Scheduler until scheduling
 * tick/tock has completed. Though very unlikely, group deschedule can be triggered
 * from multiple threads around the same time and after the wait Userspace thread
 * can win the race and get the group descheduled and free the memory for group
 * pointer before the other threads wake up and notice that group has already been
 * descheduled. To avoid the freeing in such a case, a sort of refcount is used
 * for the group which is incremented & decremented across the wait.
 */
static void wait_for_dump_complete_on_group_deschedule(struct kbase_queue_group *group)
{
#if IS_ENABLED(CONFIG_DEBUG_FS)
	struct kbase_device *kbdev = group->kctx->kbdev;
	struct kbase_context *kctx = group->kctx;
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;

	lockdep_assert_held(&kctx->csf.lock);
	lockdep_assert_held(&scheduler->lock);

	if (likely(!kbase_debug_csf_fault_dump_enabled(kbdev)))
		return;

	while ((!kbase_debug_csf_fault_dump_complete(kbdev) || (scheduler->state == SCHED_BUSY)) &&
	       queue_group_scheduled_locked(group)) {
		group->deschedule_deferred_cnt++;
		rt_mutex_unlock(&scheduler->lock);
		rt_mutex_unlock(&kctx->csf.lock);
		kbase_debug_csf_fault_wait_completion(kbdev);
		rt_mutex_lock(&kctx->csf.lock);
		rt_mutex_lock(&scheduler->lock);
		group->deschedule_deferred_cnt--;
	}
#endif
}

/**
 * schedule_actions_trigger_df() - Notify the client about the fault and
 *                                 wait for the dumping to complete.
 *
 * @kbdev: Pointer to the device
 * @kctx:  Pointer to the context associated with the CSG slot for which
 *         the timeout was seen.
 * @error: Error code indicating the type of timeout that occurred.
 *
 * This function notifies the Userspace client waiting for the faults and wait
 * for the Client to complete the dumping.
 * The function is mainly called from Scheduling tick/tock when a request sent by
 * the Scheduler to FW times out. It can be called outside the tick/tock when timeout
 * happens in the following 3 cases :-
 * - Entry to protected mode is initiated from protm event work item.
 * - Forced exit from protected mode is triggered when GPU queue of an on-slot group is kicked.
 * - CSG termination request is sent when Userspace tries to delete the queue group.
 * In the latter 3 cases there is no wait done as scheduler lock would be released
 * immediately. In the tick/tock case the function waits and releases the scheduler
 * lock before the wait. It has been ensured that the Scheduler view of the groups
 * won't change meanwhile, so no group can enter/exit the Scheduler, become
 * runnable or go off slot.
 */
static void schedule_actions_trigger_df(struct kbase_device *kbdev, struct kbase_context *kctx,
					enum dumpfault_error_type error)
{
#if IS_ENABLED(CONFIG_DEBUG_FS)
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;

	lockdep_assert_held(&scheduler->lock);

	if (!kbase_debug_csf_fault_notify(kbdev, kctx, error))
		return;

	/* Return early if the function was called outside the tick/tock */
	if (unlikely(scheduler->state != SCHED_BUSY))
		return;

	rt_mutex_unlock(&scheduler->lock);
	kbase_debug_csf_fault_wait_completion(kbdev);
	rt_mutex_lock(&scheduler->lock);
	WARN_ON(scheduler->state != SCHED_BUSY);
#endif
}

#ifdef KBASE_PM_RUNTIME
/**
 * wait_for_scheduler_to_exit_sleep() - Wait for Scheduler to exit the
 *                                      sleeping state.
 *
 * @kbdev: Pointer to the device
 *
 * This function waits until the Scheduler has exited the sleep state and
 * it is called when an on-slot group is terminated or when the suspend
 * buffer of an on-slot group needs to be captured.
 *
 * Return: 0 when the wait is successful, otherwise an error code.
 */
static int wait_for_scheduler_to_exit_sleep(struct kbase_device *kbdev)
{
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;
	int autosuspend_delay = kbdev->dev->power.autosuspend_delay;
	unsigned int sleep_exit_wait_time;
	long remaining;
	int ret = 0;

	lockdep_assert_held(&scheduler->lock);
	WARN_ON(scheduler->state != SCHED_SLEEPING);

	/* No point in waiting if autosuspend_delay value is negative.
	 * For the negative value of autosuspend_delay Driver will directly
	 * go for the suspend of Scheduler, but the autosuspend_delay value
	 * could have been changed after the sleep was initiated.
	 */
	if (autosuspend_delay < 0)
		return -EINVAL;

	if (autosuspend_delay > MAX_AUTO_SUSPEND_DELAY_MS)
		autosuspend_delay = MAX_AUTO_SUSPEND_DELAY_MS;

	/* Usually Scheduler would remain in sleeping state until the
	 * auto-suspend timer expires and all active CSGs are suspended.
	 */
	sleep_exit_wait_time = (u32)autosuspend_delay + kbdev->reset_timeout_ms;

	remaining = kbase_csf_timeout_in_jiffies(sleep_exit_wait_time);

	while ((scheduler->state == SCHED_SLEEPING) && !ret) {
		rt_mutex_unlock(&scheduler->lock);
		remaining = kbase_csf_fw_io_wait_event_timeout(&kbdev->csf.fw_io,
							       kbdev->csf.event_wait,
							       (scheduler->state != SCHED_SLEEPING),
							       remaining);
		rt_mutex_lock(&scheduler->lock);
		if (!remaining && (scheduler->state == SCHED_SLEEPING))
			ret = -ETIMEDOUT;
	}

	return ret;
}

/**
 * force_scheduler_to_exit_sleep() - Force scheduler to exit sleep state
 *
 * @kbdev: Pointer to the device
 *
 * This function will force the Scheduler to exit the sleep state by doing the
 * wake up of MCU and suspension of on-slot groups. It is called at the time of
 * system suspend.
 *
 * Return: 0 on success.
 */
static int force_scheduler_to_exit_sleep(struct kbase_device *kbdev)
{
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;
	unsigned long flags;
	int ret = 0;

	lockdep_assert_held(&scheduler->lock);
	WARN_ON(scheduler->state != SCHED_SLEEPING);
	WARN_ON(!kbdev->pm.backend.gpu_sleep_mode_active);

	kbase_pm_lock(kbdev);
	ret = kbase_pm_force_mcu_wakeup_after_sleep(kbdev);
	kbase_pm_unlock(kbdev);
	if (ret) {
		dev_warn(kbdev->dev,
			 "[%llu] Wait for MCU wake up failed on forced scheduler suspend",
			 kbase_backend_get_cycle_cnt(kbdev));
		goto out;
	}

	ret = suspend_active_groups_on_powerdown(kbdev, true);
	if (ret)
		goto out;

	kbase_pm_lock(kbdev);
	spin_lock_irqsave(&kbdev->hwaccess_lock, flags);
	kbdev->pm.backend.gpu_sleep_mode_active = false;
	kbdev->pm.backend.gpu_wakeup_override = false;
	kbase_pm_update_state(kbdev);
	spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);
	ret = kbase_pm_wait_for_desired_state(kbdev);
	kbase_pm_unlock(kbdev);
	if (ret) {
		dev_warn(kbdev->dev,
			 "[%llu] Wait for pm state change failed on forced scheduler suspend",
			 kbase_backend_get_cycle_cnt(kbdev));
		goto out;
	}

	scheduler->state = SCHED_SUSPENDED;
#if IS_ENABLED(CONFIG_MALI_TRACE_POWER_GPU_WORK_PERIOD)
	hrtimer_cancel(&scheduler->gpu_metrics_timer);
#endif
	KBASE_KTRACE_ADD(kbdev, SCHED_SUSPENDED, NULL, scheduler->state);

	return 0;

out:
	spin_lock_irqsave(&kbdev->hwaccess_lock, flags);
	kbdev->pm.backend.exit_gpu_sleep_mode = true;
	kbdev->pm.backend.gpu_wakeup_override = false;
	spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);
	kbase_csf_scheduler_invoke_tick(kbdev);

	return ret;
}
#endif

/**
 * tick_timer_callback() - Callback function for the scheduling tick hrtimer
 *
 * @timer: Pointer to the scheduling tick hrtimer
 *
 * This function will wake up kbase_csf_scheduler_kthread() to process a
 * pending scheduling tick. It will be restarted manually once a tick has been
 * processed if appropriate.
 *
 * Return: enum value to indicate that timer should not be restarted.
 */
static enum hrtimer_restart tick_timer_callback(struct hrtimer *timer)
{
	struct kbase_device *kbdev =
		container_of(timer, struct kbase_device, csf.scheduler.tick_timer);

	kbase_csf_scheduler_invoke_tick(kbdev);

	return HRTIMER_NORESTART;
}

static void release_doorbell(struct kbase_device *kbdev, int doorbell_nr)
{
	WARN_ON(doorbell_nr >= kbdev->csf.num_doorbells);

	lockdep_assert_held(&kbdev->csf.scheduler.lock);
	clear_bit(doorbell_nr, kbdev->csf.scheduler.doorbell_inuse_bitmap);
}

static int acquire_doorbell(struct kbase_device *kbdev)
{
	int doorbell_nr;
	const int doorbell_count = kbdev->csf.num_doorbells;

	lockdep_assert_held(&kbdev->csf.scheduler.lock);

	doorbell_nr =
		find_first_zero_bit(kbdev->csf.scheduler.doorbell_inuse_bitmap, doorbell_count);

	if (doorbell_nr >= doorbell_count)
		return KBASEP_USER_DB_NR_INVALID;

	set_bit(doorbell_nr, kbdev->csf.scheduler.doorbell_inuse_bitmap);

	return doorbell_nr;
}

static void unassign_user_doorbell_from_group(struct kbase_device *kbdev,
					      struct kbase_queue_group *group)
{
	lockdep_assert_held(&kbdev->csf.scheduler.lock);

	if (group->doorbell_nr != KBASEP_USER_DB_NR_INVALID) {
		release_doorbell(kbdev, group->doorbell_nr);
		group->doorbell_nr = KBASEP_USER_DB_NR_INVALID;
	}
}

static void unassign_user_doorbell_from_queue(struct kbase_device *kbdev, struct kbase_queue *queue)
{
	lockdep_assert_held(&kbdev->csf.scheduler.lock);

	mutex_lock(&kbdev->csf.reg_lock);

	if (queue->doorbell_nr != KBASEP_USER_DB_NR_INVALID) {
		queue->doorbell_nr = KBASEP_USER_DB_NR_INVALID;
		/* After this the dummy page would be mapped in */
		unmap_mapping_range(kbdev->csf.db_filp->f_inode->i_mapping,
				    (loff_t)(queue->db_file_offset << PAGE_SHIFT), PAGE_SIZE, 1);
	}

	mutex_unlock(&kbdev->csf.reg_lock);
}

static void assign_user_doorbell_to_group(struct kbase_device *kbdev,
					  struct kbase_queue_group *group)
{
	lockdep_assert_held(&kbdev->csf.scheduler.lock);

	if (group->doorbell_nr == KBASEP_USER_DB_NR_INVALID)
		group->doorbell_nr = acquire_doorbell(kbdev);
}

static void assign_user_doorbell_to_queue(struct kbase_device *kbdev,
					  struct kbase_queue *const queue)
{
	lockdep_assert_held(&kbdev->csf.scheduler.lock);

	mutex_lock(&kbdev->csf.reg_lock);

	/* If bind operation for the queue hasn't completed yet, then the
	 * CSI can't be programmed for the queue
	 * (even in stopped state) and so the doorbell also can't be assigned
	 * to it.
	 */
	if ((queue->bind_state == KBASE_CSF_QUEUE_BOUND) &&
	    (queue->doorbell_nr == KBASEP_USER_DB_NR_INVALID)) {
		WARN_ON(queue->group->doorbell_nr == KBASEP_USER_DB_NR_INVALID);
		queue->doorbell_nr = queue->group->doorbell_nr;

		/* After this the real Hw doorbell page would be mapped in */
		unmap_mapping_range(kbdev->csf.db_filp->f_inode->i_mapping,
				    (loff_t)(queue->db_file_offset << PAGE_SHIFT), PAGE_SIZE, 1);
	}

	mutex_unlock(&kbdev->csf.reg_lock);
}

static void scheduler_doorbell_init(struct kbase_device *kbdev)
{
	int doorbell_nr;

	bitmap_zero(kbdev->csf.scheduler.doorbell_inuse_bitmap, CSF_NUM_DOORBELL_MAX);

	rt_mutex_lock(&kbdev->csf.scheduler.lock);
	/* Reserve doorbell 0 for use by kernel driver */
	doorbell_nr = acquire_doorbell(kbdev);
	rt_mutex_unlock(&kbdev->csf.scheduler.lock);

	WARN_ON(doorbell_nr != CSF_KERNEL_DOORBELL_NR);
}

/**
 * update_on_slot_queues_offsets - Update active queues' INSERT & EXTRACT ofs
 *
 * @kbdev: Instance of a GPU platform device that implements a CSF interface.
 *
 * This function updates the EXTRACT offset for all queues which groups have
 * been assigned a physical slot. These values could be used to detect a
 * queue's true idleness status. This is intended to be an additional check
 * on top of the GPU idle notification to account for race conditions.
 * This function is supposed to be called only when GPU idle notification
 * interrupt is received.
 */
static void update_on_slot_queues_offsets(struct kbase_device *kbdev)
{
	struct kbase_csf_scheduler *const scheduler = &kbdev->csf.scheduler;
	/* All CSGs have the same number of CSs */
	size_t const max_streams = kbdev->csf.global_iface.groups[0].stream_num;
	size_t i;

	lockdep_assert_held(&scheduler->interrupt_lock);

	/* csg_slots_idle_mask is not used here for the looping, as it could get
	 * updated concurrently when Scheduler re-evaluates the idle status of
	 * the CSGs for which idle notification was received previously.
	 */
	for_each_set_bit(i, scheduler->csg_inuse_bitmap, kbdev->csf.global_iface.group_num) {
		struct kbase_queue_group *const group = scheduler->csg_slots[i].resident_group;
		size_t j;

		if (WARN_ON(!group))
			continue;

		for (j = 0; j < max_streams; ++j) {
			struct kbase_queue *const queue = group->bound_queues[j];

			if (queue && queue->user_io_addr) {
				u64 const *const output_addr =
					(u64 const *)(queue->user_io_addr +
						      PAGE_SIZE / sizeof(u64));

				/*
				 * This 64-bit read will be atomic on a 64-bit kernel but may not
				 * be atomic on 32-bit kernels. Support for 32-bit kernels is
				 * limited to build-only.
				 */
				queue->extract_ofs = output_addr[CS_EXTRACT_LO / sizeof(u64)];
			}
		}
	}
}

static void enqueue_gpu_idle_work(struct kbase_csf_scheduler *const scheduler)
{
	atomic_set(&scheduler->gpu_no_longer_idle, false);
	atomic_inc(&scheduler->pending_gpu_idle_work);
	complete(&scheduler->kthread_signal);
}

void kbase_csf_scheduler_process_gpu_idle_event(struct kbase_device *kbdev)
{
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;
	bool can_suspend_on_idle;

	lockdep_assert_held(&kbdev->hwaccess_lock);
	lockdep_assert_held(&scheduler->interrupt_lock);

	can_suspend_on_idle = kbase_pm_idle_groups_sched_suspendable(kbdev) &&
			      !kbase_pm_is_mcu_inactive(kbdev, kbdev->pm.backend.mcu_state);
	KBASE_KTRACE_ADD(kbdev, SCHEDULER_GPU_IDLE_EVENT_CAN_SUSPEND, NULL,
			 (((u64)can_suspend_on_idle) << 32));

	if (can_suspend_on_idle) {
		/* fast_gpu_idle_handling is protected by the
		 * interrupt_lock, which would prevent this from being
		 * updated whilst gpu_idle_worker() is executing.
		 */
		scheduler->fast_gpu_idle_handling = (kbdev->csf.gpu_idle_hysteresis_ns == 0) ||
						    !kbase_csf_scheduler_all_csgs_idle(kbdev);

		/* The GPU idle worker relies on update_on_slot_queues_offsets() to have
		 * finished. It's queued before to reduce the time it takes till execution
		 * but it'll eventually be blocked by the scheduler->interrupt_lock.
		 */
		enqueue_gpu_idle_work(scheduler);
	}

	/* The extract offsets are unused in fast GPU idle handling */
	if (!scheduler->fast_gpu_idle_handling)
		update_on_slot_queues_offsets(kbdev);
}

u32 kbase_csf_scheduler_get_nr_active_csgs_locked(struct kbase_device *kbdev)
{
	u32 nr_active_csgs;

	lockdep_assert_held(&kbdev->csf.scheduler.interrupt_lock);

	nr_active_csgs = (u32)bitmap_weight(kbdev->csf.scheduler.csg_inuse_bitmap,
					    kbdev->csf.global_iface.group_num);

	return nr_active_csgs;
}

u32 kbase_csf_scheduler_get_nr_active_csgs(struct kbase_device *kbdev)
{
	u32 nr_active_csgs;
	unsigned long flags;

	spin_lock_irqsave(&kbdev->csf.scheduler.interrupt_lock, flags);
	nr_active_csgs = kbase_csf_scheduler_get_nr_active_csgs_locked(kbdev);
	spin_unlock_irqrestore(&kbdev->csf.scheduler.interrupt_lock, flags);

	return nr_active_csgs;
}

/**
 * csg_slot_in_use - returns true if a queue group has been programmed on a
 *                   given CSG slot.
 *
 * @kbdev: Instance of a GPU platform device that implements a CSF interface.
 * @slot:  Index/number of the CSG slot in question.
 *
 * Return: the interface is actively engaged flag.
 *
 * Note: Caller must hold the scheduler lock.
 */
static inline bool csg_slot_in_use(struct kbase_device *kbdev, int slot)
{
	lockdep_assert_held(&kbdev->csf.scheduler.lock);

	return (kbdev->csf.scheduler.csg_slots[slot].resident_group != NULL);
}

static bool queue_group_suspended_locked(struct kbase_queue_group *group)
{
	lockdep_assert_held(&group->kctx->kbdev->csf.scheduler.lock);

	return (group->run_state == KBASE_CSF_GROUP_SUSPENDED ||
		group->run_state == KBASE_CSF_GROUP_SUSPENDED_ON_IDLE ||
		group->run_state == KBASE_CSF_GROUP_SUSPENDED_ON_WAIT_SYNC);
}

static bool queue_group_idle_locked(struct kbase_queue_group *group)
{
	lockdep_assert_held(&group->kctx->kbdev->csf.scheduler.lock);

	return (group->run_state == KBASE_CSF_GROUP_IDLE ||
		group->run_state == KBASE_CSF_GROUP_SUSPENDED_ON_IDLE);
}

static bool on_slot_group_idle_locked(struct kbase_queue_group *group)
{
	lockdep_assert_held(&group->kctx->kbdev->csf.scheduler.lock);

	return (group->run_state == KBASE_CSF_GROUP_IDLE);
}

static bool can_schedule_idle_group(struct kbase_queue_group *group)
{
	return (on_slot_group_idle_locked(group) ||
		(group->priority == KBASE_QUEUE_GROUP_PRIORITY_REALTIME));
}

static bool queue_group_scheduled(struct kbase_queue_group *group)
{
	return (group->run_state != KBASE_CSF_GROUP_INACTIVE &&
		group->run_state != KBASE_CSF_GROUP_TERMINATED &&
		group->run_state != KBASE_CSF_GROUP_FAULT_EVICTED);
}

static bool queue_group_scheduled_locked(struct kbase_queue_group *group)
{
	lockdep_assert_held(&group->kctx->kbdev->csf.scheduler.lock);

	return queue_group_scheduled(group);
}

static void update_idle_protm_group_state_to_runnable(struct kbase_queue_group *group)
{
	lockdep_assert_held(&group->kctx->kbdev->csf.scheduler.lock);

	group->run_state = KBASE_CSF_GROUP_RUNNABLE;
	KBASE_KTRACE_ADD_CSF_GRP(group->kctx->kbdev, CSF_GROUP_RUNNABLE, group, group->run_state);
}

/**
 * scheduler_protm_wait_quit() - Wait for GPU to exit protected mode.
 *
 * @kbdev: Pointer to the GPU device
 *
 * This function waits for the GPU to exit protected mode which is confirmed
 * when active_protm_grp is set to NULL.
 *
 * Return: true on success, false otherwise.
 */
static bool scheduler_protm_wait_quit(struct kbase_device *kbdev)
{
	struct kbase_csf_scheduler *const scheduler = &kbdev->csf.scheduler;
	const unsigned int fw_timeout_ms = kbase_get_timeout_ms(kbdev, CSF_FIRMWARE_TIMEOUT);
	long wt = kbase_csf_timeout_in_jiffies(fw_timeout_ms);
	long remaining;
	bool success = true;

	lockdep_assert_held(&scheduler->lock);

	KBASE_KTRACE_ADD(kbdev, SCHEDULER_PROTM_WAIT_QUIT_START, NULL,
			 jiffies_to_msecs((unsigned long)wt));

	/* No need to check FW I/O status, because FW is always responsive during
	 * protected mode exit.
	 */
	remaining = wait_event_timeout(kbdev->csf.event_wait,
				       !kbase_csf_scheduler_protected_mode_in_use(kbdev), wt);

	if (unlikely(!remaining)) {
		struct kbase_queue_group *group = kbdev->csf.scheduler.active_protm_grp;
		struct kbase_context *kctx = group ? group->kctx : NULL;

		dev_warn(kbdev->dev, "[%llu] Timeout (%d ms), protm_quit wait skipped",
			 kbase_backend_get_cycle_cnt(kbdev), fw_timeout_ms);
		schedule_actions_trigger_df(kbdev, kctx, DF_PROTECTED_MODE_EXIT_TIMEOUT);
		pixel_gpu_uevent_kmd_error_send(kbdev, GPU_UEVENT_INFO_PMODE_EXIT_TIMEOUT);
		success = false;
	}

	KBASE_KTRACE_ADD(kbdev, SCHEDULER_PROTM_WAIT_QUIT_END, NULL,
			 jiffies_to_msecs((unsigned long)remaining));

	return success;
}

/**
 * scheduler_force_protm_exit() - Force GPU to exit protected mode.
 *
 * @kbdev: Pointer to the GPU device
 *
 * This function sends a ping request to the firmware and waits for the GPU
 * to exit protected mode.
 *
 * If the GPU does not exit protected mode, it is considered as hang.
 * A GPU reset would then be triggered.
 */
static void scheduler_force_protm_exit(struct kbase_device *kbdev)
{
	unsigned long flags;

	lockdep_assert_held(&kbdev->csf.scheduler.lock);

	kbase_csf_firmware_ping(kbdev);

	if (scheduler_protm_wait_quit(kbdev))
		return;

	dev_err(kbdev->dev, "Possible GPU hang in Protected mode");

	spin_lock_irqsave(&kbdev->csf.scheduler.interrupt_lock, flags);
	if (kbdev->csf.scheduler.active_protm_grp) {
		dev_err(kbdev->dev,
			"Group-%d of context %d_%d ran in protected mode for too long on slot %d",
			kbdev->csf.scheduler.active_protm_grp->handle,
			kbdev->csf.scheduler.active_protm_grp->kctx->tgid,
			kbdev->csf.scheduler.active_protm_grp->kctx->id,
			kbdev->csf.scheduler.active_protm_grp->csg_nr);
	}
	spin_unlock_irqrestore(&kbdev->csf.scheduler.interrupt_lock, flags);

	/* The GPU could be stuck in Protected mode. To prevent a hang,
	 * a GPU reset is performed.
	 */
	if (kbase_prepare_to_reset_gpu(kbdev, RESET_FLAGS_NONE))
		kbase_reset_gpu(kbdev);
}

/**
 * scheduler_pm_active_handle_suspend() - Acquire the PM reference count for
 *                                        Scheduler
 *
 * @kbdev: Pointer to the device
 * @suspend_handler: Handler code for how to handle a suspend that might occur.
 * @active_after_sleep: Flag to indicate that Scheduler is being activated from
 *                      the sleeping state.
 *
 * This function is usually called when Scheduler needs to be activated.
 * The PM reference count is acquired for the Scheduler and the power on
 * of GPU is initiated.
 *
 * Return: 0 if successful or a negative error code on failure.
 */
static int scheduler_pm_active_handle_suspend(struct kbase_device *kbdev,
					      enum kbase_pm_suspend_handler suspend_handler,
					      bool active_after_sleep)
{
	unsigned long flags;
	u32 prev_count;
	int ret = 0;

	lockdep_assert_held(&kbdev->csf.scheduler.lock);

	prev_count = kbdev->csf.scheduler.pm_active_count;
	if (!WARN_ON(prev_count == U32_MAX))
		kbdev->csf.scheduler.pm_active_count++;

	/* On 0 => 1, make a pm_ctx_active request */
	if (!prev_count) {
		kbase_pm_lock(kbdev);
		kbdev->pm.backend.mcu_poweron_required = true;
		ret = kbase_pm_context_active_handle_suspend_locked(kbdev, suspend_handler);
		if (ret) {
			kbdev->csf.scheduler.pm_active_count--;
			kbdev->pm.backend.mcu_poweron_required = false;
		} else {
			spin_lock_irqsave(&kbdev->hwaccess_lock, flags);
			if (active_after_sleep) {
				kbdev->pm.backend.runtime_suspend_abort_reason = ABORT_REASON_NONE;
				kbdev->pm.backend.gpu_sleep_mode_active = false;
			}
			/* Check if the GPU is already active */
			if (kbdev->pm.active_count > 1) {
				/* GPU is already active, so need to invoke the PM state machines
				 * explicitly to turn on the MCU.
				 */
				kbdev->pm.backend.mcu_desired = true;
				kbase_pm_update_state(kbdev);
			}
			spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);
		}
		kbase_pm_unlock(kbdev);
	}

	return ret;
}

#ifdef KBASE_PM_RUNTIME
/**
 * scheduler_pm_active_after_sleep() - Acquire the PM reference count for
 *                                     Scheduler
 *
 * @kbdev: Pointer to the device
 *
 * This function is called when Scheduler needs to be activated from the
 * sleeping state.
 * The PM reference count is acquired for the Scheduler and the wake up of
 * MCU is initiated. It resets the flag that indicates to the MCU state
 * machine that MCU needs to be put in sleep state.
 *
 * Return: zero when the PM reference was taken and non-zero when the
 * system is being suspending/suspended.
 */
static int scheduler_pm_active_after_sleep(struct kbase_device *kbdev)
{
	lockdep_assert_held(&kbdev->csf.scheduler.lock);

	return scheduler_pm_active_handle_suspend(kbdev, KBASE_PM_SUSPEND_HANDLER_NOT_POSSIBLE,
						  true);
}
#endif

/**
 * scheduler_pm_idle() - Release the PM reference count held by Scheduler
 *
 * @kbdev: Pointer to the device
 *
 * This function is usually called after Scheduler is suspended.
 * The PM reference count held by the Scheduler is released to trigger the
 * power down of GPU.
 */
static void scheduler_pm_idle(struct kbase_device *kbdev)
{
	unsigned long flags;
	u32 prev_count;

	lockdep_assert_held(&kbdev->csf.scheduler.lock);

	prev_count = kbdev->csf.scheduler.pm_active_count;
	if (!WARN_ON(prev_count == 0))
		kbdev->csf.scheduler.pm_active_count--;

	if (prev_count == 1) {
		kbase_pm_lock(kbdev);
		kbdev->pm.backend.mcu_poweron_required = false;
		kbase_pm_context_idle_locked(kbdev);
		/* Check if GPU is still active */
		if (kbdev->pm.active_count) {
			/* GPU is still active, so need to invoke the PM state machines
			 * explicitly to turn off the MCU.
			 */
			spin_lock_irqsave(&kbdev->hwaccess_lock, flags);
			kbdev->pm.backend.mcu_desired = false;
			kbase_pm_update_state(kbdev);
			spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);
		}
		kbase_pm_unlock(kbdev);
	}
}

#ifdef KBASE_PM_RUNTIME
/**
 * scheduler_pm_idle_before_sleep() - Release the PM reference count and
 *                                    trigger the transition to sleep state.
 *
 * @kbdev: Pointer to the device
 *
 * This function is called on the GPU idle notification. It releases the
 * Scheduler's PM reference count and sets the flag to indicate to the
 * MCU state machine that MCU needs to be put in sleep state.
 */
static void scheduler_pm_idle_before_sleep(struct kbase_device *kbdev)
{
	unsigned long flags;

	lockdep_assert_held(&kbdev->csf.scheduler.lock);

	spin_lock_irqsave(&kbdev->hwaccess_lock, flags);
	kbdev->pm.backend.gpu_sleep_mode_active = true;
	kbdev->pm.backend.exit_gpu_sleep_mode = false;
	spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);

	scheduler_pm_idle(kbdev);
}
#endif

static void scheduler_wakeup(struct kbase_device *kbdev, bool kick)
{
	struct kbase_csf_scheduler *const scheduler = &kbdev->csf.scheduler;
	int ret;

	lockdep_assert_held(&scheduler->lock);

	if ((scheduler->state != SCHED_SUSPENDED) && (scheduler->state != SCHED_SLEEPING))
		return;

	if (scheduler->state == SCHED_SUSPENDED) {
		dev_dbg(kbdev->dev, "Re-activating the Scheduler after suspend");
		ret = scheduler_pm_active_handle_suspend(
			kbdev, KBASE_PM_SUSPEND_HANDLER_DONT_REACTIVATE, false);
#if IS_ENABLED(CONFIG_MALI_TRACE_POWER_GPU_WORK_PERIOD)
		if (!ret) {
			hrtimer_start(&scheduler->gpu_metrics_timer,
				      HR_TIMER_DELAY_NSEC(kbase_gpu_metrics_get_tp_emit_interval()),
				      HRTIMER_MODE_REL_SOFT);
		}
#endif
	} else {
#ifdef KBASE_PM_RUNTIME
		dev_dbg(kbdev->dev, "Re-activating the Scheduler out of sleep");
		ret = scheduler_pm_active_after_sleep(kbdev);
#endif
	}

	if (ret) {
		/* GPUCORE-29850 would add the handling for the case where
		 * Scheduler could not be activated due to system suspend.
		 */
		dev_dbg(kbdev->dev, "Couldn't wakeup Scheduler due to system suspend");
		return;
	}

	scheduler->state = SCHED_INACTIVE;
	KBASE_KTRACE_ADD(kbdev, SCHED_INACTIVE, NULL, scheduler->state);

	if (kick)
		scheduler_enable_tick_timer_nolock(kbdev);
}

static int scheduler_suspend(struct kbase_device *kbdev)
{
	struct kbase_csf_scheduler *const scheduler = &kbdev->csf.scheduler;

	lockdep_assert_held(&scheduler->lock);

	if (!WARN_ON(scheduler->state == SCHED_SUSPENDED)) {
#if KBASE_PM_RUNTIME
		int ret;

		ret = kbase_csf_firmware_soi_disable_on_scheduler_suspend(kbdev);
		if (ret)
			return ret;
#endif /* KBASE_PM_RUNTIME */
		dev_dbg(kbdev->dev, "Suspending the Scheduler");
		scheduler_pm_idle(kbdev);
		scheduler->state = SCHED_SUSPENDED;
#if IS_ENABLED(CONFIG_MALI_TRACE_POWER_GPU_WORK_PERIOD)
		hrtimer_cancel(&scheduler->gpu_metrics_timer);
#endif
		KBASE_KTRACE_ADD(kbdev, SCHED_SUSPENDED, NULL, scheduler->state);
	}

	return 0;
}

/**
 * update_idle_suspended_group_state() - Move the queue group to a non-idle
 *                                       suspended state.
 * @group: Pointer to the queue group.
 *
 * This function is called to change the state of queue group to non-idle
 * suspended state, if the group was suspended when all the queues bound to it
 * became empty or when some queues got blocked on a sync wait & others became
 * empty. The group is also moved to the runnable list from idle wait list in
 * the latter case.
 * So the function gets called when a queue is kicked or sync wait condition
 * gets satisfied.
 */
static void update_idle_suspended_group_state(struct kbase_queue_group *group)
{
	struct kbase_csf_scheduler *scheduler = &group->kctx->kbdev->csf.scheduler;
	int new_val;

	lockdep_assert_held(&scheduler->lock);

	if (group->run_state == KBASE_CSF_GROUP_SUSPENDED_ON_WAIT_SYNC) {
		remove_group_from_idle_wait(group);
		insert_group_to_runnable(scheduler, group, KBASE_CSF_GROUP_SUSPENDED);
	} else if (group->run_state == KBASE_CSF_GROUP_SUSPENDED_ON_IDLE) {
		group->run_state = KBASE_CSF_GROUP_SUSPENDED;
		KBASE_KTRACE_ADD_CSF_GRP(group->kctx->kbdev, CSF_GROUP_SUSPENDED, group,
					 group->run_state);

                /* If scheduler is not suspended and the given group's
		 * static priority (reflected by the scan_seq_num) is inside
		 * the current tick slot-range, or there are some on_slot
		 * idle groups, schedule an async tock.
		 */
		if (scheduler->state != SCHED_SUSPENDED) {
			unsigned long flags;
			unsigned int n_idle;
			unsigned int n_used;
			unsigned int n_slots = group->kctx->kbdev->csf.global_iface.group_num;

			spin_lock_irqsave(&scheduler->interrupt_lock, flags);
			n_idle = (unsigned int)bitmap_weight(scheduler->csg_slots_idle_mask,
							     n_slots);
			n_used = (unsigned int)bitmap_weight(scheduler->csg_inuse_bitmap, n_slots);
			spin_unlock_irqrestore(&scheduler->interrupt_lock, flags);

			if (n_idle || n_used < scheduler->num_csg_slots_for_tick ||
			    group->scan_seq_num < scheduler->num_csg_slots_for_tick)
				schedule_in_cycle(group, true);
		}
	} else
		return;

	new_val = atomic_inc_return(&scheduler->non_idle_offslot_grps);
	KBASE_KTRACE_ADD_CSF_GRP(group->kctx->kbdev, SCHEDULER_NONIDLE_OFFSLOT_GRP_INC, group,
				 (u64)new_val);
}

int kbase_csf_scheduler_group_get_slot_locked(struct kbase_queue_group *group)
{
	struct kbase_csf_scheduler *scheduler = &group->kctx->kbdev->csf.scheduler;
	int slot_num = group->csg_nr;

	lockdep_assert_held(&scheduler->interrupt_lock);

	if (slot_num >= 0) {
		if (WARN_ON(scheduler->csg_slots[slot_num].resident_group != group))
			return -1;
	}

	return slot_num;
}

int kbase_csf_scheduler_group_get_slot(struct kbase_queue_group *group)
{
	struct kbase_csf_scheduler *scheduler = &group->kctx->kbdev->csf.scheduler;
	unsigned long flags;
	int slot_num;

	spin_lock_irqsave(&scheduler->interrupt_lock, flags);
	slot_num = kbase_csf_scheduler_group_get_slot_locked(group);
	spin_unlock_irqrestore(&scheduler->interrupt_lock, flags);

	return slot_num;
}

/* kbasep_csf_scheduler_group_is_on_slot_locked() - Check if CSG is on slot.
 *
 * @group: GPU queue group to be checked
 *
 * This function needs to be called with scheduler's lock held
 *
 * Return: true if @group is on slot.
 */
static bool kbasep_csf_scheduler_group_is_on_slot_locked(struct kbase_queue_group *group)
{
	struct kbase_csf_scheduler *scheduler = &group->kctx->kbdev->csf.scheduler;
	int slot_num = group->csg_nr;

	lockdep_assert_held(&scheduler->lock);

	if (slot_num >= 0) {
		if (!WARN_ON(scheduler->csg_slots[slot_num].resident_group != group))
			return true;
	}

	return false;
}

bool kbase_csf_scheduler_group_events_enabled(struct kbase_device *kbdev,
					      struct kbase_queue_group *group)
{
	struct kbase_csf_scheduler *scheduler = &group->kctx->kbdev->csf.scheduler;
	int slot_num = group->csg_nr;

	CSTD_UNUSED(kbdev);

	lockdep_assert_held(&scheduler->interrupt_lock);

	if (WARN_ON(slot_num < 0))
		return false;

	return test_bit(slot_num, scheduler->csgs_events_enable_mask);
}

struct kbase_queue_group *kbase_csf_scheduler_get_group_on_slot(struct kbase_device *kbdev,
								u32 slot)
{
	lockdep_assert_held(&kbdev->csf.scheduler.interrupt_lock);

	return kbdev->csf.scheduler.csg_slots[slot].resident_group;
}

static u32 get_cs_req_state(struct kbase_csf_fw_io *fw_io, u32 group_id, u32 stream_id)
{
	u32 req_state;

	req_state = CS_REQ_STATE_GET(
		kbase_csf_fw_io_stream_input_read(fw_io, group_id, stream_id, CS_REQ));

	return req_state;
}

static u32 get_cs_ack_state(struct kbase_csf_fw_io *fw_io, u32 group_id, u32 stream_id)
{
	u32 ack_state;

	ack_state =
		CS_ACK_STATE_GET(kbase_csf_fw_io_stream_read(fw_io, group_id, stream_id, CS_ACK));

	return ack_state;
}

static int halt_stream_sync(struct kbase_queue *queue)
{
	struct kbase_queue_group *group = queue->group;
	struct kbase_device *kbdev = queue->kctx->kbdev;
	int csi_index = queue->csi_index;
	const unsigned int fw_timeout_ms = kbase_get_timeout_ms(kbdev, CSF_FIRMWARE_TIMEOUT);
	long remaining = kbase_csf_timeout_in_jiffies(fw_timeout_ms);
	unsigned long flags, fw_io_flags;

	if (WARN_ON(!group) || WARN_ON(!kbasep_csf_scheduler_group_is_on_slot_locked(group)))
		return -EINVAL;

	lockdep_assert_held(&kbdev->csf.scheduler.lock);

	if (get_cs_req_state(&kbdev->csf.fw_io, group->csg_nr, csi_index) == CS_REQ_STATE_START) {
		remaining = kbase_csf_fw_io_wait_event_timeout(
			&kbdev->csf.fw_io, kbdev->csf.event_wait,
			(get_cs_ack_state(&kbdev->csf.fw_io, group->csg_nr, csi_index) ==
			 CS_ACK_STATE_START),
			remaining);

		if (!remaining) {
			pixel_gpu_uevent_kmd_error_send(kbdev, GPU_UEVENT_INFO_QUEUE_START);
			dev_warn(
				kbdev->dev,
				"[%llu] Timeout (%d ms) waiting for queue to start on csi %d bound to group %d on slot %d",
				kbase_backend_get_cycle_cnt(kbdev), fw_timeout_ms, csi_index,
				group->handle, group->csg_nr);
			if (kbase_prepare_to_reset_gpu(kbdev, RESET_FLAGS_NONE))
				kbase_reset_gpu(kbdev);

			return -ETIMEDOUT;
		}

		remaining = kbase_csf_timeout_in_jiffies(fw_timeout_ms);
	}

	spin_lock_irqsave(&kbdev->csf.scheduler.interrupt_lock, flags);
	if (kbase_csf_fw_io_open(&kbdev->csf.fw_io, &fw_io_flags)) {
		/* Skip FW transaction */
		spin_unlock_irqrestore(&kbdev->csf.scheduler.interrupt_lock, flags);
		return 0;
	}
	/* Set state to STOP */
	kbase_csf_fw_io_stream_write_mask(&kbdev->csf.fw_io, group->csg_nr, csi_index, CS_REQ,
					  CS_REQ_STATE_STOP, CS_REQ_STATE_MASK);

	kbase_csf_ring_cs_kernel_doorbell(kbdev, csi_index, group->csg_nr, true);
	kbase_csf_fw_io_close(&kbdev->csf.fw_io, fw_io_flags);
	spin_unlock_irqrestore(&kbdev->csf.scheduler.interrupt_lock, flags);

	KBASE_KTRACE_ADD_CSF_GRP_Q(kbdev, CSI_STOP_REQ, group, queue, 0u);

	/* Timed wait */
	remaining = kbase_csf_fw_io_wait_event_timeout(
		&kbdev->csf.fw_io, kbdev->csf.event_wait,
		(get_cs_ack_state(&kbdev->csf.fw_io, group->csg_nr, csi_index) ==
		 CS_ACK_STATE_STOP),
		remaining);

	if (!remaining) {
		pixel_gpu_uevent_kmd_error_send(kbdev, GPU_UEVENT_INFO_QUEUE_STOP);
		dev_warn(
			kbdev->dev,
			"[%llu] Timeout (%d ms) waiting for queue to stop on csi %d bound to group %d on slot %d",
			kbase_backend_get_cycle_cnt(kbdev), fw_timeout_ms, queue->csi_index,
			group->handle, group->csg_nr);

		/* TODO GPUCORE-25328: The CSG can't be terminated, the GPU
		 * will be reset as a work-around.
		 */
		if (kbase_prepare_to_reset_gpu(kbdev, RESET_FLAGS_NONE))
			kbase_reset_gpu(kbdev);

		return -ETIMEDOUT;
	}

	return 0;
}

static bool can_halt_stream(struct kbase_device *kbdev, struct kbase_queue_group *group)
{
	struct kbase_csf_csg_slot *const csg_slot = kbdev->csf.scheduler.csg_slots;
	unsigned long flags;
	bool can_halt;
	int slot;

	if (!queue_group_scheduled(group))
		return true;

	spin_lock_irqsave(&kbdev->csf.scheduler.interrupt_lock, flags);
	slot = kbase_csf_scheduler_group_get_slot_locked(group);
	can_halt = (slot >= 0) && (atomic_read(&csg_slot[slot].state) == CSG_SLOT_RUNNING);
	spin_unlock_irqrestore(&kbdev->csf.scheduler.interrupt_lock, flags);

	return can_halt;
}

/**
 * sched_halt_stream() - Stop a GPU queue when its queue group is not running
 *                       on a CSG slot.
 * @queue: Pointer to the GPU queue to stop.
 *
 * This function handles stopping gpu queues for groups that are either not on
 * a CSG slot or are on the slot but undergoing transition to
 * resume or suspend states.
 * It waits until the queue group is scheduled on a slot and starts running,
 * which is needed as groups that were suspended may need to resume all queues
 * that were enabled and running at the time of suspension.
 *
 * Return: 0 on success, or negative on failure.
 */
static int sched_halt_stream(struct kbase_queue *queue)
{
	struct kbase_queue_group *group = queue->group;
	struct kbase_device *kbdev = queue->kctx->kbdev;
	struct kbase_csf_scheduler *const scheduler = &kbdev->csf.scheduler;
	struct kbase_csf_csg_slot *const csg_slot = kbdev->csf.scheduler.csg_slots;
	bool retry_needed = false;
	bool retried = false;
	long remaining;
	int slot;
	int err = 0;
	const u32 group_schedule_timeout = kbdev->csf.csg_suspend_timeout_ms;
	const u32 fw_timeout_ms = kbase_get_timeout_ms(kbdev, CSF_FIRMWARE_TIMEOUT);

	if (WARN_ON(!group))
		return -EINVAL;

	lockdep_assert_held(&queue->kctx->csf.lock);
	lockdep_assert_held(&scheduler->lock);

	slot = kbase_csf_scheduler_group_get_slot(group);

	if (slot >= 0) {
		WARN_ON(atomic_read(&csg_slot[slot].state) == CSG_SLOT_RUNNING);

		if (atomic_read(&csg_slot[slot].state) == CSG_SLOT_READY2RUN) {
			dev_dbg(kbdev->dev,
				"Stopping a queue on csi %d when Group-%d is in under transition to running state",
				queue->csi_index, group->handle);
			retry_needed = true;
		}
	}
retry:
	/* Update the group state so that it can get scheduled soon */
	update_idle_suspended_group_state(group);

	rt_mutex_unlock(&scheduler->lock);

	/* This function is called when the queue group is either not on a CSG
	 * slot or is on the slot but undergoing transition.
	 *
	 * To stop the queue, the function needs to wait either for the queue
	 * group to be assigned a CSG slot (and that slot has to reach the
	 * running state) or for the eviction of the queue group from the
	 * scheduler's list.
	 *
	 * In order to evaluate the latter condition, the function doesn't
	 * really need to lock the scheduler, as any update to the run_state
	 * of the queue group by sched_evict_group() would be visible due
	 * to implicit barriers provided by the kernel waitqueue macros.
	 *
	 * The group pointer cannot disappear meanwhile, as the high level
	 * CSF context is locked. Therefore, the scheduler would be
	 * the only one to update the run_state of the group.
	 */
	remaining = kbase_csf_fw_io_wait_event_timeout(
		&kbdev->csf.fw_io, kbdev->csf.event_wait, can_halt_stream(kbdev, group),
		kbase_csf_timeout_in_jiffies(group_schedule_timeout));

	rt_mutex_lock(&scheduler->lock);

	if ((remaining > 0) && queue_group_scheduled_locked(group)) {
		slot = kbase_csf_scheduler_group_get_slot(group);

		/* If the group is still on slot and slot is in running state
		 * then explicitly stop the CSI of the
		 * queue. Otherwise there are different cases to consider
		 *
		 * - If the queue group was already undergoing transition to
		 *   resume/start state when this function was entered then it
		 *   would not have disabled the CSI of the
		 *   queue being stopped and the previous wait would have ended
		 *   once the slot was in a running state with CS
		 *   interface still enabled.
		 *   Now the group is going through another transition either
		 *   to a suspend state or to a resume state (it could have
		 *   been suspended before the scheduler lock was grabbed).
		 *   In both scenarios need to wait again for the group to
		 *   come on a slot and that slot to reach the running state,
		 *   as that would guarantee that firmware will observe the
		 *   CSI as disabled.
		 *
		 * - If the queue group was either off the slot or was
		 *   undergoing transition to suspend state on entering this
		 *   function, then the group would have been resumed with the
		 *   queue's CSI in disabled state.
		 *   So now if the group is undergoing another transition
		 *   (after the resume) then just need to wait for the state
		 *   bits in the ACK register of CSI to be
		 *   set to STOP value. It is expected that firmware will
		 *   process the stop/disable request of the CS
		 *   interface after resuming the group before it processes
		 *   another state change request of the group.
		 */
		if ((slot >= 0) && (atomic_read(&csg_slot[slot].state) == CSG_SLOT_RUNNING)) {
			err = halt_stream_sync(queue);
		} else if (retry_needed && !retried) {
			retried = true;
			goto retry;
		} else if (slot >= 0) {
			if (!WARN_ON(get_cs_req_state(&kbdev->csf.fw_io, slot, queue->csi_index) !=
				     CS_REQ_STATE_STOP)) {
				/* Timed wait */
				remaining = kbase_csf_fw_io_wait_event_timeout(
					&kbdev->csf.fw_io, kbdev->csf.event_wait,
					(get_cs_ack_state(&kbdev->csf.fw_io, slot,
							  queue->csi_index) == CS_ACK_STATE_STOP),
					kbase_csf_timeout_in_jiffies(fw_timeout_ms));

				if (!remaining) {
					pixel_gpu_uevent_kmd_error_send(kbdev, GPU_UEVENT_INFO_QUEUE_STOP_ACK);
					dev_warn(
						kbdev->dev,
						"[%llu] Timeout (%d ms) waiting for queue stop ack on csi %d bound to group %d on slot %d",
						kbase_backend_get_cycle_cnt(kbdev), fw_timeout_ms,
						queue->csi_index, group->handle, group->csg_nr);

					err = -ETIMEDOUT;
				}
			}
		}
	} else if (!remaining) {
		dev_warn(
			kbdev->dev,
			"[%llu] Group-%d failed to get a slot for stopping the queue on csi %d (timeout %d ms)",
			kbase_backend_get_cycle_cnt(kbdev), group->handle, queue->csi_index,
			group_schedule_timeout);


		err = -ETIMEDOUT;
	}

	return err;
}

/**
 * scheduler_activate_on_queue_stop() - Activate the Scheduler when the GPU
 *                                      queue needs to be stopped.
 *
 * @queue: Pointer the GPU command queue
 *
 * This function is called when the CSI to which GPU queue is bound needs to
 * be stopped. For that the corresponding queue group needs to be resident on
 * the CSG slot and MCU firmware should be running. So this function makes the
 * Scheduler exit the sleeping or suspended state.
 */
static void scheduler_activate_on_queue_stop(struct kbase_queue *queue)
{
	struct kbase_device *kbdev = queue->kctx->kbdev;

	scheduler_wakeup(kbdev, true);

	/* Wait for MCU firmware to start running */
	if (kbase_csf_scheduler_wait_mcu_active(kbdev)) {
		dev_warn(
			kbdev->dev,
			"[%llu] Wait for MCU active failed for stopping queue on csi %d bound to group %d of context %d_%d on slot %d",
			kbase_backend_get_cycle_cnt(kbdev), queue->csi_index, queue->group->handle,
			queue->kctx->tgid, queue->kctx->id, queue->group->csg_nr);
	}
}

int kbase_csf_scheduler_queue_stop(struct kbase_queue *queue)
{
	struct kbase_device *kbdev = queue->kctx->kbdev;
	struct kbase_queue_group *group = queue->group;
	bool const cs_enabled = queue->enabled;
	int err = 0;

	if (WARN_ON(!group))
		return -EINVAL;

	kbase_reset_gpu_assert_failed_or_prevented(kbdev);
	lockdep_assert_held(&queue->kctx->csf.lock);
	rt_mutex_lock(&kbdev->csf.scheduler.lock);

	queue->enabled = false;
	KBASE_KTRACE_ADD_CSF_GRP_Q(kbdev, CSI_STOP, group, queue, cs_enabled);

	if (cs_enabled && queue_group_scheduled_locked(group)) {
		struct kbase_csf_csg_slot *const csg_slot = kbdev->csf.scheduler.csg_slots;
		int slot = kbase_csf_scheduler_group_get_slot(group);

		/* Since the group needs to be resumed in order to stop the queue,
		 * check if GPU needs to be powered up.
		 */
		scheduler_activate_on_queue_stop(queue);

		if ((slot >= 0) && (atomic_read(&csg_slot[slot].state) == CSG_SLOT_RUNNING))
			err = halt_stream_sync(queue);
		else
			err = sched_halt_stream(queue);

		unassign_user_doorbell_from_queue(kbdev, queue);
		kbase_csf_mcu_shared_drop_stopped_queue(kbdev, queue);
	}

	rt_mutex_unlock(&kbdev->csf.scheduler.lock);
	KBASE_KTRACE_ADD_CSF_GRP_Q(kbdev, QUEUE_STOP, group, queue, group->run_state);
	return err;
}

static void update_hw_active(struct kbase_queue *queue, bool active)
{
#if IS_ENABLED(CONFIG_MALI_NO_MALI)
	if (queue && queue->enabled) {
		u64 *output_addr = queue->user_io_addr + PAGE_SIZE / sizeof(u64);

		output_addr[CS_ACTIVE / sizeof(*output_addr)] = active;
	}
#else
	CSTD_UNUSED(queue);
	CSTD_UNUSED(active);
#endif
}

static void program_cs_extract_init(struct kbase_queue *queue)
{
	u64 *input_addr = queue->user_io_addr;
	u64 *output_addr = queue->user_io_addr + PAGE_SIZE / sizeof(u64);

	/*
	 * These 64-bit reads and writes will be atomic on a 64-bit kernel but may
	 * not be atomic on 32-bit kernels. Support for 32-bit kernels is limited to
	 * build-only.
	 */
	input_addr[CS_EXTRACT_INIT_LO / sizeof(*input_addr)] =
		output_addr[CS_EXTRACT_LO / sizeof(*output_addr)];
}

static void program_cs_trace_cfg(u32 group_id, u32 stream_id, struct kbase_queue *queue)
{
	struct kbase_device *kbdev = queue->kctx->kbdev;
	u32 const glb_version = kbdev->csf.global_iface.version;

	lockdep_assert_held(&kbdev->csf.scheduler.lock);
	kbase_csf_fw_io_assert_opened(&kbdev->csf.fw_io);

	/* If cs_trace_command not supported, nothing to program */
	if (glb_version < kbase_csf_interface_version(1, 1, 0))
		return;

	/* Program for cs_trace if enabled. In the current arrangement, it is
	 * possible for the context to enable the cs_trace after some queues
	 * has been registered in cs_trace in disabled state. This is tracked by
	 * the queue's trace buffer base address, which had been validated at the
	 * queue's register_ex call.
	 */
	if (kbase_csf_scheduler_queue_has_trace(queue)) {
		u32 cs_cfg = CS_INSTR_CONFIG_JASID_SET(queue->trace_cfg, (u32)queue->kctx->as_nr);

		kbase_csf_fw_io_stream_write(&kbdev->csf.fw_io, group_id, stream_id,
					     CS_INSTR_CONFIG, cs_cfg);
		kbase_csf_fw_io_stream_write(&kbdev->csf.fw_io, group_id, stream_id,
					     CS_INSTR_BUFFER_SIZE, queue->trace_buffer_size);

		kbase_csf_fw_io_stream_write(&kbdev->csf.fw_io, group_id, stream_id,
					     CS_INSTR_BUFFER_BASE_LO,
					     queue->trace_buffer_base & U32_MAX);
		kbase_csf_fw_io_stream_write(&kbdev->csf.fw_io, group_id, stream_id,
					     CS_INSTR_BUFFER_BASE_HI,
					     queue->trace_buffer_base >> 32);

		kbase_csf_fw_io_stream_write(&kbdev->csf.fw_io, group_id, stream_id,
					     CS_INSTR_BUFFER_OFFSET_POINTER_LO,
					     queue->trace_offset_ptr & U32_MAX);
		kbase_csf_fw_io_stream_write(&kbdev->csf.fw_io, group_id, stream_id,
					     CS_INSTR_BUFFER_OFFSET_POINTER_HI,
					     queue->trace_offset_ptr >> 32);
	} else {
		/* Place the configuration to the disabled condition */
		kbase_csf_fw_io_stream_write(&kbdev->csf.fw_io, group_id, stream_id,
					     CS_INSTR_CONFIG, 0);
		kbase_csf_fw_io_stream_write(&kbdev->csf.fw_io, group_id, stream_id,
					     CS_INSTR_BUFFER_SIZE, 0);
	}
}

static void program_cs(struct kbase_device *kbdev, struct kbase_queue *queue,
		       bool ring_csg_doorbell)
{
	struct kbase_queue_group *group = queue->group;
	struct kbase_csf_cmd_stream_group_info *ginfo;
	int csi_index = queue->csi_index;
	u64 user_input;
	u64 user_output;
	unsigned long flags, fw_io_flags;

	if (WARN_ON(!group))
		return;

	lockdep_assert_held(&kbdev->csf.scheduler.lock);

	if (WARN_ON(!kbasep_csf_scheduler_group_is_on_slot_locked(group)))
		return;

	ginfo = &kbdev->csf.global_iface.groups[group->csg_nr];

	if (WARN_ON(csi_index < 0) || WARN_ON((u32)csi_index >= ginfo->stream_num))
		return;

	if (queue->enabled) {
		assign_user_doorbell_to_queue(kbdev, queue);
		if (queue->doorbell_nr == KBASEP_USER_DB_NR_INVALID)
			return;

		WARN_ON(queue->doorbell_nr != queue->group->doorbell_nr);
	}

	if (queue->enabled && queue_group_suspended_locked(group))
		program_cs_extract_init(queue);

	spin_lock_irqsave(&kbdev->csf.scheduler.interrupt_lock, flags);

	if (kbase_csf_fw_io_open(&kbdev->csf.fw_io, &fw_io_flags))
		goto skip_fw;

	kbase_csf_fw_io_stream_write(&kbdev->csf.fw_io, group->csg_nr, csi_index, CS_BASE_LO,
				     queue->base_addr & 0xFFFFFFFF);
	kbase_csf_fw_io_stream_write(&kbdev->csf.fw_io, group->csg_nr, csi_index, CS_BASE_HI,
				     queue->base_addr >> 32);
	kbase_csf_fw_io_stream_write(&kbdev->csf.fw_io, group->csg_nr, csi_index, CS_SIZE,
				     queue->size);

	user_input = queue->user_io_gpu_va;
	WARN_ONCE(!user_input && queue->enabled, "Enabled queue should have a valid gpu_va");

	kbase_csf_fw_io_stream_write(&kbdev->csf.fw_io, group->csg_nr, csi_index, CS_USER_INPUT_LO,
				     user_input & 0xFFFFFFFF);
	kbase_csf_fw_io_stream_write(&kbdev->csf.fw_io, group->csg_nr, csi_index, CS_USER_INPUT_HI,
				     user_input >> 32);

	user_output = user_input + PAGE_SIZE;
	kbase_csf_fw_io_stream_write(&kbdev->csf.fw_io, group->csg_nr, csi_index, CS_USER_OUTPUT_LO,
				     user_output & 0xFFFFFFFF);
	kbase_csf_fw_io_stream_write(&kbdev->csf.fw_io, group->csg_nr, csi_index, CS_USER_OUTPUT_HI,
				     user_output >> 32);

	kbase_csf_fw_io_stream_write(&kbdev->csf.fw_io, group->csg_nr, csi_index, CS_CONFIG,
				     ((u32)queue->doorbell_nr << 8) | (queue->priority & 0xF));

	/* Program the queue's cs_trace configuration */
	program_cs_trace_cfg(group->csg_nr, csi_index, queue);

	/* Enable all interrupts for now */
	kbase_csf_fw_io_stream_write(&kbdev->csf.fw_io, group->csg_nr, csi_index, CS_ACK_IRQ_MASK,
				     ~((u32)0));

	/* The fault bit could be misaligned between CS_REQ and CS_ACK if the
	 * acknowledgment was deferred due to dump on fault and the group was
	 * removed from the CSG slot before the fault could be acknowledged.
	 */
	if (queue->enabled) {
		u32 const cs_ack = kbase_csf_fw_io_stream_read(&kbdev->csf.fw_io, group->csg_nr,
							       csi_index, CS_ACK);

		kbase_csf_fw_io_stream_write_mask(&kbdev->csf.fw_io, group->csg_nr, csi_index,
						  CS_REQ, cs_ack, CS_REQ_FAULT_MASK);
	}

	/*
	 * Enable the CSG idle notification once the CS's ringbuffer
	 * becomes empty or the CS becomes sync_idle, waiting sync update
	 * or protected mode switch.
	 */
	kbase_csf_fw_io_stream_write_mask(&kbdev->csf.fw_io, group->csg_nr, csi_index, CS_REQ,
					  CS_REQ_IDLE_EMPTY_MASK | CS_REQ_IDLE_SYNC_WAIT_MASK |
						  CS_REQ_IDLE_SHARED_SB_DEC_MASK,
					  CS_REQ_IDLE_EMPTY_MASK | CS_REQ_IDLE_SYNC_WAIT_MASK |
						  CS_REQ_IDLE_SHARED_SB_DEC_MASK);

	/* Set state to START/STOP */
	kbase_csf_fw_io_stream_write_mask(&kbdev->csf.fw_io, group->csg_nr, csi_index, CS_REQ,
					  queue->enabled ? CS_REQ_STATE_START : CS_REQ_STATE_STOP,
					  CS_REQ_STATE_MASK);
	kbase_csf_ring_cs_kernel_doorbell(kbdev, csi_index, group->csg_nr, ring_csg_doorbell);
	kbase_csf_fw_io_close(&kbdev->csf.fw_io, fw_io_flags);
skip_fw:
	spin_unlock_irqrestore(&kbdev->csf.scheduler.interrupt_lock, flags);
	KBASE_KTRACE_ADD_CSF_GRP_Q(kbdev, CSI_START, group, queue, queue->enabled);

	update_hw_active(queue, true);
}

static int onslot_csg_add_new_queue(struct kbase_queue *queue)
{
	struct kbase_device *kbdev = queue->kctx->kbdev;
	int err;

	lockdep_assert_held(&kbdev->csf.scheduler.lock);

	err = kbase_csf_mcu_shared_add_queue(kbdev, queue);
	if (!err)
		program_cs(kbdev, queue, true);

	return err;
}

int kbase_csf_scheduler_queue_start(struct kbase_queue *queue)
{
	struct kbase_queue_group *group = queue->group;
	struct kbase_device *kbdev = queue->kctx->kbdev;
	bool const cs_enabled = queue->enabled;
	int err = 0;
	bool evicted = false;

	kbase_reset_gpu_assert_prevented(kbdev);
	lockdep_assert_held(&queue->kctx->csf.lock);

	if (WARN_ON_ONCE(!group || queue->bind_state != KBASE_CSF_QUEUE_BOUND))
		return -EINVAL;

	rt_mutex_lock(&kbdev->csf.scheduler.lock);

#if IS_ENABLED(CONFIG_DEBUG_FS)
	if (unlikely(kbdev->csf.scheduler.state == SCHED_BUSY)) {
		rt_mutex_unlock(&kbdev->csf.scheduler.lock);
		return -EBUSY;
	}
#endif

	KBASE_KTRACE_ADD_CSF_GRP_Q(kbdev, QUEUE_START, group, queue, group->run_state);
	KBASE_KTRACE_ADD_CSF_GRP_Q(kbdev, QUEUE_SYNC_UPDATE_WAIT_STATUS, queue->group, queue,
				   queue->status_wait);

	if (group->run_state == KBASE_CSF_GROUP_FAULT_EVICTED) {
		err = -EIO;
		evicted = true;
	} else if ((group->run_state == KBASE_CSF_GROUP_SUSPENDED_ON_WAIT_SYNC) &&
		   CS_STATUS_WAIT_SYNC_WAIT_GET(queue->status_wait)) {
		dev_dbg(kbdev->dev, "blocked queue(csi_index=%d) of group %d was kicked",
			queue->csi_index, group->handle);
	} else {
		err = scheduler_group_schedule(group);

		if (!err) {
			queue->enabled = true;
			if (kbasep_csf_scheduler_group_is_on_slot_locked(group)) {
				if (cs_enabled) {
					/* In normal situation, when a queue is
					 * already running, the queue update
					 * would be a doorbell kick on user
					 * side. However, if such a kick is
					 * shortly following a start or resume,
					 * the queue may actually in transition
					 * hence the said kick would enter the
					 * kernel as the hw_active flag is yet
					 * to be set. The sheduler needs to
					 * give a kick to the corresponding
					 * user door-bell on such a case.
					 */
					kbase_csf_ring_cs_user_doorbell(kbdev, queue);
				} else {
					err = onslot_csg_add_new_queue(queue);
					/* For an on slot CSG, the only error in adding a new
					 * queue to run is that the scheduler could not map
					 * the required userio pages due to likely some resource
					 * issues. In such a case, and if the group is yet
					 * to enter its fatal error state, we return a -EBUSY
					 * to the submitter for another kick. The queue itself
					 * has yet to be programmed hence needs to remain its
					 * previous (disabled) state. If the error persists,
					 * the group will eventually reports a fatal error by
					 * the group's error reporting mechanism, when the MCU
					 * shared region map retry limit of the group is
					 * exceeded. For such a case, the expected error value
					 * is -EIO.
					 */
					if (unlikely(err)) {
						queue->enabled = cs_enabled;
						rt_mutex_unlock(&kbdev->csf.scheduler.lock);
						return (err != -EIO) ? -EBUSY : err;
					}
				}
			}
			queue_delayed_work(system_long_wq, &kbdev->csf.scheduler.ping_work,
					   msecs_to_jiffies(kbase_get_timeout_ms(
						   kbdev, CSF_FIRMWARE_PING_TIMEOUT)));
		}
	}

	rt_mutex_unlock(&kbdev->csf.scheduler.lock);

	if (evicted)
		kbase_csf_term_descheduled_queue_group(group);

	return err;
}

static u32 get_csg_ack_state(struct kbase_csf_fw_io *fw_io, u32 group_id)
{
	u32 ack_state;

	ack_state = CSG_ACK_STATE_GET(kbase_csf_fw_io_group_read(fw_io, group_id, CSG_ACK));

	return ack_state;
}

static enum kbase_csf_csg_slot_state update_csg_slot_status(struct kbase_device *kbdev, s8 slot)
{
	struct kbase_csf_csg_slot *csg_slot = &kbdev->csf.scheduler.csg_slots[slot];
	u32 state;
	enum kbase_csf_csg_slot_state slot_state;

	lockdep_assert_held(&kbdev->csf.scheduler.lock);

	state = get_csg_ack_state(&kbdev->csf.fw_io, slot);
	slot_state = atomic_read(&csg_slot->state);

	switch (slot_state) {
	case CSG_SLOT_READY2RUN:
		if ((state == CSG_ACK_STATE_START) || (state == CSG_ACK_STATE_RESUME)) {
			slot_state = CSG_SLOT_RUNNING;
			atomic_set(&csg_slot->state, slot_state);
			KBASE_KTRACE_ADD_CSF_GRP(kbdev, CSG_SLOT_RUNNING, csg_slot->resident_group,
						 state);
			dev_dbg(kbdev->dev, "Group %u running on slot %d\n",
				csg_slot->resident_group->handle, slot);
		}
		break;
	case CSG_SLOT_DOWN2STOP:
		if ((state == CSG_ACK_STATE_SUSPEND) || (state == CSG_ACK_STATE_TERMINATE)) {
			slot_state = CSG_SLOT_STOPPED;
			atomic_set(&csg_slot->state, slot_state);
			KBASE_KTRACE_ADD_CSF_GRP(kbdev, CSG_SLOT_STOPPED, csg_slot->resident_group,
						 state);
			dev_dbg(kbdev->dev, "Group %u stopped on slot %d\n",
				csg_slot->resident_group->handle, slot);
		}
		break;
	case CSG_SLOT_DOWN2STOP_TIMEDOUT:
	case CSG_SLOT_READY2RUN_TIMEDOUT:
	case CSG_SLOT_READY:
	case CSG_SLOT_RUNNING:
	case CSG_SLOT_STOPPED:
		break;
	default:
		dev_warn(kbdev->dev, "Unknown CSG slot state %d", slot_state);
		break;
	}

	return slot_state;
}

static bool csg_slot_running(struct kbase_device *kbdev, s8 slot)
{
	lockdep_assert_held(&kbdev->csf.scheduler.lock);

	return (update_csg_slot_status(kbdev, slot) == CSG_SLOT_RUNNING);
}

static bool csg_slot_stopped_locked(struct kbase_device *kbdev, s8 slot)
{
	enum kbase_csf_csg_slot_state slot_state;

	lockdep_assert_held(&kbdev->csf.scheduler.lock);

	slot_state = update_csg_slot_status(kbdev, slot);

	return (slot_state == CSG_SLOT_STOPPED || slot_state == CSG_SLOT_READY);
}

static bool csg_slot_stopped_raw(struct kbase_device *kbdev, s8 slot)
{
	u32 state;

	state = get_csg_ack_state(&kbdev->csf.fw_io, slot);

	if (state == CSG_ACK_STATE_SUSPEND || state == CSG_ACK_STATE_TERMINATE) {
		KBASE_KTRACE_ADD_CSF_GRP(kbdev, CSG_SLOT_STOPPED,
					 kbdev->csf.scheduler.csg_slots[slot].resident_group,
					 state);
		dev_dbg(kbdev->dev, "(raw status) slot %d stopped\n", slot);
		return true;
	}

	return false;
}

static void halt_csg_slot(struct kbase_queue_group *group, bool suspend)
{
	struct kbase_device *kbdev = group->kctx->kbdev;
	struct kbase_csf_csg_slot *csg_slot = kbdev->csf.scheduler.csg_slots;
	const unsigned int fw_timeout_ms = kbase_get_timeout_ms(kbdev, CSF_FIRMWARE_TIMEOUT);
	s8 slot;

	lockdep_assert_held(&kbdev->csf.scheduler.lock);

	if (WARN_ON(!kbasep_csf_scheduler_group_is_on_slot_locked(group)))
		return;

	slot = group->csg_nr;

	/* When in transition, wait for it to complete */
	if (atomic_read(&csg_slot[slot].state) == CSG_SLOT_READY2RUN) {
		long remaining = kbase_csf_timeout_in_jiffies(fw_timeout_ms);

		dev_dbg(kbdev->dev, "slot %d wait for up-running\n", slot);
		remaining = kbase_csf_fw_io_wait_event_timeout(&kbdev->csf.fw_io,
							       kbdev->csf.event_wait,
							       csg_slot_running(kbdev, slot),
							       remaining);
		if (!remaining) {
			pixel_gpu_uevent_kmd_error_send(kbdev, GPU_UEVENT_INFO_CSG_SLOT_READY);
			dev_warn(kbdev->dev, "[%llu] slot %d timeout (%d ms) on up-running\n",
				 kbase_backend_get_cycle_cnt(kbdev), slot, fw_timeout_ms);
		}
	}

	if (csg_slot_running(kbdev, slot)) {
		unsigned long flags, fw_io_flags;
		u32 halt_cmd = suspend ? CSG_REQ_STATE_SUSPEND : CSG_REQ_STATE_TERMINATE;

		dev_dbg(kbdev->dev, "Halting(suspend=%d) group %d of context %d_%d on slot %d",
			suspend, group->handle, group->kctx->tgid, group->kctx->id, slot);

		spin_lock_irqsave(&kbdev->csf.scheduler.interrupt_lock, flags);
		/* Set state to SUSPEND/TERMINATE */
		if (kbase_csf_fw_io_open(&kbdev->csf.fw_io, &fw_io_flags))
			goto skip_fw;
		kbase_csf_fw_io_group_write_mask(&kbdev->csf.fw_io, slot, CSG_REQ, halt_cmd,
						 CSG_REQ_STATE_MASK);
		kbase_csf_ring_csg_doorbell(kbdev, slot);
		kbase_csf_fw_io_close(&kbdev->csf.fw_io, fw_io_flags);
skip_fw:
		spin_unlock_irqrestore(&kbdev->csf.scheduler.interrupt_lock, flags);
		atomic_set(&csg_slot[slot].state, CSG_SLOT_DOWN2STOP);
		KBASE_KTRACE_ADD_CSF_GRP(kbdev, CSG_SLOT_STOP_REQ, group, halt_cmd);

		KBASE_TLSTREAM_TL_KBASE_DEVICE_HALTING_CSG(kbdev, kbdev->id, (u32)slot, suspend);
	}
}

static void term_csg_slot(struct kbase_queue_group *group)
{
	halt_csg_slot(group, false);
}

static void suspend_csg_slot(struct kbase_queue_group *group)
{
	halt_csg_slot(group, true);
}

static bool csf_wait_ge_condition_supported(struct kbase_device *kbdev)
{
	const uint32_t glb_major = GLB_VERSION_MAJOR_GET(kbdev->csf.global_iface.version);
	const uint32_t glb_minor = GLB_VERSION_MINOR_GET(kbdev->csf.global_iface.version);

	switch (glb_major) {
	case 0:
		break;
	case 1:
		if (glb_minor >= 4)
			return true;
		break;
	case 2:
		if (glb_minor >= 6)
			return true;
		break;
	case 3:
		if (glb_minor >= 6)
			return true;
		break;
	default:
		return true;
	}
	return false;
}
/**
 * evaluate_sync_update() - Evaluate the sync wait condition the GPU command
 *                          queue has been blocked on.
 *
 * @queue: Pointer to the GPU command queue
 *
 * Return: true if sync wait condition is satisfied.
 */
static bool evaluate_sync_update(struct kbase_queue *queue)
{
	struct kbase_vmap_struct *mapping;
	bool updated = false;
	u32 *sync_ptr;
	u32 sync_wait_size;
	u32 sync_wait_align_mask;
	u32 sync_wait_cond;
	u32 sync_current_val;
	struct kbase_device *kbdev;
	bool sync_wait_align_valid = false;
	bool sync_wait_cond_valid = false;

	if (WARN_ON(!queue))
		return false;

	kbdev = queue->kctx->kbdev;

	lockdep_assert_held(&kbdev->csf.scheduler.lock);

	sync_wait_size = CS_STATUS_WAIT_SYNC_WAIT_SIZE_GET(queue->status_wait);
	sync_wait_align_mask =
		(sync_wait_size == 0 ? BASEP_EVENT32_ALIGN_BYTES : BASEP_EVENT64_ALIGN_BYTES) - 1;
	sync_wait_align_valid = ((uintptr_t)queue->sync_ptr & sync_wait_align_mask) == 0;
	if (!sync_wait_align_valid) {
		dev_dbg(queue->kctx->kbdev->dev, "sync memory VA 0x%016llX is misaligned",
			queue->sync_ptr);
		goto out;
	}

	sync_ptr = kbase_phy_alloc_mapping_get(queue->kctx, queue->sync_ptr, &mapping);

	KBASE_KTRACE_ADD_CSF_GRP_Q(kbdev, QUEUE_SYNC_UPDATE_EVAL_START, queue->group, queue,
				   queue->sync_ptr);
	KBASE_KTRACE_ADD_CSF_GRP_Q(kbdev, QUEUE_SYNC_UPDATE_BLOCKED_REASON, queue->group, queue,
				   queue->blocked_reason);

	if (!sync_ptr) {
		dev_dbg(queue->kctx->kbdev->dev, "sync memory VA 0x%016llX already freed",
			queue->sync_ptr);
		goto out;
	}

	sync_wait_cond = CS_STATUS_WAIT_SYNC_WAIT_CONDITION_GET(queue->status_wait);
	sync_wait_cond_valid = (sync_wait_cond == CS_STATUS_WAIT_SYNC_WAIT_CONDITION_GT) ||
			       (sync_wait_cond == CS_STATUS_WAIT_SYNC_WAIT_CONDITION_LE) ||
			       ((sync_wait_cond == CS_STATUS_WAIT_SYNC_WAIT_CONDITION_GE) &&
				csf_wait_ge_condition_supported(kbdev));

	WARN_ON(!sync_wait_cond_valid);

	sync_current_val = READ_ONCE(*sync_ptr);
	KBASE_KTRACE_ADD_CSF_GRP_Q(kbdev, QUEUE_SYNC_UPDATE_CUR_VAL, queue->group, queue,
				   sync_current_val);

	KBASE_KTRACE_ADD_CSF_GRP_Q(kbdev, QUEUE_SYNC_UPDATE_TEST_VAL, queue->group, queue,
				   queue->sync_value);

	if (((sync_wait_cond == CS_STATUS_WAIT_SYNC_WAIT_CONDITION_GT) &&
	     (sync_current_val > queue->sync_value)) ||
	    ((sync_wait_cond == CS_STATUS_WAIT_SYNC_WAIT_CONDITION_GE) &&
	     (sync_current_val >= queue->sync_value) && csf_wait_ge_condition_supported(kbdev)) ||
	    ((sync_wait_cond == CS_STATUS_WAIT_SYNC_WAIT_CONDITION_LE) &&
	     (sync_current_val <= queue->sync_value))) {
		/* The sync wait condition is satisfied so the group to which
		 * queue is bound can be re-scheduled.
		 */
		updated = true;
	} else {
		dev_dbg(queue->kctx->kbdev->dev, "sync memory not updated yet(%u)",
			sync_current_val);
	}

	kbase_phy_alloc_mapping_put(queue->kctx, mapping);
out:
	KBASE_KTRACE_ADD_CSF_GRP_Q(kbdev, QUEUE_SYNC_UPDATE_EVAL_END, queue->group, queue, updated);
	return updated;
}

/**
 * save_slot_cs() -  Save the state for blocked GPU command queue.
 *
 * @fw_io: Pointer to FW I/O manager.
 * @group_id:  CSG index.
 * @queue: Pointer to the GPU command queue.
 *
 * This function will check if GPU command queue is blocked on a sync wait and
 * evaluate the wait condition. If the wait condition isn't satisfied it would
 * save the state needed to reevaluate the condition in future.
 * The group to which queue is bound shall be in idle state.
 *
 * Return: true if the queue is blocked on a sync wait operation.
 */
static bool save_slot_cs(struct kbase_csf_fw_io *fw_io, u32 group_id, struct kbase_queue *queue)
{
	u32 stream_id = queue->csi_index;
	u32 status;
	bool is_waiting = false;
	u64 cmd_ptr;

	status = kbase_csf_fw_io_stream_read(fw_io, group_id, stream_id, CS_STATUS_WAIT);
	cmd_ptr = kbase_csf_fw_io_stream_read(fw_io, group_id, stream_id, CS_STATUS_CMD_PTR_LO);

	cmd_ptr |=
		(u64)kbase_csf_fw_io_stream_read(fw_io, group_id, stream_id, CS_STATUS_CMD_PTR_HI)
		<< 32;
	queue->saved_cmd_ptr = cmd_ptr;

	KBASE_KTRACE_ADD_CSF_GRP_Q(fw_io->kbdev, QUEUE_SYNC_UPDATE_WAIT_STATUS, queue->group, queue,
				   status);

	if (CS_STATUS_WAIT_SYNC_WAIT_GET(status) || CS_STATUS_WAIT_SB_MASK_GET(status)) {
		queue->status_wait = status;
		queue->sync_ptr = kbase_csf_fw_io_stream_read(fw_io, group_id, stream_id,
							      CS_STATUS_WAIT_SYNC_POINTER_LO);
		queue->sync_ptr |= (u64)kbase_csf_fw_io_stream_read(fw_io, group_id, stream_id,
								    CS_STATUS_WAIT_SYNC_POINTER_HI)
				   << 32;
		queue->sync_value = kbase_csf_fw_io_stream_read(fw_io, group_id, stream_id,
								CS_STATUS_WAIT_SYNC_VALUE);

		queue->sb_status = CS_STATUS_SCOREBOARDS_NONZERO_GET(kbase_csf_fw_io_stream_read(
			fw_io, group_id, stream_id, CS_STATUS_SCOREBOARDS));
		queue->blocked_reason = CS_STATUS_BLOCKED_REASON_REASON_GET(
			kbase_csf_fw_io_stream_read(fw_io, group_id, stream_id,
						    CS_STATUS_BLOCKED_REASON));

		if ((queue->blocked_reason == CS_STATUS_BLOCKED_ON_SB_WAIT) ||
		    !evaluate_sync_update(queue)) {
			is_waiting = true;
		} else {
			/* Sync object already got updated & met the condition
			 * thus it doesn't need to be reevaluated and so can
			 * clear the 'status_wait' here.
			 */
			queue->status_wait = 0;
		}
	} else {
		/* Invalidate wait status info that would have been recorded if
		 * this queue was blocked when the group (in idle state) was
		 * suspended previously. After that the group could have been
		 * unblocked due to the kicking of another queue bound to it &
		 * so the wait status info would have stuck with this queue.
		 */
		queue->status_wait = 0;
	}

	return is_waiting;
}

static void schedule_in_cycle(struct kbase_queue_group *group, bool force)
{
	struct kbase_context *kctx = group->kctx;
	struct kbase_device *kbdev = kctx->kbdev;
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;

	lockdep_assert_held(&scheduler->lock);

	/* Only try to schedule work for this event if no requests are pending,
	 * otherwise the function will end up canceling previous work requests,
	 * and scheduler is configured to wake up periodically (or the schedule
	 * of work needs to be enforced in situation such as entering into
	 * protected mode).
	 */
	if (likely(kbase_csf_scheduler_timer_is_enabled(kbdev)) || force) {
		dev_dbg(kbdev->dev, "Kicking async for group %d\n", group->handle);
		kbase_csf_scheduler_invoke_tock(kbdev);
	}
}

static void ktrace_log_group_state(struct kbase_queue_group *const group)
{
	switch (group->run_state) {
	case KBASE_CSF_GROUP_INACTIVE:
		KBASE_KTRACE_ADD_CSF_GRP(group->kctx->kbdev, CSF_GROUP_INACTIVE, group,
					 group->run_state);
		break;
	case KBASE_CSF_GROUP_RUNNABLE:
		KBASE_KTRACE_ADD_CSF_GRP(group->kctx->kbdev, CSF_GROUP_RUNNABLE, group,
					 group->run_state);
		break;
	case KBASE_CSF_GROUP_IDLE:
		KBASE_KTRACE_ADD_CSF_GRP(group->kctx->kbdev, CSF_GROUP_IDLE, group,
					 group->run_state);
		break;
	case KBASE_CSF_GROUP_SUSPENDED:
		KBASE_KTRACE_ADD_CSF_GRP(group->kctx->kbdev, CSF_GROUP_SUSPENDED, group,
					 group->run_state);
		break;
	case KBASE_CSF_GROUP_SUSPENDED_ON_IDLE:
		KBASE_KTRACE_ADD_CSF_GRP(group->kctx->kbdev, CSF_GROUP_SUSPENDED_ON_IDLE, group,
					 group->run_state);
		break;
	case KBASE_CSF_GROUP_SUSPENDED_ON_WAIT_SYNC:
		KBASE_KTRACE_ADD_CSF_GRP(group->kctx->kbdev, CSF_GROUP_SUSPENDED_ON_WAIT_SYNC,
					 group, group->run_state);
		break;
	case KBASE_CSF_GROUP_FAULT_EVICTED:
		KBASE_KTRACE_ADD_CSF_GRP(group->kctx->kbdev, CSF_GROUP_FAULT_EVICTED, group,
					 group->run_state);
		break;
	case KBASE_CSF_GROUP_TERMINATED:
		KBASE_KTRACE_ADD_CSF_GRP(group->kctx->kbdev, CSF_GROUP_TERMINATED, group,
					 group->run_state);
		break;
	}
}

static void insert_group_to_runnable(struct kbase_csf_scheduler *const scheduler,
				     struct kbase_queue_group *const group,
				     enum kbase_csf_group_state run_state)
{
	struct kbase_context *const kctx = group->kctx;
	struct kbase_device *const kbdev = kctx->kbdev;

	lockdep_assert_held(&scheduler->lock);

	WARN_ON(group->run_state != KBASE_CSF_GROUP_INACTIVE);

	if (WARN_ON(group->priority >= KBASE_QUEUE_GROUP_PRIORITY_COUNT))
		return;

	group->run_state = run_state;

	ktrace_log_group_state(group);

	if (run_state == KBASE_CSF_GROUP_RUNNABLE)
		group->prepared_seq_num = KBASEP_GROUP_PREPARED_SEQ_NUM_INVALID;

	list_add_tail(&group->link, &kctx->csf.sched.runnable_groups[group->priority]);
	kctx->csf.sched.num_runnable_grps++;
	KBASE_KTRACE_ADD_CSF_GRP(kbdev, GROUP_RUNNABLE_INSERT, group,
				 kctx->csf.sched.num_runnable_grps);

	/* Add the kctx if not yet in runnable kctxs */
	if (kctx->csf.sched.num_runnable_grps == 1) {
		/* First runnable csg, adds to the runnable_kctxs */
		INIT_LIST_HEAD(&kctx->csf.link);
		list_add_tail(&kctx->csf.link, &scheduler->runnable_kctxs);
		KBASE_KTRACE_ADD(kbdev, SCHEDULER_RUNNABLE_KCTX_INSERT, kctx, 0u);
	}

	scheduler->total_runnable_grps++;

	if (likely(kbase_csf_scheduler_timer_is_enabled(kbdev)) &&
	    (scheduler->total_runnable_grps == 1 || scheduler->state == SCHED_SUSPENDED ||
	     scheduler->state == SCHED_SLEEPING)) {
		dev_dbg(kbdev->dev, "Kicking scheduler on first runnable group\n");
		/* Fire a scheduling to start the time-slice */
		kbase_csf_scheduler_invoke_tick(kbdev);
	} else
		schedule_in_cycle(group, false);

	/* Since a new group has become runnable, check if GPU needs to be
	 * powered up.
	 */
	scheduler_wakeup(kbdev, false);
}

static void cancel_tick_work(struct kbase_csf_scheduler *const scheduler)
{
	hrtimer_cancel(&scheduler->tick_timer);
	atomic_set(&scheduler->pending_tick_work, false);
}

static void cancel_tock_work(struct kbase_csf_scheduler *const scheduler)
{
	atomic_set(&scheduler->pending_tock_work, false);
}

static void remove_group_from_runnable(struct kbase_csf_scheduler *const scheduler,
				       struct kbase_queue_group *group,
				       enum kbase_csf_group_state run_state)
{
	struct kbase_context *kctx = group->kctx;
	struct kbase_queue_group *new_head_grp;
	struct list_head *list = &kctx->csf.sched.runnable_groups[group->priority];
	unsigned long flags;

	lockdep_assert_held(&scheduler->lock);

	WARN_ON(!queue_group_scheduled_locked(group));

	group->run_state = run_state;

	ktrace_log_group_state(group);

	list_del_init(&group->link);

	spin_lock_irqsave(&scheduler->interrupt_lock, flags);
	/* The below condition will be true when the group running in protected
	 * mode is being terminated but the protected mode exit interrupt was't
	 * received. This can happen if the FW got stuck during protected mode
	 * for some reason (like GPU page fault or some internal error).
	 * In normal cases FW is expected to send the protected mode exit
	 * interrupt before it handles the CSG termination request.
	 */
	if (unlikely(scheduler->active_protm_grp == group)) {
		/* CSG slot cleanup should have happened for the pmode group */
		WARN_ON(kbasep_csf_scheduler_group_is_on_slot_locked(group));
		WARN_ON(group->run_state != KBASE_CSF_GROUP_INACTIVE);
		/* Initiate a GPU reset, in case it wasn't initiated yet,
		 * in order to rectify the anomaly.
		 */
		if (kbase_prepare_to_reset_gpu(kctx->kbdev, RESET_FLAGS_NONE))
			kbase_reset_gpu(kctx->kbdev);

		KBASE_KTRACE_ADD_CSF_GRP(kctx->kbdev, SCHEDULER_PROTM_EXIT,
					 scheduler->active_protm_grp, 0u);
		kctx->protm_exit_ts = ktime_get_raw();
		scheduler->active_protm_grp = NULL;
	}
	spin_unlock_irqrestore(&scheduler->interrupt_lock, flags);

	if (scheduler->top_grp == group) {
		/*
		 * Note: this disables explicit rotation in the next scheduling
		 * cycle. However, removing the top_grp is the same as an
		 * implicit rotation (e.g. if we instead rotated the top_kctx
		 * and then remove top_grp)
		 *
		 * This implicit rotation is assumed by the scheduler rotate
		 * functions.
		 */
		scheduler->top_grp = NULL;

		/*
		 * Trigger a scheduling tock for a CSG containing protected
		 * content in case there has been any in order to minimise
		 * latency.
		 */
		group = scheduler_get_protm_enter_async_group(kctx->kbdev, NULL);
		if (group)
			schedule_in_cycle(group, true);
	}

	kctx->csf.sched.num_runnable_grps--;
	KBASE_KTRACE_ADD_CSF_GRP(kctx->kbdev, GROUP_RUNNABLE_REMOVE, group,
				 kctx->csf.sched.num_runnable_grps);
	new_head_grp =
		(!list_empty(list)) ? list_first_entry(list, struct kbase_queue_group, link) : NULL;
	KBASE_KTRACE_ADD_CSF_GRP(kctx->kbdev, GROUP_RUNNABLE_HEAD, new_head_grp, 0u);

	if (kctx->csf.sched.num_runnable_grps == 0) {
		struct kbase_context *new_head_kctx;
		struct list_head *kctx_list = &scheduler->runnable_kctxs;
		/* drop the kctx */
		list_del_init(&kctx->csf.link);
		if (scheduler->top_kctx == kctx)
			scheduler->top_kctx = NULL;
		KBASE_KTRACE_ADD(kctx->kbdev, SCHEDULER_RUNNABLE_KCTX_REMOVE, kctx, 0u);
		new_head_kctx =
			(!list_empty(kctx_list)) ?
				      list_first_entry(kctx_list, struct kbase_context, csf.link) :
				      NULL;
		KBASE_KTRACE_ADD(kctx->kbdev, SCHEDULER_RUNNABLE_KCTX_HEAD, new_head_kctx, 0u);
	}

	WARN_ON(scheduler->total_runnable_grps == 0);
	scheduler->total_runnable_grps--;
	if (!scheduler->total_runnable_grps) {
		dev_dbg(kctx->kbdev->dev, "Scheduler idle has no runnable groups");
		cancel_tick_work(scheduler);
		WARN_ON(atomic_read(&scheduler->non_idle_offslot_grps));
		if (scheduler->state != SCHED_SUSPENDED)
			enqueue_gpu_idle_work(scheduler);
	}
	KBASE_KTRACE_ADD_CSF_GRP(kctx->kbdev, SCHEDULER_TOP_GRP, scheduler->top_grp,
				 scheduler->num_active_address_spaces |
					 (((u64)scheduler->total_runnable_grps) << 32));
}

static void insert_group_to_idle_wait(struct kbase_queue_group *const group)
{
	struct kbase_context *kctx = group->kctx;

	lockdep_assert_held(&kctx->kbdev->csf.scheduler.lock);

	WARN_ON(group->run_state != KBASE_CSF_GROUP_IDLE);

	list_add_tail(&group->link, &kctx->csf.sched.idle_wait_groups);
	kctx->csf.sched.num_idle_wait_grps++;
	KBASE_KTRACE_ADD_CSF_GRP(kctx->kbdev, GROUP_IDLE_WAIT_INSERT, group,
				 kctx->csf.sched.num_idle_wait_grps);
	group->run_state = KBASE_CSF_GROUP_SUSPENDED_ON_WAIT_SYNC;
	KBASE_KTRACE_ADD_CSF_GRP(kctx->kbdev, CSF_GROUP_SUSPENDED_ON_WAIT_SYNC, group,
				 group->run_state);
	dev_dbg(kctx->kbdev->dev, "Group-%d suspended on sync_wait, total wait_groups: %u\n",
		group->handle, kctx->csf.sched.num_idle_wait_grps);
}

static void remove_group_from_idle_wait(struct kbase_queue_group *const group)
{
	struct kbase_context *kctx = group->kctx;
	struct list_head *list = &kctx->csf.sched.idle_wait_groups;
	struct kbase_queue_group *new_head_grp;

	lockdep_assert_held(&kctx->kbdev->csf.scheduler.lock);

	WARN_ON(group->run_state != KBASE_CSF_GROUP_SUSPENDED_ON_WAIT_SYNC);

	list_del_init(&group->link);
	WARN_ON(kctx->csf.sched.num_idle_wait_grps == 0);
	kctx->csf.sched.num_idle_wait_grps--;
	KBASE_KTRACE_ADD_CSF_GRP(kctx->kbdev, GROUP_IDLE_WAIT_REMOVE, group,
				 kctx->csf.sched.num_idle_wait_grps);
	new_head_grp =
		(!list_empty(list)) ? list_first_entry(list, struct kbase_queue_group, link) : NULL;
	KBASE_KTRACE_ADD_CSF_GRP(kctx->kbdev, GROUP_IDLE_WAIT_HEAD, new_head_grp, 0u);
	group->run_state = KBASE_CSF_GROUP_INACTIVE;
	KBASE_KTRACE_ADD_CSF_GRP(kctx->kbdev, CSF_GROUP_INACTIVE, group, group->run_state);
}

static void deschedule_idle_wait_group(struct kbase_csf_scheduler *scheduler,
				       struct kbase_queue_group *group)
{
	lockdep_assert_held(&scheduler->lock);

	if (WARN_ON(!group))
		return;

	remove_group_from_runnable(scheduler, group, KBASE_CSF_GROUP_IDLE);
	insert_group_to_idle_wait(group);
}

static void update_offslot_non_idle_cnt(struct kbase_queue_group *group)
{
	struct kbase_device *kbdev = group->kctx->kbdev;
	struct kbase_csf_scheduler *const scheduler = &kbdev->csf.scheduler;

	lockdep_assert_held(&scheduler->lock);

	if (group->prepared_seq_num < scheduler->non_idle_scanout_grps) {
		int new_val = atomic_dec_return(&scheduler->non_idle_offslot_grps);
		KBASE_KTRACE_ADD_CSF_GRP(kbdev, SCHEDULER_NONIDLE_OFFSLOT_GRP_DEC, group,
					 (u64)new_val);
	}
}

static void update_offslot_non_idle_cnt_for_onslot_grp(struct kbase_queue_group *group)
{
	struct kbase_device *kbdev = group->kctx->kbdev;
	struct kbase_csf_scheduler *const scheduler = &kbdev->csf.scheduler;

	lockdep_assert_held(&scheduler->lock);

	WARN_ON(group->csg_nr < 0);

	if (group->prepared_seq_num < scheduler->non_idle_scanout_grps) {
		int new_val = atomic_dec_return(&scheduler->non_idle_offslot_grps);
		KBASE_KTRACE_ADD_CSF_GRP(kbdev, SCHEDULER_NONIDLE_OFFSLOT_GRP_DEC, group,
					 (u64)new_val);
	}
}

static void update_offslot_non_idle_cnt_on_grp_suspend(struct kbase_queue_group *group)
{
	struct kbase_device *kbdev = group->kctx->kbdev;
	struct kbase_csf_scheduler *const scheduler = &kbdev->csf.scheduler;

	lockdep_assert_held(&scheduler->lock);

	if (scheduler->state == SCHED_BUSY) {
		/* active phase or, async entering the protected mode */
		if (group->prepared_seq_num >= scheduler->non_idle_scanout_grps) {
			/* At scanout, it was tagged as on-slot idle */
			if (group->run_state == KBASE_CSF_GROUP_SUSPENDED) {
				int new_val = atomic_inc_return(&scheduler->non_idle_offslot_grps);
				KBASE_KTRACE_ADD_CSF_GRP(kbdev, SCHEDULER_NONIDLE_OFFSLOT_GRP_INC,
							 group, (u64)new_val);
			}
		} else {
			if (group->run_state != KBASE_CSF_GROUP_SUSPENDED) {
				int new_val = atomic_dec_return(&scheduler->non_idle_offslot_grps);
				KBASE_KTRACE_ADD_CSF_GRP(kbdev, SCHEDULER_NONIDLE_OFFSLOT_GRP_DEC,
							 group, (u64)new_val);
			}
		}
	} else {
		/* async phases */
		if (group->run_state == KBASE_CSF_GROUP_SUSPENDED) {
			int new_val = atomic_inc_return(&scheduler->non_idle_offslot_grps);
			KBASE_KTRACE_ADD_CSF_GRP(kbdev, SCHEDULER_NONIDLE_OFFSLOT_GRP_INC, group,
						 (u64)new_val);
		}
	}
}

static bool confirm_cmd_buf_empty(struct kbase_queue const *queue)
{
	bool cs_empty;
	bool cs_idle;
	u32 sb_status = 0;

	struct kbase_device *kbdev = queue->group->kctx->kbdev;
	struct kbase_csf_global_iface const *const iface = &kbdev->csf.global_iface;

	u32 glb_version = iface->version;

	u64 const *input_addr = (u64 const *)queue->user_io_addr;
	u64 const *output_addr = (u64 const *)(queue->user_io_addr + PAGE_SIZE / sizeof(u64));

	if (glb_version >= kbase_csf_interface_version(1, 0, 0)) {
		/* CS_STATUS_SCOREBOARD supported from CSF 1.0 */
		sb_status = CS_STATUS_SCOREBOARDS_NONZERO_GET(
			kbase_csf_fw_io_stream_read(&kbdev->csf.fw_io, queue->group->csg_nr,
						    queue->csi_index, CS_STATUS_SCOREBOARDS));
	}

	/*
	 * These 64-bit reads and writes will be atomic on a 64-bit kernel but may
	 * not be atomic on 32-bit kernels. Support for 32-bit kernels is limited to
	 * build-only.
	 */
	cs_empty = (input_addr[CS_INSERT_LO / sizeof(u64)] ==
		    output_addr[CS_EXTRACT_LO / sizeof(u64)]);
	cs_idle = cs_empty && (!sb_status);

	return cs_idle;
}

static void save_csg_slot(struct kbase_queue_group *group)
{
	struct kbase_device *kbdev = group->kctx->kbdev;
	struct kbase_csf_scheduler *const scheduler = &kbdev->csf.scheduler;
	struct kbase_csf_cmd_stream_group_info *ginfo;
	u32 state;

	lockdep_assert_held(&scheduler->lock);

	if (WARN_ON(!kbasep_csf_scheduler_group_is_on_slot_locked(group)))
		return;

	ginfo = &kbdev->csf.global_iface.groups[group->csg_nr];

	state = get_csg_ack_state(&kbdev->csf.fw_io, group->csg_nr);

	if (!WARN_ON((state != CSG_ACK_STATE_SUSPEND) && (state != CSG_ACK_STATE_TERMINATE))) {
		u32 max_streams = ginfo->stream_num;
		u32 i;
		bool sync_wait = false;
		bool idle;

		idle = kbase_csf_fw_io_group_read(&kbdev->csf.fw_io, group->csg_nr,
						  CSG_STATUS_STATE) &
		       CSG_STATUS_STATE_IDLE_MASK;

#if IS_ENABLED(CONFIG_MALI_NO_MALI)
		for (i = 0; i < max_streams; i++)
			update_hw_active(group->bound_queues[i], false);
#endif /* CONFIG_MALI_NO_MALI */
		for (i = 0; idle && i < max_streams; i++) {
			struct kbase_queue *const queue = group->bound_queues[i];

			if (!queue || !queue->enabled)
				continue;

			if (save_slot_cs(&kbdev->csf.fw_io, group->csg_nr, queue)) {
				/* sync_wait is only true if the queue is blocked on
				 * a CQS and not a scoreboard.
				 */
				if (queue->blocked_reason != CS_STATUS_BLOCKED_ON_SB_WAIT)
					sync_wait = true;
			} else {
				/* Need to confirm if ringbuffer of the GPU
				 * queue is empty or not. A race can arise
				 * between the flush of GPU queue and suspend
				 * of CSG. If a queue is flushed after FW has
				 * set the IDLE bit in CSG_STATUS_STATE, then
				 * Scheduler will incorrectly consider CSG
				 * as idle. And there may not be any further
				 * flush call for the GPU queue, which would
				 * have de-idled the CSG.
				 */
				idle = confirm_cmd_buf_empty(queue);
			}
		}

		if (idle) {
			/* Take the suspended group out of the runnable_groups
			 * list of the context and move it to the
			 * idle_wait_groups list.
			 */
			if (sync_wait)
				deschedule_idle_wait_group(scheduler, group);
			else {
				group->run_state = KBASE_CSF_GROUP_SUSPENDED_ON_IDLE;
				KBASE_KTRACE_ADD_CSF_GRP(kbdev, CSF_GROUP_SUSPENDED_ON_IDLE, group,
							 group->run_state);
				dev_dbg(kbdev->dev, "Group-%d suspended: idle", group->handle);
			}
		} else {
			group->run_state = KBASE_CSF_GROUP_SUSPENDED;
			KBASE_KTRACE_ADD_CSF_GRP(kbdev, CSF_GROUP_SUSPENDED, group,
						 group->run_state);
		}

		update_offslot_non_idle_cnt_on_grp_suspend(group);
		kbase_csf_tiler_heap_reclaim_sched_notify_grp_suspend(group);
	}
}

/* Cleanup_csg_slot after it has been vacated, ready for next csg run.
 * Return whether there is a kctx address fault associated with the group
 * for which the clean-up is done.
 */
static bool cleanup_csg_slot(struct kbase_queue_group *group)
{
	struct kbase_context *kctx = group->kctx;
	struct kbase_device *kbdev = kctx->kbdev;
	struct kbase_csf_global_iface *global_iface = &kbdev->csf.global_iface;
	struct kbase_csf_cmd_stream_group_info *ginfo;
	s8 slot;
	struct kbase_csf_csg_slot *csg_slot;
	unsigned long flags, fw_io_flags;
	u32 csg_req, csg_ack, i;
	bool as_fault = false;

	lockdep_assert_held(&kbdev->csf.scheduler.lock);

	if (WARN_ON(!kbasep_csf_scheduler_group_is_on_slot_locked(group)))
		return as_fault;

	slot = group->csg_nr;
	csg_slot = &kbdev->csf.scheduler.csg_slots[slot];
	ginfo = &global_iface->groups[slot];

	/* Now loop through all the bound CSs, and clean them via a stop */
	for (i = 0; i < ginfo->stream_num; i++) {
		if (group->bound_queues[i]) {
			if (group->bound_queues[i]->enabled) {
				/* Write to FW regardless of its status to prevent the queue
				 * from running after the GPU resumes.
				 */
				kbase_csf_fw_io_open_force(&kbdev->csf.fw_io, &fw_io_flags);
				kbase_csf_fw_io_stream_write_mask(&kbdev->csf.fw_io, slot, i,
								  CS_REQ, CS_REQ_STATE_STOP,
								  CS_REQ_STATE_MASK);
				kbase_csf_fw_io_close(&kbdev->csf.fw_io, fw_io_flags);
			}

			unassign_user_doorbell_from_queue(kbdev, group->bound_queues[i]);
		}
	}

	unassign_user_doorbell_from_group(kbdev, group);

	/* The csg does not need cleanup other than drop its AS */
	spin_lock_irqsave(&kctx->kbdev->hwaccess_lock, flags);
	as_fault = kbase_ctx_flag(kctx, KCTX_AS_DISABLED_ON_FAULT);
	kbase_ctx_sched_release_ctx(kctx);
	if (unlikely(group->faulted))
		as_fault = true;
	spin_unlock_irqrestore(&kctx->kbdev->hwaccess_lock, flags);

#if IS_ENABLED(CONFIG_MALI_TRACE_POWER_GPU_WORK_PERIOD)
	spin_lock_bh(&kbdev->csf.scheduler.gpu_metrics_lock);
	emit_gpu_metrics_to_frontend_for_off_slot_group(group);
#endif
	/* now marking the slot is vacant */
	spin_lock_irqsave(&kbdev->csf.scheduler.interrupt_lock, flags);
	/* Process pending SYNC_UPDATE, if any */

	csg_req = kbase_csf_fw_io_group_input_read(&kbdev->csf.fw_io, slot, CSG_REQ);
	csg_ack = kbase_csf_fw_io_group_read(&kbdev->csf.fw_io, slot, CSG_ACK);
	/* Write to FW regardless of its status to prevent the queue
	 * from running after the GPU resumes.
	 */
	kbase_csf_fw_io_open_force(&kbdev->csf.fw_io, &fw_io_flags);
	kbase_csf_handle_csg_sync_update(kbdev, slot, group, csg_req, csg_ack);
	kbase_csf_fw_io_close(&kbdev->csf.fw_io, fw_io_flags);

	kbdev->csf.scheduler.csg_slots[slot].resident_group = NULL;
	clear_bit(slot, kbdev->csf.scheduler.csg_slots_idle_mask);
	KBASE_KTRACE_ADD_CSF_GRP(kbdev, CSG_SLOT_IDLE_CLEAR, group,
				 kbdev->csf.scheduler.csg_slots_idle_mask[0]);

	group->csg_nr = KBASEP_CSG_NR_INVALID;
	set_bit(slot, kbdev->csf.scheduler.csgs_events_enable_mask);
	clear_bit(slot, kbdev->csf.scheduler.csg_inuse_bitmap);
	spin_unlock_irqrestore(&kbdev->csf.scheduler.interrupt_lock, flags);
#if IS_ENABLED(CONFIG_MALI_TRACE_POWER_GPU_WORK_PERIOD)
	spin_unlock_bh(&kbdev->csf.scheduler.gpu_metrics_lock);
#endif

	atomic_set(&csg_slot->state, CSG_SLOT_READY);

	KBASE_KTRACE_ADD_CSF_GRP(kbdev, CSG_SLOT_CLEANED, group, (u64)slot);
	dev_dbg(kbdev->dev, "Cleanup done for group %d on slot %d\n", group->handle, slot);

	KBASE_TLSTREAM_TL_KBASE_DEVICE_DEPROGRAM_CSG(kbdev, kbdev->id, (u32)slot);

	/* Notify the group is off-slot and the csg_reg might be available for
	 * resue with other groups in a 'lazy unbinding' style.
	 */
	kbase_csf_mcu_shared_set_group_csg_reg_unused(kbdev, group);

	return as_fault;
}

static void update_csg_slot_priority(struct kbase_queue_group *group, u8 prio)
{
	struct kbase_device *kbdev = group->kctx->kbdev;
	struct kbase_csf_csg_slot *csg_slot;
	s8 slot;
	u8 prev_prio;
	u64 ep_cfg;
	u32 csg_req;
	unsigned long flags, fw_io_flags;

	lockdep_assert_held(&kbdev->csf.scheduler.lock);

	if (WARN_ON(!kbasep_csf_scheduler_group_is_on_slot_locked(group)))
		return;

	slot = group->csg_nr;
	csg_slot = &kbdev->csf.scheduler.csg_slots[slot];

	/* CSGs remaining on-slot can be either idle or runnable.
	 * This also applies in protected mode.
	 */
	WARN_ON(!((group->run_state == KBASE_CSF_GROUP_RUNNABLE) ||
		  (group->run_state == KBASE_CSF_GROUP_IDLE)));

	/* Update consumes a group from scanout */
	update_offslot_non_idle_cnt_for_onslot_grp(group);

	if (csg_slot->priority == prio)
		return;

	spin_lock_irqsave(&kbdev->csf.scheduler.interrupt_lock, flags);
	if (kbase_csf_fw_io_open(&kbdev->csf.fw_io, &fw_io_flags)) {
		spin_unlock_irqrestore(&kbdev->csf.scheduler.interrupt_lock, flags);
		goto exit;
	}

	ep_cfg = kbase_csf_fw_io_group_input_read(&kbdev->csf.fw_io, slot, CSG_EP_REQ_LO);

	prev_prio = CSG_EP_REQ_PRIORITY_GET(ep_cfg);
	ep_cfg = CSG_EP_REQ_PRIORITY_SET(ep_cfg, prio);
	kbase_csf_fw_io_group_write(&kbdev->csf.fw_io, slot, CSG_EP_REQ_LO, ep_cfg);

	csg_req = kbase_csf_fw_io_group_read(&kbdev->csf.fw_io, slot, CSG_ACK);
	csg_req ^= CSG_REQ_EP_CFG_MASK;
	kbase_csf_fw_io_group_write_mask(&kbdev->csf.fw_io, slot, CSG_REQ, csg_req,
					 CSG_REQ_EP_CFG_MASK);
	kbase_csf_ring_csg_doorbell(kbdev, slot);
	kbase_csf_fw_io_close(&kbdev->csf.fw_io, fw_io_flags);
	spin_unlock_irqrestore(&kbdev->csf.scheduler.interrupt_lock, flags);

	dev_dbg(kbdev->dev,
		"Priority for group %d of context %d_%d on slot %d to be updated from %u to %u\n",
		group->handle, group->kctx->tgid, group->kctx->id, slot, prev_prio, prio);
exit:
	csg_slot->priority = prio;

	KBASE_KTRACE_ADD_CSF_GRP(kbdev, CSG_SLOT_PRIO_UPDATE, group, prio);

	set_bit(slot, kbdev->csf.scheduler.csg_slots_prio_update);
}

static void program_csg_slot(struct kbase_queue_group *group, s8 slot, u8 prio)
{
	struct kbase_context *kctx = group->kctx;
	struct kbase_device *kbdev = kctx->kbdev;
	struct kbase_csf_global_iface *global_iface = &kbdev->csf.global_iface;
	const u64 shader_core_mask = kbase_pm_get_present_cores(kbdev, KBASE_PM_CORE_SHADER);
	const u64 tiler_core_mask = kbase_pm_get_present_cores(kbdev, KBASE_PM_CORE_TILER);
	const u64 compute_mask = shader_core_mask & group->compute_mask;
	const u64 fragment_mask = shader_core_mask & group->fragment_mask;
	const u64 tiler_mask = tiler_core_mask & group->tiler_mask;
	const u8 compute_max = min(kbdev->gpu_props.num_cores, group->compute_max);
	const u8 fragment_max = min(kbdev->gpu_props.num_cores, group->fragment_max);
	const u8 tiler_max = min(CSG_TILER_MAX, group->tiler_max);
	u64 ep_cfg = 0;
	u32 state;
	int i;
	unsigned long flags, fw_io_flags;
	u64 normal_suspend_buf;
	u64 protm_suspend_buf;
	struct kbase_csf_csg_slot *csg_slot = &kbdev->csf.scheduler.csg_slots[slot];
	struct kbase_csf_fw_io *fw_io = &kbdev->csf.fw_io;

	lockdep_assert_held(&kbdev->csf.scheduler.lock);

	if (WARN_ON(slot < 0) && WARN_ON((u32)slot >= global_iface->group_num))
		return;

	WARN_ON(atomic_read(&csg_slot->state) != CSG_SLOT_READY);

	if (unlikely(kbase_csf_mcu_shared_group_bind_csg_reg(kbdev, group))) {
		dev_warn(kbdev->dev,
			 "Couldn't bind MCU shared csg_reg for group %d of context %d_%d, slot=%u",
			 group->handle, group->kctx->tgid, kctx->id, slot);
		kbase_csf_mcu_shared_set_group_csg_reg_unused(kbdev, group);
		return;
	}

	/* The suspend buf has already been mapped through binding to csg_reg */
	normal_suspend_buf = group->normal_suspend_buf.gpu_va;
	protm_suspend_buf = group->protected_suspend_buf.gpu_va;
	WARN_ONCE(!normal_suspend_buf, "Normal suspend buffer not mapped");

	/* Pick an available address space for this context */
	mutex_lock(&kbdev->mmu_hw_mutex);
	spin_lock_irqsave(&kbdev->hwaccess_lock, flags);
	kbase_ctx_sched_retain_ctx(kctx);
	spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);
	mutex_unlock(&kbdev->mmu_hw_mutex);

	if (kctx->as_nr == KBASEP_AS_NR_INVALID) {
		dev_warn(kbdev->dev,
			 "Could not get a valid AS for group %d of context %d_%d on slot %d\n",
			 group->handle, kctx->tgid, kctx->id, slot);
		kbase_csf_mcu_shared_set_group_csg_reg_unused(kbdev, group);
		return;
	}

#if IS_ENABLED(CONFIG_MALI_TRACE_POWER_GPU_WORK_PERIOD)
	spin_lock_bh(&kbdev->csf.scheduler.gpu_metrics_lock);
#endif
	spin_lock_irqsave(&kbdev->csf.scheduler.interrupt_lock, flags);
	set_bit(slot, kbdev->csf.scheduler.csg_inuse_bitmap);
	kbdev->csf.scheduler.csg_slots[slot].resident_group = group;
	group->csg_nr = slot;

	spin_unlock_irqrestore(&kbdev->csf.scheduler.interrupt_lock, flags);
#if IS_ENABLED(CONFIG_MALI_TRACE_POWER_GPU_WORK_PERIOD)
	spin_unlock_bh(&kbdev->csf.scheduler.gpu_metrics_lock);
#endif

	assign_user_doorbell_to_group(kbdev, group);

	/* Now loop through all the bound & kicked CSs, and program them */
	for (i = 0; i < MAX_SUPPORTED_STREAMS_PER_GROUP; i++) {
		struct kbase_queue *queue = group->bound_queues[i];

		if (queue)
			program_cs(kbdev, queue, false);
	}

	spin_lock_irqsave(&kbdev->csf.scheduler.interrupt_lock, flags);
	/* Set state to START/RESUME */
	if (queue_group_suspended_locked(group)) {
		state = CSG_REQ_STATE_RESUME;
	} else {
		WARN_ON(group->run_state != KBASE_CSF_GROUP_RUNNABLE);
		state = CSG_REQ_STATE_START;
	}

	if (kbase_csf_fw_io_open(fw_io, &fw_io_flags)) {
		spin_unlock_irqrestore(&kbdev->csf.scheduler.interrupt_lock, flags);
		goto skip_fw;
	}

	/* Endpoint programming for CSG */
	kbase_csf_fw_io_group_write(fw_io, slot, CSG_ALLOW_COMPUTE_LO, compute_mask & U32_MAX);
	kbase_csf_fw_io_group_write(fw_io, slot, CSG_ALLOW_COMPUTE_HI, compute_mask >> 32);
	kbase_csf_fw_io_group_write(fw_io, slot, CSG_ALLOW_FRAGMENT_LO, fragment_mask & U32_MAX);
	kbase_csf_fw_io_group_write(fw_io, slot, CSG_ALLOW_FRAGMENT_HI, fragment_mask >> 32);

	kbase_csf_fw_io_group_write(fw_io, slot, CSG_ALLOW_OTHER, tiler_mask & U32_MAX);

	/* Register group UID with firmware */
	kbase_csf_fw_io_group_write(fw_io, slot, CSG_ITER_TRACE_CONFIG, group->group_uid);

	ep_cfg = CSG_EP_REQ_COMPUTE_EP_SET(ep_cfg, compute_max);
	ep_cfg = CSG_EP_REQ_FRAGMENT_EP_SET(ep_cfg, fragment_max);
	ep_cfg = CSG_EP_REQ_TILER_EP_SET(ep_cfg, tiler_max);
	ep_cfg = CSG_EP_REQ_PRIORITY_SET(ep_cfg, prio);
	kbase_csf_fw_io_group_write(fw_io, slot, CSG_EP_REQ_LO, ep_cfg & U32_MAX);

	/* EP_CFG request to handle endpoint configuration changes should not be sent
	 * at this point, as the config registers are read on starting or resuming a CSG.
	 */

	/* Program the address space number assigned to the context */
	kbase_csf_fw_io_group_write(fw_io, slot, CSG_CONFIG, (u32)kctx->as_nr);

	kbase_csf_fw_io_group_write(fw_io, slot, CSG_SUSPEND_BUF_LO, normal_suspend_buf & U32_MAX);
	kbase_csf_fw_io_group_write(fw_io, slot, CSG_SUSPEND_BUF_HI, normal_suspend_buf >> 32);

	/* Note, we program the P-mode buffer pointer here, but actual runtime
	 * enter into pmode execution is controlled by the P-mode phy pages are
	 * allocated and mapped with the bound csg_reg, which has a specific flag
	 * for indicating this P-mode runnable condition before a group is
	 * granted its p-mode section entry. Without a P-mode entry, the buffer
	 * pointed is not going to be accessed at all.
	 */
	kbase_csf_fw_io_group_write(fw_io, slot, CSG_PROTM_SUSPEND_BUF_LO,
				    protm_suspend_buf & U32_MAX);
	kbase_csf_fw_io_group_write(fw_io, slot, CSG_PROTM_SUSPEND_BUF_HI, protm_suspend_buf >> 32);

	if (group->dvs_buf) {
		kbase_csf_fw_io_group_write(fw_io, slot, CSG_DVS_BUF_LO, group->dvs_buf & U32_MAX);
		kbase_csf_fw_io_group_write(fw_io, slot, CSG_DVS_BUF_HI, group->dvs_buf >> 32);
	}

	/* Enable all interrupts for now */
	kbase_csf_fw_io_group_write(fw_io, slot, CSG_ACK_IRQ_MASK, ~((u32)0));

	kbase_csf_fw_io_group_write_mask(fw_io, slot, CSG_REQ, state, CSG_REQ_STATE_MASK);
	kbase_csf_ring_csg_doorbell(kbdev, slot);
	kbase_csf_fw_io_close(fw_io, fw_io_flags);

	spin_unlock_irqrestore(&kbdev->csf.scheduler.interrupt_lock, flags);
skip_fw:
	/* Update status before rings the door-bell, marking ready => run */
	atomic_set(&csg_slot->state, CSG_SLOT_READY2RUN);
	csg_slot->priority = prio;

	/* Trace the programming of the CSG on the slot */
	KBASE_TLSTREAM_TL_KBASE_DEVICE_PROGRAM_CSG(kbdev, kbdev->id, group->kctx->id, group->handle,
						   (u32)slot,
						   (state == CSG_REQ_STATE_RESUME) ? 1 : 0);

	dev_dbg(kbdev->dev, "Starting group %d of context %d_%d on slot %d with priority %u\n",
		group->handle, kctx->tgid, kctx->id, slot, prio);

	KBASE_KTRACE_ADD_CSF_GRP(kbdev, CSG_SLOT_START_REQ, group,
				 (((u64)ep_cfg) << 32) | ((((u32)kctx->as_nr) & 0xF) << 16) |
					 (state & (CSG_REQ_STATE_MASK >> CS_REQ_STATE_SHIFT)));

	/* Update the heap reclaim manager */
	kbase_csf_tiler_heap_reclaim_sched_notify_grp_active(group);

	/* Programming a slot consumes a group from scanout */
	update_offslot_non_idle_cnt_for_onslot_grp(group);

	/* Notify the group's bound csg_reg is now in active use */
	kbase_csf_mcu_shared_set_group_csg_reg_active(kbdev, group);
}

static void remove_scheduled_group(struct kbase_device *kbdev, struct kbase_queue_group *group)
{
	struct kbase_csf_scheduler *const scheduler = &kbdev->csf.scheduler;

	lockdep_assert_held(&scheduler->lock);

	WARN_ON(group->prepared_seq_num == KBASEP_GROUP_PREPARED_SEQ_NUM_INVALID);
	WARN_ON(list_empty(&group->link_to_schedule));

	list_del_init(&group->link_to_schedule);
	scheduler->ngrp_to_schedule--;
	group->prepared_seq_num = KBASEP_GROUP_PREPARED_SEQ_NUM_INVALID;
	group->kctx->csf.sched.ngrp_to_schedule--;
}

static void sched_evict_group(struct kbase_queue_group *group, bool fault,
			      bool update_non_idle_offslot_grps_cnt_from_run_state)
{
	struct kbase_context *kctx = group->kctx;
	struct kbase_device *kbdev = kctx->kbdev;
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;

	lockdep_assert_held(&kbdev->csf.scheduler.lock);

	if (queue_group_scheduled_locked(group)) {
		u32 i;

		if (update_non_idle_offslot_grps_cnt_from_run_state &&
		    (group->run_state == KBASE_CSF_GROUP_SUSPENDED ||
		     group->run_state == KBASE_CSF_GROUP_RUNNABLE)) {
			int new_val = atomic_dec_return(&scheduler->non_idle_offslot_grps);
			KBASE_KTRACE_ADD_CSF_GRP(kbdev, SCHEDULER_NONIDLE_OFFSLOT_GRP_DEC, group,
						 (u64)new_val);
		}

		for (i = 0; i < MAX_SUPPORTED_STREAMS_PER_GROUP; i++) {
			if (group->bound_queues[i])
				group->bound_queues[i]->enabled = false;
		}

		if (group->prepared_seq_num != KBASEP_GROUP_PREPARED_SEQ_NUM_INVALID) {
			if (!update_non_idle_offslot_grps_cnt_from_run_state)
				update_offslot_non_idle_cnt(group);
			remove_scheduled_group(kbdev, group);
		}

		if (group->run_state == KBASE_CSF_GROUP_SUSPENDED_ON_WAIT_SYNC)
			remove_group_from_idle_wait(group);
		else {
			remove_group_from_runnable(scheduler, group, KBASE_CSF_GROUP_INACTIVE);
		}

		WARN_ON(group->run_state != KBASE_CSF_GROUP_INACTIVE);

		if (fault) {
			group->run_state = KBASE_CSF_GROUP_FAULT_EVICTED;
			KBASE_KTRACE_ADD_CSF_GRP(kbdev, CSF_GROUP_FAULT_EVICTED, group,
						 scheduler->total_runnable_grps);
		}

		KBASE_KTRACE_ADD_CSF_GRP(kbdev, GROUP_EVICT, group,
					 (((u64)scheduler->total_runnable_grps) << 32) |
						 ((u32)group->run_state));
		dev_dbg(kbdev->dev, "group %d exited scheduler, num_runnable_grps %d\n",
			group->handle, scheduler->total_runnable_grps);
		/* Notify a group has been evicted */
		wake_up_all(&kbdev->csf.event_wait);
	}

	kbase_csf_tiler_heap_reclaim_sched_notify_grp_evict(group);

	/* Clear all the bound shared regions and unmap any in-place MMU maps */
	kbase_csf_mcu_shared_clear_evicted_group_csg_reg(kbdev, group);
}

static int term_group_sync(struct kbase_queue_group *group)
{
	struct kbase_device *kbdev = group->kctx->kbdev;
	unsigned int group_term_timeout_ms = kbase_get_timeout_ms(kbdev, CSF_CSG_TERM_TIMEOUT);
	long remaining = 0;
	int err = 0;

	term_csg_slot(group);

	if (kbdev->protected_mode) {
		/* When GPU is in protected mode, CSG termination would only be
		 * performed after FW suspends the CSG running in protected mode
		 * and it switches to normal mode. So increase the timeout value
		 * to account for the extra work needed before the actual handling
		 * of CSG termination request can begin.
		 */
		group_term_timeout_ms += kbdev->csf.csg_suspend_timeout_ms;

		if (IS_ENABLED(CONFIG_MALI_REAL_HW) && !IS_ENABLED(CONFIG_MALI_IS_FPGA) &&
		    unlikely(group_term_timeout_ms >= MAX_TIMEOUT_MS))
			group_term_timeout_ms = MAX_TIMEOUT_MS;
	}

	remaining = kbase_csf_timeout_in_jiffies(group_term_timeout_ms);
	remaining = kbase_csf_fw_io_wait_event_timeout(
		&kbdev->csf.fw_io, kbdev->csf.event_wait,
		group->cs_unrecoverable || csg_slot_stopped_locked(kbdev, group->csg_nr),
		remaining);

	if (unlikely(!remaining)) {
		enum dumpfault_error_type error_type = DF_CSG_TERMINATE_TIMEOUT;
		pixel_gpu_uevent_kmd_error_send(kbdev, GPU_UEVENT_INFO_GROUP_TERM);

		dev_warn(
			kbdev->dev,
			"[%llu] term request timeout (%d ms) for group %d of context %d_%d on slot %d",
			kbase_backend_get_cycle_cnt(kbdev), group_term_timeout_ms, group->handle,
			group->kctx->tgid, group->kctx->id, group->csg_nr);
		if (kbase_csf_firmware_ping_wait(kbdev, FW_PING_AFTER_ERROR_TIMEOUT_MS))
			error_type = DF_PING_REQUEST_TIMEOUT;
		schedule_actions_trigger_df(kbdev, group->kctx, error_type);
		if (kbase_prepare_to_reset_gpu(kbdev, RESET_FLAGS_NONE))
			kbase_reset_gpu(kbdev);

		err = -ETIMEDOUT;
	}

	return err;
}

void kbase_csf_scheduler_group_deschedule(struct kbase_queue_group *group)
{
	struct kbase_device *kbdev = group->kctx->kbdev;
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;
	bool wait_for_termination = true;
	bool on_slot;

	kbase_reset_gpu_assert_failed_or_prevented(kbdev);
	lockdep_assert_held(&group->kctx->csf.lock);
	rt_mutex_lock(&scheduler->lock);

	KBASE_KTRACE_ADD_CSF_GRP(kbdev, GROUP_DESCHEDULE, group, group->run_state);
	wait_for_dump_complete_on_group_deschedule(group);
	if (!queue_group_scheduled_locked(group))
		goto unlock;

	on_slot = kbasep_csf_scheduler_group_is_on_slot_locked(group);

#ifdef KBASE_PM_RUNTIME
	/* If the queue group is on slot and Scheduler is in SLEEPING state,
	 * then we need to wake up the Scheduler to exit the sleep state rather
	 * than waiting for the runtime suspend or power down of GPU.
	 * The group termination is usually triggered in the context of Application
	 * thread and it has been seen that certain Apps can destroy groups at
	 * random points and not necessarily when the App is exiting.
	 */
	if (on_slot && (scheduler->state == SCHED_SLEEPING)) {
		scheduler_wakeup(kbdev, true);

		/* Wait for MCU firmware to start running */
		if (kbase_csf_scheduler_wait_mcu_active(kbdev)) {
			dev_warn(
				kbdev->dev,
				"[%llu] Wait for MCU active failed when terminating group %d of context %d_%d on slot %d",
				kbase_backend_get_cycle_cnt(kbdev), group->handle,
				group->kctx->tgid, group->kctx->id, group->csg_nr);
			/* No point in waiting for CSG termination if MCU didn't
			 * become active.
			 */
			wait_for_termination = false;
		}
	}
#endif
	if (!on_slot) {
		sched_evict_group(group, false, true);
	} else {
		bool as_faulty;

		if (likely(wait_for_termination))
			term_group_sync(group);
		else
			term_csg_slot(group);

		/* Treat the csg been terminated */
		as_faulty = cleanup_csg_slot(group);
		/* remove from the scheduler list */
		sched_evict_group(group, as_faulty, false);
	}

	WARN_ON(queue_group_scheduled_locked(group));

unlock:
	rt_mutex_unlock(&scheduler->lock);
}

/**
 * scheduler_group_schedule() - Schedule a GPU command queue group on firmware
 *
 * @group: Pointer to the queue group to be scheduled.
 *
 * This function would enable the scheduling of GPU command queue group on
 * firmware.
 *
 * Return: 0 on success, or negative on failure.
 */
static int scheduler_group_schedule(struct kbase_queue_group *group)
{
	struct kbase_context *kctx = group->kctx;
	struct kbase_device *kbdev = kctx->kbdev;
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;

	lockdep_assert_held(&kctx->csf.lock);
	lockdep_assert_held(&scheduler->lock);

	KBASE_KTRACE_ADD_CSF_GRP(kbdev, GROUP_SCHEDULE, group, group->run_state);
	if (group->run_state == KBASE_CSF_GROUP_SUSPENDED_ON_WAIT_SYNC)
		update_idle_suspended_group_state(group);
	else if (queue_group_idle_locked(group)) {
		WARN_ON(kctx->csf.sched.num_runnable_grps == 0);
		WARN_ON(kbdev->csf.scheduler.total_runnable_grps == 0);

		if (group->run_state == KBASE_CSF_GROUP_SUSPENDED_ON_IDLE)
			update_idle_suspended_group_state(group);
		else {
			struct kbase_queue_group *protm_grp;
			unsigned long flags;

			WARN_ON(!kbasep_csf_scheduler_group_is_on_slot_locked(group));

			group->run_state = KBASE_CSF_GROUP_RUNNABLE;
			KBASE_KTRACE_ADD_CSF_GRP(kbdev, CSF_GROUP_RUNNABLE, group,
						 group->run_state);

			/* A normal mode CSG could be idle onslot during
			 * protected mode. In this case clear the
			 * appropriate bit in csg_slots_idle_mask.
			 */
			spin_lock_irqsave(&scheduler->interrupt_lock, flags);
			protm_grp = scheduler->active_protm_grp;
			if (protm_grp && protm_grp != group) {
				clear_bit((unsigned int)group->csg_nr,
					  scheduler->csg_slots_idle_mask);
				/* Request the update to confirm the condition inferred. */
				group->reevaluate_idle_status = true;
				KBASE_KTRACE_ADD_CSF_GRP(kbdev, CSG_SLOT_IDLE_CLEAR, group,
							 scheduler->csg_slots_idle_mask[0]);
			}
			spin_unlock_irqrestore(&scheduler->interrupt_lock, flags);

			/* If GPU is in protected mode then any doorbells rang
			 * would have no effect. Check if GPU is in protected
			 * mode and if this group has higher priority than the
			 * active protected mode group. If so prompt the FW
			 * to exit protected mode.
			 */
			if (protm_grp && group->scan_seq_num < protm_grp->scan_seq_num) {
				/* Prompt the FW to exit protected mode */
				scheduler_force_protm_exit(kbdev);
			}
		}
	} else if (!queue_group_scheduled_locked(group)) {
		int new_val;

		insert_group_to_runnable(&kbdev->csf.scheduler, group, KBASE_CSF_GROUP_RUNNABLE);
		/* A new group into the scheduler */
		new_val = atomic_inc_return(&kbdev->csf.scheduler.non_idle_offslot_grps);
		KBASE_KTRACE_ADD_CSF_GRP(kbdev, SCHEDULER_NONIDLE_OFFSLOT_GRP_INC, group,
					 (u64)new_val);
	}

	/* Since a group has become active now, check if GPU needs to be
	 * powered up. Also rekick the Scheduler.
	 */
	scheduler_wakeup(kbdev, true);

	return 0;
}

/**
 * set_max_csg_slots() - Set the number of available CSG slots
 *
 * @kbdev: Pointer of the GPU device.
 *
 * This function would set/limit the number of CSG slots that
 * can be used in the given tick/tock. It would be less than the total CSG
 * slots supported by firmware if the number of GPU address space slots
 * required to utilize all the CSG slots is more than the available
 * address space slots.
 */
static inline void set_max_csg_slots(struct kbase_device *kbdev)
{
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;
	unsigned int total_csg_slots = kbdev->csf.global_iface.group_num;
	unsigned int max_address_space_slots =
		(unsigned int)kbdev->nr_hw_address_spaces - NUM_RESERVED_AS_SLOTS;

	WARN_ON(scheduler->num_active_address_spaces > total_csg_slots);

	if (likely(scheduler->num_active_address_spaces <= max_address_space_slots))
		scheduler->num_csg_slots_for_tick = total_csg_slots;
}

/**
 * count_active_address_space() - Count the number of GPU address space slots
 *
 * @kbdev: Pointer of the GPU device.
 * @kctx: Pointer of the Kbase context.
 *
 * This function would update the counter that is tracking the number of GPU
 * address space slots that would be required to program the CS
 * group slots from the groups at the head of groups_to_schedule list.
 */
static inline void count_active_address_space(struct kbase_device *kbdev,
					      struct kbase_context *kctx)
{
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;
	unsigned int total_csg_slots = kbdev->csf.global_iface.group_num;
	unsigned int max_address_space_slots =
		(unsigned int)kbdev->nr_hw_address_spaces - NUM_RESERVED_AS_SLOTS;

	if (scheduler->ngrp_to_schedule <= total_csg_slots) {
		if (kctx->csf.sched.ngrp_to_schedule == 1)
			scheduler->num_active_address_spaces++;

		if (scheduler->num_active_address_spaces <= max_address_space_slots)
			scheduler->num_csg_slots_for_tick++;
	}
}

/* Two schemes are used in assigning the priority to CSG slots for a given
 * CSG from the 'groups_to_schedule' list.
 * This is needed as an idle on-slot group is deprioritized by moving it to
 * the tail of 'groups_to_schedule' list. As a result it can either get
 * evicted from the CSG slot in current tick/tock dealing, or its position
 * can be after the lower priority non-idle groups in the 'groups_to_schedule'
 * list. The latter case can result in the on-slot subset containing both
 * non-idle and idle CSGs, and is handled through the 2nd scheme described
 * below.
 *
 * First scheme :- If all the slots are going to be occupied by the non-idle or
 * idle groups, then a simple assignment of the priority is done as per the
 * position of a group in the 'groups_to_schedule' list. So maximum priority
 * gets assigned to the slot of a group which is at the head of the list.
 * Here the 'groups_to_schedule' list would effectively be ordered as per the
 * static priority of groups.
 *
 * Second scheme :- If the slots are going to be occupied by a mix of idle and
 * non-idle groups then the priority assignment needs to ensure that the
 * priority of a slot belonging to a higher priority idle group will always be
 * greater than the priority of a slot belonging to a lower priority non-idle
 * group, reflecting the original position of a group in the scan order (i.e
 * static priority) 'sched_act_seq_num', which is set during the prepare phase
 * of a tick/tock before the group is moved to 'idle_groups_to_schedule' list if
 * it is idle.
 * The priority range [MAX_CSG_SLOT_PRIORITY, 0] is partitioned with the first
 * 'slots_for_tick' groups in the original scan order are assigned a priority in
 * the subrange [MAX_CSG_SLOT_PRIORITY, MAX_CSG_SLOT_PRIORITY - slots_for_tick),
 * whereas rest of the groups are assigned the priority in the subrange
 * [MAX_CSG_SLOT_PRIORITY - slots_for_tick, 0]. This way even if an idle higher
 * priority group ends up after the non-idle lower priority groups in the
 * 'groups_to_schedule' list, it will get a higher slot priority. And this will
 * enable the FW to quickly start the execution of higher priority group when it
 * gets de-idled.
 */
static u8 get_slot_priority(struct kbase_queue_group *group)
{
	struct kbase_csf_scheduler *scheduler = &group->kctx->kbdev->csf.scheduler;
	u8 slot_prio;
	u32 slots_for_tick = scheduler->num_csg_slots_for_tick;
	u32 used_slots = slots_for_tick - scheduler->remaining_tick_slots;
	/* Check if all the slots are going to be occupied by the non-idle or
	 * idle groups.
	 */
	if (scheduler->non_idle_scanout_grps >= slots_for_tick ||
	    !scheduler->non_idle_scanout_grps) {
		slot_prio = (u8)(MAX_CSG_SLOT_PRIORITY - used_slots);
	} else {
		/* There will be a mix of idle and non-idle groups. */
		if (group->sched_act_seq_num < slots_for_tick)
			slot_prio = (u8)(MAX_CSG_SLOT_PRIORITY - group->sched_act_seq_num);
		else if (MAX_CSG_SLOT_PRIORITY > (slots_for_tick + used_slots))
			slot_prio = (u8)(MAX_CSG_SLOT_PRIORITY - (slots_for_tick + used_slots));
		else
			slot_prio = 0;
	}
	return slot_prio;
}

/**
 * update_resident_groups_priority() - Update the priority of resident groups
 *
 * @kbdev:    The GPU device.
 *
 * This function will update the priority of all resident queue groups
 * that are at the head of groups_to_schedule list, preceding the first
 * non-resident group.
 *
 * This function will also adjust kbase_csf_scheduler.remaining_tick_slots on
 * the priority update.
 */
static void update_resident_groups_priority(struct kbase_device *kbdev)
{
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;
	u32 num_groups = scheduler->num_csg_slots_for_tick;

	lockdep_assert_held(&kbdev->csf.scheduler.lock);
	while (!list_empty(&scheduler->groups_to_schedule)) {
		struct kbase_queue_group *group = list_first_entry(
			&scheduler->groups_to_schedule, struct kbase_queue_group, link_to_schedule);
		bool resident = kbasep_csf_scheduler_group_is_on_slot_locked(group);

		if ((group->prepared_seq_num >= num_groups) || !resident)
			break;

		update_csg_slot_priority(group, get_slot_priority(group));

		/* Drop the head group from the list */
		remove_scheduled_group(kbdev, group);
		scheduler->remaining_tick_slots--;
	}
}

/**
 * program_group_on_vacant_csg_slot() - Program a non-resident group on the
 *                                      given vacant CSG slot.
 * @kbdev:    Pointer to the GPU device.
 * @slot:     Vacant CSG slot number.
 *
 * This function will program a non-resident group at the head of
 * kbase_csf_scheduler.groups_to_schedule list on the given vacant
 * CSG slot, provided the initial position of the non-resident
 * group in the list is less than the number of CSG slots and there is
 * an available GPU address space slot.
 * kbase_csf_scheduler.remaining_tick_slots would also be adjusted after
 * programming the slot.
 */
static void program_group_on_vacant_csg_slot(struct kbase_device *kbdev, s8 slot)
{
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;
	struct kbase_queue_group *const group =
		list_empty(&scheduler->groups_to_schedule) ?
			      NULL :
			      list_first_entry(&scheduler->groups_to_schedule, struct kbase_queue_group,
					 link_to_schedule);
	u32 num_groups = scheduler->num_csg_slots_for_tick;

	lockdep_assert_held(&kbdev->csf.scheduler.lock);
	if (group && (group->prepared_seq_num < num_groups)) {
		bool ret = kbasep_csf_scheduler_group_is_on_slot_locked(group);

		if (!WARN_ON(ret)) {
			if (kctx_as_enabled(group->kctx) && !group->faulted) {
				program_csg_slot(group, slot, get_slot_priority(group));

				if (likely(csg_slot_in_use(kbdev, slot))) {
					/* Drop the head group from the list */
					remove_scheduled_group(kbdev, group);
					scheduler->remaining_tick_slots--;
				}
			} else {
				update_offslot_non_idle_cnt(group);
				remove_scheduled_group(kbdev, group);
			}
		}
	}
}

/**
 * program_vacant_csg_slot() - Program the vacant CSG slot with a non-resident
 *                             group and update the priority of resident groups.
 *
 * @kbdev:    Pointer to the GPU device.
 * @slot:     Vacant CSG slot number.
 *
 * This function will first update the priority of all resident queue groups
 * that are at the head of groups_to_schedule list, preceding the first
 * non-resident group, it will then try to program the given CS
 * group slot with the non-resident group. Finally update the priority of all
 * resident queue groups following the non-resident group.
 *
 * kbase_csf_scheduler.remaining_tick_slots would also be adjusted.
 */
static void program_vacant_csg_slot(struct kbase_device *kbdev, s8 slot)
{
	struct kbase_csf_scheduler *const scheduler = &kbdev->csf.scheduler;
	struct kbase_csf_csg_slot *const csg_slot = scheduler->csg_slots;

	lockdep_assert_held(&kbdev->csf.scheduler.lock);
	WARN_ON(atomic_read(&csg_slot[slot].state) != CSG_SLOT_READY);

	/* First update priority for already resident groups (if any)
	 * before the non-resident group
	 */
	update_resident_groups_priority(kbdev);

	/* Now consume the vacant slot for the non-resident group */
	program_group_on_vacant_csg_slot(kbdev, slot);

	/* Now update priority for already resident groups (if any)
	 * following the non-resident group
	 */
	update_resident_groups_priority(kbdev);
}

static bool slots_state_changed(struct kbase_device *kbdev, unsigned long *slots_mask,
				bool (*state_check_func)(struct kbase_device *, s8))
{
	u32 num_groups = kbdev->csf.global_iface.group_num;
	DECLARE_BITMAP(changed_slots, MAX_SUPPORTED_CSGS) = { 0 };
	bool changed = false;
	u32 i;

	for_each_set_bit(i, slots_mask, num_groups) {
		if (state_check_func(kbdev, (s8)i)) {
			set_bit(i, changed_slots);
			changed = true;
		}
	}

	if (changed)
		bitmap_copy(slots_mask, changed_slots, MAX_SUPPORTED_CSGS);

	return changed;
}

/**
 * program_suspending_csg_slots() - Program the CSG slots vacated on suspension
 *                                  of queue groups running on them.
 *
 * @kbdev:    Pointer to the GPU device.
 *
 * This function will first wait for the ongoing suspension to complete on a
 * CSG slot and will then program the vacant slot with the
 * non-resident queue group inside the groups_to_schedule list.
 * The programming of the non-resident queue group on the vacant slot could
 * fail due to unavailability of free GPU address space slot and so the
 * programming is re-attempted after the ongoing suspension has completed
 * for all the CSG slots.
 * The priority of resident groups before and after the non-resident group
 * in the groups_to_schedule list would also be updated.
 * This would be repeated for all the slots undergoing suspension.
 * GPU reset would be initiated if the wait for suspend times out.
 */
static void program_suspending_csg_slots(struct kbase_device *kbdev)
{
	u32 num_groups = kbdev->csf.global_iface.group_num;
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;
	DECLARE_BITMAP(slot_mask, MAX_SUPPORTED_CSGS);
	DECLARE_BITMAP(evicted_mask, MAX_SUPPORTED_CSGS) = { 0 };
	bool suspend_wait_failed = false;

	lockdep_assert_held(&kbdev->csf.scheduler.lock);

	/* In the current implementation, csgs_events_enable_mask would be used
	 * only to indicate suspending CSGs.
	 */
	bitmap_complement(slot_mask, scheduler->csgs_events_enable_mask, MAX_SUPPORTED_CSGS);

	while (!bitmap_empty(slot_mask, MAX_SUPPORTED_CSGS)) {
		DECLARE_BITMAP(changed, MAX_SUPPORTED_CSGS);
		long remaining = kbase_csf_timeout_in_jiffies(kbdev->csf.csg_suspend_timeout_ms);

		bitmap_copy(changed, slot_mask, MAX_SUPPORTED_CSGS);

		remaining = kbase_csf_fw_io_wait_event_timeout(
			&kbdev->csf.fw_io, kbdev->csf.event_wait,
			slots_state_changed(kbdev, changed, csg_slot_stopped_raw), remaining);

		if (likely(remaining > 0)) {
			u32 i;

			for_each_set_bit(i, changed, num_groups) {
				struct kbase_queue_group *group =
					scheduler->csg_slots[i].resident_group;

				if (WARN_ON(!csg_slot_stopped_locked(kbdev, (s8)i)))
					continue;

				/* The on slot csg is now stopped */
				clear_bit(i, slot_mask);

				if (likely(group)) {
					bool as_fault;
					/* Only do save/cleanup if the
					 * group is not terminated during
					 * the sleep.
					 */

					/* Only emit suspend, if there was no AS fault */
					if (kctx_as_enabled(group->kctx) && !group->faulted)
						KBASE_TLSTREAM_TL_KBASE_DEVICE_SUSPEND_CSG(
							kbdev, kbdev->id, i);
					save_csg_slot(group);
					as_fault = cleanup_csg_slot(group);
					/* If AS fault detected, evict it */
					if (as_fault) {
						sched_evict_group(group, true, true);
						set_bit(i, evicted_mask);
					}
				}

				program_vacant_csg_slot(kbdev, (s8)i);
			}
		} else if (!remaining) {
			u32 i;

			/* Groups that have failed to suspend in time shall
			 * raise a fatal error as they could no longer be
			 * safely resumed.
			 */
			for_each_set_bit(i, slot_mask, num_groups) {
				struct kbase_queue_group *const group =
					scheduler->csg_slots[i].resident_group;
				enum dumpfault_error_type error_type = DF_CSG_SUSPEND_TIMEOUT;

				struct base_gpu_queue_group_error const
					err_payload = { .error_type =
								BASE_GPU_QUEUE_GROUP_ERROR_FATAL,
							.payload = {
								.fatal_group = {
									.status =
										GPU_EXCEPTION_TYPE_SW_FAULT_2,
								} } };

				if (unlikely(group == NULL))
					continue;

				/* TODO GPUCORE-25328: The CSG can't be
				 * terminated, the GPU will be reset as a
				 * work-around.
				 */
				pixel_gpu_uevent_kmd_error_send(kbdev, GPU_UEVENT_INFO_CSG_GROUP_SUSPEND);
				dev_warn(
					kbdev->dev,
					"[%llu] Group %d of context %d_%d on slot %u failed to suspend (timeout %d ms)",
					kbase_backend_get_cycle_cnt(kbdev), group->handle,
					group->kctx->tgid, group->kctx->id, i,
					kbase_get_timeout_ms(kbdev, CSF_FIRMWARE_TIMEOUT));
				if (kbase_csf_firmware_ping_wait(kbdev,
								 FW_PING_AFTER_ERROR_TIMEOUT_MS))
					error_type = DF_PING_REQUEST_TIMEOUT;
				schedule_actions_trigger_df(kbdev, group->kctx, error_type);

				kbase_csf_add_group_fatal_error(group, &err_payload);
				kbase_event_wakeup_nosync(group->kctx);

				/* The group has failed suspension, stop
				 * further examination.
				 */
				clear_bit(i, slot_mask);
				set_bit(i, scheduler->csgs_events_enable_mask);
			}

			suspend_wait_failed = true;
		}
	}

	if (!bitmap_empty(evicted_mask, MAX_SUPPORTED_CSGS))
		dev_info(kbdev->dev, "Scheduler evicting slots: 0x%*pb\n", num_groups,
			 evicted_mask);

	if (likely(!suspend_wait_failed)) {
		u32 i;

		while (scheduler->ngrp_to_schedule && scheduler->remaining_tick_slots) {
			i = find_first_zero_bit(scheduler->csg_inuse_bitmap, num_groups);
			if (WARN_ON(i == num_groups))
				break;
			program_vacant_csg_slot(kbdev, (s8)i);
			if (!csg_slot_in_use(kbdev, (int)i)) {
				dev_warn(kbdev->dev,
					 "Couldn't use CSG slot %d despite being vacant", i);
				break;
			}
		}
	} else {
		if (kbase_prepare_to_reset_gpu(kbdev, RESET_FLAGS_NONE))
			kbase_reset_gpu(kbdev);
	}
}

static void suspend_queue_group(struct kbase_queue_group *group)
{
	unsigned long flags;
	struct kbase_csf_scheduler *const scheduler = &group->kctx->kbdev->csf.scheduler;

	spin_lock_irqsave(&scheduler->interrupt_lock, flags);
	/* This shall be used in program_suspending_csg_slots() where we
	 * assume that whilst CSGs are being suspended, this bitmask is not
	 * used by anything else i.e., it indicates only the CSGs going
	 * through suspension.
	 */
	clear_bit(group->csg_nr, scheduler->csgs_events_enable_mask);
	spin_unlock_irqrestore(&scheduler->interrupt_lock, flags);

	/* If AS fault detected, terminate the group */
	if (!kctx_as_enabled(group->kctx) || group->faulted)
		term_csg_slot(group);
	else
		suspend_csg_slot(group);
}

static void wait_csg_slots_start(struct kbase_device *kbdev)
{
	u32 num_groups = kbdev->csf.global_iface.group_num;
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;
	long remaining =
		kbase_csf_timeout_in_jiffies(kbase_get_timeout_ms(kbdev, CSF_FIRMWARE_TIMEOUT));
	DECLARE_BITMAP(slot_mask, MAX_SUPPORTED_CSGS) = { 0 };
	u32 i;

	lockdep_assert_held(&kbdev->csf.scheduler.lock);

	/* extract start slot flags for check */
	for (i = 0; i < num_groups; i++) {
		if (atomic_read(&scheduler->csg_slots[i].state) == CSG_SLOT_READY2RUN)
			set_bit(i, slot_mask);
	}

	while (!bitmap_empty(slot_mask, MAX_SUPPORTED_CSGS)) {
		DECLARE_BITMAP(changed, MAX_SUPPORTED_CSGS);

		bitmap_copy(changed, slot_mask, MAX_SUPPORTED_CSGS);

		remaining = kbase_csf_fw_io_wait_event_timeout(
			&kbdev->csf.fw_io, kbdev->csf.event_wait,
			slots_state_changed(kbdev, changed, csg_slot_running), remaining);

		if (likely(remaining > 0)) {
			for_each_set_bit(i, changed, num_groups) {
				struct kbase_queue_group *group =
					scheduler->csg_slots[i].resident_group;

				/* The on slot csg is now running */
				clear_bit(i, slot_mask);
				group->run_state = KBASE_CSF_GROUP_RUNNABLE;
				KBASE_KTRACE_ADD_CSF_GRP(kbdev, CSF_GROUP_RUNNABLE, group,
							 group->run_state);
			}
		} else if (!remaining) {
			const int csg_nr = ffs(slot_mask[0]) - 1;
			struct kbase_queue_group *group =
				scheduler->csg_slots[csg_nr].resident_group;
			enum dumpfault_error_type error_type = DF_CSG_START_TIMEOUT;

			pixel_gpu_uevent_kmd_error_send(kbdev, GPU_UEVENT_INFO_CSG_SLOTS_START);
			dev_err(kbdev->dev,
				"[%llu] Timeout (%d ms) waiting for CSG slots to start, slots: 0x%*pb\n",
				kbase_backend_get_cycle_cnt(kbdev),
				kbase_get_timeout_ms(kbdev, CSF_FIRMWARE_TIMEOUT), num_groups,
				slot_mask);

			if (kbase_csf_firmware_ping_wait(kbdev, FW_PING_AFTER_ERROR_TIMEOUT_MS))
				error_type = DF_PING_REQUEST_TIMEOUT;
			schedule_actions_trigger_df(kbdev, group->kctx, error_type);

			if (kbase_prepare_to_reset_gpu(kbdev, RESET_FLAGS_NONE))
				kbase_reset_gpu(kbdev);
			break;
		}
	}
}

/**
 * group_on_slot_is_idle() - Check if the given slot has a CSG-idle state
 *                           flagged after the completion of a CSG status
 *                           update command
 *
 * @kbdev:  Pointer to the GPU device.
 * @slot:   The given slot for checking an occupying resident group's idle
 *          state.
 *
 * This function is called at the start of scheduling tick to check the
 * idle status of a queue group resident on a CSG slot.
 * The caller must make sure the corresponding status update command has
 * been called and completed before checking this status.
 *
 * Return: true if the group resident on slot is idle, otherwise false.
 */
static bool group_on_slot_is_idle(struct kbase_device *kbdev, unsigned long slot)
{
	bool idle;

	lockdep_assert_held(&kbdev->csf.scheduler.lock);

	idle = kbase_csf_fw_io_group_read(&kbdev->csf.fw_io, slot, CSG_STATUS_STATE) &
	       CSG_STATUS_STATE_IDLE_MASK;

	return idle;
}

/**
 * slots_update_state_changed() -  Check the handshake state of a subset of
 *                                 command group slots.
 *
 * @kbdev:          The GPU device.
 * @field_mask:     The field mask for checking the state in the csg_req/ack.
 * @slots_mask:     A bit_map specifying the slots to check.
 * @slots_done:     A cleared bit_map for returning the slots that
 *                  have finished update.
 *
 * Checks the state of a subset of slots selected through the slots_mask
 * bit_map. Records which slots' handshake completed and send it back in the
 * slots_done bit_map.
 *
 * Return: true if the slots_done is set for at least one slot.
 *         Otherwise false.
 */
static bool slots_update_state_changed(struct kbase_device *kbdev, u32 field_mask,
				       const unsigned long *slots_mask, unsigned long *slots_done)
{
	u32 num_groups = kbdev->csf.global_iface.group_num;
	bool changed = false;
	u32 i;

	lockdep_assert_held(&kbdev->csf.scheduler.lock);

	for_each_set_bit(i, slots_mask, num_groups) {
		u32 state = kbase_csf_fw_io_group_input_read(&kbdev->csf.fw_io, i, CSG_REQ);

		state ^= kbase_csf_fw_io_group_read(&kbdev->csf.fw_io, i, CSG_ACK);

		if (!(state & field_mask)) {
			set_bit(i, slots_done);
			changed = true;
		}
	}

	return changed;
}

/**
 * wait_csg_slots_handshake_ack - Wait the req/ack handshakes to complete on
 *                                the specified groups.
 *
 * @kbdev:           Pointer to the GPU device.
 * @field_mask:      The field mask for checking the state in the csg_req/ack.
 * @slot_mask:       Bitmap reflecting the slots, the function will modify
 *                   the acknowledged slots by clearing their corresponding
 *                   bits.
 * @wait_in_jiffies: Wait duration in jiffies, controlling the time-out.
 *
 * This function waits for the acknowledgment of the request that have
 * already been placed for the CSG slots by the caller. Currently used for
 * the CSG priority update and status update requests.
 *
 * Return: 0 on all specified slots acknowledged; otherwise -ETIMEDOUT. For
 *         timed out condition with unacknowledged slots, their bits remain
 *         set in the slot_mask.
 */
static int wait_csg_slots_handshake_ack(struct kbase_device *kbdev, u32 field_mask,
					unsigned long *slot_mask, long wait_in_jiffies)
{
	const u32 num_groups = kbdev->csf.global_iface.group_num;
	long remaining = wait_in_jiffies;

	lockdep_assert_held(&kbdev->csf.scheduler.lock);

	while (!bitmap_empty(slot_mask, num_groups) && !kbase_reset_gpu_is_active(kbdev)) {
		DECLARE_BITMAP(dones, MAX_SUPPORTED_CSGS) = { 0 };

		remaining = kbase_csf_fw_io_wait_event_timeout(
			&kbdev->csf.fw_io, kbdev->csf.event_wait,
			slots_update_state_changed(kbdev, field_mask, slot_mask, dones), remaining);

		if (likely(remaining > 0))
			bitmap_andnot(slot_mask, slot_mask, dones, num_groups);
		else if (!remaining) {

			/* Timed-out on the wait */
			return -ETIMEDOUT;
		}
	}

	return 0;
}

static void wait_csg_slots_finish_prio_update(struct kbase_device *kbdev)
{
	unsigned long *slot_mask = kbdev->csf.scheduler.csg_slots_prio_update;
	const unsigned int fw_timeout_ms = kbase_get_timeout_ms(kbdev, CSF_FIRMWARE_TIMEOUT);
	long wait_time = kbase_csf_timeout_in_jiffies(fw_timeout_ms);
	int ret = wait_csg_slots_handshake_ack(kbdev, CSG_REQ_EP_CFG_MASK, slot_mask, wait_time);

	lockdep_assert_held(&kbdev->csf.scheduler.lock);

	if (unlikely(ret != 0)) {
		const int csg_nr = ffs(slot_mask[0]) - 1;
		struct kbase_queue_group *group =
			kbdev->csf.scheduler.csg_slots[csg_nr].resident_group;
		enum dumpfault_error_type error_type = DF_CSG_EP_CFG_TIMEOUT;
		/* The update timeout is not regarded as a serious
		 * issue, no major consequences are expected as a
		 * result, so just warn the case.
		 */
		pixel_gpu_uevent_kmd_error_send(kbdev, GPU_UEVENT_INFO_CSG_EP_CFG);
		dev_warn(
			kbdev->dev,
			"[%llu] Timeout (%d ms) on CSG_REQ:EP_CFG, skipping the update wait: slot mask=0x%lx",
			kbase_backend_get_cycle_cnt(kbdev), fw_timeout_ms, slot_mask[0]);
		if (kbase_csf_firmware_ping_wait(kbdev, FW_PING_AFTER_ERROR_TIMEOUT_MS))
			error_type = DF_PING_REQUEST_TIMEOUT;
		schedule_actions_trigger_df(kbdev, group->kctx, error_type);

		/* Timeout could indicate firmware is unresponsive so trigger a GPU reset. */
		if (kbase_prepare_to_reset_gpu(kbdev, RESET_FLAGS_HWC_UNRECOVERABLE_ERROR))
			kbase_reset_gpu(kbdev);
	}
}

static void report_csg_termination(struct kbase_queue_group *const group)
{
	struct base_gpu_queue_group_error
		err = { .error_type = BASE_GPU_QUEUE_GROUP_ERROR_FATAL,
			.payload = { .fatal_group = {
					     .status = GPU_EXCEPTION_TYPE_SW_FAULT_2,
				     } } };

	kbase_csf_add_group_fatal_error(group, &err);
}

void kbase_csf_scheduler_evict_ctx_slots(struct kbase_device *kbdev, struct kbase_context *kctx,
					 struct list_head *evicted_groups)
{
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;
	struct kbase_queue_group *group;
	u32 num_groups = kbdev->csf.global_iface.group_num;
	u32 slot;
	DECLARE_BITMAP(slot_mask, MAX_SUPPORTED_CSGS) = { 0 };

	lockdep_assert_held(&kctx->csf.lock);
	rt_mutex_lock(&scheduler->lock);

	/* This code is only called during reset, so we don't wait for the CSG
	 * slots to be stopped
	 */
	WARN_ON(!kbase_reset_gpu_is_active(kbdev));

	KBASE_KTRACE_ADD(kbdev, SCHEDULER_EVICT_CTX_SLOTS_START, kctx, 0u);
	for (slot = 0; slot < num_groups; slot++) {
		group = kbdev->csf.scheduler.csg_slots[slot].resident_group;
		if (group && group->kctx == kctx) {
			bool as_fault;

			dev_dbg(kbdev->dev, "Evicting group [%d] running on slot [%d] due to reset",
				group->handle, group->csg_nr);

			term_csg_slot(group);
			as_fault = cleanup_csg_slot(group);
			/* remove the group from the scheduler list */
			sched_evict_group(group, as_fault, false);
			/* signal Userspace that CSG is being terminated */
			report_csg_termination(group);
			/* return the evicted group to the caller */
			list_add_tail(&group->link, evicted_groups);
			set_bit(slot, slot_mask);
		}
	}

	dev_info(kbdev->dev, "Evicting context %d_%d slots: 0x%*pb\n", kctx->tgid, kctx->id,
		 num_groups, slot_mask);

	if (kctx->protm_enter_ts) {
		dev_info(kbdev->dev, "Context %d_%d protm_enter_ts=%lluns protm_exit_ts=%lluns\n",
			kctx->tgid, kctx->id,
			ktime_to_ns(kctx->protm_enter_ts),
			ktime_to_ns(kctx->protm_exit_ts));
	}

	/* Fatal errors may have been the cause of the GPU reset
	 * taking place, in which case we want to make sure that
	 * we wake up the fatal event queue to notify userspace
	 * only once. Otherwise, we may have duplicate event
	 * notifications between the time the first notification
	 * occurs and the time the GPU is reset.
	 */
	kbase_event_wakeup_nosync(kctx);

	rt_mutex_unlock(&scheduler->lock);
	KBASE_KTRACE_ADD(kbdev, SCHEDULER_EVICT_CTX_SLOTS_END, kctx, num_groups);
}

/**
 * scheduler_slot_protm_ack - Acknowledging the protected region requests
 * from the resident group on a given slot.
 *
 * @kbdev:  Pointer to the GPU device.
 * @group:  Pointer to the resident group on the given slot.
 * @slot:   The slot that the given group is actively operating on.
 *
 * The function assumes that the given slot is in stable running state and
 * has already been judged by the caller on that any pending protected region
 * requests of the resident group should be acknowledged.
 *
 * Return: true if the group has pending protm request(s) and is acknowledged.
 *         The caller should arrange to enter the protected mode for servicing
 *         it. Otherwise return false, indicating the group has no pending protm
 *         request.
 */
static bool scheduler_slot_protm_ack(struct kbase_device *const kbdev,
				     struct kbase_queue_group *const group, const int slot)
{
	struct kbase_csf_scheduler *const scheduler = &kbdev->csf.scheduler;
	bool protm_ack = false;
	struct kbase_csf_cmd_stream_group_info *ginfo = &kbdev->csf.global_iface.groups[slot];
	u32 max_csi;
	u32 i;
	unsigned long fw_io_flags;

	if (WARN_ON(scheduler->csg_slots[slot].resident_group != group))
		return protm_ack;

	lockdep_assert_held(&scheduler->lock);
	lockdep_assert_held(&group->kctx->kbdev->csf.scheduler.interrupt_lock);

	if (kbase_csf_fw_io_open(&kbdev->csf.fw_io, &fw_io_flags))
		return protm_ack;

	max_csi = ginfo->stream_num;
	for (i = find_first_bit(group->protm_pending_bitmap, max_csi); i < max_csi;
	     i = find_next_bit(group->protm_pending_bitmap, max_csi, i + 1)) {
		struct kbase_queue *queue = group->bound_queues[i];

		clear_bit(i, group->protm_pending_bitmap);
		KBASE_KTRACE_ADD_CSF_GRP_Q(kbdev, CSI_PROTM_PEND_CLEAR, group, queue,
					   group->protm_pending_bitmap[0]);

		if (!WARN_ON(!queue) && queue->enabled) {
			u32 cs_protm_ack =
				kbase_csf_fw_io_stream_read(&kbdev->csf.fw_io, slot, i, CS_ACK) &
				CS_ACK_PROTM_PEND_MASK;
			u32 cs_protm_req = kbase_csf_fw_io_stream_input_read(&kbdev->csf.fw_io,
									     slot, i, CS_REQ) &
					   CS_REQ_PROTM_PEND_MASK;

			KBASE_KTRACE_ADD_CSF_GRP_Q(kbdev, CSI_PROTM_ACK, group, queue,
						   cs_protm_ack ^ cs_protm_req);

			if (cs_protm_ack == cs_protm_req) {
				dev_dbg(kbdev->dev,
					"PROTM-ack already done for queue-%d group-%d slot-%d",
					queue->csi_index, group->handle, slot);
				continue;
			}

			kbase_csf_fw_io_stream_write_mask(&kbdev->csf.fw_io, slot, i, CS_REQ,
							  cs_protm_ack, CS_ACK_PROTM_PEND_MASK);
			protm_ack = true;
			dev_dbg(kbdev->dev, "PROTM-ack for queue-%d, group-%d slot-%d",
				queue->csi_index, group->handle, slot);
		}
	}

	kbase_csf_fw_io_close(&kbdev->csf.fw_io, fw_io_flags);

	return protm_ack;
}

/**
 * protm_enter_set_next_pending_seq - Update the scheduler's field of
 * tick_protm_pending_seq to that from the next available on-slot protm
 * pending CSG.
 *
 * @kbdev:     Pointer to the GPU device.
 *
 * If applicable, the function updates the scheduler's tick_protm_pending_seq
 * field from the next available on-slot protm pending CSG. If not, the field
 * is set to KBASEP_TICK_PROTM_PEND_SCAN_SEQ_NR_INVALID.
 */
static void protm_enter_set_next_pending_seq(struct kbase_device *const kbdev)
{
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;
	u32 num_groups = kbdev->csf.global_iface.group_num;
	u32 num_csis = kbdev->csf.global_iface.groups[0].stream_num;
	u32 i;

	kbase_csf_scheduler_spin_lock_assert_held(kbdev);

	/* Reset the tick's pending protm seq number to invalid initially */
	scheduler->tick_protm_pending_seq = KBASEP_TICK_PROTM_PEND_SCAN_SEQ_NR_INVALID;
	for_each_set_bit(i, scheduler->csg_inuse_bitmap, num_groups) {
		struct kbase_queue_group *group = scheduler->csg_slots[i].resident_group;

		/* Set to the next pending protm group's scan_seq_number */
		if ((group != scheduler->active_protm_grp) &&
		    (!bitmap_empty(group->protm_pending_bitmap, num_csis)) &&
		    (group->scan_seq_num < scheduler->tick_protm_pending_seq))
			scheduler->tick_protm_pending_seq = group->scan_seq_num;
	}
}

/**
 * scheduler_group_check_protm_enter - Request the given group to be evaluated
 * for triggering the protected mode.
 *
 * @kbdev:     Pointer to the GPU device.
 * @input_grp: Pointer to the GPU queue group.
 *
 * The function assumes the given group is either an active running group or
 * the scheduler internally maintained field scheduler->top_grp.
 *
 * If the GPU is not already running in protected mode and the input group
 * has protected region requests from its bound queues, the requests are
 * acknowledged and the GPU is instructed to enter the protected mode.
 */
static void scheduler_group_check_protm_enter(struct kbase_device *const kbdev,
					      struct kbase_queue_group *const input_grp)
{
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;
	struct kbase_protected_suspend_buffer *sbuf = &input_grp->protected_suspend_buf;
	unsigned long flags;
	bool protm_in_use;

	lockdep_assert_held(&scheduler->lock);

	/* Return early if the physical pages have not been allocated yet */
	if (unlikely(!sbuf->pma))
		return;

	/* This lock is taken to prevent the issuing of MMU command during the
	 * transition to protected mode. This helps avoid the scenario where the
	 * entry to protected mode happens with a memory region being locked and
	 * the same region is then accessed by the GPU in protected mode.
	 */
	down_write(&kbdev->csf.mmu_sync_sem);
	spin_lock_irqsave(&scheduler->interrupt_lock, flags);

	/* Check if the previous transition to enter & exit the protected
	 * mode has completed or not.
	 */
	protm_in_use = kbase_csf_scheduler_protected_mode_in_use(kbdev) || kbdev->protected_mode;
	KBASE_KTRACE_ADD_CSF_GRP(kbdev, SCHEDULER_PROTM_ENTER_CHECK, input_grp, protm_in_use);

	/* Firmware samples the PROTM_PEND ACK bit for CSs when
	 * Host sends PROTM_ENTER global request. So if PROTM_PEND ACK bit
	 * is set for a CS after Host has sent the PROTM_ENTER
	 * Global request, then there is no guarantee that firmware will
	 * notice that prior to switching to protected mode. And firmware
	 * may not again raise the PROTM_PEND interrupt for that CS
	 * later on. To avoid that uncertainty PROTM_PEND ACK bit
	 * is not set for a CS if the request to enter protected
	 * mode has already been sent. It will be set later (after the exit
	 * from protected mode has taken place) when the group to which
	 * CS is bound becomes the top group.
	 *
	 * The actual decision of entering protected mode is hinging on the
	 * input group is the top priority group, or, in case the previous
	 * top-group is evicted from the scheduler during the tick, its would
	 * be replacement, and that it is currently in a stable state (i.e. the
	 * slot state is running).
	 */
	if (!protm_in_use && !WARN_ON(!input_grp)) {
		const int slot = kbase_csf_scheduler_group_get_slot_locked(input_grp);

		/* check the input_grp is running and requesting protected mode
		 */
		if (slot >= 0 &&
		    atomic_read(&scheduler->csg_slots[slot].state) == CSG_SLOT_RUNNING) {
			if (kctx_as_enabled(input_grp->kctx) &&
			    scheduler_slot_protm_ack(kbdev, input_grp, slot)) {
				int err;

				/* Option of acknowledging to multiple
				 * CSGs from the same kctx is dropped,
				 * after consulting with the
				 * architecture team. See the comment in
				 * GPUCORE-21394.
				 */

				/* Switch to protected mode */
				scheduler->active_protm_grp = input_grp;
				KBASE_KTRACE_ADD_CSF_GRP(kbdev, SCHEDULER_PROTM_ENTER, input_grp,
							 0u);

#if IS_ENABLED(CONFIG_MALI_CORESIGHT)
				spin_unlock_irqrestore(&scheduler->interrupt_lock, flags);

				/* Coresight must be disabled before entering protected mode. */
				kbase_debug_coresight_csf_disable_pmode_enter(kbdev);

				spin_lock_irqsave(&scheduler->interrupt_lock, flags);
#endif /* IS_ENABLED(CONFIG_MALI_CORESIGHT) */

				if (kbase_csf_enter_protected_mode(kbdev)) {
					dev_err(kbdev->dev, "Failed to enter protected mode");
					goto unlock;
				}
				/* Set the pending protm seq number to the next one */
				protm_enter_set_next_pending_seq(kbdev);

				spin_unlock_irqrestore(&scheduler->interrupt_lock, flags);

				err = kbase_csf_wait_protected_mode_enter(kbdev);
				up_write(&kbdev->csf.mmu_sync_sem);

				if (err) {
					pixel_gpu_uevent_kmd_error_send(kbdev, GPU_UEVENT_INFO_PMODE_ENTRY_FAILURE);
					schedule_actions_trigger_df(
						kbdev, input_grp->kctx,
						DF_PROTECTED_MODE_ENTRY_FAILURE);
				}

				scheduler->protm_enter_time = ktime_get_raw();
				input_grp->kctx->protm_enter_ts = scheduler->protm_enter_time;

				return;
			}
		}
	}

unlock:
	spin_unlock_irqrestore(&scheduler->interrupt_lock, flags);
	up_write(&kbdev->csf.mmu_sync_sem);
}

/**
 * scheduler_check_pmode_progress - Check if protected mode execution is progressing
 *
 * @kbdev:     Pointer to the GPU device.
 *
 * This function is called when the GPU is in protected mode.
 *
 * It will check if the time spent in protected mode is less
 * than CSF_SCHED_PROTM_PROGRESS_TIMEOUT. If not, a PROTM_EXIT
 * request is sent to the FW.
 */
static void scheduler_check_pmode_progress(struct kbase_device *kbdev)
{
	u64 protm_spent_time_ms;
	u64 protm_progress_timeout = kbase_get_timeout_ms(kbdev, CSF_SCHED_PROTM_PROGRESS_TIMEOUT);
	s64 diff_ms_signed = ktime_ms_delta(ktime_get_raw(), kbdev->csf.scheduler.protm_enter_time);

	if (diff_ms_signed < 0)
		return;

	lockdep_assert_held(&kbdev->csf.scheduler.lock);

	protm_spent_time_ms = (u64)diff_ms_signed;
	if (protm_spent_time_ms < protm_progress_timeout)
		return;

	dev_dbg(kbdev->dev, "Protected mode progress timeout: %llu >= %llu", protm_spent_time_ms,
		protm_progress_timeout);

	/* Prompt the FW to exit protected mode */
	scheduler_force_protm_exit(kbdev);
}

static void scheduler_apply(struct kbase_device *kbdev)
{
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;
	const u32 total_csg_slots = kbdev->csf.global_iface.group_num;
	const u32 available_csg_slots = scheduler->num_csg_slots_for_tick;
	u32 resident_cnt = 0;
	struct kbase_queue_group *group;
	u32 i;
	u32 spare;

	lockdep_assert_held(&scheduler->lock);

	/* Suspend those resident groups not in the run list */
	for (i = 0; i < total_csg_slots; i++) {
		group = scheduler->csg_slots[i].resident_group;
		if (group) {
			resident_cnt++;
			if (group->prepared_seq_num >= available_csg_slots)
				suspend_queue_group(group);
		}
	}

	/* Initialize the remaining available csg slots for the tick/tock */
	scheduler->remaining_tick_slots = available_csg_slots;

	/* If there are spare slots, apply heads in the list */
	spare = (available_csg_slots > resident_cnt) ? (available_csg_slots - resident_cnt) : 0;
	while (!list_empty(&scheduler->groups_to_schedule)) {
		group = list_first_entry(&scheduler->groups_to_schedule, struct kbase_queue_group,
					 link_to_schedule);

		if (kbasep_csf_scheduler_group_is_on_slot_locked(group) &&
		    group->prepared_seq_num < available_csg_slots) {
			/* One of the resident remainders */
			update_csg_slot_priority(group, get_slot_priority(group));
		} else if (spare != 0) {
			s8 slot = (s8)find_first_zero_bit(kbdev->csf.scheduler.csg_inuse_bitmap,
							  total_csg_slots);

			if (WARN_ON(slot >= (s8)total_csg_slots))
				break;

			if (!kctx_as_enabled(group->kctx) || group->faulted) {
				/* Drop the head group and continue */
				update_offslot_non_idle_cnt(group);
				remove_scheduled_group(kbdev, group);
				continue;
			}
			program_csg_slot(group, slot, get_slot_priority(group));
			if (unlikely(!csg_slot_in_use(kbdev, slot)))
				break;

			spare--;
		} else
			break;

		/* Drop the head csg from the list */
		remove_scheduled_group(kbdev, group);
		if (!WARN_ON(!scheduler->remaining_tick_slots))
			scheduler->remaining_tick_slots--;
	}

	/* Dealing with groups currently going through suspend */
	program_suspending_csg_slots(kbdev);
}

/**
 * scheduler_scan_add_group_to_sched_list - Add a schedulable group to one of the scheduler's
 *                                          schedulable lists following exactly the scan sequence.
 *
 * @kbdev:      Pointer to the GPU device.
 * @sched_list: the scheduling list for appending the given group
 * @group:      the group that is to be added forllowing the scan out procedure.
 *
 * This function is called when a group is required to be added to a scheduler maintained list
 * for a tick/tock scheduling operation.
 */
static void scheduler_scan_add_group_to_sched_list(struct kbase_device *kbdev,
						   struct list_head *sched_list,
						   struct kbase_queue_group *group)
{
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;

	lockdep_assert_held(&scheduler->lock);
	lockdep_assert_held(&scheduler->interrupt_lock);

	list_add_tail(&group->link_to_schedule, sched_list);
	/* Set the act_seq_num when a group is added to a schedulable list.
	 * Otherwise it's a don't care case, as the group would not be involved
	 * with onslot running.
	 */
	group->sched_act_seq_num = scheduler->csg_scan_sched_count++;
}

static void scheduler_ctx_scan_groups(struct kbase_device *kbdev, struct kbase_context *kctx,
				      int priority, struct list_head *privileged_groups,
				      struct list_head *active_groups)
{
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;
	struct kbase_queue_group *group;

	lockdep_assert_held(&scheduler->lock);
	lockdep_assert_held(&scheduler->interrupt_lock);
	if (WARN_ON(priority < 0) || WARN_ON(priority >= KBASE_QUEUE_GROUP_PRIORITY_COUNT))
		return;

	if (!kctx_as_enabled(kctx))
		return;

	list_for_each_entry(group, &kctx->csf.sched.runnable_groups[priority], link) {
		bool protm_req;

		if (WARN_ON(!list_empty(&group->link_to_schedule)))
			/* This would be a bug */
			list_del_init(&group->link_to_schedule);

		if (unlikely(group->faulted))
			continue;

		/* Set the scanout sequence number, starting from 0 */
		group->scan_seq_num = scheduler->csg_scan_count_for_tick++;

		protm_req = !bitmap_empty(group->protm_pending_bitmap,
					  kbdev->csf.global_iface.groups[0].stream_num);

		if (scheduler->tick_protm_pending_seq ==
		    KBASEP_TICK_PROTM_PEND_SCAN_SEQ_NR_INVALID) {
			if (protm_req)
				scheduler->tick_protm_pending_seq = group->scan_seq_num;
		}

		if (protm_req && on_slot_group_idle_locked(group))
			update_idle_protm_group_state_to_runnable(group);
		else if (queue_group_idle_locked(group)) {
			if (can_schedule_idle_group(group))
				scheduler_scan_add_group_to_sched_list(
					kbdev, &scheduler->idle_groups_to_schedule, group);
			continue;
		}

		if (protm_req && (group->priority == KBASE_QUEUE_GROUP_PRIORITY_REALTIME)) {
			scheduler_scan_add_group_to_sched_list(kbdev, privileged_groups, group);
			continue;
		}

		scheduler_scan_add_group_to_sched_list(kbdev, active_groups, group);
	}
}

/**
 * scheduler_rotate_groups() - Rotate the runnable queue groups to provide
 *                             fairness of scheduling within a single
 *                             kbase_context.
 *
 * @kbdev:    Pointer to the GPU device.
 *
 * Since only kbase_csf_scheduler's top_grp (i.e. the queue group assigned
 * the highest slot priority) is guaranteed to get the resources that it
 * needs we only rotate the kbase_context corresponding to it -
 * kbase_csf_scheduler's top_kctx.
 *
 * The priority level chosen for rotation is the one containing the previous
 * scheduling cycle's kbase_csf_scheduler's top_grp.
 *
 * In a 'fresh-slice-cycle' this always corresponds to the highest group
 * priority in use by kbase_csf_scheduler's top_kctx. That is, it's the priority
 * level of the previous scheduling cycle's first runnable kbase_context.
 *
 * We choose this priority level because when higher priority work is
 * scheduled, we should always cause the scheduler to run and do a scan. The
 * scan always enumerates the highest priority work first (whether that be
 * based on process priority or group priority), and thus
 * kbase_csf_scheduler's top_grp will point to the first of those high priority
 * groups, which necessarily must be the highest priority group in
 * kbase_csf_scheduler's top_kctx. The fresh-slice-cycle will run later and pick
 * up that group appropriately.
 *
 * If kbase_csf_scheduler's top_grp was instead evicted (and thus is NULL),
 * then no explicit rotation occurs on the next fresh-slice-cycle schedule, but
 * will set up kbase_csf_scheduler's top_kctx again for the next scheduling
 * cycle. Implicitly, a rotation had already occurred by removing
 * the kbase_csf_scheduler's top_grp
 *
 * If kbase_csf_scheduler's top_grp became idle and all other groups belonging
 * to kbase_csf_scheduler's top_grp's priority level in kbase_csf_scheduler's
 * top_kctx are also idle, then the effect of this will be to rotate idle
 * groups, which might not actually become resident in the next
 * scheduling slice. However this is acceptable since a queue group becoming
 * idle is implicitly a rotation (as above with evicted queue groups), as it
 * automatically allows a new queue group to take the maximum slot priority
 * whilst the idle kbase_csf_scheduler's top_grp ends up near the back of
 * the kbase_csf_scheduler's groups_to_schedule list. In this example, it will
 * be for a group in the next lowest priority level or in absence of those the
 * next kbase_context's queue groups.
 */
static void scheduler_rotate_groups(struct kbase_device *kbdev)
{
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;
	struct kbase_context *const top_kctx = scheduler->top_kctx;
	struct kbase_queue_group *const top_grp = scheduler->top_grp;

	lockdep_assert_held(&scheduler->lock);
	if (top_kctx && top_grp) {
		struct list_head *list = &top_kctx->csf.sched.runnable_groups[top_grp->priority];

		WARN_ON(top_grp->kctx != top_kctx);
		if (!WARN_ON(list_empty(list))) {
			struct kbase_queue_group *new_head_grp;

			list_move_tail(&top_grp->link, list);
			new_head_grp =
				(!list_empty(list)) ?
					      list_first_entry(list, struct kbase_queue_group, link) :
					      NULL;
			KBASE_KTRACE_ADD_CSF_GRP(kbdev, GROUP_RUNNABLE_ROTATE, top_grp,
						 top_kctx->csf.sched.num_runnable_grps);
			KBASE_KTRACE_ADD_CSF_GRP(kbdev, GROUP_RUNNABLE_HEAD, new_head_grp, 0u);
			dev_dbg(kbdev->dev,
				"groups rotated for a context, num_runnable_groups: %u\n",
				scheduler->top_kctx->csf.sched.num_runnable_grps);
		}
	}
}

static void scheduler_rotate_ctxs(struct kbase_device *kbdev)
{
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;
	struct list_head *list = &scheduler->runnable_kctxs;

	lockdep_assert_held(&scheduler->lock);
	if (scheduler->top_kctx) {
		if (!WARN_ON(list_empty(list))) {
			struct kbase_context *pos_kctx;
			bool found = false;

			/* Locate the ctx on the list */
			list_for_each_entry(pos_kctx, list, csf.link) {
				if (scheduler->top_kctx == pos_kctx) {
					found = true;
					break;
				}
			}

			if (!WARN_ON(!found)) {
				struct kbase_context *new_head_kctx;

				list_move_tail(&pos_kctx->csf.link, list);
				KBASE_KTRACE_ADD(kbdev, SCHEDULER_RUNNABLE_KCTX_ROTATE, pos_kctx,
						 0u);
				new_head_kctx = (!list_empty(list)) ?
							      list_first_entry(list, struct kbase_context,
									 csf.link) :
							      NULL;
				KBASE_KTRACE_ADD(kbdev, SCHEDULER_RUNNABLE_KCTX_HEAD, new_head_kctx,
						 0u);
				dev_dbg(kbdev->dev, "contexts rotated\n");
			}
		}
	}
}

/**
 * scheduler_update_idle_slots_status() - Get the status update for the CSG
 *                       slots for which the IDLE notification was received
 *                        previously.
 *
 * @kbdev:             Pointer to the GPU device.
 * @csg_bitmap:        Bitmap of the CSG slots for which
 *                     the status update request completed successfully.
 * @failed_csg_bitmap: Bitmap of the idle CSG slots for which
 *                     the status update request timedout.
 *
 * This function sends a CSG status update request for all the CSG slots
 * present in the bitmap scheduler->csg_slots_idle_mask. Additionally, if
 * the group's 'reevaluate_idle_status' field is set, the nominally non-idle
 * slots are also included in the status update for a confirmation of their
 * status. The function wait for the status update request to complete and
 * returns the update completed slots bitmap and any timed out idle-flagged
 * slots bitmap.
 *
 * The bits set in the scheduler->csg_slots_idle_mask bitmap are cleared by
 * this function.
 */
static void scheduler_update_idle_slots_status(struct kbase_device *kbdev,
					       unsigned long *csg_bitmap,
					       unsigned long *failed_csg_bitmap)
{
	struct kbase_csf_scheduler *const scheduler = &kbdev->csf.scheduler;
	const u32 num_groups = kbdev->csf.global_iface.group_num;
	unsigned long flags, fw_io_flags, i;
	u32 active_chk = 0;

	lockdep_assert_held(&scheduler->lock);

	spin_lock_irqsave(&scheduler->interrupt_lock, flags);
	if (kbase_csf_fw_io_open(&kbdev->csf.fw_io, &fw_io_flags)) {
		spin_unlock_irqrestore(&scheduler->interrupt_lock, flags);
		return;
	}

	for_each_set_bit(i, scheduler->csg_inuse_bitmap, num_groups) {
		struct kbase_csf_csg_slot *csg_slot = &scheduler->csg_slots[i];
		struct kbase_queue_group *group = csg_slot->resident_group;
		u32 csg_req;
		bool idle_flag;

		if (WARN_ON(!group)) {
			clear_bit(i, scheduler->csg_inuse_bitmap);
			clear_bit(i, scheduler->csg_slots_idle_mask);
			continue;
		}

		idle_flag = test_bit(i, scheduler->csg_slots_idle_mask);
		if (idle_flag || group->reevaluate_idle_status) {
			if (idle_flag) {
#ifdef CONFIG_MALI_DEBUG
				struct kbase_csf_cmd_stream_group_info *const ginfo =
					&kbdev->csf.global_iface.groups[i];
				if (!bitmap_empty(group->protm_pending_bitmap, ginfo->stream_num)) {
					dev_warn(
						kbdev->dev,
						"Idle bit set for group %d of ctx %d_%d on slot %d with pending protm execution",
						group->handle, group->kctx->tgid, group->kctx->id,
						(int)i);
				}
#endif
				clear_bit(i, scheduler->csg_slots_idle_mask);
				KBASE_KTRACE_ADD_CSF_GRP(kbdev, CSG_SLOT_IDLE_CLEAR, group,
							 scheduler->csg_slots_idle_mask[0]);
			} else {
				/* Updates include slots for which reevaluation is needed.
				 * Here one tracks the extra included slots in active_chk.
				 * For protm pending slots, their status of activeness are
				 * assured so no need to request an update.
				 */
				active_chk |= BIT(i);
				group->reevaluate_idle_status = false;
			}

			KBASE_KTRACE_ADD_CSF_GRP(kbdev, CSG_UPDATE_IDLE_SLOT_REQ, group, i);
			csg_req = kbase_csf_fw_io_group_read(&kbdev->csf.fw_io, i, CSG_ACK);
			csg_req ^= CSG_REQ_STATUS_UPDATE_MASK;
			kbase_csf_fw_io_group_write_mask(&kbdev->csf.fw_io, i, CSG_REQ, csg_req,
							 CSG_REQ_STATUS_UPDATE_MASK);

			/* Track the slot update requests in csg_bitmap.
			 * Note, if the scheduler requested extended update, the resulting
			 * csg_bitmap would be the idle_flags + active_chk. Otherwise it's
			 * identical to the idle_flags.
			 */
			set_bit(i, csg_bitmap);
		} else {
			group->run_state = KBASE_CSF_GROUP_RUNNABLE;
			KBASE_KTRACE_ADD_CSF_GRP(kbdev, CSF_GROUP_RUNNABLE, group,
						 group->run_state);
		}
	}

	/* The groups are aggregated into a single kernel doorbell request */
	if (!bitmap_empty(csg_bitmap, num_groups)) {
		long wt =
			kbase_csf_timeout_in_jiffies(CSG_STATUS_UPDATE_REQ_TIMEOUT_MS);
		u32 db_slots = (u32)csg_bitmap[0];

		kbase_csf_ring_csg_slots_doorbell(kbdev, db_slots);
		kbase_csf_fw_io_close(&kbdev->csf.fw_io, fw_io_flags);
		spin_unlock_irqrestore(&scheduler->interrupt_lock, flags);

		if (wait_csg_slots_handshake_ack(kbdev, CSG_REQ_STATUS_UPDATE_MASK, csg_bitmap,
						 wt)) {
			const int csg_nr = ffs(csg_bitmap[0]) - 1;
			struct kbase_queue_group *group =
				scheduler->csg_slots[csg_nr].resident_group;
			pixel_gpu_uevent_kmd_error_send(kbdev, GPU_UEVENT_INFO_CSG_REQ_STATUS_UPDATE);

			dev_warn(
				kbdev->dev,
				"[%llu] Timeout (%d ms) on CSG_REQ:STATUS_UPDATE, treat groups as not idle: slot mask=0x%lx",
				kbase_backend_get_cycle_cnt(kbdev), CSG_STATUS_UPDATE_REQ_TIMEOUT_MS,
				csg_bitmap[0]);
			schedule_actions_trigger_df(kbdev, group->kctx,
						    DF_CSG_STATUS_UPDATE_TIMEOUT);

			/* Store the bitmap of timed out slots */
			bitmap_copy(failed_csg_bitmap, csg_bitmap, num_groups);
			csg_bitmap[0] = ~csg_bitmap[0] & db_slots;

			/* Mask off any failed bit position contributed from active ones, as the
			 * intention is to retain the failed bit pattern contains only those from
			 * idle flags reporting back to the caller. This way, any failed to update
			 * original idle flag would be kept as 'idle' (an informed guess, as the
			 * update did not come to a conclusive result). So will be the failed
			 * active ones be treated as still 'non-idle'. This is for a graceful
			 * handling to the unexpected timeout condition.
			 */
			failed_csg_bitmap[0] &= ~active_chk;

		} else {
			KBASE_KTRACE_ADD(kbdev, SCHEDULER_UPDATE_IDLE_SLOTS_ACK, NULL, db_slots);
			csg_bitmap[0] = db_slots;
		}
	} else {
		kbase_csf_fw_io_close(&kbdev->csf.fw_io, fw_io_flags);
		spin_unlock_irqrestore(&scheduler->interrupt_lock, flags);
	}
}

/**
 * scheduler_handle_idle_slots() - Update the idle status of queue groups
 *                    resident on CSG slots for which the
 *                    IDLE notification was received previously.
 *
 * @kbdev:  Pointer to the GPU device.
 *
 * This function is called at the start of scheduling tick/tock to reconfirm
 * the idle status of queue groups resident on CSG slots for
 * which idle notification was received previously, i.e. all the CSG slots
 * present in the bitmap scheduler->csg_slots_idle_mask.
 * The confirmation is done by sending the CSG status update request to the
 * firmware. On completion, the firmware will mark the idleness at the
 * slot's interface CSG_STATUS_STATE register accordingly.
 *
 * The run state of the groups resident on still idle CSG slots is changed to
 * KBASE_CSF_GROUP_IDLE and the bitmap scheduler->csg_slots_idle_mask is
 * updated accordingly.
 * The bits corresponding to slots for which the status update request timedout
 * remain set in scheduler->csg_slots_idle_mask.
 */
static void scheduler_handle_idle_slots(struct kbase_device *kbdev)
{
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;
	u32 num_groups = kbdev->csf.global_iface.group_num;
	unsigned long flags, i;
	DECLARE_BITMAP(csg_bitmap, MAX_SUPPORTED_CSGS) = { 0 };
	DECLARE_BITMAP(failed_csg_bitmap, MAX_SUPPORTED_CSGS) = { 0 };

	lockdep_assert_held(&scheduler->lock);

	scheduler_update_idle_slots_status(kbdev, csg_bitmap, failed_csg_bitmap);

	spin_lock_irqsave(&scheduler->interrupt_lock, flags);
	for_each_set_bit(i, csg_bitmap, num_groups) {
		struct kbase_csf_csg_slot *csg_slot = &scheduler->csg_slots[i];
		struct kbase_queue_group *group = csg_slot->resident_group;

		if (WARN_ON(atomic_read(&csg_slot->state) != CSG_SLOT_RUNNING))
			continue;
		if (WARN_ON(!group))
			continue;
		if (WARN_ON(group->run_state != KBASE_CSF_GROUP_RUNNABLE &&
			    group->run_state != KBASE_CSF_GROUP_IDLE))
			continue;
		if (WARN_ON(group->priority >= KBASE_QUEUE_GROUP_PRIORITY_COUNT))
			continue;

		if (group_on_slot_is_idle(kbdev, i)) {
			group->run_state = KBASE_CSF_GROUP_IDLE;
			KBASE_KTRACE_ADD_CSF_GRP(kbdev, CSF_GROUP_IDLE, group, group->run_state);
			set_bit(i, scheduler->csg_slots_idle_mask);
			KBASE_KTRACE_ADD_CSF_GRP(kbdev, CSG_SLOT_IDLE_SET, group,
						 scheduler->csg_slots_idle_mask[0]);
		} else {
			group->run_state = KBASE_CSF_GROUP_RUNNABLE;
			KBASE_KTRACE_ADD_CSF_GRP(kbdev, CSF_GROUP_RUNNABLE, group,
						 group->run_state);
		}
	}

	bitmap_or(scheduler->csg_slots_idle_mask, scheduler->csg_slots_idle_mask, failed_csg_bitmap,
		  num_groups);
	KBASE_KTRACE_ADD_CSF_GRP(kbdev, SCHEDULER_HANDLE_IDLE_SLOTS, NULL,
				 scheduler->csg_slots_idle_mask[0]);
	spin_unlock_irqrestore(&scheduler->interrupt_lock, flags);
}

static void scheduler_scan_group_list(struct kbase_device *kbdev, struct list_head *groups)
{
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;
	struct kbase_queue_group *group, *n;

	list_for_each_entry_safe(group, n, groups, link_to_schedule) {
		if (!scheduler->ngrp_to_schedule) {
			/* keep the top csg''s origin */
			scheduler->top_kctx = group->kctx;
			scheduler->top_grp = group;
		}

		group->prepared_seq_num = scheduler->ngrp_to_schedule++;
		list_move_tail(&group->link_to_schedule, &scheduler->groups_to_schedule);

		group->kctx->csf.sched.ngrp_to_schedule++;
		count_active_address_space(kbdev, group->kctx);
	}
}

static void scheduler_rotate(struct kbase_device *kbdev)
{
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;

	lockdep_assert_held(&scheduler->lock);

	/* Dealing with rotation */
	scheduler_rotate_groups(kbdev);
	scheduler_rotate_ctxs(kbdev);
}

static struct kbase_queue_group *get_tock_top_group(struct kbase_csf_scheduler *const scheduler)
{
	struct kbase_context *kctx;
	int i;

	lockdep_assert_held(&scheduler->lock);
	for (i = 0; i < KBASE_QUEUE_GROUP_PRIORITY_COUNT; ++i) {
		list_for_each_entry(kctx, &scheduler->runnable_kctxs, csf.link) {
			struct kbase_queue_group *group;

			list_for_each_entry(group, &kctx->csf.sched.runnable_groups[i], link) {
				if (queue_group_idle_locked(group))
					continue;

				return group;
			}
		}
	}

	return NULL;
}

/**
 * suspend_active_groups_on_powerdown() - Suspend active CSG groups upon
 *                                        suspend or GPU IDLE.
 *
 * @kbdev:          Pointer to the device
 * @system_suspend: Flag to indicate it's for system suspend.
 *
 * This function will suspend all active CSG groups upon either
 * system suspend, runtime suspend or GPU IDLE.
 *
 * Return: 0 on success, -1 otherwise.
 */
static int suspend_active_groups_on_powerdown(struct kbase_device *kbdev, bool system_suspend)
{
	struct kbase_csf_scheduler *const scheduler = &kbdev->csf.scheduler;
	DECLARE_BITMAP(slot_mask, MAX_SUPPORTED_CSGS) = { 0 };

	if (unlikely(suspend_active_queue_groups(kbdev, slot_mask, false))) {
		if (!is_gpu_level_suspend_supported(kbdev)) {
			const int csg_nr = ffs(slot_mask[0]) - 1;
			struct kbase_queue_group *group;
			enum dumpfault_error_type error_type = DF_CSG_SUSPEND_TIMEOUT;

			group = scheduler->csg_slots[csg_nr].resident_group;

			/* The suspend of CSGs failed,
			 * trigger the GPU reset to be in a deterministic state.
			 */
			pixel_gpu_uevent_kmd_error_send(kbdev, GPU_UEVENT_INFO_CSG_SLOTS_SUSPEND);
			dev_warn(
				kbdev->dev,
				"[%llu] Timeout (%d ms) waiting for CSG slots to suspend on power down, slot_mask: 0x%*pb\n",
				kbase_backend_get_cycle_cnt(kbdev),
				kbase_get_timeout_ms(kbdev, CSF_FIRMWARE_TIMEOUT),
				kbdev->csf.global_iface.group_num, slot_mask);
			if (kbase_csf_firmware_ping_wait(kbdev, FW_PING_AFTER_ERROR_TIMEOUT_MS))
				error_type = DF_PING_REQUEST_TIMEOUT;
			schedule_actions_trigger_df(kbdev, group->kctx, error_type);
		}

		if (kbase_prepare_to_reset_gpu(kbdev, RESET_FLAGS_NONE))
			kbase_reset_gpu(kbdev);

		return -1;
	}

	kbdev->csf.mcu_halted = false;

	/* Check if the groups became active whilst the suspend was ongoing,
	 * but only for the case where the system suspend is not in progress
	 */
	if (!system_suspend && atomic_read(&scheduler->non_idle_offslot_grps))
		return -1;

	return 0;
}

/**
 * all_on_slot_groups_remained_idle - Live check for all groups' idleness
 *
 * @kbdev: Pointer to the device.
 *
 * Returns false if any of the queues inside any of the groups that have been
 * assigned a physical CSG slot have work to execute, or have executed work
 * since having received a GPU idle notification. This function is used to
 * handle a race condition between firmware reporting GPU idle and userspace
 * submitting more work by directly ringing a doorbell.
 *
 * Return: false if any queue inside any resident group has work to be processed
 *         or has processed work since GPU idle event, true otherwise.
 */
static bool all_on_slot_groups_remained_idle(struct kbase_device *kbdev)
{
	struct kbase_csf_scheduler *const scheduler = &kbdev->csf.scheduler;
	/* All CSGs have the same number of CSs */
	size_t const max_streams = kbdev->csf.global_iface.groups[0].stream_num;
	size_t i;

	lockdep_assert_held(&scheduler->lock);
	lockdep_assert_held(&scheduler->interrupt_lock);

	for_each_set_bit(i, scheduler->csg_slots_idle_mask, kbdev->csf.global_iface.group_num) {
		struct kbase_queue_group *const group = scheduler->csg_slots[i].resident_group;
		size_t j;

		for (j = 0; j < max_streams; ++j) {
			struct kbase_queue const *const queue = group->bound_queues[j];
			u64 const *output_addr;
			u64 cur_extract_ofs;

			if (!queue || !queue->user_io_addr)
				continue;

			output_addr = (u64 const *)(queue->user_io_addr + PAGE_SIZE / sizeof(u64));
			/*
			 * These 64-bit reads and writes will be atomic on a 64-bit kernel
			 * but may not be atomic on 32-bit kernels. Support for 32-bit
			 * kernels is limited to build-only.
			 */
			cur_extract_ofs = output_addr[CS_EXTRACT_LO / sizeof(u64)];
			if (cur_extract_ofs != queue->extract_ofs) {
				/* More work has been executed since the idle
				 * notification.
				 */
				return false;
			}
		}
	}

	return true;
}

static bool scheduler_idle_suspendable(struct kbase_device *kbdev)
{
	bool suspend;
	unsigned long flags;
	struct kbase_csf_scheduler *const scheduler = &kbdev->csf.scheduler;

	lockdep_assert_held(&scheduler->lock);

	if ((scheduler->state == SCHED_SUSPENDED) || (scheduler->state == SCHED_SLEEPING))
		return false;

	spin_lock_irqsave(&kbdev->hwaccess_lock, flags);
	spin_lock(&scheduler->interrupt_lock);

	if (scheduler->total_runnable_grps) {
		/* Check both on-slots and off-slots groups idle status */
		suspend = (scheduler->fast_gpu_idle_handling ||
			   kbase_csf_scheduler_all_csgs_idle(kbdev)) &&
			  !atomic_read(&scheduler->non_idle_offslot_grps) &&
			  kbase_pm_idle_groups_sched_suspendable(kbdev);
	} else
		suspend = kbase_pm_no_runnables_sched_suspendable(kbdev);

	if (suspend && unlikely(atomic_read(&scheduler->gpu_no_longer_idle)))
		suspend = false;

	/* Confirm that all groups are actually idle before proceeding with
	 * suspension as groups might potentially become active again without
	 * informing the scheduler in case userspace rings a doorbell directly.
	 */
	if (suspend && !scheduler->fast_gpu_idle_handling &&
	    unlikely(!all_on_slot_groups_remained_idle(kbdev))) {
		dev_dbg(kbdev->dev, "GPU suspension skipped due to active CSGs");
		suspend = false;
	}

	scheduler->fast_gpu_idle_handling = false;
	spin_unlock(&scheduler->interrupt_lock);
	spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);

	return suspend;
}

#ifdef KBASE_PM_RUNTIME
/**
 * scheduler_sleep_on_idle - Put the Scheduler in sleeping state on GPU
 *                           becoming idle.
 *
 * @kbdev: Pointer to the device.
 *
 * This function is called on GPU idle notification to trigger the transition of
 * GPU to sleep state, where MCU firmware pauses execution and L2 cache is
 * turned off. Scheduler's state is changed to sleeping and all the active queue
 * groups remain on the CSG slots.
 */
static void scheduler_sleep_on_idle(struct kbase_device *kbdev)
{
	struct kbase_csf_scheduler *const scheduler = &kbdev->csf.scheduler;

	lockdep_assert_held(&scheduler->lock);

	dev_dbg(kbdev->dev, "Scheduler to be put to sleep on GPU becoming idle");
	cancel_tick_work(scheduler);
	scheduler_pm_idle_before_sleep(kbdev);
	scheduler->state = SCHED_SLEEPING;
	KBASE_KTRACE_ADD(kbdev, SCHED_SLEEPING, NULL, scheduler->state);
}
#endif

/**
 * scheduler_suspend_on_idle - Put the Scheduler in suspended state on GPU
 *                             becoming idle.
 *
 * @kbdev: Pointer to the device.
 *
 * This function is called on GPU idle notification to trigger the power down of
 * GPU. Scheduler's state is changed to suspended and all the active queue
 * groups are suspended before halting the MCU firmware.
 *
 * Return: true if scheduler will be suspended or false if suspend is aborted.
 */
static bool scheduler_suspend_on_idle(struct kbase_device *kbdev)
{
	struct kbase_csf_scheduler *const scheduler = &kbdev->csf.scheduler;
	int ret = suspend_active_groups_on_powerdown(kbdev, false);

	if (ret) {
		dev_dbg(kbdev->dev, "Aborting suspend scheduler (grps: %d)",
			atomic_read(&kbdev->csf.scheduler.non_idle_offslot_grps));
		/* Bring forward the next tick */
		kbase_csf_scheduler_invoke_tick(kbdev);
		return false;
	}

	dev_dbg(kbdev->dev, "Scheduler to be suspended on GPU becoming idle");
	ret = scheduler_suspend(kbdev);
	if (!ret) {
		cancel_tick_work(scheduler);
		return true;
	}

	return false;
}

static void gpu_idle_worker(struct kbase_device *kbdev)
{
	struct kbase_csf_scheduler *const scheduler = &kbdev->csf.scheduler;
	bool scheduler_is_idle_suspendable = false;
	bool all_groups_suspended = false;

	WARN_ON_ONCE(atomic_read(&scheduler->pending_gpu_idle_work) == 0);

	KBASE_KTRACE_ADD(kbdev, SCHEDULER_GPU_IDLE_WORKER_START, NULL, 0u);

#define __ENCODE_KTRACE_INFO(reset, idle, all_suspend) \
	(((u32)reset) | (((u32)idle) << 4) | (((u32)all_suspend) << 8))

	if (kbase_reset_gpu_try_prevent(kbdev)) {
		dev_warn(kbdev->dev, "Quit idle for failing to prevent gpu reset.\n");
		KBASE_KTRACE_ADD(kbdev, SCHEDULER_GPU_IDLE_WORKER_END, NULL,
				 __ENCODE_KTRACE_INFO(true, false, false));
		goto exit;
	}
	kbase_debug_csf_fault_wait_completion(kbdev);
	rt_mutex_lock(&scheduler->lock);

#if IS_ENABLED(CONFIG_DEBUG_FS)
	if (unlikely(scheduler->state == SCHED_BUSY)) {
		rt_mutex_unlock(&scheduler->lock);
		kbase_reset_gpu_allow(kbdev);
		goto exit;
	}
#endif

	scheduler_is_idle_suspendable = scheduler_idle_suspendable(kbdev);
	if (scheduler_is_idle_suspendable) {
		KBASE_KTRACE_ADD(kbdev, SCHEDULER_GPU_IDLE_WORKER_HANDLING_START, NULL,
				 kbase_csf_ktrace_gpu_cycle_cnt(kbdev));
#ifdef KBASE_PM_RUNTIME
		if (kbase_pm_gpu_sleep_allowed(kbdev) &&
		    kbase_csf_scheduler_get_nr_active_csgs(kbdev))
			scheduler_sleep_on_idle(kbdev);
		else
#endif
			all_groups_suspended = scheduler_suspend_on_idle(kbdev);

		KBASE_KTRACE_ADD(kbdev, SCHEDULER_GPU_IDLE_WORKER_HANDLING_END, NULL, 0u);
	}

	rt_mutex_unlock(&scheduler->lock);
	kbase_reset_gpu_allow(kbdev);
	KBASE_KTRACE_ADD(kbdev, SCHEDULER_GPU_IDLE_WORKER_END, NULL,
			 __ENCODE_KTRACE_INFO(false, scheduler_is_idle_suspendable,
					      all_groups_suspended));
#undef __ENCODE_KTRACE_INFO

exit:
	atomic_dec(&scheduler->pending_gpu_idle_work);
}

static int scheduler_prepare(struct kbase_device *kbdev)
{
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;
	struct list_head privileged_groups, active_groups;
	unsigned long flags;
	int i;

	lockdep_assert_held(&scheduler->lock);

	/* Empty the groups_to_schedule */
	while (!list_empty(&scheduler->groups_to_schedule)) {
		struct kbase_queue_group *grp = list_first_entry(
			&scheduler->groups_to_schedule, struct kbase_queue_group, link_to_schedule);

		remove_scheduled_group(kbdev, grp);
	}

	/* Pre-scan init scheduler fields */
	if (WARN_ON(scheduler->ngrp_to_schedule != 0))
		scheduler->ngrp_to_schedule = 0;
	scheduler->top_kctx = NULL;
	scheduler->top_grp = NULL;
	scheduler->csg_scan_count_for_tick = 0;
	WARN_ON(!list_empty(&scheduler->idle_groups_to_schedule));
	scheduler->num_active_address_spaces = 0;
	scheduler->num_csg_slots_for_tick = 0;
	scheduler->csg_scan_sched_count = 0;
	bitmap_zero(scheduler->csg_slots_prio_update, MAX_SUPPORTED_CSGS);
	INIT_LIST_HEAD(&privileged_groups);
	INIT_LIST_HEAD(&active_groups);

	spin_lock_irqsave(&scheduler->interrupt_lock, flags);
	scheduler->tick_protm_pending_seq = KBASEP_TICK_PROTM_PEND_SCAN_SEQ_NR_INVALID;
	/* Scan out to run groups */
	for (i = 0; i < KBASE_QUEUE_GROUP_PRIORITY_COUNT; ++i) {
		struct kbase_context *kctx;

		/* Scan the per-priority list of groups twice, firstly for the
		 * prioritised contexts, then the normal ones.
		 */
		list_for_each_entry(kctx, &scheduler->runnable_kctxs, csf.link) {
			if (atomic_read(&kctx->prioritized))
				scheduler_ctx_scan_groups(kbdev, kctx, i, &privileged_groups,
							  &active_groups);
		}
		list_for_each_entry(kctx, &scheduler->runnable_kctxs, csf.link) {
			if (!atomic_read(&kctx->prioritized))
				scheduler_ctx_scan_groups(kbdev, kctx, i, &privileged_groups,
							  &active_groups);
		}
	}
	spin_unlock_irqrestore(&scheduler->interrupt_lock, flags);

	/* Adds privileged (RT + p.mode) groups to the scanout list */
	scheduler_scan_group_list(kbdev, &privileged_groups);

	/* Adds remainder of active groups to the scanout list */
	scheduler_scan_group_list(kbdev, &active_groups);

	/* Update this tick's non-idle groups */
	scheduler->non_idle_scanout_grps = scheduler->ngrp_to_schedule;

	/* Initial number of non-idle off-slot groups, before the scheduler's
	 * scheduler_apply() operation. This gives a sensible start point view
	 * of the tick. It will be subject to up/downs during the scheduler
	 * active phase.
	 */
	atomic_set(&scheduler->non_idle_offslot_grps, (int)scheduler->non_idle_scanout_grps);
	KBASE_KTRACE_ADD_CSF_GRP(kbdev, SCHEDULER_NONIDLE_OFFSLOT_GRP_INC, NULL,
				 scheduler->non_idle_scanout_grps);

	/* Adds those idle but runnable groups to the scanout list */
	scheduler_scan_group_list(kbdev, &scheduler->idle_groups_to_schedule);

	WARN_ON(scheduler->csg_scan_count_for_tick < scheduler->ngrp_to_schedule);

	KBASE_KTRACE_ADD_CSF_GRP(kbdev, SCHEDULER_TOP_GRP, scheduler->top_grp,
				 scheduler->num_active_address_spaces |
					 (((u64)scheduler->ngrp_to_schedule) << 32));
	set_max_csg_slots(kbdev);
	dev_dbg(kbdev->dev, "prepared groups length: %u, num_active_address_spaces: %u\n",
		scheduler->ngrp_to_schedule, scheduler->num_active_address_spaces);
	return 0;
}

/**
 * keep_lru_on_slots() - Check the condition for LRU is met.
 *
 * @kbdev: Pointer to the device.
 *
 * This function tries to maintain the Last-Recent-Use case on slots, when
 * the scheduler has no non-idle off-slot CSGs for a replacement
 * consideration. This effectively extends the previous scheduling results
 * for the new one. That is, the last recent used CSGs are retained on slots
 * for the new tick/tock action.
 *
 * Return: true for avoiding on-slot CSGs changes (i.e. keep existing LRU),
 *         otherwise false.
 */
static bool keep_lru_on_slots(struct kbase_device *kbdev)
{
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;
	bool keep_lru = false;
	int on_slots =
		bitmap_weight(scheduler->csg_inuse_bitmap, kbdev->csf.global_iface.group_num);

	lockdep_assert_held(&scheduler->lock);

	if (on_slots && !atomic_read(&scheduler->non_idle_offslot_grps)) {
		unsigned long flags;

		spin_lock_irqsave(&scheduler->interrupt_lock, flags);
		/* All on-slots are idle, no non-idle off-slot CSGs available
		 * for considering a meaningful change. Set keep_lru.
		 */
		keep_lru = kbase_csf_scheduler_all_csgs_idle(kbdev);

		spin_unlock_irqrestore(&scheduler->interrupt_lock, flags);

		dev_dbg(kbdev->dev, "Keep_LRU: %d, CSGs on-slots: %d\n", keep_lru, on_slots);
	}

	return keep_lru;
}

/**
 * prepare_fast_local_tock() - making preparation arrangement for exercizing
 *                             a fast local tock inside scheduling-actions.
 *
 * @kbdev:  Pointer to the GPU device.
 *
 * The function assumes that a scheduling action of firing a fast local tock
 * call (i.e. an equivalent tock action without dropping the lock) is desired
 * if there are idle onslot CSGs. The function updates those affected CSGs'
 * run-state as a preparation. This should only be called from inside the
 * schedule_actions(), where the previous idle-flags are still considered to
 * be reflective, following its earlier idle confirmation operational call,
 * plus some potential newly idle CSGs in the scheduling action committing
 * steps.
 *
 * Return: number of on-slots CSGs that can be considered for replacing.
 */
static int prepare_fast_local_tock(struct kbase_device *kbdev)
{
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;
	u32 num_groups = kbdev->csf.global_iface.group_num;
	unsigned long flags, i;
	DECLARE_BITMAP(csg_bitmap, MAX_SUPPORTED_CSGS) = { 0 };

	lockdep_assert_held(&scheduler->lock);

	spin_lock_irqsave(&scheduler->interrupt_lock, flags);
	bitmap_copy(csg_bitmap, scheduler->csg_slots_idle_mask, num_groups);
	spin_unlock_irqrestore(&scheduler->interrupt_lock, flags);

	/* Marking the flagged idle CSGs' run state to IDLE, so
	 * the intended fast local tock can replacing them with off-slots
	 * non-idle CSGs.
	 */
	for_each_set_bit(i, csg_bitmap, num_groups) {
		struct kbase_csf_csg_slot *csg_slot = &scheduler->csg_slots[i];
		struct kbase_queue_group *group = csg_slot->resident_group;

		if (!queue_group_idle_locked(group)) {
			group->run_state = KBASE_CSF_GROUP_IDLE;
			KBASE_KTRACE_ADD_CSF_GRP(kbdev, CSF_GROUP_IDLE, group, group->run_state);
		}
	}

	/* Return the number of idle slots for potential replacement */
	return bitmap_weight(csg_bitmap, num_groups);
}

static int wait_csg_slots_suspend(struct kbase_device *kbdev, unsigned long *slot_mask)
{
	struct kbase_csf_scheduler *const scheduler = &kbdev->csf.scheduler;
	u32 num_groups = kbdev->csf.global_iface.group_num;
	int err = 0;
	DECLARE_BITMAP(slot_mask_local, MAX_SUPPORTED_CSGS);

	lockdep_assert_held(&scheduler->lock);

	bitmap_copy(slot_mask_local, slot_mask, MAX_SUPPORTED_CSGS);

	while (!bitmap_empty(slot_mask_local, MAX_SUPPORTED_CSGS)) {
		long remaining = kbase_csf_timeout_in_jiffies(kbdev->csf.csg_suspend_timeout_ms);
		DECLARE_BITMAP(changed, MAX_SUPPORTED_CSGS);

		bitmap_copy(changed, slot_mask_local, MAX_SUPPORTED_CSGS);
		remaining = kbase_csf_fw_io_wait_event_timeout(
			&kbdev->csf.fw_io, kbdev->csf.event_wait,
			slots_state_changed(kbdev, changed, csg_slot_stopped_locked), remaining);

		if (likely(remaining > 0)) {
			u32 i;

			for_each_set_bit(i, changed, num_groups) {
				struct kbase_queue_group *group;

				if (WARN_ON(!csg_slot_stopped_locked(kbdev, (s8)i)))
					continue;

				/* The on slot csg is now stopped */
				clear_bit(i, slot_mask_local);

				group = scheduler->csg_slots[i].resident_group;
				if (likely(group)) {
					/* Only do save/cleanup if the
					 * group is not terminated during
					 * the sleep.
					 */

					/* Only emit suspend, if there was no AS fault */
					if (kctx_as_enabled(group->kctx) && !group->faulted)
						KBASE_TLSTREAM_TL_KBASE_DEVICE_SUSPEND_CSG(
							kbdev, kbdev->id, i);

					save_csg_slot(group);
					if (cleanup_csg_slot(group)) {
						sched_evict_group(group, true, true);
					}
				}
			}
		} else if (!remaining) {
			dev_warn(
				kbdev->dev,
				"[%llu] Suspend request sent on CSG slots 0x%lx timed out for slots 0x%lx",
				kbase_backend_get_cycle_cnt(kbdev), slot_mask[0],
				slot_mask_local[0]);
			/* Return the bitmask of the timed out slots to the caller */
			bitmap_copy(slot_mask, slot_mask_local, MAX_SUPPORTED_CSGS);
			err = -ETIMEDOUT;
			break;
		}
	}

	return err;
}

/**
 * evict_lru_or_blocked_csg() - Evict the least-recently-used idle or blocked CSG
 *
 * @kbdev: Pointer to the device
 *
 * Used to allow for speedier starting/resumption of another CSG. The worst-case
 * scenario of the evicted CSG being scheduled next is expected to be rare.
 * Also, the eviction will not be applied if the GPU is running in protected mode.
 * Otherwise the the eviction attempt would force the MCU to quit the execution of
 * the protected mode, and likely re-request to enter it again.
 */
static void evict_lru_or_blocked_csg(struct kbase_device *kbdev)
{
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;
	size_t i;
	struct kbase_queue_group *lru_idle_group = NULL;
	const u32 total_csg_slots = kbdev->csf.global_iface.group_num;
	const bool all_addr_spaces_used =
		(scheduler->num_active_address_spaces >=
		 (u32)(kbdev->nr_hw_address_spaces - NUM_RESERVED_AS_SLOTS));
	u8 as_usage[BASE_MAX_NR_AS] = { 0 };

	lockdep_assert_held(&scheduler->lock);
	if (kbase_csf_scheduler_protected_mode_in_use(kbdev))
		return;

	BUILD_BUG_ON(MAX_SUPPORTED_CSGS > (sizeof(int) * BITS_PER_BYTE));
	if ((u32)fls(scheduler->csg_inuse_bitmap[0]) != total_csg_slots)
		return; /* Some CSG slots remain unused */

	if (all_addr_spaces_used) {
		for (i = 0; i != total_csg_slots; ++i) {
			if (scheduler->csg_slots[i].resident_group != NULL) {
				if (WARN_ON(scheduler->csg_slots[i].resident_group->kctx->as_nr <
					    0))
					continue;

				as_usage[scheduler->csg_slots[i].resident_group->kctx->as_nr]++;
			}
		}
	}

	for (i = 0; i != total_csg_slots; ++i) {
		struct kbase_queue_group *const group = scheduler->csg_slots[i].resident_group;

		/* We expect that by this point all groups would normally be
		 * assigned a physical CSG slot, but if circumstances have
		 * changed then bail out of this optimisation.
		 */
		if (group == NULL)
			return;

		/* Real-time priority CSGs must be kept on-slot even when
		 * idle.
		 */
		if ((group->run_state == KBASE_CSF_GROUP_IDLE) &&
		    (group->priority != KBASE_QUEUE_GROUP_PRIORITY_REALTIME) &&
		    ((lru_idle_group == NULL) ||
		     (lru_idle_group->prepared_seq_num < group->prepared_seq_num))) {
			if (WARN_ON(group->kctx->as_nr < 0))
				continue;

			/* If all address spaces are used, we need to ensure the group does not
			 * share the AS with other active CSGs. Or CSG would be freed without AS
			 * and this optimization would not work.
			 */
			if ((!all_addr_spaces_used) || (as_usage[group->kctx->as_nr] == 1))
				lru_idle_group = group;
		}
	}

	if (lru_idle_group != NULL) {
		unsigned long slot_mask = 1UL << lru_idle_group->csg_nr;

		dev_dbg(kbdev->dev, "Suspending LRU idle group %d of context %d_%d on slot %d",
			lru_idle_group->handle, lru_idle_group->kctx->tgid,
			lru_idle_group->kctx->id, lru_idle_group->csg_nr);
		suspend_queue_group(lru_idle_group);
		if (wait_csg_slots_suspend(kbdev, &slot_mask)) {
			enum dumpfault_error_type error_type = DF_CSG_SUSPEND_TIMEOUT;
			pixel_gpu_uevent_kmd_error_send(kbdev, GPU_UEVENT_INFO_CSG_SUSPEND);
			dev_warn(
				kbdev->dev,
				"[%llu] LRU idle group %d of context %d_%d failed to suspend on slot %d (timeout %d ms)",
				kbase_backend_get_cycle_cnt(kbdev), lru_idle_group->handle,
				lru_idle_group->kctx->tgid, lru_idle_group->kctx->id,
				lru_idle_group->csg_nr,
				kbase_get_timeout_ms(kbdev, CSF_FIRMWARE_TIMEOUT));
			if (kbase_csf_firmware_ping_wait(kbdev, FW_PING_AFTER_ERROR_TIMEOUT_MS))
				error_type = DF_PING_REQUEST_TIMEOUT;
			schedule_actions_trigger_df(kbdev, lru_idle_group->kctx, error_type);
		}
	}
}

static void schedule_actions(struct kbase_device *kbdev, bool is_tick)
{
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;
	unsigned long flags;
	struct kbase_queue_group *protm_grp;
	bool skip_scheduling_actions;
	bool skip_idle_slots_update;
	bool new_protm_top_grp = false;
	int local_tock_slots = 0;

	kbase_reset_gpu_assert_prevented(kbdev);
	lockdep_assert_held(&scheduler->lock);

	if (kbase_csf_scheduler_wait_mcu_active(kbdev)) {
		dev_err(kbdev->dev, "Wait for MCU power on failed on scheduling tick/tock");
		return;
	}

	spin_lock_irqsave(&scheduler->interrupt_lock, flags);
	skip_idle_slots_update = kbase_csf_scheduler_protected_mode_in_use(kbdev);
	skip_scheduling_actions = !skip_idle_slots_update && kbdev->protected_mode;
	spin_unlock_irqrestore(&scheduler->interrupt_lock, flags);

	/* Skip scheduling actions as GPU reset hasn't been performed yet to
	 * rectify the anomaly that happened when pmode exit interrupt wasn't
	 * received before the termination of group running in pmode.
	 */
	if (unlikely(skip_scheduling_actions)) {
		dev_info(kbdev->dev, "Scheduling actions skipped due to anomaly in pmode");
		return;
	}

	if (!skip_idle_slots_update) {
		/* Updating on-slot idle CSGs when not in protected mode. */
		scheduler_handle_idle_slots(kbdev);

		/* Determine whether the condition is met for keeping the
		 * Last-Recent-Use. If true, skipping the remaining action
		 * steps and thus extending the previous tick's arrangement,
		 * in particular, no alterations to on-slot CSGs.
		 */
		if (keep_lru_on_slots(kbdev)) {
			return;
		}
	}

	if (is_tick)
		scheduler_rotate(kbdev);

redo_local_tock:
	scheduler_prepare(kbdev);
	/* Need to specifically enqueue the GPU idle work if there are no groups
	 * to schedule despite the runnable groups. This scenario will happen
	 * if System suspend is done when all groups are idle and and no work
	 * is submitted for the groups after the System resume.
	 */
	if (unlikely(!scheduler->ngrp_to_schedule && scheduler->total_runnable_grps)) {
		dev_dbg(kbdev->dev, "No groups to schedule in the tick");
		enqueue_gpu_idle_work(scheduler);
		return;
	}
	spin_lock_irqsave(&scheduler->interrupt_lock, flags);
	protm_grp = scheduler->active_protm_grp;

	/* Avoid update if the top-group remains unchanged and in protected
	 * mode. For the said case, all the slots update is effectively
	 * competing against the active protected mode group (typically the
	 * top-group). If we update other slots, even on leaving the
	 * top-group slot untouched, the firmware would exit the protected mode
	 * for interacting with the host-driver. After it, as the top-group
	 * would again raise the request for entering protected mode, we would
	 * be actively doing the switching over twice without progressing the
	 * queue jobs.
	 */
	if (protm_grp && scheduler->top_grp == protm_grp) {
		dev_dbg(kbdev->dev, "Scheduler keep protm exec: group-%d", protm_grp->handle);
		spin_unlock_irqrestore(&scheduler->interrupt_lock, flags);

		update_offslot_non_idle_cnt_for_onslot_grp(protm_grp);
		remove_scheduled_group(kbdev, protm_grp);
		scheduler_check_pmode_progress(kbdev);
	} else if (scheduler->top_grp) {
		if (protm_grp)
			dev_dbg(kbdev->dev, "Scheduler drop protm exec: group-%d",
				protm_grp->handle);

		if (!bitmap_empty(scheduler->top_grp->protm_pending_bitmap,
				  kbdev->csf.global_iface.groups[0].stream_num)) {
			dev_dbg(kbdev->dev,
				"Scheduler prepare protm exec: group-%d of context %d_%d",
				scheduler->top_grp->handle, scheduler->top_grp->kctx->tgid,
				scheduler->top_grp->kctx->id);

			/* When entering protected mode all CSG slots can be occupied
			 * but only the protected mode CSG will be running. Any event
			 * that would trigger the execution of an on-slot idle CSG will
			 * need to be handled by the host during protected mode.
			 */
			new_protm_top_grp = true;
		}

		spin_unlock_irqrestore(&scheduler->interrupt_lock, flags);

		scheduler_apply(kbdev);

		/* Scheduler is dropping the exec of the previous protm_grp,
		 * Until the protm quit completes, the GPU is effectively
		 * locked in the secure mode.
		 */
		if (protm_grp)
			scheduler_force_protm_exit(kbdev);

		wait_csg_slots_start(kbdev);
		wait_csg_slots_finish_prio_update(kbdev);

		if (new_protm_top_grp) {
			scheduler_group_check_protm_enter(kbdev, scheduler->top_grp);
		} else if (!local_tock_slots && atomic_read(&scheduler->non_idle_offslot_grps)) {
			/* If during the scheduling action, we have off-slot
			 * non-idle CSGs in waiting, if it happens to have
			 * some new idle slots emerging during the committed
			 * action steps, trigger a one-off fast local tock.
			 */
			local_tock_slots = prepare_fast_local_tock(kbdev);

			if (local_tock_slots) {
				dev_dbg(kbdev->dev, "In-cycle %d idle slots available\n",
					local_tock_slots);
				goto redo_local_tock;
			}
		}
	} else {
		spin_unlock_irqrestore(&scheduler->interrupt_lock, flags);
	}

	evict_lru_or_blocked_csg(kbdev);
}

/**
 * can_skip_scheduling() - Check if the scheduling actions can be skipped.
 *
 * @kbdev: Pointer to the device
 *
 * This function is called on a scheduling tick or tock to determine if the
 * scheduling actions can be skipped.
 * If Scheduler is in sleeping state and exit from the sleep state is allowed
 * then activation of MCU will be triggered. The tick or tock work item could
 * have been in flight when the state of Scheduler was changed to sleeping.
 *
 * Return: true if the scheduling actions can be skipped.
 */
static bool can_skip_scheduling(struct kbase_device *kbdev)
{
	struct kbase_csf_scheduler *const scheduler = &kbdev->csf.scheduler;

	lockdep_assert_held(&scheduler->lock);

	if (unlikely(!kbase_reset_gpu_is_not_pending(kbdev)))
		return true;

	if (scheduler->state == SCHED_SUSPENDED)
		return true;

#ifdef KBASE_PM_RUNTIME
	if (scheduler->state == SCHED_SLEEPING) {
		unsigned long flags;

		spin_lock_irqsave(&kbdev->hwaccess_lock, flags);
		if (kbdev->pm.backend.exit_gpu_sleep_mode) {
			int ret;

			spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);
			ret = scheduler_pm_active_after_sleep(kbdev);
			if (!ret) {
				scheduler->state = SCHED_INACTIVE;
				KBASE_KTRACE_ADD(kbdev, SCHED_INACTIVE, NULL, scheduler->state);
				return false;
			}

			dev_info(kbdev->dev, "Skip scheduling due to system suspend");
			return true;
		}
		spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);
		return true;
	}
#endif

	return false;
}

static void schedule_on_tock(struct kbase_device *kbdev)
{
	struct kbase_csf_scheduler *const scheduler = &kbdev->csf.scheduler;
	int err;

	err = kbase_reset_gpu_try_prevent(kbdev);
	/* Regardless of whether reset failed or is currently happening, exit
	 * early
	 */
	if (err)
		return;

	kbase_debug_csf_fault_wait_completion(kbdev);
	rt_mutex_lock(&scheduler->lock);
	if (can_skip_scheduling(kbdev))
	{
		atomic_set(&scheduler->pending_tock_work, false);
		goto exit_no_schedule_unlock;
	}

	WARN_ON(!(scheduler->state == SCHED_INACTIVE));
	scheduler->state = SCHED_BUSY;
	KBASE_KTRACE_ADD(kbdev, SCHED_BUSY, NULL, scheduler->state);

	/* Undertaking schedule action steps */
	KBASE_KTRACE_ADD(kbdev, SCHEDULER_TOCK_START, NULL, 0u);
	while (atomic_cmpxchg(&scheduler->pending_tock_work, true, false) == true)
		schedule_actions(kbdev, false);

	/* Record time information on a non-skipped tock */
	scheduler->last_schedule = jiffies;

	scheduler->state = SCHED_INACTIVE;
	KBASE_KTRACE_ADD(kbdev, SCHED_INACTIVE, NULL, scheduler->state);
	if (!scheduler->total_runnable_grps)
		enqueue_gpu_idle_work(scheduler);
	rt_mutex_unlock(&scheduler->lock);
	kbase_reset_gpu_allow(kbdev);

	KBASE_KTRACE_ADD(kbdev, SCHEDULER_TOCK_END, NULL, 0u);
	return;

exit_no_schedule_unlock:
	rt_mutex_unlock(&scheduler->lock);
	kbase_reset_gpu_allow(kbdev);
}

static void schedule_on_tick(struct kbase_device *kbdev)
{
	struct kbase_csf_scheduler *const scheduler = &kbdev->csf.scheduler;

	int err = kbase_reset_gpu_try_prevent(kbdev);
	/* Regardless of whether reset failed or is currently happening, exit
	 * early
	 */
	if (err)
		return;

	kbase_debug_csf_fault_wait_completion(kbdev);
	rt_mutex_lock(&scheduler->lock);

	if (can_skip_scheduling(kbdev))
		goto exit_no_schedule_unlock;

	scheduler->state = SCHED_BUSY;
	KBASE_KTRACE_ADD(kbdev, SCHED_BUSY, NULL, scheduler->state);

	/* Undertaking schedule action steps */
	KBASE_KTRACE_ADD(kbdev, SCHEDULER_TICK_START, NULL, scheduler->total_runnable_grps);
	schedule_actions(kbdev, true);

	/* Record time information */
	scheduler->last_schedule = jiffies;

	/* Kicking next scheduling if needed */
	if (likely(kbase_csf_scheduler_timer_is_enabled(kbdev)) &&
	    (scheduler->total_runnable_grps > 0)) {
		hrtimer_start(&scheduler->tick_timer,
			      HR_TIMER_DELAY_MSEC(scheduler->csg_scheduling_period_ms),
			      HRTIMER_MODE_REL);
		dev_dbg(kbdev->dev, "scheduling for next tick, num_runnable_groups:%u\n",
			scheduler->total_runnable_grps);
	} else if (!scheduler->total_runnable_grps) {
		enqueue_gpu_idle_work(scheduler);
	}

	scheduler->state = SCHED_INACTIVE;
	rt_mutex_unlock(&scheduler->lock);
	KBASE_KTRACE_ADD(kbdev, SCHED_INACTIVE, NULL, scheduler->state);
	kbase_reset_gpu_allow(kbdev);

	KBASE_KTRACE_ADD(kbdev, SCHEDULER_TICK_END, NULL, scheduler->total_runnable_grps);
	return;

exit_no_schedule_unlock:
	rt_mutex_unlock(&scheduler->lock);
	kbase_reset_gpu_allow(kbdev);
}


static int suspend_active_queue_groups(struct kbase_device *kbdev, unsigned long *slot_mask,
				       bool reset)
{
	struct kbase_csf_scheduler *const scheduler = &kbdev->csf.scheduler;
	u32 num_groups = kbdev->csf.global_iface.group_num;
	struct kbase_queue_group *group;
	u32 slot_num;
	int ret;

	lockdep_assert_held(&scheduler->lock);

		for (slot_num = 0; slot_num < num_groups; slot_num++) {
			group = scheduler->csg_slots[slot_num].resident_group;

			if (group) {
				suspend_queue_group(group);
				set_bit(slot_num, slot_mask);
			}
		}

		ret = wait_csg_slots_suspend(kbdev, slot_mask);
	return ret;
}

static int suspend_active_queue_groups_on_reset(struct kbase_device *kbdev)
{
	struct kbase_csf_scheduler *const scheduler = &kbdev->csf.scheduler;
	DECLARE_BITMAP(slot_mask, MAX_SUPPORTED_CSGS) = { 0 };
	int ret;
	int ret2;

	rt_mutex_lock(&scheduler->lock);

	ret = suspend_active_queue_groups(kbdev, slot_mask, true);

	if (ret) {
		dev_warn(
			kbdev->dev,
			"Timeout waiting for CSG slots to suspend before reset, slot_mask: 0x%*pb\n",
			kbdev->csf.global_iface.group_num, slot_mask);
		//TODO: should introduce SSCD report if this happens.
		kbase_gpu_timeout_debug_message(kbdev, "");
		dev_warn(kbdev->dev, "[%llu] Firmware ping %d",
				kbase_backend_get_cycle_cnt(kbdev),
				kbase_csf_firmware_ping_wait(kbdev, 0));
	}

	/* Need to flush the GPU cache to ensure suspend buffer
	 * contents are not lost on reset of GPU.
	 * Do this even if suspend operation had timed out for some of
	 * the CSG slots.
	 * In case the scheduler already in suspended state, the
	 * cache clean is required as the async reset request from
	 * the debugfs may race against the scheduler suspend operation
	 * due to the extra context ref-count, which prevents the
	 * L2 powering down cache clean operation in the non racing
	 * case.
	 */
	kbase_gpu_start_cache_clean(kbdev, GPU_COMMAND_CACHE_CLN_INV_L2_LSC);
	ret2 = kbase_gpu_wait_cache_clean_timeout(
		kbdev, kbase_get_timeout_ms(kbdev, MMU_AS_INACTIVE_WAIT_TIMEOUT));
	if (ret2) {
		dev_err(kbdev->dev, "[%llu] Timeout waiting for CACHE_CLN_INV_L2_LSC",
			kbase_backend_get_cycle_cnt(kbdev));
		if (!ret)
			ret = ret2;
	}

	rt_mutex_unlock(&scheduler->lock);

	return ret;
}

/**
 * scheduler_handle_reset_in_protected_mode() - Update the state of normal mode
 *                                              groups when reset is done during
 *                                              protected mode execution and GPU
 *                                              can't exit the protected mode in
 *                                              response to the ping request.
 *
 * @kbdev: Pointer to the device.
 *
 * This function is called at the time of GPU reset, before the suspension of queue groups,
 * to handle the case when the reset is getting performed whilst GPU is in protected mode.
 * If GPU is in protected mode, then the function attempts to bring it back to the normal
 * mode by sending a ping request.
 * - If GPU exited the protected mode, then the function returns success to the caller
 *   indicating that the on-slot groups can be suspended in a regular way.
 * - If GPU didn't exit the protected mode then as a recovery measure the function marks
 *   the normal mode on-slot groups as suspended to avoid any loss of work for those groups.
 *   All on-slot groups, except the top group that executes in protected mode, are implicitly
 *   suspended by the FW just before entering protected mode. So the failure to exit protected
 *   mode is attributed to the top group and it is thus forcefully evicted by the Scheduler
 *   later in the request sequence.
 *
 * Return: true if the groups remaining on the CSG slots need to be suspended in
 *         the regular way by sending CSG SUSPEND requests to FW, otherwise false.
 */
static bool scheduler_handle_reset_in_protected_mode(struct kbase_device *kbdev)
{
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;
	u32 const num_groups = kbdev->csf.global_iface.group_num;
	struct kbase_queue_group *protm_grp;
	bool suspend_on_slot_groups = true;
	bool pmode_active;
	unsigned long flags;
	u32 csg_nr;

	rt_mutex_lock(&scheduler->lock);

	spin_lock_irqsave(&scheduler->interrupt_lock, flags);
	protm_grp = scheduler->active_protm_grp;
	pmode_active = kbdev->protected_mode;
	spin_unlock_irqrestore(&scheduler->interrupt_lock, flags);

	/* GPU not in protected mode. Suspend the on-slot groups normally */
	if (likely(!pmode_active))
		goto unlock;

	/* Send the ping request to force exit from protected mode */
	if (!kbase_csf_firmware_ping_wait(kbdev, FW_PING_ON_GPU_RESET_IN_PMODE_TIMEOUT_MS)) {
		/* FW should have exited protected mode before acknowledging the ping request */
		WARN_ON_ONCE(scheduler->active_protm_grp);

		/* Explicitly suspend all groups. The protected mode group would be terminated
		 * if it had faulted.
		 */
		goto unlock;
	}

	dev_warn(
		kbdev->dev,
		"Timeout for ping request sent to force exit from protected mode before GPU reset");

	/* GPU didn't exit the protected mode, so FW is most probably unresponsive.
	 * All the on-slot groups barring the protected mode group can be marked
	 * as suspended right away. The protected mode group would be forcefully
	 * evicted from the csg slot.
	 */
	suspend_on_slot_groups = false;
	for (csg_nr = 0; csg_nr < num_groups; csg_nr++) {
		struct kbase_queue_group *const group =
			kbdev->csf.scheduler.csg_slots[csg_nr].resident_group;
		int new_val;

		if (!group || (group == protm_grp))
			continue;

		cleanup_csg_slot(group);
		group->run_state = KBASE_CSF_GROUP_SUSPENDED;
		KBASE_KTRACE_ADD_CSF_GRP(kbdev, CSF_GROUP_SUSPENDED, group, group->run_state);

		/* Simply treat the normal mode groups as non-idle. The tick
		 * scheduled after the reset will re-initialize the counter
		 * anyways.
		 */
		new_val = atomic_inc_return(&scheduler->non_idle_offslot_grps);
		KBASE_KTRACE_ADD_CSF_GRP(kbdev, SCHEDULER_NONIDLE_OFFSLOT_GRP_INC, group,
					 (u64)new_val);
	}

unlock:
	rt_mutex_unlock(&scheduler->lock);
	return suspend_on_slot_groups;
}

static void scheduler_inner_reset(struct kbase_device *kbdev)
{
	u32 const num_groups = kbdev->csf.global_iface.group_num;
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;
	unsigned long flags;

	WARN_ON(kbase_csf_scheduler_get_nr_active_csgs(kbdev));

	/* Cancel any potential queued delayed work(s) */
	cancel_tick_work(scheduler);
	cancel_tock_work(scheduler);
	/* gpu_idle_worker() might already be running at this point, which
	 * could decrement the pending_gpu_idle_worker counter to below 0.
	 * It'd be safer to let it run if one has already been scheduled.
	 */
	cancel_delayed_work_sync(&scheduler->ping_work);

	rt_mutex_lock(&scheduler->lock);

	spin_lock_irqsave(&scheduler->interrupt_lock, flags);
	bitmap_fill(scheduler->csgs_events_enable_mask, MAX_SUPPORTED_CSGS);
	if (scheduler->active_protm_grp) {
		KBASE_KTRACE_ADD_CSF_GRP(kbdev, SCHEDULER_PROTM_EXIT, scheduler->active_protm_grp,
					 0u);
		if (scheduler->active_protm_grp->kctx) {
			scheduler->active_protm_grp->kctx->protm_exit_ts = ktime_get_raw();
		}
	}

	scheduler->active_protm_grp = NULL;
	memset(kbdev->csf.scheduler.csg_slots, 0, num_groups * sizeof(struct kbase_csf_csg_slot));
	bitmap_zero(kbdev->csf.scheduler.csg_inuse_bitmap, num_groups);
	spin_unlock_irqrestore(&scheduler->interrupt_lock, flags);

	scheduler->top_kctx = NULL;
	scheduler->top_grp = NULL;

	atomic_set(&scheduler->gpu_idle_timer_enabled, false);

	KBASE_KTRACE_ADD_CSF_GRP(kbdev, SCHEDULER_TOP_GRP, scheduler->top_grp,
				 scheduler->num_active_address_spaces |
					 (((u64)scheduler->total_runnable_grps) << 32));

#ifdef KBASE_PM_RUNTIME
	if (scheduler->state == SCHED_SLEEPING) {
#if IS_ENABLED(CONFIG_MALI_TRACE_POWER_GPU_WORK_PERIOD)
		hrtimer_cancel(&scheduler->gpu_metrics_timer);
#endif
		scheduler->state = SCHED_SUSPENDED;
		KBASE_KTRACE_ADD(kbdev, SCHED_SUSPENDED, NULL, scheduler->state);
	}
#endif
	rt_mutex_unlock(&scheduler->lock);
}

void kbase_csf_scheduler_reset(struct kbase_device *kbdev)
{
	struct kbase_context *kctx;
	WARN_ON(!kbase_reset_gpu_is_active(kbdev));

	KBASE_KTRACE_ADD(kbdev, SCHEDULER_RESET_START, NULL, 0u);

	if (kbase_reset_gpu_is_active(kbdev))
		kbase_debug_csf_fault_wait_completion(kbdev);


	if (scheduler_handle_reset_in_protected_mode(kbdev) &&
	    !suspend_active_queue_groups_on_reset(kbdev)) {
		/* As all groups have been successfully evicted from the CSG
		 * slots, clear out thee scheduler data fields and return
		 */
		scheduler_inner_reset(kbdev);
		return;
	}

	mutex_lock(&kbdev->kctx_list_lock);

	/* The loop to iterate over the kbase contexts is present due to lock
	 * ordering issue between kctx->csf.lock & kbdev->csf.scheduler.lock.
	 * CSF ioctls first take kctx->csf.lock which is context-specific and
	 * then take kbdev->csf.scheduler.lock for global actions like assigning
	 * a CSG slot.
	 * If the lock ordering constraint was not there then could have
	 * directly looped over the active queue groups.
	 */
	list_for_each_entry(kctx, &kbdev->kctx_list, kctx_list_link) {
		/* Firmware reload would reinitialize the CSG & CS interface IO
		 * pages, so just need to internally mark the currently active
		 * queue groups as terminated (similar to the unexpected OoM
		 * event case).
		 * No further work can now get executed for the active groups
		 * (new groups would have to be created to execute work) and
		 * in near future Clients would be duly informed of this
		 * reset. The resources (like User IO pages, GPU queue memory)
		 * allocated for the associated queues would be freed when the
		 * Clients do the teardown when they become aware of the reset.
		 */
		kbase_csf_active_queue_groups_reset(kbdev, kctx);
	}

	mutex_unlock(&kbdev->kctx_list_lock);

	KBASE_KTRACE_ADD(kbdev, SCHEDULER_RESET_END, NULL, 0u);

	/* After queue groups reset, the scheduler data fields clear out */
	scheduler_inner_reset(kbdev);
}

static void firmware_aliveness_monitor(struct work_struct *work)
{
	struct kbase_device *kbdev =
		container_of(work, struct kbase_device, csf.scheduler.ping_work.work);
	int err;

	/* Ensure that reset will not be occurring while this function is being
	 * executed as otherwise calling kbase_reset_gpu when reset is already
	 * occurring is a programming error.
	 *
	 * We must use the 'try' variant as the Reset worker can try to flush
	 * this workqueue, which would otherwise deadlock here if we tried to
	 * wait for the reset (and thus ourselves) to complete.
	 */
	err = kbase_reset_gpu_try_prevent(kbdev);
	if (err) {
		/* It doesn't matter whether the value was -EAGAIN or a fatal
		 * error, just stop processing. In case of -EAGAIN, the Reset
		 * worker will restart the scheduler later to resume ping
		 */
		return;
	}

	rt_mutex_lock(&kbdev->csf.scheduler.lock);

#ifdef CONFIG_MALI_DEBUG
	if (fw_debug) {
		/* ping requests cause distraction in firmware debugging */
		goto exit;
	}
#endif

	if (kbdev->csf.scheduler.state == SCHED_SUSPENDED ||
	    kbdev->csf.scheduler.state == SCHED_SLEEPING)
		goto exit;

	if (kbase_csf_scheduler_get_nr_active_csgs(kbdev) != 1)
		goto exit;

	if (kbase_csf_scheduler_protected_mode_in_use(kbdev))
		goto exit;

	kbase_pm_lock(kbdev);
	if (kbase_pm_context_active_handle_suspend_locked(kbdev,
							  KBASE_PM_SUSPEND_HANDLER_DONT_INCREASE)) {
		kbase_pm_unlock(kbdev);
		/* Suspend pending - no real need to ping */
		goto exit;
	}
	kbase_pm_unlock(kbdev);

	if (kbase_csf_scheduler_wait_mcu_active(kbdev)) {
		dev_err(kbdev->dev, "Wait for MCU power on failed at fw aliveness monitor");
		kbase_pm_context_idle(kbdev);
		goto exit;
	}

	err = kbase_csf_firmware_ping_wait(kbdev,
					   kbase_get_timeout_ms(kbdev, CSF_FIRMWARE_TIMEOUT));

	if (err) {
		/* It is acceptable to enqueue a reset whilst we've prevented
		 * them, it will happen after we've allowed them again
		 */
		if (kbase_prepare_to_reset_gpu(kbdev, RESET_FLAGS_HWC_UNRECOVERABLE_ERROR))
			kbase_reset_gpu(kbdev);
	} else if (kbase_csf_scheduler_get_nr_active_csgs(kbdev) == 1) {
		queue_delayed_work(
			system_long_wq, &kbdev->csf.scheduler.ping_work,
			msecs_to_jiffies(kbase_get_timeout_ms(kbdev, CSF_FIRMWARE_PING_TIMEOUT)));
	}

	kbase_pm_context_idle(kbdev);
exit:
	rt_mutex_unlock(&kbdev->csf.scheduler.lock);
	kbase_reset_gpu_allow(kbdev);
}

int kbase_csf_scheduler_group_copy_suspend_buf(struct kbase_queue_group *group,
					       struct kbase_suspend_copy_buffer *sus_buf)
{
	struct kbase_context *const kctx = group->kctx;
	struct kbase_device *const kbdev = kctx->kbdev;
	struct kbase_csf_scheduler *const scheduler = &kbdev->csf.scheduler;
	bool on_slot;
	int err = 0;

	kbase_reset_gpu_assert_prevented(kbdev);
	lockdep_assert_held(&kctx->csf.lock);
	rt_mutex_lock(&scheduler->lock);

	on_slot = kbasep_csf_scheduler_group_is_on_slot_locked(group);

#ifdef KBASE_PM_RUNTIME
	if (on_slot && (scheduler->state == SCHED_SLEEPING)) {
		if (wait_for_scheduler_to_exit_sleep(kbdev)) {
			dev_warn(
				kbdev->dev,
				"Wait for scheduler to exit sleep state timedout when copying suspend buffer for group %d of ctx %d_%d on slot %d",
				group->handle, group->kctx->tgid, group->kctx->id, group->csg_nr);

			scheduler_wakeup(kbdev, true);

			/* Wait for MCU firmware to start running */
			if (kbase_csf_scheduler_wait_mcu_active(kbdev))
				dev_warn(
					kbdev->dev,
					"Wait for MCU active failed when copying suspend buffer for group %d of ctx %d_%d on slot %d",
					group->handle, group->kctx->tgid, group->kctx->id,
					group->csg_nr);
		}

		/* Check the group state again as scheduler lock would have been
		 * released when waiting for the exit from SLEEPING state.
		 */
		on_slot = kbasep_csf_scheduler_group_is_on_slot_locked(group);
	}
#endif
	if (on_slot) {
		DECLARE_BITMAP(slot_mask, MAX_SUPPORTED_CSGS) = { 0 };

		set_bit(kbase_csf_scheduler_group_get_slot(group), slot_mask);

		if (!WARN_ON(scheduler->state == SCHED_SUSPENDED))
			suspend_queue_group(group);
		err = wait_csg_slots_suspend(kbdev, slot_mask);
		if (err) {
			pixel_gpu_uevent_kmd_error_send(kbdev, GPU_UEVENT_INFO_CSG_GROUP_SUSPEND);
			dev_warn(kbdev->dev,
				 "[%llu] Timeout waiting for the group %d to suspend on slot %d",
				 kbase_backend_get_cycle_cnt(kbdev), group->handle, group->csg_nr);
			goto exit;
		}
	}

	if (queue_group_suspended_locked(group)) {
		unsigned int target_page_nr = 0, i = 0;
		u64 offset = sus_buf->offset;
		size_t to_copy = sus_buf->size;
		const u32 csg_suspend_buf_nr_pages =
			PFN_UP(kbdev->csf.global_iface.groups[0].suspend_size);

		if (scheduler->state != SCHED_SUSPENDED) {
			/* Similar to the case of HW counters, need to flush
			 * the GPU L2 cache before reading from the suspend buffer
			 * pages as they are mapped and cached on GPU side.
			 * Flushing LSC is not done here, since only the flush of
			 * CSG suspend buffer contents is needed from the L2 cache.
			 */
			kbase_gpu_start_cache_clean(kbdev, GPU_COMMAND_CACHE_CLN_INV_L2);
			kbase_gpu_wait_cache_clean(kbdev);
		} else {
			/* Make sure power down transitions have completed,
			 * i.e. L2 has been powered off as that would ensure
			 * its contents are flushed to memory.
			 * This is needed as Scheduler doesn't wait for the
			 * power down to finish.
			 */
			kbase_pm_wait_for_desired_state(kbdev);
		}

		for (i = 0; i < csg_suspend_buf_nr_pages && target_page_nr < sus_buf->nr_pages;
		     i++) {
			struct page *pg = as_page(group->normal_suspend_buf.phy[i]);
			void *sus_page = kbase_kmap(pg);

			if (sus_page) {
				kbase_sync_single_for_cpu(kbdev, kbase_dma_addr(pg), PAGE_SIZE,
							  DMA_BIDIRECTIONAL);

				err = kbase_mem_copy_to_pinned_user_pages(sus_buf->pages, sus_page,
									  &to_copy,
									  sus_buf->nr_pages,
									  &target_page_nr, offset);
				kbase_kunmap(pg, sus_page);
				if (err)
					break;
			} else {
				err = -ENOMEM;
				break;
			}
		}
		schedule_in_cycle(group, false);
	} else {
		/* If addr-space fault, the group may have been evicted */
		err = -EIO;
	}

exit:
	rt_mutex_unlock(&scheduler->lock);
	return err;
}

KBASE_EXPORT_TEST_API(kbase_csf_scheduler_group_copy_suspend_buf);

/**
 * group_sync_updated() - Evaluate sync wait condition of all blocked command
 *                        queues of the group.
 *
 * @group: Pointer to the command queue group that has blocked command queue(s)
 *         bound to it.
 *
 * Return: true if sync wait condition is satisfied for at least one blocked
 *         queue of the group.
 */
static bool group_sync_updated(struct kbase_queue_group *group)
{
	bool updated = false;
	int stream;

	/* Groups can also be blocked on-slot during protected mode. */
	WARN_ON(group->run_state != KBASE_CSF_GROUP_SUSPENDED_ON_WAIT_SYNC &&
		group->run_state != KBASE_CSF_GROUP_IDLE);

	for (stream = 0; stream < MAX_SUPPORTED_STREAMS_PER_GROUP; ++stream) {
		struct kbase_queue *const queue = group->bound_queues[stream];

		/* To check the necessity of sync-wait evaluation,
		 * we rely on the cached 'status_wait' instead of reading it
		 * directly from shared memory as the CSG has been already
		 * evicted from the CSG slot, thus this CSG doesn't have
		 * valid information in the shared memory.
		 */
		if (queue && queue->enabled && CS_STATUS_WAIT_SYNC_WAIT_GET(queue->status_wait))
			if (evaluate_sync_update(queue)) {
				updated = true;
				queue->status_wait = 0;
			}
	}

	return updated;
}

/**
 * scheduler_get_protm_enter_async_group() -  Check if the GPU queue group
 *                          can be now allowed to execute in protected mode.
 *
 * @kbdev:    Pointer to the GPU device.
 * @group:    Pointer to the GPU queue group.
 *
 * This function is called outside the scheduling tick/tock to determine
 * if the given GPU queue group can now execute in protected mode or not.
 * If the group pointer passed is NULL then the evaluation is done for the
 * highest priority group on the scheduler maintained group lists without
 * tick associated rotation actions. This is referred as the 'top-group'
 * in a tock action sense.
 *
 * It returns the same group pointer, that was passed as an argument, if that
 * group matches the highest priority group and has pending protected region
 * requests otherwise NULL is returned.
 *
 * If the group pointer passed is NULL then the internal evaluated highest
 * priority group is returned if that has pending protected region requests
 * otherwise NULL is returned.
 *
 * The evaluated highest priority group may not necessarily be the same as the
 * scheduler->top_grp. This can happen if there is dynamic de-idle update
 * during the tick interval for some on-slots groups that were idle during the
 * scheduler normal scheduling action, where the scheduler->top_grp was set.
 * The recorded scheduler->top_grp is untouched by this evualuation, so will not
 * affect the scheduler context/priority list rotation arrangement.
 *
 * Return: the pointer to queue group that can currently execute in protected
 *         mode or NULL.
 */
static struct kbase_queue_group *
scheduler_get_protm_enter_async_group(struct kbase_device *const kbdev,
				      struct kbase_queue_group *const group)
{
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;
	struct kbase_queue_group *match_grp, *input_grp;

	lockdep_assert_held(&scheduler->lock);

	if (scheduler->state != SCHED_INACTIVE)
		return NULL;

	match_grp = get_tock_top_group(scheduler);
	input_grp = group ? group : match_grp;

	if (input_grp && (input_grp == match_grp)) {
		struct kbase_csf_cmd_stream_group_info *ginfo = &kbdev->csf.global_iface.groups[0];
		unsigned long *pending = input_grp->protm_pending_bitmap;
		unsigned long flags;

		spin_lock_irqsave(&scheduler->interrupt_lock, flags);

		if (bitmap_empty(pending, ginfo->stream_num)) {
			dev_dbg(kbdev->dev,
				"Pmode requested for group %d of ctx %d_%d with no pending queues",
				input_grp->handle, input_grp->kctx->tgid, input_grp->kctx->id);
			input_grp = NULL;
		} else if (kbase_csf_scheduler_protected_mode_in_use(kbdev)) {
			kbase_csf_scheduler_invoke_tock(kbdev);
			input_grp = NULL;
		}

		spin_unlock_irqrestore(&scheduler->interrupt_lock, flags);
	} else {
		if (group && (group->priority == KBASE_QUEUE_GROUP_PRIORITY_REALTIME))
			kbase_csf_scheduler_invoke_tock(kbdev);

		input_grp = NULL;
	}

	return input_grp;
}

void kbase_csf_scheduler_group_protm_enter(struct kbase_queue_group *group)
{
	struct kbase_device *const kbdev = group->kctx->kbdev;
	struct kbase_csf_scheduler *const scheduler = &kbdev->csf.scheduler;

	int err = kbase_reset_gpu_try_prevent(kbdev);
	/* Regardless of whether reset failed or is currently happening, exit
	 * early
	 */
	if (err)
		return;

	rt_mutex_lock(&scheduler->lock);

	if (on_slot_group_idle_locked(group))
		update_idle_protm_group_state_to_runnable(group);
	/* Check if the group is now eligible for execution in protected mode. */
	if (scheduler_get_protm_enter_async_group(kbdev, group))
		scheduler_group_check_protm_enter(kbdev, group);

	rt_mutex_unlock(&scheduler->lock);
	kbase_reset_gpu_allow(kbdev);
}

/**
 * check_sync_update_for_on_slot_group() - Check the sync wait condition
 *                                         for all the queues bound to
 *                                         the given on-slot group.
 *
 * @group:    Pointer to the on-slot group that requires evaluation.
 *
 * This function is called if the GPU is in protected mode and there are on
 * slot idle groups with higher priority than the active protected mode group
 * or this function is called when CQS object is signaled whilst GPU is in
 * sleep state.
 * This function will evaluate the sync condition, if any, of all the queues
 * bound to the given group.
 *
 * Return: true if the sync condition of at least one queue has been satisfied.
 */
static bool check_sync_update_for_on_slot_group(struct kbase_queue_group *group)
{
	struct kbase_device *const kbdev = group->kctx->kbdev;
	struct kbase_csf_scheduler *const scheduler = &kbdev->csf.scheduler;
	bool sync_update_done = false;
	int i;
	unsigned long flags;

	lockdep_assert_held(&scheduler->lock);

	for (i = 0; i < MAX_SUPPORTED_STREAMS_PER_GROUP; i++) {
		struct kbase_queue *queue = group->bound_queues[i];

		if (queue && queue->enabled && !sync_update_done) {
			u32 group_id = group->csg_nr;
			u32 stream_id = queue->csi_index;
			u32 status;

			status = kbase_csf_fw_io_stream_read(&kbdev->csf.fw_io, group_id, stream_id,
							     CS_STATUS_WAIT);

			KBASE_KTRACE_ADD_CSF_GRP_Q(kbdev, QUEUE_SYNC_UPDATE_WAIT_STATUS,
						   queue->group, queue, status);

			if (!CS_STATUS_WAIT_SYNC_WAIT_GET(status))
				continue;

			/* Save the information of sync object of the command
			 * queue so the callback function, 'group_sync_updated'
			 * can evaluate the sync object when it gets updated
			 * later.
			 */
			queue->status_wait = status;
			queue->sync_ptr =
				kbase_csf_fw_io_stream_read(&kbdev->csf.fw_io, group_id, stream_id,
							    CS_STATUS_WAIT_SYNC_POINTER_LO);
			queue->sync_ptr |= (u64)kbase_csf_fw_io_stream_read(
						   &kbdev->csf.fw_io, group_id, stream_id,
						   CS_STATUS_WAIT_SYNC_POINTER_HI)
					   << 32;
			queue->sync_value = kbase_csf_fw_io_stream_read(
				&kbdev->csf.fw_io, group_id, stream_id, CS_STATUS_WAIT_SYNC_VALUE);
			queue->blocked_reason = CS_STATUS_BLOCKED_REASON_REASON_GET(
				kbase_csf_fw_io_stream_read(&kbdev->csf.fw_io, group_id, stream_id,
							    CS_STATUS_BLOCKED_REASON));

			if (!evaluate_sync_update(queue))
				continue;

			/* Update csg_slots_idle_mask and group's run_state */
			if (group->run_state != KBASE_CSF_GROUP_RUNNABLE) {
				/* Only clear the group's idle flag if it has been dealt
				 * with by the scheduler's tick/tock action, otherwise
				 * leave it untouched.
				 */
				spin_lock_irqsave(&scheduler->interrupt_lock, flags);
				clear_bit((unsigned int)group_id, scheduler->csg_slots_idle_mask);
				KBASE_KTRACE_ADD_CSF_GRP(kbdev, CSG_SLOT_IDLE_CLEAR, group,
							 scheduler->csg_slots_idle_mask[0]);
				spin_unlock_irqrestore(&scheduler->interrupt_lock, flags);
				/* Request the scheduler to confirm the condition inferred
				 * here inside the protected mode.
				 */
				group->reevaluate_idle_status = true;
				group->run_state = KBASE_CSF_GROUP_RUNNABLE;
				KBASE_KTRACE_ADD_CSF_GRP(kbdev, CSF_GROUP_RUNNABLE, group,
							 group->run_state);
			}

			KBASE_KTRACE_ADD_CSF_GRP(kbdev, GROUP_SYNC_UPDATE_DONE, group, 0u);
			sync_update_done = true;
		}
	}

	return sync_update_done;
}

/**
 * check_sync_update_for_idle_groups_protm() - Check the sync wait condition
 *                                             for the idle groups on slot
 *                                             during protected mode.
 *
 * @kbdev:    Pointer to the GPU device
 *
 * This function checks the gpu queues of all the idle groups on slot during
 * protected mode that has a higher priority than the active protected mode
 * group.
 *
 * Return: true if the sync condition of at least one queue in a group has been
 * satisfied.
 */
static bool check_sync_update_for_idle_groups_protm(struct kbase_device *kbdev)
{
	struct kbase_csf_scheduler *const scheduler = &kbdev->csf.scheduler;
	struct kbase_queue_group *protm_grp;
	bool exit_protm = false;
	unsigned long flags;
	u32 num_groups;
	u32 i;

	lockdep_assert_held(&scheduler->lock);

	spin_lock_irqsave(&scheduler->interrupt_lock, flags);
	protm_grp = scheduler->active_protm_grp;
	spin_unlock_irqrestore(&scheduler->interrupt_lock, flags);

	if (!protm_grp)
		return exit_protm;

	num_groups = kbdev->csf.global_iface.group_num;

	for_each_set_bit(i, scheduler->csg_slots_idle_mask, num_groups) {
		struct kbase_csf_csg_slot *csg_slot = &scheduler->csg_slots[i];
		struct kbase_queue_group *group = csg_slot->resident_group;

		if (group->scan_seq_num < protm_grp->scan_seq_num) {
			/* If sync update has been performed for the group that
			 * has a higher priority than the protm group, then we
			 * need to exit protected mode.
			 */
			if (check_sync_update_for_on_slot_group(group))
				exit_protm = true;
		}
	}

	return exit_protm;
}

/**
 * wait_for_mcu_sleep_before_sync_update_check() - Wait for MCU sleep request to
 *                                    complete before performing the sync update
 *                                    check for all the on-slot groups.
 *
 * @kbdev:    Pointer to the GPU device
 *
 * This function is called before performing the sync update check on the GPU queues
 * of all the on-slot groups when a CQS object is signaled and Scheduler was in
 * SLEEPING state.
 */
static void wait_for_mcu_sleep_before_sync_update_check(struct kbase_device *kbdev)
{
	/* Handling of sleep request should be relatively quick as GPU
	 * expected to be idle but CSG(s) could become active by the time
	 * FW starts handling the sleep request. So CSG suspend timeout is
	 * a more appropriate choice here for the timeout value.
	 */
	long timeout = kbase_csf_timeout_in_jiffies(kbdev->csf.csg_suspend_timeout_ms);
	bool can_wait_for_mcu_sleep;
	unsigned long flags;

	lockdep_assert_held(&kbdev->csf.scheduler.lock);

	if (WARN_ON_ONCE(kbdev->csf.scheduler.state != SCHED_SLEEPING))
		return;

	spin_lock_irqsave(&kbdev->hwaccess_lock, flags);
	/* If exit from sleep state has already been triggered then there is no need
	 * to wait, as scheduling is anyways going to be resumed and also MCU would
	 * have already transitioned to the sleep state.
	 * Also there is no need to wait if the Scheduler's PM refcount is not zero,
	 * which implies that MCU needs to be turned on.
	 */
	can_wait_for_mcu_sleep = !kbdev->pm.backend.exit_gpu_sleep_mode &&
				 !kbdev->csf.scheduler.pm_active_count;
	spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);
	if (!can_wait_for_mcu_sleep)
		return;

	/* Wait until MCU enters sleep state or there is a pending GPU reset */
	if (!wait_event_timeout(kbdev->pm.backend.gpu_in_desired_state_wait,
				kbase_csf_firmware_mcu_halted(kbdev) ||
					kbdev->pm.backend.exit_gpu_sleep_mode ||
					!kbase_reset_gpu_is_not_pending(kbdev),
				timeout))
		dev_warn(kbdev->dev, "Wait for MCU sleep timed out");
}

static void check_sync_update_in_sleep_mode(struct kbase_device *kbdev)
{
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;
	u32 const num_groups = kbdev->csf.global_iface.group_num;
	u32 csg_nr;

	lockdep_assert_held(&scheduler->lock);

	/* Wait for MCU to enter the sleep state to ensure that FW has published
	 * the status of CSGs/CSIs, otherwise we can miss detecting that a GPU
	 * queue stuck on SYNC_WAIT has been unblocked.
	 */
	wait_for_mcu_sleep_before_sync_update_check(kbdev);

	for (csg_nr = 0; csg_nr < num_groups; csg_nr++) {
		struct kbase_queue_group *const group =
			kbdev->csf.scheduler.csg_slots[csg_nr].resident_group;

		if (!group)
			continue;

		if (check_sync_update_for_on_slot_group(group)) {
			/* SYNC_UPDATE event shall invalidate GPU idle event */
			atomic_set(&scheduler->gpu_no_longer_idle, true);
			scheduler_wakeup(kbdev, true);
			return;
		}
	}
}

/**
 * check_group_sync_update_worker() - Check the sync wait condition for all the
 *                                    blocked queue groups
 *
 * @kctx: The context to evaluate the wait condition for all the queue groups
 *        in idle_wait_groups list.
 *
 * This function checks the gpu queues of all the groups present in both
 * idle_wait_groups list of a context and all on slot idle groups (if GPU
 * is in protected mode).
 * If the sync wait condition for at least one queue bound to the group has
 * been satisfied then the group is moved to the per context list of
 * runnable groups so that Scheduler can consider scheduling the group
 * in next tick or exit protected mode.
 */
static void check_group_sync_update_worker(struct kbase_context *kctx)
{
	struct kbase_device *const kbdev = kctx->kbdev;
	struct kbase_csf_scheduler *const scheduler = &kbdev->csf.scheduler;
	bool sync_updated = false;

	rt_mutex_lock(&scheduler->lock);

	KBASE_KTRACE_ADD(kbdev, SCHEDULER_GROUP_SYNC_UPDATE_WORKER_START, kctx, 0u);
	if (kctx->csf.sched.num_idle_wait_grps != 0) {
		struct kbase_queue_group *group, *temp;

		list_for_each_entry_safe(group, temp, &kctx->csf.sched.idle_wait_groups, link) {
			if (group_sync_updated(group)) {
				sync_updated = true;
				/* Move this group back in to the runnable
				 * groups list of the context.
				 */
				update_idle_suspended_group_state(group);
				KBASE_KTRACE_ADD_CSF_GRP(kbdev, GROUP_SYNC_UPDATE_DONE, group, 0u);
			}
		}
	} else {
		WARN_ON(!list_empty(&kctx->csf.sched.idle_wait_groups));
	}

	if (check_sync_update_for_idle_groups_protm(kbdev)) {
		scheduler_force_protm_exit(kbdev);
		sync_updated = true;
	}

	/* If scheduler is in sleep or suspended state, re-activate it
	 * to serve on-slot CSGs blocked on CQS which has been signaled.
	 */
	if (!sync_updated && (scheduler->state == SCHED_SLEEPING)) {
		/* Wait for sleep transition to complete to ensure the
		 * CS_STATUS_WAIT registers are updated by the MCU.
		 */
		check_sync_update_in_sleep_mode(kbdev);
	}

	KBASE_KTRACE_ADD(kbdev, SCHEDULER_GROUP_SYNC_UPDATE_WORKER_END, kctx, 0u);

	rt_mutex_unlock(&scheduler->lock);
}

static enum kbase_csf_event_callback_action check_group_sync_update_cb(void *param)
{
	struct kbase_context *const kctx = param;

	KBASE_KTRACE_ADD(kctx->kbdev, SCHEDULER_GROUP_SYNC_UPDATE_EVENT, kctx, 0u);

	kbase_csf_scheduler_enqueue_sync_update_work(kctx);

	return KBASE_CSF_EVENT_CALLBACK_KEEP;
}

int kbase_csf_scheduler_context_init(struct kbase_context *kctx)
{
	int priority;
	int err;
	struct kbase_device *kbdev = kctx->kbdev;

	WARN_ON_ONCE(!kbdev->csf.scheduler.kthread_running);

#if IS_ENABLED(CONFIG_MALI_TRACE_POWER_GPU_WORK_PERIOD)
	err = gpu_metrics_ctx_init(kctx);
	if (err)
		return err;
#endif /* CONFIG_MALI_TRACE_POWER_GPU_WORK_PERIOD */

	for (priority = 0; priority < KBASE_QUEUE_GROUP_PRIORITY_COUNT; ++priority) {
		INIT_LIST_HEAD(&kctx->csf.sched.runnable_groups[priority]);
	}

	INIT_LIST_HEAD(&kctx->csf.sched.idle_wait_groups);

	INIT_LIST_HEAD(&kctx->csf.sched.sync_update_work);

	kbase_csf_tiler_heap_reclaim_ctx_init(kctx);

	err = kbase_csf_event_wait_add(kctx, check_group_sync_update_cb, kctx);

	if (err) {
		dev_err(kbdev->dev, "Failed to register a sync update callback");
		goto event_wait_add_failed;
	}

	return err;

event_wait_add_failed:
	kbase_ctx_sched_remove_ctx(kctx);
#if IS_ENABLED(CONFIG_MALI_TRACE_POWER_GPU_WORK_PERIOD)
	gpu_metrics_ctx_term(kctx);
#endif /* CONFIG_MALI_TRACE_POWER_GPU_WORK_PERIOD */
	return err;
}

void kbase_csf_scheduler_context_term(struct kbase_context *kctx)
{
	kbase_csf_event_wait_remove(kctx, check_group_sync_update_cb, kctx);

	/* Drain a pending SYNC_UPDATE work if any */
	kbase_csf_scheduler_wait_for_kthread_pending_work(kctx->kbdev,
							  &kctx->csf.pending_sync_update);

	kbase_ctx_sched_remove_ctx(kctx);
#if IS_ENABLED(CONFIG_MALI_TRACE_POWER_GPU_WORK_PERIOD)
	gpu_metrics_ctx_term(kctx);
#endif /* CONFIG_MALI_TRACE_POWER_GPU_WORK_PERIOD */
}

static void handle_pending_sync_update_works(struct kbase_csf_scheduler *scheduler)
{
	struct kbase_context *sync_update_ctx;

	if (atomic_cmpxchg(&scheduler->pending_sync_update_works, true, false) == false)
		return;

	do {
		unsigned long flags;

		spin_lock_irqsave(&scheduler->sync_update_work_ctxs_lock, flags);
		sync_update_ctx = NULL;
		if (!list_empty(&scheduler->sync_update_work_ctxs)) {
			sync_update_ctx = list_first_entry(&scheduler->sync_update_work_ctxs,
							   struct kbase_context,
							   csf.sched.sync_update_work);
			list_del_init(&sync_update_ctx->csf.sched.sync_update_work);
		}
		spin_unlock_irqrestore(&scheduler->sync_update_work_ctxs_lock, flags);

		if (sync_update_ctx != NULL) {
			WARN_ON_ONCE(atomic_read(&sync_update_ctx->csf.pending_sync_update) == 0);
			check_group_sync_update_worker(sync_update_ctx);
			atomic_dec(&sync_update_ctx->csf.pending_sync_update);
		}
	} while (sync_update_ctx != NULL);
}

static void handle_pending_protm_requests(struct kbase_csf_scheduler *scheduler)
{
	struct kbase_queue_group *protm_grp;

	if (atomic_cmpxchg(&scheduler->pending_protm_event_works, true, false) == false)
		return;

	do {
		unsigned long flags;

		spin_lock_irqsave(&scheduler->protm_event_work_grps_lock, flags);
		protm_grp = NULL;
		if (!list_empty(&scheduler->protm_event_work_grps)) {
			protm_grp = list_first_entry(&scheduler->protm_event_work_grps,
						     struct kbase_queue_group, protm_event_work);
			list_del_init(&protm_grp->protm_event_work);
		}
		spin_unlock_irqrestore(&scheduler->protm_event_work_grps_lock, flags);

		if (protm_grp != NULL) {
			WARN_ON_ONCE(atomic_read(&protm_grp->pending_protm_event_work) == 0);
			kbase_csf_process_protm_event_request(protm_grp);
			atomic_dec(&protm_grp->pending_protm_event_work);
		}
	} while (protm_grp != NULL);
}

static void handle_pending_kcpuq_commands(struct kbase_csf_scheduler *scheduler)
{
	struct kbase_kcpu_command_queue *kcpuq;

	if (atomic_cmpxchg(&scheduler->pending_kcpuq_works, true, false) == false)
		return;

	do {
		unsigned long flags;

		spin_lock_irqsave(&scheduler->kcpuq_work_queues_lock, flags);
		kcpuq = NULL;
		if (!list_empty(&scheduler->kcpuq_work_queues)) {
			kcpuq = list_first_entry(&scheduler->kcpuq_work_queues,
						 struct kbase_kcpu_command_queue, high_prio_work);
			list_del_init(&kcpuq->high_prio_work);
		}
		spin_unlock_irqrestore(&scheduler->kcpuq_work_queues_lock, flags);

		if (kcpuq != NULL) {
			WARN_ON_ONCE(atomic_read(&kcpuq->pending_kick) == 0);

			mutex_lock(&kcpuq->lock);
			kbase_csf_kcpu_queue_process(kcpuq, false);
			mutex_unlock(&kcpuq->lock);

			atomic_dec(&kcpuq->pending_kick);
		}
	} while (kcpuq != NULL);
}

static void handle_pending_queue_kicks(struct kbase_device *kbdev)
{
	struct kbase_csf_scheduler *const scheduler = &kbdev->csf.scheduler;
	struct kbase_queue *queue;

	if (atomic_cmpxchg(&kbdev->csf.pending_gpuq_kicks, true, false) == false)
		return;

	do {
		u8 prio;

		spin_lock(&kbdev->csf.pending_gpuq_kick_queues_lock);
		queue = NULL;
		for (prio = 0; prio != KBASE_QUEUE_GROUP_PRIORITY_COUNT; ++prio) {
			if (!list_empty(&kbdev->csf.pending_gpuq_kick_queues[prio])) {
				queue = list_first_entry(&kbdev->csf.pending_gpuq_kick_queues[prio],
							 struct kbase_queue, pending_kick_link);
				list_del_init(&queue->pending_kick_link);
				break;
			}
		}
		spin_unlock(&kbdev->csf.pending_gpuq_kick_queues_lock);

		if (queue != NULL) {
			WARN_ONCE(
				prio != queue->group_priority,
				"Queue %pK has priority %u but instead its kick was handled at priority %u",
				(void *)queue, queue->group_priority, prio);
			WARN_ON_ONCE(atomic_read(&queue->pending_kick) == 0);

			kbase_csf_process_queue_kick(queue);

			/* Perform a scheduling tock for high-priority queue groups if
			 * required.
			 */
			BUILD_BUG_ON(KBASE_QUEUE_GROUP_PRIORITY_REALTIME != 0);
			BUILD_BUG_ON(KBASE_QUEUE_GROUP_PRIORITY_HIGH != 1);
			if ((prio <= KBASE_QUEUE_GROUP_PRIORITY_HIGH) &&
			    atomic_read(&scheduler->pending_tock_work))
				schedule_on_tock(kbdev);
		}
	} while (queue != NULL);
}

static int kbase_csf_scheduler_kthread(void *data)
{
	struct kbase_device *const kbdev = data;
	struct kbase_csf_scheduler *const scheduler = &kbdev->csf.scheduler;

	while (scheduler->kthread_running) {
		if (wait_for_completion_interruptible(&scheduler->kthread_signal) != 0)
			continue;
		reinit_completion(&scheduler->kthread_signal);

		/*
		 * The order in which these requests are handled is based on
		 * how they would influence each other's decisions. As a
		 * result, the tick & tock requests must be handled after all
		 * other requests, but before the GPU IDLE work.
		 */

		handle_pending_sync_update_works(scheduler);
		handle_pending_protm_requests(scheduler);
		handle_pending_kcpuq_commands(scheduler);
		handle_pending_queue_kicks(kbdev);

		/* Check if we need to perform a scheduling tick/tock. A tick
		 * event shall override a tock event but not vice-versa.
		 */
		if (atomic_cmpxchg(&scheduler->pending_tick_work, true, false) == true) {
			atomic_set(&scheduler->pending_tock_work, false);
			schedule_on_tick(kbdev);
		} else if (atomic_read(&scheduler->pending_tock_work)) {
			schedule_on_tock(kbdev);
		}

		/* Drain pending GPU idle works */
		while (atomic_read(&scheduler->pending_gpu_idle_work) > 0)
			gpu_idle_worker(kbdev);

		/* Update GLB_IDLE timer/FW Sleep-on-Idle config (which might
		 * have been disabled during FW boot et. al.).
		 */
		kbase_csf_firmware_soi_update(kbdev);
		kbase_csf_firmware_glb_idle_timer_update(kbdev);

		dev_dbg(kbdev->dev, "Waking up for event after a scheduling iteration.");
		wake_up_all(&kbdev->csf.event_wait);

		/* Inform platform of scheduling event */
		kbasep_platform_event_tick_tock(kbdev);
	}

	/* Wait for the other thread, that signaled the exit, to call kthread_stop() */
	while (!kthread_should_stop())
		;

	return 0;
}

int kbase_csf_scheduler_init(struct kbase_device *kbdev)
{
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;
	u32 num_groups = kbdev->csf.global_iface.group_num;

	bitmap_zero(scheduler->csg_inuse_bitmap, num_groups);
	bitmap_zero(scheduler->csg_slots_idle_mask, num_groups);

	scheduler->csg_slots = kcalloc(num_groups, sizeof(*scheduler->csg_slots), GFP_KERNEL);
	if (!scheduler->csg_slots) {
		dev_err(kbdev->dev, "Failed to allocate memory for csg slot status array\n");
		return -ENOMEM;
	}

	init_completion(&scheduler->kthread_signal);
	scheduler->kthread_running = true;
	scheduler->gpuq_kthread =
		kbase_kthread_run_rt(kbdev, &kbase_csf_scheduler_kthread, kbdev, "mali-gpuq-kthread");
	if (IS_ERR_OR_NULL(scheduler->gpuq_kthread)) {
		kfree(scheduler->csg_slots);
		scheduler->csg_slots = NULL;

		dev_err(kbdev->dev, "Failed to spawn the GPU queue submission worker thread");
		return -ENOMEM;
	}

#if IS_ENABLED(CONFIG_MALI_TRACE_POWER_GPU_WORK_PERIOD)
#if !IS_ENABLED(CONFIG_MALI_NO_MALI)
	scheduler->gpu_metrics_tb =
		kbase_csf_firmware_get_trace_buffer(kbdev, KBASE_CSFFW_GPU_METRICS_BUF_NAME);
	if (!scheduler->gpu_metrics_tb) {
		scheduler->kthread_running = false;
		complete(&scheduler->kthread_signal);
		kthread_stop(scheduler->gpuq_kthread);
		scheduler->gpuq_kthread = NULL;

		kfree(scheduler->csg_slots);
		scheduler->csg_slots = NULL;

		dev_err(kbdev->dev, "Failed to get the handler of gpu_metrics from trace buffer");
		return -ENOENT;
	}
#endif /* !CONFIG_MALI_NO_MALI */

	spin_lock_init(&scheduler->gpu_metrics_lock);
	hrtimer_init(&scheduler->gpu_metrics_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL_SOFT);
	scheduler->gpu_metrics_timer.function = gpu_metrics_timer_callback;
#endif /* CONFIG_MALI_TRACE_POWER_GPU_WORK_PERIOD */

	atomic_set(&scheduler->gpu_idle_timer_enabled, false);

	return kbase_csf_mcu_shared_regs_data_init(kbdev);
}

int kbase_csf_scheduler_early_init(struct kbase_device *kbdev)
{
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;

	atomic_set(&scheduler->timer_enabled, true);

	INIT_DEFERRABLE_WORK(&scheduler->ping_work, firmware_aliveness_monitor);

	rt_mutex_init(&scheduler->lock);
	spin_lock_init(&scheduler->interrupt_lock);

	/* Internal lists */
	INIT_LIST_HEAD(&scheduler->runnable_kctxs);
	INIT_LIST_HEAD(&scheduler->groups_to_schedule);
	INIT_LIST_HEAD(&scheduler->idle_groups_to_schedule);

	BUILD_BUG_ON(MAX_SUPPORTED_CSGS >
		     (sizeof(scheduler->csgs_events_enable_mask) * BITS_PER_BYTE));
	bitmap_fill(scheduler->csgs_events_enable_mask, MAX_SUPPORTED_CSGS);
	scheduler->state = SCHED_SUSPENDED;
	KBASE_KTRACE_ADD(kbdev, SCHED_SUSPENDED, NULL, scheduler->state);
	scheduler->csg_scheduling_period_ms = CSF_SCHEDULER_TIME_TICK_MS;
	scheduler_doorbell_init(kbdev);
	hrtimer_init(&scheduler->tick_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	scheduler->tick_timer.function = tick_timer_callback;

	atomic_set(&scheduler->pending_sync_update_works, false);
	spin_lock_init(&scheduler->sync_update_work_ctxs_lock);
	INIT_LIST_HEAD(&scheduler->sync_update_work_ctxs);
	atomic_set(&scheduler->pending_protm_event_works, false);
	spin_lock_init(&scheduler->protm_event_work_grps_lock);
	INIT_LIST_HEAD(&scheduler->protm_event_work_grps);
	atomic_set(&scheduler->pending_kcpuq_works, false);
	spin_lock_init(&scheduler->kcpuq_work_queues_lock);
	INIT_LIST_HEAD(&scheduler->kcpuq_work_queues);
	atomic_set(&scheduler->pending_tick_work, false);
	atomic_set(&scheduler->pending_tock_work, false);
	atomic_set(&scheduler->pending_gpu_idle_work, 0);

	return kbase_csf_tiler_heap_reclaim_mgr_init(kbdev);
}

void kbase_csf_scheduler_term(struct kbase_device *kbdev)
{
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;

	if (!IS_ERR_OR_NULL(scheduler->gpuq_kthread)) {
		scheduler->kthread_running = false;
		complete(&scheduler->kthread_signal);
		kthread_stop(scheduler->gpuq_kthread);
	}

	if (kbdev->csf.scheduler.csg_slots) {
		WARN_ON(atomic_read(&kbdev->csf.scheduler.non_idle_offslot_grps));
		/* The unload of Driver can take place only when all contexts have
		 * been terminated. The groups that were not terminated by the User
		 * are terminated on context termination. So no CSGs are expected
		 * to be active at the time of Driver unload.
		 */
		WARN_ON(kbase_csf_scheduler_get_nr_active_csgs(kbdev));
		rt_mutex_lock(&kbdev->csf.scheduler.lock);

		if (kbdev->csf.scheduler.state != SCHED_SUSPENDED) {
			unsigned long flags;
			/* The power policy could prevent the Scheduler from
			 * getting suspended when GPU becomes idle.
			 */
			spin_lock_irqsave(&kbdev->hwaccess_lock, flags);
			WARN_ON(kbase_pm_idle_groups_sched_suspendable(kbdev));
			spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);
			scheduler_suspend(kbdev);
		}

		rt_mutex_unlock(&kbdev->csf.scheduler.lock);
		cancel_delayed_work_sync(&kbdev->csf.scheduler.ping_work);
		kfree(kbdev->csf.scheduler.csg_slots);
		kbdev->csf.scheduler.csg_slots = NULL;
	}
	KBASE_KTRACE_ADD_CSF_GRP(kbdev, CSF_GROUP_TERMINATED, NULL,
				 kbase_csf_scheduler_get_nr_active_csgs(kbdev));
	/* Terminating the MCU shared regions, following the release of slots */
	kbase_csf_mcu_shared_regs_data_term(kbdev);
}

void kbase_csf_scheduler_early_term(struct kbase_device *kbdev)
{
	kbase_csf_tiler_heap_reclaim_mgr_term(kbdev);
}

/**
 * scheduler_enable_tick_timer_nolock - Enable the scheduler tick timer.
 *
 * @kbdev: Instance of a GPU platform device that implements a CSF interface.
 *
 * This function will restart the scheduler tick so that regular scheduling can
 * be resumed without any explicit trigger (like kicking of GPU queues). This
 * is a variant of kbase_csf_scheduler_enable_tick_timer() that assumes the
 * CSF scheduler lock to already have been held.
 */
static void scheduler_enable_tick_timer_nolock(struct kbase_device *kbdev)
{
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;

	lockdep_assert_held(&kbdev->csf.scheduler.lock);

	if (unlikely(!kbase_csf_scheduler_timer_is_enabled(kbdev)))
		return;

	WARN_ON((scheduler->state != SCHED_INACTIVE) && (scheduler->state != SCHED_SUSPENDED) &&
		(scheduler->state != SCHED_SLEEPING));

	if (scheduler->total_runnable_grps > 0) {
		kbase_csf_scheduler_invoke_tick(kbdev);
		dev_dbg(kbdev->dev, "Re-enabling the scheduler timer\n");
	} else if (scheduler->state != SCHED_SUSPENDED) {
		enqueue_gpu_idle_work(scheduler);
	}
}

void kbase_csf_scheduler_enable_tick_timer(struct kbase_device *kbdev)
{
	rt_mutex_lock(&kbdev->csf.scheduler.lock);
	scheduler_enable_tick_timer_nolock(kbdev);
	rt_mutex_unlock(&kbdev->csf.scheduler.lock);
}

void kbase_csf_scheduler_timer_set_enabled(struct kbase_device *kbdev, bool enable)
{
	struct kbase_csf_scheduler *const scheduler = &kbdev->csf.scheduler;
	bool currently_enabled;

	/* This lock is taken to prevent this code being executed concurrently
	 * by userspace.
	 */
	rt_mutex_lock(&scheduler->lock);

	currently_enabled = kbase_csf_scheduler_timer_is_enabled(kbdev);
	if (currently_enabled && !enable) {
		atomic_set(&scheduler->timer_enabled, false);
		cancel_tick_work(scheduler);
	} else if (!currently_enabled && enable) {
		atomic_set(&scheduler->timer_enabled, true);
		kbase_csf_scheduler_invoke_tick(kbdev);
	}

	rt_mutex_unlock(&scheduler->lock);
}

void kbase_csf_scheduler_kick(struct kbase_device *kbdev)
{
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;

	if (unlikely(kbase_csf_scheduler_timer_is_enabled(kbdev)))
		return;

	/* This lock is taken to prevent this code being executed concurrently
	 * by userspace.
	 */
	rt_mutex_lock(&scheduler->lock);

	kbase_csf_scheduler_invoke_tick(kbdev);
	dev_dbg(kbdev->dev, "Kicking the scheduler manually\n");

	rt_mutex_unlock(&scheduler->lock);
}

int kbase_csf_scheduler_pm_suspend_no_lock(struct kbase_device *kbdev)
{
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;
	int result = 0;

	lockdep_assert_held(&scheduler->lock);

#if IS_ENABLED(CONFIG_DEBUG_FS)
	if (unlikely(scheduler->state == SCHED_BUSY))
		return -EBUSY;
#endif

#ifdef KBASE_PM_RUNTIME
	/* If scheduler is in sleeping state, then MCU needs to be activated
	 * to suspend CSGs.
	 */
	if (scheduler->state == SCHED_SLEEPING) {
		dev_info(kbdev->dev, "Activating MCU out of sleep on system suspend");
		result = force_scheduler_to_exit_sleep(kbdev);
		if (result) {
			dev_warn(kbdev->dev, "Scheduler failed to exit from sleep");
			goto exit;
		}
	}
#endif
	if (scheduler->state != SCHED_SUSPENDED) {
		result = suspend_active_groups_on_powerdown(kbdev, true);
		if (result) {
			dev_warn(kbdev->dev, "failed to suspend active groups");
			goto exit;
		} else {
			dev_dbg(kbdev->dev, "Scheduler PM suspend");
			result = scheduler_suspend(kbdev);
			if (!result)
				cancel_tick_work(scheduler);
		}
	}

exit:
	return result;
}

int kbase_csf_scheduler_pm_suspend(struct kbase_device *kbdev)
{
	int result = 0;
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;

	/* Cancel any potential queued delayed work(s) */
	cancel_tick_work(scheduler);
	cancel_tock_work(scheduler);

	result = kbase_reset_gpu_prevent_and_wait(kbdev);
	if (result) {
		dev_warn(kbdev->dev, "Stop PM suspending for failing to prevent gpu reset.\n");
		return result;
	}

	rt_mutex_lock(&scheduler->lock);

	result = kbase_csf_scheduler_pm_suspend_no_lock(kbdev);
	rt_mutex_unlock(&scheduler->lock);

	kbase_reset_gpu_allow(kbdev);

	return result;
}
KBASE_EXPORT_TEST_API(kbase_csf_scheduler_pm_suspend);

void kbase_csf_scheduler_pm_resume_no_lock(struct kbase_device *kbdev)
{
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;

	lockdep_assert_held(&scheduler->lock);
	if ((scheduler->total_runnable_grps > 0) && (scheduler->state == SCHED_SUSPENDED)) {
		dev_dbg(kbdev->dev, "Scheduler PM resume");
		scheduler_wakeup(kbdev, true);
	}
}

void kbase_csf_scheduler_pm_resume(struct kbase_device *kbdev)
{
	rt_mutex_lock(&kbdev->csf.scheduler.lock);
	kbase_csf_scheduler_pm_resume_no_lock(kbdev);
	rt_mutex_unlock(&kbdev->csf.scheduler.lock);
}
KBASE_EXPORT_TEST_API(kbase_csf_scheduler_pm_resume);

void kbase_csf_scheduler_pm_active(struct kbase_device *kbdev)
{
	/* Here the lock is taken to synchronize against the runtime suspend
	 * callback function, which may need to wake up the MCU for suspending
	 * the CSGs before powering down the GPU.
	 */
	rt_mutex_lock(&kbdev->csf.scheduler.lock);
	scheduler_pm_active_handle_suspend(kbdev, KBASE_PM_SUSPEND_HANDLER_NOT_POSSIBLE, false);
	rt_mutex_unlock(&kbdev->csf.scheduler.lock);
}
KBASE_EXPORT_TEST_API(kbase_csf_scheduler_pm_active);

void kbase_csf_scheduler_pm_idle(struct kbase_device *kbdev)
{
	/* Here the lock is taken just to maintain symmetry with
	 * kbase_csf_scheduler_pm_active().
	 */
	rt_mutex_lock(&kbdev->csf.scheduler.lock);
	scheduler_pm_idle(kbdev);
	rt_mutex_unlock(&kbdev->csf.scheduler.lock);
}
KBASE_EXPORT_TEST_API(kbase_csf_scheduler_pm_idle);

static int scheduler_wait_mcu_active(struct kbase_device *kbdev, bool killable_wait)
{
	struct kbase_csf_scheduler *const scheduler = &kbdev->csf.scheduler;
	unsigned long flags;
	int err;

	kbase_pm_lock(kbdev);
	WARN_ON(!kbdev->pm.active_count);
	WARN_ON(!scheduler->pm_active_count);
	kbase_pm_unlock(kbdev);

	if (killable_wait)
		err = kbase_pm_killable_wait_for_poweroff_work_complete(kbdev);
	else
		err = kbase_pm_wait_for_poweroff_work_complete(kbdev);
	if (err)
		return err;

	if (killable_wait)
		err = kbase_pm_killable_wait_for_desired_state(kbdev);
	else
		err = kbase_pm_wait_for_desired_state(kbdev);
	if (!err) {
		spin_lock_irqsave(&kbdev->hwaccess_lock, flags);
		WARN_ON(kbdev->pm.backend.mcu_state != KBASE_MCU_ON);
		spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);
	}

	return err;
}

int kbase_csf_scheduler_killable_wait_mcu_active(struct kbase_device *kbdev)
{
	return scheduler_wait_mcu_active(kbdev, true);
}

int kbase_csf_scheduler_wait_mcu_active(struct kbase_device *kbdev)
{
	return scheduler_wait_mcu_active(kbdev, false);
}

KBASE_EXPORT_TEST_API(kbase_csf_scheduler_wait_mcu_active);

#ifdef KBASE_PM_RUNTIME
int kbase_csf_scheduler_handle_runtime_suspend(struct kbase_device *kbdev)
{
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;
	unsigned long flags;
	int ret;

	dev_dbg(kbdev->dev, "Handling runtime suspend");

	kbase_reset_gpu_assert_prevented(kbdev);
	lockdep_assert_held(&scheduler->lock);
	WARN_ON(scheduler->pm_active_count);

	if (scheduler->state == SCHED_SUSPENDED) {
		WARN_ON(kbdev->pm.backend.gpu_sleep_mode_active);
		return 0;
	}

	ret = suspend_active_groups_on_powerdown(kbdev, false);

	if (ret) {
		dev_dbg(kbdev->dev, "Aborting runtime suspend (grps: %d)",
			atomic_read(&scheduler->non_idle_offslot_grps));

		spin_lock_irqsave(&kbdev->hwaccess_lock, flags);
		kbdev->pm.backend.exit_gpu_sleep_mode = true;
		kbdev->pm.backend.runtime_suspend_abort_reason = ABORT_REASON_NON_IDLE_CGS;
		spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);

		kbase_csf_scheduler_invoke_tick(kbdev);
		return ret;
	}

	scheduler->state = SCHED_SUSPENDED;
#if IS_ENABLED(CONFIG_MALI_TRACE_POWER_GPU_WORK_PERIOD)
	hrtimer_cancel(&scheduler->gpu_metrics_timer);
#endif
	KBASE_KTRACE_ADD(kbdev, SCHED_SUSPENDED, NULL, scheduler->state);
	spin_lock_irqsave(&kbdev->hwaccess_lock, flags);
	kbdev->pm.backend.gpu_sleep_mode_active = false;
	spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);

	wake_up_all(&kbdev->csf.event_wait);
	return 0;
}

void kbase_csf_scheduler_enqueue_sync_update_work(struct kbase_context *kctx)
{
	struct kbase_csf_scheduler *const scheduler = &kctx->kbdev->csf.scheduler;
	unsigned long flags;

	spin_lock_irqsave(&scheduler->sync_update_work_ctxs_lock, flags);
	if (list_empty(&kctx->csf.sched.sync_update_work)) {
		list_add_tail(&kctx->csf.sched.sync_update_work, &scheduler->sync_update_work_ctxs);
		atomic_inc(&kctx->csf.pending_sync_update);
		if (atomic_cmpxchg(&scheduler->pending_sync_update_works, false, true) == false)
			complete(&scheduler->kthread_signal);
	}
	spin_unlock_irqrestore(&scheduler->sync_update_work_ctxs_lock, flags);
}

void kbase_csf_scheduler_enqueue_protm_event_work(struct kbase_queue_group *group)
{
	struct kbase_context *const kctx = group->kctx;
	struct kbase_csf_scheduler *const scheduler = &kctx->kbdev->csf.scheduler;
	unsigned long flags;

	spin_lock_irqsave(&scheduler->protm_event_work_grps_lock, flags);
	if (list_empty(&group->protm_event_work)) {
		list_add_tail(&group->protm_event_work, &scheduler->protm_event_work_grps);
		atomic_inc(&group->pending_protm_event_work);
		if (atomic_cmpxchg(&scheduler->pending_protm_event_works, false, true) == false)
			complete(&scheduler->kthread_signal);
	}
	spin_unlock_irqrestore(&scheduler->protm_event_work_grps_lock, flags);
}

void kbase_csf_scheduler_enqueue_kcpuq_work(struct kbase_kcpu_command_queue *queue)
{
	struct kbase_csf_scheduler *const scheduler = &queue->kctx->kbdev->csf.scheduler;
	unsigned long flags;

	spin_lock_irqsave(&scheduler->kcpuq_work_queues_lock, flags);
	if (list_empty(&queue->high_prio_work)) {
		list_add_tail(&queue->high_prio_work, &scheduler->kcpuq_work_queues);
		atomic_inc(&queue->pending_kick);
		if (atomic_cmpxchg(&scheduler->pending_kcpuq_works, false, true) == false)
			complete(&scheduler->kthread_signal);
	}
	spin_unlock_irqrestore(&scheduler->kcpuq_work_queues_lock, flags);
}

void kbase_csf_scheduler_wait_for_kthread_pending_work(struct kbase_device *kbdev,
						       atomic_t *pending)
{
	/*
	 * Signal kbase_csf_scheduler_kthread() to allow for the
	 * eventual completion of the current iteration. Once the work is
	 * done, the event_wait wait queue shall be signalled.
	 */

	complete(&kbdev->csf.scheduler.kthread_signal);
	wait_event(kbdev->csf.event_wait, atomic_read(pending) == 0);
}

void kbase_csf_scheduler_reval_idleness_post_sleep(struct kbase_device *kbdev)
{
	u32 csg_nr;

	lockdep_assert_held(&kbdev->hwaccess_lock);

	WARN_ON(kbdev->pm.backend.mcu_state != KBASE_MCU_IN_SLEEP);

	for (csg_nr = 0; csg_nr < kbdev->csf.global_iface.group_num; csg_nr++) {
		bool csg_idle;

		if (!kbdev->csf.scheduler.csg_slots[csg_nr].resident_group)
			continue;

		csg_idle = kbase_csf_fw_io_group_read(&kbdev->csf.fw_io, csg_nr, CSG_STATUS_STATE) &
			   CSG_STATUS_STATE_IDLE_MASK;

		if (!csg_idle) {
			dev_dbg(kbdev->dev, "Re-activate Scheduler after MCU sleep");
			kbdev->pm.backend.exit_gpu_sleep_mode = true;
			kbase_csf_scheduler_invoke_tick(kbdev);
			break;
		}
	}
}

void kbase_csf_scheduler_force_sleep(struct kbase_device *kbdev)
{
	struct kbase_csf_scheduler *const scheduler = &kbdev->csf.scheduler;

	rt_mutex_lock(&scheduler->lock);
	if (kbase_pm_gpu_sleep_allowed(kbdev) && (scheduler->state == SCHED_INACTIVE))
		scheduler_sleep_on_idle(kbdev);
	rt_mutex_unlock(&scheduler->lock);
}
#endif

void kbase_csf_scheduler_force_wakeup(struct kbase_device *kbdev)
{
	struct kbase_csf_scheduler *const scheduler = &kbdev->csf.scheduler;

	rt_mutex_lock(&scheduler->lock);
	scheduler_wakeup(kbdev, true);
	rt_mutex_unlock(&scheduler->lock);
}
KBASE_EXPORT_TEST_API(kbase_csf_scheduler_force_wakeup);

