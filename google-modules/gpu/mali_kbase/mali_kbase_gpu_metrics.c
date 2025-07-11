// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2023-2024 ARM Limited. All rights reserved.
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

#if IS_ENABLED(CONFIG_MALI_TRACE_POWER_GPU_WORK_PERIOD)

#include "mali_power_gpu_work_period_trace.h"
#include <mali_kbase_gpu_metrics.h>
#include <mali_kbase_config_defaults.h>
#include <mali_kbase.h>

#include <linux/module.h>
#include <linux/slab.h>

static unsigned long gpu_metrics_tp_emit_interval_ns = DEFAULT_GPU_METRICS_TP_EMIT_INTERVAL_NS;

module_param(gpu_metrics_tp_emit_interval_ns, ulong, 0444);
MODULE_PARM_DESC(gpu_metrics_tp_emit_interval_ns,
		 "Time interval in nano seconds at which GPU metrics tracepoints are emitted");

static inline void validate_tracepoint_data(struct kbase_gpu_metrics_ctx *gpu_metrics_ctx,
					    u64 start_time, u64 end_time, u64 total_active)
{
#ifdef CONFIG_MALI_DEBUG
	WARN(start_time >= end_time, "start_time %llu >= end_time %llu for aid %u active_cnt %u",
	     start_time, end_time, gpu_metrics_ctx->aid, gpu_metrics_ctx->active_cnt);

	WARN(total_active > (end_time - start_time),
	     "total_active %llu > end_time %llu - start_time %llu for aid %u active_cnt %u",
	     total_active, end_time, start_time, gpu_metrics_ctx->aid, gpu_metrics_ctx->active_cnt);
#endif
}

static u64 get_last_cycle_count(struct kbase_device *kbdev)
{
	u64 cycles = kbase_backend_get_cycle_cnt(kbdev);
#if !MALI_USE_CSF
	if (kbdev->pm.backend.l2_state != KBASE_L2_ON)
	{
		cycles = kbdev->last_cycle_count;
	}
#endif
	return cycles;
}

static void emit_tracepoint_for_active_gpu_metrics_ctx(
	struct kbase_device *kbdev, struct kbase_gpu_metrics_ctx *gpu_metrics_ctx, u64 current_time)
{
	const u64 start_time = gpu_metrics_ctx->active_start_time;
	u64 total_active, end_time = current_time;
	const u64 start_cycles = gpu_metrics_ctx->active_start_cycles;
	u64 total_cycles, end_cycles = get_last_cycle_count(kbdev);

	/* Check if the GPU activity is currently ongoing */
	if (gpu_metrics_ctx->active_cnt) {
		/* The following check is to handle the race on CSF GPUs that can happen between
		 * the draining of trace buffer and FW emitting the ACT=1 event .
		 */
		if (unlikely(end_time == start_time))
			end_time++;
		gpu_metrics_ctx->active_start_time = end_time;
		gpu_metrics_ctx->active_start_cycles = end_cycles;
	}

	total_active = end_time - start_time;
	total_cycles = end_cycles - start_cycles;
	trace_gpu_work_period(kbdev->id, gpu_metrics_ctx->aid, start_time, end_time, total_active);

	if (kbdev->platform_context) {
		struct pixel_context *pc = kbdev->platform_context;
		struct gpu_dvfs_metrics_uid_stats *entry = NULL;
		unsigned long flags;
		u8 uid_hash = gpu_dvfs_hash_uid_stats(gpu_metrics_ctx->aid);

		/* Grab the lock before modifying the entry upon a work period event. */
		spin_lock_irqsave(&pc->dvfs.metrics.lock, flags);
		hash_for_each_possible(
				pc->dvfs.metrics.uid_stats_table,
				entry,
				uid_list_node, uid_hash) {
			if(__kuid_val(entry->uid) == gpu_metrics_ctx->aid) {
				entry->gpu_cycles_last += total_cycles;
				entry->gpu_active_ns_last += total_active;
				entry->tis_stats[pc->dvfs.level].time_total += total_active;
				break;
			}
		}
		spin_unlock_irqrestore(&pc->dvfs.metrics.lock, flags);
	}

	validate_tracepoint_data(gpu_metrics_ctx, start_time, end_time, total_active);
	gpu_metrics_ctx->active_end_time = end_time;
}

void kbase_gpu_metrics_ctx_put(struct kbase_device *kbdev,
			       struct kbase_gpu_metrics_ctx *gpu_metrics_ctx)
{
	WARN_ON(list_empty(&gpu_metrics_ctx->link));
	WARN_ON(!gpu_metrics_ctx->kctx_count);

	gpu_metrics_ctx->kctx_count--;
	if (gpu_metrics_ctx->kctx_count)
		return;

	/* Generate a tracepoint if there's still activity */
	if (gpu_metrics_ctx->active_cnt)
		emit_tracepoint_for_active_gpu_metrics_ctx(kbdev, gpu_metrics_ctx,
							   ktime_get_raw_ns());

	list_del_init(&gpu_metrics_ctx->link);
	kfree(gpu_metrics_ctx);
}

struct kbase_gpu_metrics_ctx *kbase_gpu_metrics_ctx_get(struct kbase_device *kbdev, u32 aid)
{
	struct kbase_gpu_metrics *gpu_metrics = &kbdev->gpu_metrics;
	struct kbase_gpu_metrics_ctx *gpu_metrics_ctx;

	list_for_each_entry(gpu_metrics_ctx, &gpu_metrics->active_list, link) {
		if (gpu_metrics_ctx->aid == aid) {
			WARN_ON(!gpu_metrics_ctx->kctx_count);
			gpu_metrics_ctx->kctx_count++;
			return gpu_metrics_ctx;
		}
	}

	list_for_each_entry(gpu_metrics_ctx, &gpu_metrics->inactive_list, link) {
		if (gpu_metrics_ctx->aid == aid) {
			WARN_ON(!gpu_metrics_ctx->kctx_count);
			gpu_metrics_ctx->kctx_count++;
			return gpu_metrics_ctx;
		}
	}

	return NULL;
}

void kbase_gpu_metrics_ctx_init(struct kbase_device *kbdev,
				struct kbase_gpu_metrics_ctx *gpu_metrics_ctx, unsigned int aid)
{
	gpu_metrics_ctx->active_start_time = 0;
	gpu_metrics_ctx->active_start_cycles = 0;
	gpu_metrics_ctx->active_end_time = 0;
	gpu_metrics_ctx->aid = aid;
	gpu_metrics_ctx->kctx_count = 1;
	gpu_metrics_ctx->active_cnt = 0;
	list_add_tail(&gpu_metrics_ctx->link, &kbdev->gpu_metrics.inactive_list);
}

void kbase_gpu_metrics_ctx_start_activity(struct kbase_context *kctx, u64 timestamp_ns)
{
	struct kbase_gpu_metrics_ctx *gpu_metrics_ctx = kctx->gpu_metrics_ctx;

	gpu_metrics_ctx->active_cnt++;
	if (gpu_metrics_ctx->active_cnt == 1) {
		gpu_metrics_ctx->active_start_time = timestamp_ns;
		gpu_metrics_ctx->active_start_cycles = get_last_cycle_count(kctx->kbdev);
		list_move_tail(&gpu_metrics_ctx->link, &kctx->kbdev->gpu_metrics.active_list);
	}
}

void kbase_gpu_metrics_ctx_end_activity(struct kbase_context *kctx, u64 timestamp_ns)
{
	struct kbase_gpu_metrics_ctx *gpu_metrics_ctx = kctx->gpu_metrics_ctx;

	if (WARN_ON_ONCE(!gpu_metrics_ctx->active_cnt))
		return;

	/* Do not emit tracepoint if GPU activity still continues. */
	if (--gpu_metrics_ctx->active_cnt)
		return;

	if (likely(timestamp_ns > gpu_metrics_ctx->active_start_time)) {
		emit_tracepoint_for_active_gpu_metrics_ctx(kctx->kbdev, gpu_metrics_ctx,
							   timestamp_ns);
		return;
	}

	/* Due to conversion from system timestamp to CPU timestamp (which involves rounding)
	 * the value for start and end timestamp could come as same on CSF GPUs.
	 */
	if (timestamp_ns == gpu_metrics_ctx->active_start_time) {
		emit_tracepoint_for_active_gpu_metrics_ctx(kctx->kbdev, gpu_metrics_ctx,
							   timestamp_ns + 1);
		return;
	}

	/* The following check is to detect the situation on CSF GPUs, where 'ACT=0' event was not
	 * visible to the Kbase even though the system timestamp value sampled by FW was less than
	 * the system timestamp value sampled by Kbase just before the draining of trace buffer.
	 */
	if (gpu_metrics_ctx->active_end_time == gpu_metrics_ctx->active_start_time) {
		emit_tracepoint_for_active_gpu_metrics_ctx(kctx->kbdev, gpu_metrics_ctx,
							   gpu_metrics_ctx->active_end_time + 1);
		return;
	}

	WARN_ON_ONCE(1);
}

void kbase_gpu_metrics_emit_tracepoint(struct kbase_device *kbdev, u64 ts)
{
	struct kbase_gpu_metrics *gpu_metrics = &kbdev->gpu_metrics;
	struct kbase_gpu_metrics_ctx *gpu_metrics_ctx, *tmp;

	list_for_each_entry_safe(gpu_metrics_ctx, tmp, &gpu_metrics->active_list, link) {
		if (gpu_metrics_ctx->active_cnt) {
			emit_tracepoint_for_active_gpu_metrics_ctx(kbdev, gpu_metrics_ctx, ts);
			continue;
		}

		list_move_tail(&gpu_metrics_ctx->link, &gpu_metrics->inactive_list);
	}
}

int kbase_gpu_metrics_init(struct kbase_device *kbdev)
{
	INIT_LIST_HEAD(&kbdev->gpu_metrics.active_list);
	INIT_LIST_HEAD(&kbdev->gpu_metrics.inactive_list);

	if (!gpu_metrics_tp_emit_interval_ns || (gpu_metrics_tp_emit_interval_ns >= NSEC_PER_SEC)) {
		dev_warn(
			kbdev->dev,
			"Invalid value (%lu ns) for module param gpu_metrics_tp_emit_interval_ns. Using default value: %u ns",
			gpu_metrics_tp_emit_interval_ns, DEFAULT_GPU_METRICS_TP_EMIT_INTERVAL_NS);
		gpu_metrics_tp_emit_interval_ns = DEFAULT_GPU_METRICS_TP_EMIT_INTERVAL_NS;
	}

	dev_info(kbdev->dev, "GPU metrics tracepoint support enabled");
	return 0;
}

void kbase_gpu_metrics_term(struct kbase_device *kbdev)
{
	WARN_ON_ONCE(!list_empty(&kbdev->gpu_metrics.active_list));
	WARN_ON_ONCE(!list_empty(&kbdev->gpu_metrics.inactive_list));
}

unsigned long kbase_gpu_metrics_get_tp_emit_interval(void)
{
	return gpu_metrics_tp_emit_interval_ns;
}

#endif
