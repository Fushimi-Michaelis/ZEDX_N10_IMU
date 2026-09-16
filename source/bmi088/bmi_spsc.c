// SPDX-License-Identifier: GPL-2.0-only
// SPDX-FileCopyrightText: Copyright (c) 2024, STEREOLABS. All rights reserved.

/**
 * @file bmi_spsc_core.c
 * @brief BMI088 SPSC Ring Buffer Core Implementation
 * 
 * Phase 1.1: Extract SPSC ring buffer code into reusable module
 * This file implements the core SPSC ring buffer functionality that will
 * be used by the BMI088 SPSC driver implementation.
 */

#include "bmi_spsc.h"
#include "bmi088_core.h"

#define BMI_SPSC_CLASS_NAME "bmi_spsc"
#define BMI_SPSC_DEVICE_NAME "spsc_bmi"

/* Global state for device management */
static dev_t bmi_spsc_devno_base;
static struct class *bmi_spsc_class;
static int bmi_spsc_major = 0;

/* Timer configuration for 200Hz data acquisition */
#define BMI_SPSC_TIMER_PERIOD_NS (5000000)  /* 5ms = 200Hz */

/* Ring buffer mmap size: 8 pages = 32KB (order 3 allocation) */
#define BMI_SPSC_MMAP_ORDER     3
#define BMI_SPSC_MMAP_SIZE      (PAGE_SIZE << BMI_SPSC_MMAP_ORDER)  /* 32KB */

/**
 * @brief Allocate and initialize SPSC ring buffer
 * @return Pointer to allocated ring buffer or NULL on failure
 */
struct spsc_ring *spsc_ring_alloc(void)
{
    struct spsc_ring *ring;
    
    /* Allocate ring buffer (8 pages = 32KB for ring + data) */
    ring = (struct spsc_ring *)__get_free_pages(GFP_KERNEL | __GFP_ZERO, BMI_SPSC_MMAP_ORDER);
    if (!ring) {
        pr_err("BMI_SPSC: Failed to allocate ring buffer memory\n");
        return NULL;
    }
    
    /* Initialize ring buffer state */
    memset(ring, 0, sizeof(struct spsc_ring));
    ring->head = 0;
    ring->tail = 0;
    
    // pr_info("BMI_SPSC: Allocated ring buffer at %px (size: %zu bytes)\n", ring, sizeof(struct spsc_ring));
    
    return ring;
}

/**
 * @brief Free SPSC ring buffer
 * @param ring Pointer to ring buffer to free
 */
void spsc_ring_free(struct spsc_ring *ring)
{
    if (ring) {
        // pr_info("BMI_SPSC: Freeing ring buffer at %px\n", ring);
        free_pages((unsigned long)ring, BMI_SPSC_MMAP_ORDER);
    }
}

/**
 * @brief Check if ring buffer is empty
 * @param ring Pointer to ring buffer
 * @return true if empty, false otherwise
 */
bool spsc_ring_empty(struct spsc_ring *ring)
{
    if (!ring)
        return true;
    return ring->head == ring->tail;
}

/**
 * @brief Check if ring buffer is full
 * @param ring Pointer to ring buffer
 * @return true if full, false otherwise
 */
bool spsc_ring_full(struct spsc_ring *ring)
{
    if (!ring)
        return true;
    return ((ring->head + 1) & (BMI_SPSC_BUFFER_SIZE - 1)) == ring->tail;
}

/**
 * @brief Get number of available entries in ring buffer
 * @param ring Pointer to ring buffer
 * @return Number of available entries
 */
__u32 spsc_ring_count(struct spsc_ring *ring)
{
    if (!ring)
        return 0;
    return (ring->head - ring->tail) & (BMI_SPSC_BUFFER_SIZE - 1);
}

/**
 * @brief Write data to SPSC ring buffer (producer side)
 * @param ring Pointer to ring buffer
 * @param data Pointer to data to write
 * @return 0 on success, -ENOSPC if buffer is full
 */
int spsc_ring_write(struct spsc_ring *ring, const struct bmi_sensor_data *data)
{
    __u32 next_head;
    
    if (!ring || !data)
        return -EINVAL;
    
    next_head = (ring->head + 1) & (BMI_SPSC_BUFFER_SIZE - 1);
    
    if (next_head == ring->tail)
        return -ENOSPC;  /* Buffer full */
    
    /* Copy data to ring buffer */
    ring->data[ring->head] = *data;
    
    /* Memory barrier to ensure data is written before head update */
    smp_wmb();
    
    /* Update head pointer */
    ring->head = next_head;
    
    return 0;
}

/**
 * @brief Read data from SPSC ring buffer (consumer side)
 * @param ring Pointer to ring buffer
 * @param data Pointer to buffer to store read data
 * @return 0 on success, -ENODATA if buffer is empty
 */
int spsc_ring_read(struct spsc_ring *ring, struct bmi_sensor_data *data)
{
    if (!ring || !data)
        return -EINVAL;
    
    if (ring->tail == ring->head)
        return -ENODATA;  /* Buffer empty */
    
    /* Copy data from ring buffer */
    *data = ring->data[ring->tail];
    
    /* Memory barrier to ensure data is read before tail update */
    smp_rmb();
    
    /* Update tail pointer */
    ring->tail = (ring->tail + 1) & (BMI_SPSC_BUFFER_SIZE - 1);
    
    return 0;
}

/**
 * @brief Get dephasing multiplier for BMI device ID
 * @param bmi_id BMI device ID (0-3)
 * @return Multiplier value for phase offset calculation
 * 
 * Maps BMI IDs to create 5ms spacing between gyro FIFO reads:
 * ID 0 -> multiplier 0 (0ms offset from base 5ms tick)
 * ID 1 -> multiplier 1 (5ms offset - starts at 5ms)
 * ID 2 -> multiplier 2 (10ms offset - starts at 10ms)
 * ID 3 -> multiplier 3 (15ms offset - starts at 15ms)
 * 
 * This creates a 20ms cycle where each camera reads FIFO at a different time:
 * t=0ms: BMI0 reads, t=5ms: BMI1 reads, t=10ms: BMI2 reads, t=15ms: BMI3 reads
 */
static inline int bmi_get_dephasing_multiplier(int bmi_id)
{
    /* Direct mapping: BMI ID = multiplier (0, 1, 2, 3) */
    if (bmi_id >= 0 && bmi_id < 4)
        return bmi_id;
    
    return 0; /* Default to 0 for invalid IDs */
}

/**
 * @brief Get ring buffer statistics
 * @param ring Pointer to ring buffer
 * @param head_out Current head position (output)
 * @param tail_out Current tail position (output)
 * @param count_out Current entry count (output)
 * @return 0 on success, -EINVAL if ring is NULL
 */
int spsc_ring_get_stats(struct spsc_ring *ring, __u32 *head_out, 
                        __u32 *tail_out, __u32 *count_out)
{
    if (!ring)
        return -EINVAL;
    
    if (head_out)
        *head_out = ring->head;
    if (tail_out)
        *tail_out = ring->tail;
    if (count_out)
        *count_out = spsc_ring_count(ring);
    
    return 0;
}

/**
 * @brief Reset ring buffer to empty state and zero all data
 * @param ring Pointer to ring buffer
 * @return 0 on success, -EINVAL if ring is NULL
 */
int spsc_ring_reset(struct spsc_ring *ring)
{
    if (!ring)
        return -EINVAL;
    
    /* Zero all data in ring buffer */
    memset(ring->data, 0, sizeof(ring->data));
    
    ring->head = 0;
    ring->tail = 0;
    
    /* Memory barrier to ensure reset is complete */
    smp_mb();
    
    // pr_info("BMI_SPSC: Ring buffer reset\n");
    return 0;
}


/**
 * @brief High-resolution timer callback for SPSC data acquisition
 * @param timer Pointer to hrtimer structure
 * @return HRTIMER_RESTART to continue periodic execution
 *
 * Simplified: Just schedules work, timestamp captured in work function.
 */
static enum hrtimer_restart bmi_spsc_timer_callback(struct hrtimer *timer)
{
    struct bmi_spsc_device *spsc_dev = container_of(timer, struct bmi_spsc_device, timer);
    ktime_t now;
    
    /* Schedule I2C work on target CPU - only if not already pending */
    if (!work_pending(&spsc_dev->i2c_work)) {
        queue_work_on(spsc_dev->target_cpu, spsc_dev->i2c_workqueue, &spsc_dev->i2c_work);
    }
    
    /* Re-arm timer for next 5ms interval */
    now = hrtimer_cb_get_time(timer);
    hrtimer_forward(timer, now, ns_to_ktime(BMI_SPSC_TIMER_PERIOD_NS));
    return HRTIMER_RESTART;
}

/**
 * @brief Assign CPU for BMI device based on round-robin
 * @param bmi_id BMI device ID
 * @return CPU number assigned
 */
static int assign_bmi_cpu(int bmi_id)
{
    int target = (bmi_id) % num_online_cpus();
    
    // pr_info("BMI_SPSC: BMI%d assigned to CPU%d\n", bmi_id, target);
    return target;
}

/**
 * @brief Process FIFO data and push samples to ring buffer
 * @param spsc_dev SPSC device
 * @param fifo_data FIFO data from get_fifo
 *
 * PID CONTROLLER TIMESTAMP RECONSTRUCTION ALGORITHM:
 * 
 * P-term (Position Error)
 * 
 * D-term (Rate Error)
 *
 * I-term (Integral Error)
 *
 * Monotonic Continuation
 * 
 * Accelerometer matching:
 *   - For each gyro sample, find closest accel by timestamp
 *   - Interpolate if we run out of unused accel samples
 */
static void bmi_spsc_process_fifo_data(struct bmi_spsc_device *spsc_dev,
                                      struct fifo_burst_fifo_data *fifo_data)
{
    struct bmi_sensor_data sensor_data;
    int i, j, ret;
    u8 gyro_count = fifo_data->sample_count;
    u64 poll_time;
    u64 sample_timestamp;
    u64 min_time_diff;
    u64 interval_ns;
    u64 last_timestamp_local;
    int accel_used[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    int closest_accel_idx;
    
    if (gyro_count == 0) {
        dev_err((struct device*)spsc_dev->device_ptr,
                "BMI%d: FIFO empty (BMI desync) at sample %lld\n",
                spsc_dev->bmi_id, spsc_dev->sample_count);
        return;
    }
    
    if (gyro_count > BMI_FIFO_MAX_FRAMES) {
        dev_warn((struct device*)spsc_dev->device_ptr,
                "BMI%d: FIFO overflow (%d samples > max %d) at sample %lld\n",
                spsc_dev->bmi_id, gyro_count, BMI_FIFO_MAX_FRAMES, spsc_dev->sample_count);
        return;
    }
    
    poll_time = fifo_data->fifo_status_lock_timestamp;
    
    if (poll_time == 0) {
        poll_time = ktime_get_ns();
        pr_warn_ratelimited("BMI_SPSC: BMI%d got zero fifo_status_lock_timestamp (using fallback)\n",
                           spsc_dev->bmi_id);
    }
    
    if (spsc_dev->gyro_sample_interval_ns == 0) {
        if (gyro_count > 5) {
            dev_info((struct device*)spsc_dev->device_ptr,
                    "BMI%d: Flushing %d accumulated samples on first read\n",
                    spsc_dev->bmi_id, gyro_count);
            return;
        }
        
        spsc_dev->gyro_sample_interval_ns = 5000000ULL;
        spsc_dev->last_poll_time = poll_time;
        spsc_dev->last_assigned_timestamp = poll_time - (gyro_count * 5000000ULL);
    }
    
    interval_ns = spsc_dev->gyro_sample_interval_ns;
    last_timestamp_local = spsc_dev->last_assigned_timestamp;
    
    if (spsc_dev->last_poll_time != 0 && spsc_dev->last_poll_time != poll_time) {
        u8 Kp = spsc_dev->pid_kp;
        u8 Kd = spsc_dev->pid_kd;
        u8 Ki = spsc_dev->pid_ki;
        s64 position_error_us;
        s64 rate_error_us;
        s64 p_correction_ns;
        s64 d_correction_ns;
        s64 i_correction_ns;
        s64 total_correction_ns;
        s64 dt_span;
        s64 actual_sample_count;
        s64 newest_sample_ts;
        s64 interval_error_us;
        s64 actual_mean_interval_ns;
        s64 integral_error_ns;
        int oldest_idx;
        int the_one_before_idx;
        int idx;
        int k;
        
        spsc_dev->batch_history[spsc_dev->batch_head].poll_time = poll_time;
        spsc_dev->batch_history[spsc_dev->batch_head].sample_count = gyro_count;
        spsc_dev->batch_head = (spsc_dev->batch_head + 1) & 5;
        if (spsc_dev->batch_valid_count < 6)
            spsc_dev->batch_valid_count++;

        /* Use current estimated interval instead of hardcoded 5ms for smoother estimation */
        newest_sample_ts = last_timestamp_local + (gyro_count * interval_ns);
        position_error_us = ((s64)poll_time - 2500000LL - newest_sample_ts) / 1000LL;
        
        dt_span = 0;
        actual_sample_count = 0;
        if (spsc_dev->batch_valid_count >= 2) {
            oldest_idx = (spsc_dev->batch_head - spsc_dev->batch_valid_count) & 5;
            idx = (oldest_idx + 1) & 5;
            for (k = 0; k < spsc_dev->batch_valid_count - 1; k++) {
                actual_sample_count += spsc_dev->batch_history[idx].sample_count;
                idx = (idx + 1) & 5;
            }
            dt_span = (s64)(poll_time - spsc_dev->batch_history[oldest_idx].poll_time);
        }
        
        if (actual_sample_count > 10) {
            actual_mean_interval_ns = dt_span / actual_sample_count;
            rate_error_us = actual_mean_interval_ns/1000LL - 5000LL;
        } else {
            rate_error_us = 0;
        }

        interval_error_us = ((s64)interval_ns - 5000000LL) / 1000LL;
        /* Get the index of the entry BEFORE the one we just added (since batch_head already incremented) */
        the_one_before_idx = (spsc_dev->batch_head - 2) & 5;
        
        /* Collect: Get integral error from struct (in microseconds, convert to ns internally) */
        integral_error_ns = spsc_dev->integral_error;
        
        /* Calculate: Update integral error internally */
        integral_error_ns += (interval_error_us * spsc_dev->batch_history[the_one_before_idx].sample_count);
        
        /* Send: Write back to struct as microseconds */
        spsc_dev->integral_error = integral_error_ns;
        
        /* Calculate: Compute individual PID correction terms */
        p_correction_ns = (Kp == 0) ? 0 : (Kp * position_error_us) / gyro_count;
        d_correction_ns = (Kd == 0) ? 0 : (Kd * rate_error_us) / gyro_count;
        i_correction_ns = (Ki == 0) ? 0 : (Ki * (integral_error_ns / 1000LL)) / gyro_count;
        
        /* Clamp: Apply individual limits to each correction term */
        if (p_correction_ns > 1000000LL)
            p_correction_ns = 1000000LL;
        if (p_correction_ns < -1000000LL)
            p_correction_ns = -1000000LL;

        if (d_correction_ns > 150000LL)
            d_correction_ns = 150000LL;
        if (d_correction_ns < -150000LL)
            d_correction_ns = -150000LL;

        if (i_correction_ns > 150000LL)
            i_correction_ns = 150000LL;
        if (i_correction_ns < -150000LL)
            i_correction_ns = -150000LL;
        
        /* Calculate total correction from clamped individual terms */
        total_correction_ns = (p_correction_ns + d_correction_ns + i_correction_ns);
        
        interval_ns = 5000000LL + total_correction_ns;

        spsc_dev->gyro_sample_interval_ns = interval_ns;
        
        /* Track timestamp validation: recalculate with NEW interval after all corrections */
        newest_sample_ts = last_timestamp_local + (gyro_count * interval_ns);
        spsc_dev->total_batches++;
        if (newest_sample_ts >= (poll_time - 5160000LL) && newest_sample_ts <= poll_time) {
            spsc_dev->bounded_batches++;
        }
    }
    
    spsc_dev->last_poll_time = poll_time;
    spsc_dev->total_samples_received += gyro_count;
    
    for (i = 0; i < gyro_count; i++) {
        sample_timestamp = last_timestamp_local + interval_ns;
        
        if (sample_timestamp <= last_timestamp_local) {
            pr_err("BMI_SPSC: BMI%d CRITICAL - timestamp not advancing! sample_ts=%llu, last_ts=%llu, interval=%llu\n",
                   spsc_dev->bmi_id, sample_timestamp, last_timestamp_local, interval_ns);
            continue;
        }
        
        last_timestamp_local = sample_timestamp;
        
        closest_accel_idx = -1;
        min_time_diff = ULLONG_MAX;
        
        for (j = 0; j < 8; j++) {
            if (!accel_used[j]) {
                u64 time_diff = (spsc_dev->accel_history[j].timestamp > sample_timestamp) ?
                                (spsc_dev->accel_history[j].timestamp - sample_timestamp) :
                                (sample_timestamp - spsc_dev->accel_history[j].timestamp);
                if (time_diff < min_time_diff) {
                    min_time_diff = time_diff;
                    closest_accel_idx = j;
                }
            }
        }
        
        if (closest_accel_idx >= 0) {
            sensor_data.ax = spsc_dev->accel_history[closest_accel_idx].ax;
            sensor_data.ay = spsc_dev->accel_history[closest_accel_idx].ay;
            sensor_data.az = spsc_dev->accel_history[closest_accel_idx].az;
            accel_used[closest_accel_idx] = 1;
        } else {
            int idx0 = -1, idx1 = -1;
            u64 ts0 = 0, ts1 = ULLONG_MAX;
            
            for (j = 0; j < 8; j++) {
                u64 accel_ts = spsc_dev->accel_history[j].timestamp;
                if (accel_ts <= sample_timestamp && accel_ts > ts0) {
                    ts0 = accel_ts;
                    idx0 = j;
                }
                if (accel_ts >= sample_timestamp && accel_ts < ts1) {
                    ts1 = accel_ts;
                    idx1 = j;
                }
            }
            
            if (idx0 >= 0 && idx1 >= 0 && ts1 > ts0) {
                s64 dt_total = ts1 - ts0;
                s64 dt_partial = sample_timestamp - ts0;
                
                sensor_data.ax = spsc_dev->accel_history[idx0].ax + 
                                 (s16)(((s64)(spsc_dev->accel_history[idx1].ax - spsc_dev->accel_history[idx0].ax) * dt_partial) / dt_total);
                sensor_data.ay = spsc_dev->accel_history[idx0].ay + 
                                 (s16)(((s64)(spsc_dev->accel_history[idx1].ay - spsc_dev->accel_history[idx0].ay) * dt_partial) / dt_total);
                sensor_data.az = spsc_dev->accel_history[idx0].az + 
                                 (s16)(((s64)(spsc_dev->accel_history[idx1].az - spsc_dev->accel_history[idx0].az) * dt_partial) / dt_total);
            } else if (idx0 >= 0) {
                sensor_data.ax = spsc_dev->accel_history[idx0].ax;
                sensor_data.ay = spsc_dev->accel_history[idx0].ay;
                sensor_data.az = spsc_dev->accel_history[idx0].az;
            } else if (idx1 >= 0) {
                sensor_data.ax = spsc_dev->accel_history[idx1].ax;
                sensor_data.ay = spsc_dev->accel_history[idx1].ay;
                sensor_data.az = spsc_dev->accel_history[idx1].az;
            } else {
                sensor_data.ax = 0;
                sensor_data.ay = 0;
                sensor_data.az = 0;
            }
        }
        
        sensor_data.gx = fifo_data->gyro_samples[i].x;
        sensor_data.gy = fifo_data->gyro_samples[i].y;
        sensor_data.gz = fifo_data->gyro_samples[i].z;
        sensor_data.timestamp = sample_timestamp;
        
        ret = spsc_ring_write(spsc_dev->ring, &sensor_data);
        
        /* The consumer alone owns tail; never discard its first sample. */
        if (ret == -ENOSPC)
            spsc_dev->ring_drops++;
    }
    
    spsc_dev->last_assigned_timestamp = last_timestamp_local;
    spsc_dev->gyro_sample_interval_ns = interval_ns;

    if (gyro_count > 0) {
        wait_queue_head_t *wq = (wait_queue_head_t *)spsc_dev->wait_queue_ptr;
        if (wq) {
            smp_mb();
            wake_up_interruptible_sync(wq);
        }
    }
}
/**
 * @brief High-priority I2C work function for sensor data acquisition
 * 
 * - Every 5ms (all cameras): Read accelerometer data
 * - Every 20ms (per camera, staggered by 5ms): Read gyro FIFO
 * 
 * Camera timing with 5ms base period:
 * - BMI0: reads gyro at tick 0, 4, 8, 12... (every 4th tick = 20ms)
 * - BMI1: reads gyro at tick 1, 5, 9, 13... (every 4th tick = 20ms)
 * - BMI2: reads gyro at tick 2, 6, 10, 14... (every 4th tick = 20ms)
 * - BMI3: reads gyro at tick 3, 7, 11, 15... (every 4th tick = 20ms)
 */
static void bmi_spsc_i2c_work(struct work_struct *work)
{
    struct bmi_spsc_device *spsc_dev = container_of(work, struct bmi_spsc_device, i2c_work);
    struct fifo_burst_len_data accel_data;
    struct fifo_burst_fifo_data gyro_data;
    int ret;
    bool should_read_gyro;
    
    /* Increment per-device sample count */
    spsc_dev->sample_count++;
    
    /* Check if buffer is full and no consumer (mmap_count == 0) - stop timer */
    if (spsc_ring_full(spsc_dev->ring)) {
        if (atomic_read(&spsc_dev->mmap_count) == 0) {
            /* No active mmap = no consumer. Stop timer and reset buffer. */
            hrtimer_cancel(&spsc_dev->timer);
            spsc_ring_reset(spsc_dev->ring);
            return;
        }
    }
    
    /* Determine if this is a gyro read tick for this camera */
    /* Each camera reads gyro every 4 ticks (20ms), offset by bmi_id */
    should_read_gyro = ((spsc_dev->sample_count % 4) == (spsc_dev->bmi_id % 4));
    
    /* Step 1: Always read accelerometer data every 5ms */
    ret = spsc_dev->fn_dev->get_accel(spsc_dev->st, 
                                      (u8*)&accel_data, sizeof(accel_data));
    if (ret < 0) {
        spsc_dev->i2c_errors++;

        /* -EIO = I2C failure (likely unplug). Stop timer, log once, notify userspace. */
        if (ret == -EIO) {
            hrtimer_cancel(&spsc_dev->timer);
            dev_err((struct device *)spsc_dev->device_ptr,
                    "%s: I/O error, stopping acquisition (use timer_control to restart)\n",
                    spsc_dev->device_name);
        }
        return;
    }
    
    /* Store accelerometer data */
    spsc_dev->accel_history[spsc_dev->accel_head].ax = accel_data.ax;
    spsc_dev->accel_history[spsc_dev->accel_head].ay = accel_data.ay;
    spsc_dev->accel_history[spsc_dev->accel_head].az = accel_data.az;
    spsc_dev->accel_history[spsc_dev->accel_head].timestamp = accel_data.timestamp;
    spsc_dev->accel_head = (spsc_dev->accel_head + 1) % 8;
    
    /* Step 2: Conditionally read gyro FIFO every 20ms (staggered) */
    if (should_read_gyro) {
        ret = spsc_dev->fn_dev->get_gyro(spsc_dev->st, 
                                         (u8*)&gyro_data, sizeof(gyro_data),
                                         true);  /* read_fifo = true */
        if (ret == ERR_DESYNC) {
            /* FIFO length is zero - no gyro data available */
            return;
        } else if (ret < 0) {
            spsc_dev->i2c_errors++;
            if (ret == -EIO) {
                hrtimer_cancel(&spsc_dev->timer);
                dev_err((struct device *)spsc_dev->device_ptr,
                        "%s: I/O error, stopping acquisition\n",
                        spsc_dev->device_name);
            }
            return;
        }
        
        /* Process FIFO gyro data and push to ring buffer */
        bmi_spsc_process_fifo_data(spsc_dev, &gyro_data);
    }
}

/**
 * @brief Initialize high-priority I2C workqueue
 * @param spsc_dev SPSC device
 * @return 0 on success, negative error code on failure
 */
static int bmi_spsc_i2c_workqueue_init(struct bmi_spsc_device *spsc_dev)
{
    char wq_name[32];
    
    /* Create dedicated workqueue with high priority and CPU affinity */
    snprintf(wq_name, sizeof(wq_name), "bmi_i2c_%s", spsc_dev->device_name);
    
    spsc_dev->i2c_workqueue = alloc_workqueue(wq_name, 
                                             WQ_HIGHPRI | WQ_CPU_INTENSIVE, 
                                             1); /* max_active = 1 for FIFO ordering */
    if (!spsc_dev->i2c_workqueue) {
        pr_err("BMI_SPSC: Failed to create I2C workqueue for %s\n", spsc_dev->device_name);
        return -ENOMEM;
    }

    /* Initialize work item */
    INIT_WORK(&spsc_dev->i2c_work, bmi_spsc_i2c_work);
    
    // pr_info("BMI_SPSC: I2C workqueue initialized for %s\n", spsc_dev->device_name);
    return 0;
}

/**
 * @brief Cleanup I2C workqueue
 * @param spsc_dev SPSC device
 */
static void bmi_spsc_i2c_workqueue_cleanup(struct bmi_spsc_device *spsc_dev)
{
    if (spsc_dev->i2c_workqueue) {
        /* Cancel any pending work */
        cancel_work_sync(&spsc_dev->i2c_work);
        
        /* Destroy workqueue */
        destroy_workqueue(spsc_dev->i2c_workqueue);
        spsc_dev->i2c_workqueue = NULL;
        
        // pr_info("BMI_SPSC: I2C workqueue cleaned up for %s\n", spsc_dev->device_name);
    }
}

/**
 * @brief Release function for kref - actual memory freeing happens here
 * @param ref Pointer to kref structure
 */
static void bmi_spsc_ref_release(struct kref *ref)
{
    struct bmi_spsc_device *spsc_dev = container_of(ref, struct bmi_spsc_device, ref);
    wait_queue_head_t *wq;
    
    // pr_info("BMI_SPSC: Releasing final reference to device %s\n", spsc_dev->device_name);

    /* Free wait queue */
    wq = (wait_queue_head_t *)spsc_dev->wait_queue_ptr;
    if (wq) {
        kfree(wq);
    }
    
    /* Free ring buffer */
    if (spsc_dev->ring) {
        spsc_ring_free(spsc_dev->ring);
    }
    
    /* Free device structure */
    kfree(spsc_dev);
}

/**
 * @brief Character device open operation
 * 
 * Multiple opens allowed - ownership tracked via mmap refcount.
 * Device is "in use" when mmap_count > 0.
 */
static int bmi_spsc_open(struct inode *inode, struct file *filp)
{
    struct bmi_spsc_device *spsc_dev;

    spsc_dev = container_of(inode->i_cdev, struct bmi_spsc_device, cdev);

    if (!try_module_get(THIS_MODULE))
        return -ENODEV;

    kref_get(&spsc_dev->ref);
    filp->private_data = spsc_dev;
    return 0;
}

/**
 * @brief Character device release operation
 * 
 * File close - mmap refcount handles actual ownership release.
 */
static int bmi_spsc_release(struct inode *inode, struct file *filp)
{
    struct bmi_spsc_device *spsc_dev = filp->private_data;

    if (spsc_dev)
        kref_put(&spsc_dev->ref, bmi_spsc_ref_release);
        
    module_put(THIS_MODULE);
    return 0;
}

/**
 * @brief VMA open callback - called on mmap and fork/dup
 */
static void bmi_spsc_vma_open(struct vm_area_struct *vma)
{
    struct bmi_spsc_device *spsc_dev = vma->vm_private_data;
    if (spsc_dev) {
        kref_get(&spsc_dev->ref);
        atomic_inc(&spsc_dev->mmap_count);
    }
}

/**
 * @brief VMA close callback - called on munmap and process exit
 */
static void bmi_spsc_vma_close(struct vm_area_struct *vma)
{
    struct bmi_spsc_device *spsc_dev = vma->vm_private_data;
    if (spsc_dev) {
        atomic_dec(&spsc_dev->mmap_count);
        kref_put(&spsc_dev->ref, bmi_spsc_ref_release);
    }
}

static const struct vm_operations_struct bmi_spsc_vm_ops = {
    .open = bmi_spsc_vma_open,
    .close = bmi_spsc_vma_close,
};

/**
 * @brief Character device mmap operation
 * 
 * Ownership tracked via VMA refcount. Device is "in use" while mmap_count > 0.
 * When all VMAs are unmapped (process exit, munmap), device becomes free.
 */
static int bmi_spsc_mmap(struct file *filp, struct vm_area_struct *vma)
{
    struct bmi_spsc_device *spsc_dev = filp->private_data;
    unsigned long requested_size;
    unsigned long pfn;
    int ret;

    if (!spsc_dev || !spsc_dev->ring) {
        pr_err("BMI_SPSC: No ring buffer available for mmap\n");
        return -EINVAL;
    }

    /* Check if device is already owned (mmap'd by another process) */
    if (atomic_read(&spsc_dev->mmap_count) > 0) {
        dev_warn((struct device *)spsc_dev->device_ptr,
                 "%s: device busy (mmap_count=%d)\n",
                 spsc_dev->device_name, atomic_read(&spsc_dev->mmap_count));
        return -EBUSY;
    }

    /* SECURITY: Validate offset - must be zero (no partial mappings) */
    if (vma->vm_pgoff != 0) {
        pr_err("BMI_SPSC: mmap denied - non-zero offset not allowed (pgoff: %lu)\n",
               vma->vm_pgoff);
        return -EINVAL;
    }

    /* SECURITY: Validate requested size */
    requested_size = vma->vm_end - vma->vm_start;
    if (requested_size != BMI_SPSC_MMAP_SIZE) {
        pr_err("BMI_SPSC: mmap denied - invalid size %lu (expected: %lu)\n",
               requested_size, BMI_SPSC_MMAP_SIZE);
        return -EINVAL;
    }

    /* Setup VMA operations for refcount tracking */
    vma->vm_ops = &bmi_spsc_vm_ops;
    vma->vm_private_data = spsc_dev;

    pfn = virt_to_phys(spsc_dev->ring) >> PAGE_SHIFT;
    
    /* Map the kernel memory to userspace */
    ret = remap_pfn_range(vma, vma->vm_start, pfn, BMI_SPSC_MMAP_SIZE, vma->vm_page_prot);
    if (ret == 0) {
        /* Capture reference and mmap tracking */
        bmi_spsc_vma_open(vma);
    }
    return ret;
}

/**
 * @brief Character device poll operation
 */
static unsigned int bmi_spsc_poll(struct file *filp, poll_table *wait)
{
    struct bmi_spsc_device *spsc_dev = filp->private_data;
    wait_queue_head_t *wq;
    
    if (!spsc_dev)
        return POLLERR;
        
    wq = (wait_queue_head_t *)spsc_dev->wait_queue_ptr;
    if (wq) {
        poll_wait(filp, wq, wait);
        if (!spsc_ring_empty(spsc_dev->ring))
            return POLLIN | POLLRDNORM;
    }
    
    return 0;
}

/* Character device file operations */
static struct file_operations bmi_spsc_fops = {
    .owner = THIS_MODULE,
    .open = bmi_spsc_open,
    .release = bmi_spsc_release,
    .mmap = bmi_spsc_mmap,
    .poll = bmi_spsc_poll,
};

/**
 * @brief Show target CPU sysfs attribute
 */
static ssize_t target_cpu_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_spsc_device *spsc_dev = dev_get_drvdata(dev);
    if (!spsc_dev)
        return -EINVAL;
    return sprintf(buf, "%d\n", spsc_dev->target_cpu);
}

/**
 * @brief Show sample count sysfs attribute
 */
static ssize_t sample_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_spsc_device *spsc_dev = dev_get_drvdata(dev);
    if (!spsc_dev)
        return -EINVAL;
    return sprintf(buf, "%lld\n", spsc_dev->sample_count);
}

/**
 * @brief Show I2C error count sysfs attribute
 */
static ssize_t i2c_errors_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_spsc_device *spsc_dev = dev_get_drvdata(dev);
    if (!spsc_dev)
        return -EINVAL;
    return sprintf(buf, "%lld\n", spsc_dev->i2c_errors);
}

static ssize_t ring_drops_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_spsc_device *st = dev_get_drvdata(dev);
    return sysfs_emit(buf, "%lld\n", st->ring_drops);
}

static ssize_t fifo_overruns_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_spsc_device *st = dev_get_drvdata(dev);
    return sysfs_emit(buf, "%lld\n", st->fifo_overruns);
}

/**
 * @brief Show ring buffer mmap size in bytes (read-only)
 * 
 * User-space must read this value before calling mmap() to ensure
 * the correct buffer size is used. This prevents size mismatches
 * between kernel and user-space.
 */
static ssize_t ring_buffer_size_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
    /* Return the exact mmap size in bytes - always available */
    return sprintf(buf, "%lu\n", BMI_SPSC_MMAP_SIZE);
}

/**
 * @brief Show ring buffer status sysfs attribute
 */
static ssize_t ring_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_spsc_device *spsc_dev = dev_get_drvdata(dev);
    __u32 head, tail, count;
    
    if (!spsc_dev || !spsc_dev->ring)
        return sprintf(buf, "no_ring\n");
    
    spsc_ring_get_stats(spsc_dev->ring, &head, &tail, &count);
    return sprintf(buf, "head=%u tail=%u count=%u size=%d\n", 
                   head, tail, count, BMI_SPSC_BUFFER_SIZE);
}

/**
 * @brief Show timer status (running/stopped)
 */
static ssize_t timer_status_show(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
    struct bmi_spsc_device *spsc_dev = dev_get_drvdata(dev);
    
    return sprintf(buf, "%s\n", 
                   hrtimer_active(&spsc_dev->timer) ? "running" : "stopped");
}

/**
 * @brief Start/stop timer via sysfs (1=start, 0=stop)
 */
static ssize_t timer_control_store(struct device *dev,
                                 struct device_attribute *attr,
                                 const char *buf, size_t count)
{
    struct bmi_spsc_device *spsc_dev = dev_get_drvdata(dev);
    unsigned long val;
    int ret;
    // Convert input string to unsigned long
    ret = kstrtoul(buf, DECIMAL, &val);
    if (ret)
        return ret;

    if (val == 1) {
        ret = bmi_spsc_start(spsc_dev);
        if (ret) {
            dev_err(dev, "Failed to start timer: %d\n", ret);
            return ret;
        }
        // dev_info(dev, "Timer started\n");
    } else if (val == 0) {
        bmi_spsc_stop(spsc_dev);
        // dev_info(dev, "Timer stopped\n");
    } else {
        return -EINVAL;
    }

    return count;
}


/**
 * @brief Show available accelerometer ranges
 */
static ssize_t accel_available_ranges_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_spsc_device *spsc_dev = dev_get_drvdata(dev);
    ssize_t count = 0;
    int i;
    
    if (!spsc_dev || !spsc_dev->st)
        return -EINVAL;
    
    /* From bmi_rr_acc_bmi088[] in core - 4 ranges (indices 0-3) */
    for (i = 0; i < 4; i++) {
        count += sprintf(buf + count, "%d ", i);
    }
    if (count > 0)
        buf[count - 1] = '\n'; /* Replace last space with newline */
    
    return count;
}

/**
 * @brief Show available gyroscope ranges  
 */
static ssize_t gyro_available_ranges_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_spsc_device *spsc_dev = dev_get_drvdata(dev);
    ssize_t count = 0;
    int i;
    
    if (!spsc_dev || !spsc_dev->st)
        return -EINVAL;
    
    /* From bmi_rr_gyr[] in core - 5 ranges (indices 0-4) */
    for (i = 0; i < 5; i++) {
        count += sprintf(buf + count, "%d ", i);
    }
    if (count > 0)
        buf[count - 1] = '\n'; /* Replace last space with newline */
    
    return count;
}

/**
 * @brief Show available accelerometer frequencies
 */
static ssize_t accel_available_frequencies_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_spsc_device *spsc_dev = dev_get_drvdata(dev);
    ssize_t count = 0;
    
    if (!spsc_dev || !spsc_dev->st)
        return -EINVAL;
    
    /* From bmi_odrs_acc[] in core: 12.5, 25, 50, 100, 200, 400, 800, 1600 Hz */
    count += sprintf(buf + count, "12 25 50 100 200 400 800 1600\n");
    
    return count;
}

/**
 * @brief Show available gyroscope frequencies
 */
static ssize_t gyro_available_frequencies_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_spsc_device *spsc_dev = dev_get_drvdata(dev);
    ssize_t count = 0;
    
    if (!spsc_dev || !spsc_dev->st)
        return -EINVAL;
    
    /* From bmi_odrs_gyr[] in core: 100, 200, 400, 1000, 2000 Hz */
    count += sprintf(buf + count, "100 200 400 1000 2000\n");
    
    return count;
}

/**
 * @brief Show current accelerometer range
 */
static ssize_t accel_range_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_spsc_device *spsc_dev = dev_get_drvdata(dev);
    
    if (!spsc_dev || !spsc_dev->st || !spsc_dev->fn_dev || !spsc_dev->fn_dev->get_accel_range)
        return -EINVAL;
    
    return sprintf(buf, "%d\n", spsc_dev->fn_dev->get_accel_range(spsc_dev->st));
}

/**
 * @brief Show current gyroscope range
 */
static ssize_t gyro_range_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_spsc_device *spsc_dev = dev_get_drvdata(dev);
    
    if (!spsc_dev || !spsc_dev->st || !spsc_dev->fn_dev || !spsc_dev->fn_dev->get_gyro_range)
        return -EINVAL;
    
    return sprintf(buf, "%d\n", spsc_dev->fn_dev->get_gyro_range(spsc_dev->st));
}

/**
 * @brief Show current accelerometer frequency
 */
static ssize_t accel_frequency_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_spsc_device *spsc_dev = dev_get_drvdata(dev);
    
    if (!spsc_dev || !spsc_dev->st || !spsc_dev->fn_dev || !spsc_dev->fn_dev->get_accel_freq)
        return -EINVAL;
    
    return sprintf(buf, "%d\n", spsc_dev->fn_dev->get_accel_freq(spsc_dev->st));
}

/**
 * @brief Show current gyroscope frequency
 */
static ssize_t gyro_frequency_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_spsc_device *spsc_dev = dev_get_drvdata(dev);
    
    if (!spsc_dev || !spsc_dev->st || !spsc_dev->fn_dev || !spsc_dev->fn_dev->get_gyro_freq)
        return -EINVAL;
    
    return sprintf(buf, "%d\n", spsc_dev->fn_dev->get_gyro_freq(spsc_dev->st));
}

/**
 * @brief Set accelerometer range
 */
static ssize_t accel_range_store(struct device *dev, struct device_attribute *attr,
                                const char *buf, size_t count)
{
    struct bmi_spsc_device *spsc_dev = dev_get_drvdata(dev);
    unsigned long range_idx;
    int ret;

    if (!spsc_dev || !spsc_dev->st || !spsc_dev->fn_dev)
        return -EINVAL;

    // Convert input string to unsigned long
    ret = kstrtoul(buf, DECIMAL, &range_idx);
    if (ret)
        return ret;

    /* Clamp to valid range (0-3 for accelerometer) */
    if (range_idx > 3)
        range_idx = 3;

    /* Use max_range function which accepts index directly */
    ret = spsc_dev->fn_dev->max_range(spsc_dev->st, BMI_HW_ACC, (int)range_idx);
    if (ret) {
        dev_err(dev, "Failed to set accelerometer range to %lu: %d\n", range_idx, ret);
        return ret;
    }

    dev_dbg(dev, "Accelerometer range set to index %lu\n", range_idx);
    return count; 
}

/**
 * @brief Set gyroscope range
 */
static ssize_t gyro_range_store(struct device *dev, struct device_attribute *attr,
                               const char *buf, size_t count)
{
    struct bmi_spsc_device *spsc_dev = dev_get_drvdata(dev);
    unsigned long range_idx;
    int ret;

    if (!spsc_dev || !spsc_dev->st || !spsc_dev->fn_dev)
        return -EINVAL;

    // Convert input string to unsigned long
    ret = kstrtoul(buf, DECIMAL, &range_idx);
    if (ret)
        return ret;

    /* Clamp to valid range (0-4 for gyroscope) */
    if (range_idx > 4)
        range_idx = 4;

    /* Use max_range function which accepts index directly */
    ret = spsc_dev->fn_dev->max_range(spsc_dev->st, BMI_HW_GYR, (int)range_idx);
    if (ret) {
        dev_err(dev, "Failed to set gyroscope range to %lu: %d\n", range_idx, ret);
        return ret;
    }

    dev_dbg(dev, "Gyroscope range set to index %lu\n", range_idx);
    return count; 
}

/**
 * @brief Set accelerometer frequency
 */
static ssize_t accel_frequency_store(struct device *dev, struct device_attribute *attr,
                                    const char *buf, size_t count)
{
    struct bmi_spsc_device *spsc_dev = dev_get_drvdata(dev);
    unsigned long freq_hz;
    int ret;

    if (!spsc_dev || !spsc_dev->st || !spsc_dev->fn_dev)
        return -EINVAL;

    // Convert input string to unsigned long
    ret = kstrtoul(buf, DECIMAL, &freq_hz);
    if (ret)
        return ret;

    /* Use existing freq_write function with Hz values */
    /* Common accelerometer frequencies: 12, 25, 50, 100, 200, 400, 800, 1600 Hz */
    ret = spsc_dev->fn_dev->freq_write(spsc_dev->st, BMI_HW_ACC, (int)freq_hz, 0);
    if (ret) {
        dev_err(dev, "Failed to set accelerometer frequency to %lu Hz: %d\n", freq_hz, ret);
        return ret;
    }

    dev_dbg(dev, "Accelerometer frequency set to %lu Hz\n", freq_hz);
    return count; 
}

/**
 * @brief Set gyroscope frequency
 */
static ssize_t gyro_frequency_store(struct device *dev, struct device_attribute *attr,
                                   const char *buf, size_t count)
{
    struct bmi_spsc_device *spsc_dev = dev_get_drvdata(dev);
    unsigned long freq_hz;
    int ret;

    if (!spsc_dev || !spsc_dev->st || !spsc_dev->fn_dev)
        return -EINVAL;

    // Convert input string to unsigned long
    ret = kstrtoul(buf, DECIMAL, &freq_hz);
    if (ret)
        return ret;

    /* Use existing freq_write function with Hz values */
    /* Common gyroscope frequencies: 100, 200, 400, 1000, 2000 Hz */
    ret = spsc_dev->fn_dev->freq_write(spsc_dev->st, BMI_HW_GYR, (int)freq_hz, 0);
    if (ret) {
        dev_err(dev, "Failed to set gyroscope frequency to %lu Hz: %d\n", freq_hz, ret);
        return ret;
    }

    dev_dbg(dev, "Gyroscope frequency set to %lu Hz\n", freq_hz);
    return count; 
}

/**
 * @brief Show accelerometer temperature
 */
static ssize_t accel_temperature_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_spsc_device *spsc_dev = dev_get_drvdata(dev);
    int val, val2;
    int ret;

    if (!spsc_dev || !spsc_dev->st || !spsc_dev->fn_dev || !spsc_dev->fn_dev->temp)
        return -EINVAL;

    ret = spsc_dev->fn_dev->temp(spsc_dev->st, BMI_HW_ACC, &val, &val2);
    if (ret) {
        dev_err(dev, "Failed to read temperature: %d\n", ret);
        return ret;
    }

    // Temperature is returned in millidegrees Celsius
    return snprintf(buf, PAGE_SIZE, "%d.%03d\n", val / 1000, val % 1000);
}

/**
 * @brief Show I2C bus number
 */
static ssize_t i2c_bus_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_spsc_device *spsc_dev = dev_get_drvdata(dev);
    int bus_nr;

    if (!spsc_dev || !spsc_dev->st || !spsc_dev->fn_dev || !spsc_dev->fn_dev->i2c_bus)
        return -EINVAL;

    bus_nr = spsc_dev->fn_dev->i2c_bus(spsc_dev->st);
    if (bus_nr < 0)
        return bus_nr;

    return snprintf(buf, PAGE_SIZE, "%d\n", bus_nr);
}

/**
 * @brief Show accelerometer I2C address
 */
static ssize_t acc_addr_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_spsc_device *spsc_dev = dev_get_drvdata(dev);
    int addr;

    if (!spsc_dev || !spsc_dev->st || !spsc_dev->fn_dev || !spsc_dev->fn_dev->i2c_addr)
        return -EINVAL;

    addr = spsc_dev->fn_dev->i2c_addr(spsc_dev->st, BMI_HW_ACC);
    if (addr < 0)
        return addr;

    return snprintf(buf, PAGE_SIZE, "0x%02x\n", addr);
}

/**
 * @brief Show gyroscope I2C address
 */
static ssize_t gyro_addr_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_spsc_device *spsc_dev = dev_get_drvdata(dev);
    int addr;

    if (!spsc_dev || !spsc_dev->st || !spsc_dev->fn_dev || !spsc_dev->fn_dev->i2c_addr)
        return -EINVAL;

    addr = spsc_dev->fn_dev->i2c_addr(spsc_dev->st, BMI_HW_GYR);
    if (addr < 0)
        return addr;

    return snprintf(buf, PAGE_SIZE, "0x%02x\n", addr);
}

/**
 * @brief Show device name (always "bmi088")
 */
static ssize_t name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "bmi088\n");
}

/**
 * @brief Show timer phase offset (Phase 8)
 */
static ssize_t timer_phase_offset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_spsc_device *spsc_dev = dev_get_drvdata(dev);
    
    if (!spsc_dev)
        return -EINVAL;
    
    /* Phase offset = multiplier * 1.25ms = multiplier * 1250 microseconds */
    return sprintf(buf, "%d\n", bmi_get_dephasing_multiplier(spsc_dev->bmi_id) * 1250);
}

/**
 * @brief Show next timer expiry time (Phase 8)
 */
static ssize_t timer_next_expiry_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_spsc_device *spsc_dev = dev_get_drvdata(dev);
    ktime_t next_expiry;
    u64 expiry_ns;
    
    if (!spsc_dev)
        return -EINVAL;
    
    if (!hrtimer_active(&spsc_dev->timer))
        return sprintf(buf, "inactive\n");
    
    next_expiry = hrtimer_get_expires(&spsc_dev->timer);
    expiry_ns = ktime_to_ns(next_expiry);
    
    return sprintf(buf, "%llu\n", expiry_ns);
}

/**
 * @brief Show timer synchronization status (Phase 8)
 */
static ssize_t timer_synchronization_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_spsc_device *spsc_dev = dev_get_drvdata(dev);
    ktime_t now, next_expiry;
    s64 time_to_expiry;
    
    if (!spsc_dev)
        return -EINVAL;
    
    if (!hrtimer_active(&spsc_dev->timer))
        return sprintf(buf, "inactive\n");
    
    now = ktime_get();
    next_expiry = hrtimer_get_expires(&spsc_dev->timer);
    time_to_expiry = ktime_to_ns(ktime_sub(next_expiry, now));
    
    return sprintf(buf, "phase_offset=%dus next_expiry=%lldns time_to_expiry=%lldns\n",
                   bmi_get_dephasing_multiplier(spsc_dev->bmi_id) * 1250, ktime_to_ns(next_expiry), time_to_expiry);
}

/**
 * @brief Dump all BMI088 registers (both accelerometer and gyroscope)
 */
static ssize_t dump_regs_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_spsc_device *spsc_dev = dev_get_drvdata(dev);
    ssize_t count = 0;
    int ret;
    
    if (!spsc_dev || !spsc_dev->st || !spsc_dev->fn_dev || !spsc_dev->fn_dev->regs)
        return -EINVAL;
    
    /* Dump accelerometer registers */
    count += sprintf(buf + count, "=== ACCELEROMETER REGISTERS ===\n");
    ret = spsc_dev->fn_dev->regs(spsc_dev->st, BMI_HW_ACC, buf + count);
    if (ret < 0) {
        count += sprintf(buf + count, "Error reading accelerometer registers: %d\n", ret);
    } else {
        count += ret;
    }
    
    /* Dump gyroscope registers */
    count += sprintf(buf + count, "\n=== GYROSCOPE REGISTERS ===\n");
    ret = spsc_dev->fn_dev->regs(spsc_dev->st, BMI_HW_GYR, buf + count);
    if (ret < 0) {
        count += sprintf(buf + count, "Error reading gyroscope registers: %d\n", ret);
    } else {
        count += ret;
    }
    
    return count;
}

/**
 * @brief Show PID timestamp validation statistics
 */
static __maybe_unused ssize_t pid_stats_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_spsc_device *spsc_dev = dev_get_drvdata(dev);
    long long percentage;
    
    if (!spsc_dev)
        return -EINVAL;
    
    if (spsc_dev->total_batches == 0)
        return sprintf(buf, "total_batches=0 bounded_batches=0 percentage=0%%\n");
    
    percentage = (spsc_dev->bounded_batches * 100) / spsc_dev->total_batches;
    
    return sprintf(buf, "total_batches=%lld bounded_batches=%lld percentage=%lld%%\n",
                   spsc_dev->total_batches, spsc_dev->bounded_batches, percentage);
}

/**
 * @brief Show PID Kp gain
 */
static __maybe_unused ssize_t pid_kp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_spsc_device *spsc_dev = dev_get_drvdata(dev);
    
    if (!spsc_dev)
        return -EINVAL;
    
    return sprintf(buf, "%lld\n", spsc_dev->pid_kp);
}

/**
 * @brief Set PID Kp gain (applied on next start)
 */
static __maybe_unused ssize_t pid_kp_store(struct device *dev, struct device_attribute *attr,
                           const char *buf, size_t count)
{
    struct bmi_spsc_device *spsc_dev = dev_get_drvdata(dev);
    long long val;
    int ret;
    
    if (!spsc_dev)
        return -EINVAL;
    
    ret = kstrtoll(buf, 10, &val);
    if (ret)
        return ret;
    
    spsc_dev->pid_kp = val;
    dev_info(dev, "PID Kp set to %lld (will apply on next start)\n", val);
    
    return count;
}

/**
 * @brief Show PID Kd gain
 */
static __maybe_unused ssize_t pid_kd_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_spsc_device *spsc_dev = dev_get_drvdata(dev);
    
    if (!spsc_dev)
        return -EINVAL;
    
    return sprintf(buf, "%lld\n", spsc_dev->pid_kd);
}

/**
 * @brief Set PID Kd gain (applied on next start)
 */
static __maybe_unused ssize_t pid_kd_store(struct device *dev, struct device_attribute *attr,
                           const char *buf, size_t count)
{
    struct bmi_spsc_device *spsc_dev = dev_get_drvdata(dev);
    long long val;
    int ret;
    
    if (!spsc_dev)
        return -EINVAL;
    
    ret = kstrtoll(buf, 10, &val);
    if (ret)
        return ret;
    
    spsc_dev->pid_kd = val;
    // dev_info(dev, "PID Kd set to %lld (will apply on next start)\n", val);
    dev_info(dev, "PID Kd set to %lld (removed_test)\n", val);
    
    return count;
}

/**
 * @brief Show PID Ki gain
 */
static __maybe_unused ssize_t pid_ki_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bmi_spsc_device *spsc_dev = dev_get_drvdata(dev);
    
    if (!spsc_dev)
        return -EINVAL;
    
    return sprintf(buf, "%lld\n", spsc_dev->pid_ki);
}

/**
 * @brief Set PID Ki gain (applied on next start)
 */
static __maybe_unused ssize_t pid_ki_store(struct device *dev, struct device_attribute *attr,
                           const char *buf, size_t count)
{
    struct bmi_spsc_device *spsc_dev = dev_get_drvdata(dev);
    long long val;
    int ret;
    
    if (!spsc_dev)
        return -EINVAL;
    
    ret = kstrtoll(buf, 10, &val);
    if (ret)
        return ret;
    
    spsc_dev->pid_ki = val;
    // dev_info(dev, "PID Ki set to %lld (will apply on next start)\n", val);
    dev_info(dev, "PID Ki set to %lld (removed_test)\n", val);
    
    return count;
}

static DEVICE_ATTR_RO(accel_available_ranges);
static DEVICE_ATTR_RO(gyro_available_ranges);
static DEVICE_ATTR_RO(accel_available_frequencies);
static DEVICE_ATTR_RO(gyro_available_frequencies);
static DEVICE_ATTR_RW(accel_range);
static DEVICE_ATTR_RW(gyro_range);
static DEVICE_ATTR_RW(accel_frequency);
static DEVICE_ATTR_RW(gyro_frequency);
static DEVICE_ATTR_RO(accel_temperature);
static DEVICE_ATTR_RO(i2c_bus);
static DEVICE_ATTR_RO(acc_addr);
static DEVICE_ATTR_RO(gyro_addr);
static DEVICE_ATTR_RO(name);
static DEVICE_ATTR_RO(target_cpu);
static DEVICE_ATTR_RO(sample_count);
static DEVICE_ATTR_RO(i2c_errors);
static DEVICE_ATTR_RO(ring_drops);
static DEVICE_ATTR_RO(fifo_overruns);
static DEVICE_ATTR_RO(ring_status);
static DEVICE_ATTR_RO(ring_buffer_size);
static DEVICE_ATTR_RO(timer_status);
static DEVICE_ATTR_WO(timer_control);
static DEVICE_ATTR_RO(timer_phase_offset);
static DEVICE_ATTR_RO(timer_next_expiry);
static DEVICE_ATTR_RO(timer_synchronization);
static DEVICE_ATTR_RO(dump_regs);
/* PID sysfs attributes removed: PID implementation remains internal and is not exposed to userspace. */

static struct attribute *bmi_spsc_attrs[] = {
    &dev_attr_target_cpu.attr,
    &dev_attr_sample_count.attr,
    &dev_attr_i2c_errors.attr,
    &dev_attr_ring_drops.attr,
    &dev_attr_fifo_overruns.attr,
    &dev_attr_ring_status.attr,
    &dev_attr_ring_buffer_size.attr,
    &dev_attr_timer_status.attr,
    &dev_attr_timer_control.attr,
    &dev_attr_timer_phase_offset.attr,
    &dev_attr_timer_next_expiry.attr,
    &dev_attr_timer_synchronization.attr,
    &dev_attr_accel_available_ranges.attr,
    &dev_attr_gyro_available_ranges.attr,
    &dev_attr_accel_available_frequencies.attr,
    &dev_attr_gyro_available_frequencies.attr,
    &dev_attr_accel_range.attr,
    &dev_attr_gyro_range.attr,
    &dev_attr_accel_frequency.attr,
    &dev_attr_gyro_frequency.attr,
    &dev_attr_accel_temperature.attr,
    &dev_attr_name.attr,
    &dev_attr_i2c_bus.attr,
    &dev_attr_acc_addr.attr,
    &dev_attr_gyro_addr.attr,
    &dev_attr_dump_regs.attr,
    NULL,
};

static struct attribute_group bmi_spsc_attr_group = {
    .attrs = bmi_spsc_attrs,
};

static const struct attribute_group *bmi_spsc_attr_groups[] = {
    &bmi_spsc_attr_group,
    NULL,
};

/**
 * @brief Cleanup partial device initialization on error path
 * @param spsc_dev Device structure to cleanup
 * @param wq Wait queue pointer (may be NULL)
 * @param stage Cleanup stage: 0=basic, 1=after workqueue, 2=after cdev_add, 3=after device_create
 */
static void bmi_spsc_device_init_cleanup(struct bmi_spsc_device *spsc_dev, 
                                        wait_queue_head_t *wq, int stage)
{
    if (!spsc_dev)
        return;

    if (stage > 3 || stage < 0)
        return;

    if (stage == 3) {
        /* device_create failed */
        cdev_del(&spsc_dev->cdev);
    }

    if (stage >= 2) {
        /* cdev_add failed */
        if (wq)
            kfree(wq);
    }

    if (stage >= 1) {
        /* workqueue init failed */
        bmi_spsc_i2c_workqueue_cleanup(spsc_dev);
    }

    if (stage >= 0) {
        /* basic cleanup for early failures */
        if (spsc_dev->ring) {
            spsc_ring_free(spsc_dev->ring);
        }
        kfree(spsc_dev);
    }
}

/**
 * @brief Initialize BMI SPSC device
 * @param bmi_id Sequential BMI device ID
 * @param st Back-reference to BMI spsc state
 * @return Pointer to allocated device or NULL on failure
 */
struct bmi_spsc_device *bmi_spsc_device_init(int bmi_id, struct bmi_state *st, struct bmi_spsc_fn_dev *fn_dev)
{
    struct bmi_spsc_device *spsc_dev;
    struct device *device;
    wait_queue_head_t *wq;
    int ret;
    
    // pr_info("BMI_SPSC: Initializing device bmi_id=%d\n", bmi_id);
    
    /* Allocate device structure */
    spsc_dev = kzalloc(sizeof(struct bmi_spsc_device), GFP_KERNEL);
    if (!spsc_dev) {
        pr_err("BMI_SPSC: Failed to allocate device structure\n");
        return NULL;
    }
    
    /* Initialize device fields */
    spsc_dev->bmi_id = bmi_id;
    spsc_dev->target_cpu = assign_bmi_cpu(bmi_id);
    snprintf(spsc_dev->device_name, sizeof(spsc_dev->device_name), 
             "spsc_bmi%d", bmi_id);
    spsc_dev->st = st;
    spsc_dev->fn_dev = fn_dev;
    spsc_dev->sample_count = 0;
    spsc_dev->i2c_errors = 0;

    /* Initialize VMA-based ownership tracking */
    atomic_set(&spsc_dev->mmap_count, 0);
    
    /* Initialize PID timestamp validation statistics */
    spsc_dev->total_batches = 0;
    spsc_dev->bounded_batches = 0;
    
    /* Initialize FIFO history management */
    memset(spsc_dev->accel_history, 0, sizeof(spsc_dev->accel_history));
    memset(spsc_dev->phase1_timestamps, 0, sizeof(spsc_dev->phase1_timestamps));
    memset(spsc_dev->phase2_timestamps, 0, sizeof(spsc_dev->phase2_timestamps));
    spsc_dev->accel_head = 0;
    
    /* Initialize batch history for D-term sliding window */
    memset(spsc_dev->batch_history, 0, sizeof(spsc_dev->batch_history));
    spsc_dev->batch_head = 0;
    spsc_dev->batch_valid_count = 0;
    spsc_dev->integral_error = 0;
    spsc_dev->phase_error_prev = 0;
    
    /* Initialize PID controller gains with default values */
    spsc_dev->pid_kp = 500LL;
    spsc_dev->pid_kd = 300LL;
    spsc_dev->pid_ki = 10LL;
    
    /* Initialize first sample tracking for tail synchronization */
    
    /* Allocate SPSC ring buffer */
    spsc_dev->ring = spsc_ring_alloc();
    if (!spsc_dev->ring) {
        pr_err("BMI_SPSC: Failed to allocate ring buffer for bmi_id=%d\n", bmi_id);
        kfree(spsc_dev);
        return NULL;
    }
    
    /* Initialize high-priority I2C workqueue */
    ret = bmi_spsc_i2c_workqueue_init(spsc_dev);
    if (ret) {
        pr_err("BMI_SPSC: Failed to initialize I2C workqueue for bmi_id=%d\n", bmi_id);
        bmi_spsc_device_init_cleanup(spsc_dev, NULL, 0);
        return NULL;
    }
    
    /* Initialize 200Hz timer for data acquisition */
    hrtimer_init(&spsc_dev->timer, CLOCK_REALTIME, HRTIMER_MODE_ABS_PINNED);
    spsc_dev->timer.function = bmi_spsc_timer_callback;
    
    /* Initialize reference count */
    kref_init(&spsc_dev->ref);

    /* Allocate and initialize wait queue */
    wq = kzalloc(sizeof(wait_queue_head_t), GFP_KERNEL);
    if (!wq) {
        pr_err("BMI_SPSC: Failed to allocate wait queue\n");
        bmi_spsc_device_init_cleanup(spsc_dev, NULL, 1);
        return NULL;
    }
    init_waitqueue_head(wq);
    spsc_dev->wait_queue_ptr = wq;
    
    /* Allocate and initialize character device */
    /* cdev is now embedded in spsc_dev */
    
    /* Setup device numbers */
    spsc_dev->devno_major = bmi_spsc_major;
    spsc_dev->devno_minor = bmi_id;
    
    /* Initialize character device */
    cdev_init(&spsc_dev->cdev, &bmi_spsc_fops);
    spsc_dev->cdev.owner = THIS_MODULE;
    
    /* Add character device to system */
    ret = cdev_add(&spsc_dev->cdev, MKDEV(bmi_spsc_major, bmi_id), 1);
    if (ret) {
        pr_err("BMI_SPSC: Failed to add cdev for bmi_id=%d, ret=%d\n", bmi_id, ret);
        bmi_spsc_device_init_cleanup(spsc_dev, wq, 1);
        return NULL;
    }
    
    /* Create device node */
    device = device_create(bmi_spsc_class, NULL, MKDEV(bmi_spsc_major, bmi_id), 
                          spsc_dev, spsc_dev->device_name);
    if (IS_ERR(device)) {
        pr_err("BMI_SPSC: Failed to create device for bmi_id=%d\n", bmi_id);
        bmi_spsc_device_init_cleanup(spsc_dev, wq, 2);
        return NULL;
    }
    spsc_dev->device_ptr = device;
    
    /* Create sysfs attributes */
    ret = sysfs_create_groups(&device->kobj, bmi_spsc_attr_groups);
    if (ret) {
        pr_warn("BMI_SPSC: Failed to create sysfs attributes for bmi_id=%d\n", bmi_id);
        /* Continue without sysfs attributes */
    }
    
    return spsc_dev;
}
EXPORT_SYMBOL_GPL(bmi_spsc_device_init);

/**
 * @brief Cleanup BMI SPSC device
 * @param spsc_dev Pointer to SPSC device to cleanup
 */
void bmi_spsc_device_cleanup(struct bmi_spsc_device *spsc_dev)
{
    struct device *device;
    
    if (!spsc_dev) {
        pr_warn("BMI_SPSC: Attempted to cleanup NULL device\n");
        return;
    }
    
    // pr_info("BMI_SPSC: Cleaning up device %s\n", spsc_dev->device_name);
    
    /* Remove sysfs attributes */
    device = (struct device *)spsc_dev->device_ptr;
    if (device) {
        sysfs_remove_groups(&device->kobj, bmi_spsc_attr_groups);
        device_destroy(bmi_spsc_class, MKDEV(spsc_dev->devno_major, spsc_dev->devno_minor));
    }
    
    /* Remove character device */
    cdev_del(&spsc_dev->cdev);
    
    /* Stop and cleanup timer */
    hrtimer_cancel(&spsc_dev->timer);
    
    /* Cleanup I2C workqueue */
    bmi_spsc_i2c_workqueue_cleanup(spsc_dev);
    
    /* Release reference - if last, will free memory */
    kref_put(&spsc_dev->ref, bmi_spsc_ref_release);
    
    // pr_info("BMI_SPSC: Device cleanup complete\n");
}
EXPORT_SYMBOL_GPL(bmi_spsc_device_cleanup);

/**
 * @brief Initialize BMI SPSC device management subsystem
 * @return 0 on success, negative error code on failure
 */
static int __init bmi_spsc_device_init_subsystem(void)
{
    int ret;
    
    // pr_info("BMI_SPSC: Initializing device management subsystem\n");
    
    /* Allocate major device number */
    ret = alloc_chrdev_region(&bmi_spsc_devno_base, 0, 256, BMI_SPSC_DEVICE_NAME);
    if (ret) {
        pr_err("BMI_SPSC: Failed to allocate chrdev region, ret=%d\n", ret);
        return ret;
    }
    bmi_spsc_major = MAJOR(bmi_spsc_devno_base);
    
    /* Create device class */
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 0, 0)
    bmi_spsc_class = class_create(THIS_MODULE, BMI_SPSC_CLASS_NAME);
#else
    bmi_spsc_class = class_create(BMI_SPSC_CLASS_NAME);
#endif
    if (IS_ERR(bmi_spsc_class)) {
        pr_err("BMI_SPSC: Failed to create device class\n");
        unregister_chrdev_region(bmi_spsc_devno_base, 256);
        return PTR_ERR(bmi_spsc_class);
    }
    
    // pr_info("BMI_SPSC: Device management subsystem initialized (major=%d)\n", 
    
    return 0;
}

/**
 * @brief Cleanup BMI SPSC device management subsystem
 */
static void __exit bmi_spsc_device_exit_subsystem(void)
{
    // pr_info("BMI_SPSC: Cleaning up device management subsystem\n");
    
    if (bmi_spsc_class) {
        class_destroy(bmi_spsc_class);
    }
    
    if (bmi_spsc_major) {
        unregister_chrdev_region(bmi_spsc_devno_base, 256);
    }
    
    // pr_info("BMI_SPSC: Device management subsystem cleanup complete\n");
}

/**
 * @brief Start SPSC data acquisition timer with absolute timing and phase offset
 * @param spsc_dev Pointer to SPSC device
 * @return 0 on success, negative error code on failure
 */
int bmi_spsc_start(struct bmi_spsc_device *spsc_dev)
{
    ktime_t base_time, start_time;
    u64 current_ns;
    int ret;
    int multiplier;
    u64 phase_offset_ns;
    if (!spsc_dev) {
        pr_err("BMI_SPSC: Cannot start NULL device\n");
        return -EINVAL;
    }
    if (hrtimer_active(&spsc_dev->timer))
        return -EBUSY;
    
    // pr_info("BMI_SPSC: Starting 200Hz data acquisition for %s (BMI ID: %d)\n", 
    //         spsc_dev->device_name, spsc_dev->bmi_id);
    
    /* Enable both accelerometer and gyroscope sensors */
    ret = spsc_dev->fn_dev->enable(spsc_dev->st, BMI_HW_ACC, 1);
    if (ret) {
        pr_err("BMI_SPSC: Failed to enable accelerometer: %d\n", ret);
        return ret;
    }
    
    ret = spsc_dev->fn_dev->enable(spsc_dev->st, BMI_HW_GYR, 1);
    if (ret) {
        pr_err("BMI_SPSC: Failed to enable gyroscope: %d\n", ret);
        /* Disable accel if gyro failed */
        spsc_dev->fn_dev->enable(spsc_dev->st, BMI_HW_ACC, 0);
        return ret;
    }
    
    usleep_range(40000, 45000); /* 40-45ms delay for sensors to stabilize */

    /* Flush FIFO to clear any accumulated data from startup */
    ret = spsc_dev->fn_dev->flush_fifo(spsc_dev->st, BMI_HW_GYR);
    if (ret) {
        pr_warn("BMI_SPSC: Failed to flush FIFO (continuing anyway): %d\n", ret);
        /* Don't fail startup for flush failure */
    } 
    
    /* Reset ring buffer to clear any stale data from previous session */
    spsc_ring_reset(spsc_dev->ring);
    
    /* Mark that we need to sync tail with head after first sample */
    spsc_dev->first_sample_written = false;
    
    // pr_info("BMI_SPSC: BMI%d ring buffer reset and zeroed\n", spsc_dev->bmi_id);

    /* Reset timestamp state for clean start */
    spsc_dev->gyro_sample_interval_ns = 0;      /* Force reinitialization */
    spsc_dev->last_assigned_timestamp = 0;       /* Clear old timestamps */
    spsc_dev->last_poll_time = 0;                /* Reset poll time tracking */
    memset(spsc_dev->accel_history, 0, sizeof(spsc_dev->accel_history));
    memset(spsc_dev->batch_history, 0, sizeof(spsc_dev->batch_history));
    spsc_dev->accel_head = 0;
    spsc_dev->batch_head = 0;
    spsc_dev->batch_valid_count = 0;
    spsc_dev->integral_error = 0;
    spsc_dev->phase_error_prev = 0;
    spsc_dev->ring_drops = 0;
    spsc_dev->fifo_overruns = 0;
    spsc_dev->i2c_errors = 0;
    
    // pr_info("BMI_SPSC: BMI%d timestamp state reset for fresh start\n", spsc_dev->bmi_id);
    
    /* Reset sample count for clean start */
    spsc_dev->sample_count = 0;
    
    /* Reset PID validation statistics for this session */
    spsc_dev->total_batches = 0;
    spsc_dev->bounded_batches = 0;
    
    /* Calculate absolute start time with dephasing offset per camera */
    current_ns = ktime_get_ns();
    
    /* Base time: round up (current + 10ms) to next 10ms boundary for synchronized start 
     * This ensures all devices start at the same absolute time boundary */
    base_time = ktime_set(0, ((current_ns + 10000000ULL + (2 * BMI_SPSC_TIMER_PERIOD_NS) - 1) / 
                              (2 * BMI_SPSC_TIMER_PERIOD_NS)) * (2 * BMI_SPSC_TIMER_PERIOD_NS));
    
    /* Phase offset: multiplier * 1.25ms = multiplier * 1250000ns
     * Multipliers: [0, 2, 1, 3] for IDs [0, 1, 2, 3] = [0ms, 2.5ms, 1.25ms, 3.75ms]
     * This spreads devices across the 5ms window with better I2C bus spacing */
    multiplier = bmi_get_dephasing_multiplier(spsc_dev->bmi_id);
    phase_offset_ns = multiplier * 1250000ULL;
    start_time = ktime_add_ns(base_time, phase_offset_ns);
    
    /* Start high-resolution timer with absolute time and CPU pinning */
    hrtimer_start(&spsc_dev->timer, start_time, HRTIMER_MODE_ABS_PINNED);
    
    return 0;
}

/**
 * @brief Stop SPSC data acquisition timer
 * @param spsc_dev Pointer to SPSC device
 */
void bmi_spsc_stop(struct bmi_spsc_device *spsc_dev)
{
    if (!spsc_dev) {
        pr_warn("BMI_SPSC: Cannot stop NULL device\n");
        return;
    }
    
    // pr_info("BMI_SPSC: Stopping data acquisition for %s\n", spsc_dev->device_name);
    
    /* Cancel timer */
    hrtimer_cancel(&spsc_dev->timer);
    
    /* Flush any pending work */
    flush_workqueue(spsc_dev->i2c_workqueue);

#ifdef VERBOSE
    /* Print PID timestamp validation statistics */
    if (spsc_dev->total_batches > 0) {
        long long percentage = (spsc_dev->bounded_batches * 100) / spsc_dev->total_batches;
        pr_info("BMI_SPSC: BMI%d PID Stats: %lld/%lld batches bounded (%lld%%) [newest_ts in poll_time [-5ms,0ms]]\n",
                spsc_dev->bmi_id, spsc_dev->bounded_batches, spsc_dev->total_batches, percentage);
    }
#endif
    
    
    /* Disable sensors (reverse order of start: gyro off then accel) */
    spsc_dev->fn_dev->enable(spsc_dev->st, BMI_HW_GYR, 0);
    spsc_dev->fn_dev->enable(spsc_dev->st, BMI_HW_ACC, 0);
}


module_init(bmi_spsc_device_init_subsystem);
module_exit(bmi_spsc_device_exit_subsystem);


MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("BMI088 SPSC Ring Buffer Core Implementation");
MODULE_AUTHOR("STEREOLABS <support@stereolabs.com>");
MODULE_VERSION("1.0.1");
