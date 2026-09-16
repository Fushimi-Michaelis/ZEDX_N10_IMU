#ifndef _BMI_SPSC_H_
#define _BMI_SPSC_H_

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/poll.h>
#include <linux/ktime.h>
#include <linux/hrtimer.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/random.h>
#include <linux/cdev.h>
#include <linux/wait.h>
#include <linux/atomic.h>
#include <linux/kref.h>
#include <asm/barrier.h>
#include <linux/errno.h>
#include <linux/cpumask.h>
#include <asm/unaligned.h>
#include <linux/types.h>
#include <linux/kernel.h>  /* For PAGE_SIZE */

/* Forward declarations */
struct bmi_state;

/* SPSC-related structures for BMI088 core */
struct bmi_spsc_float {
	int ival;
	int fval;
};

struct bmi_spsc_sensor_cfg {
	const char *name;
	int snsr_id;
	unsigned int ch_n;
	const char *part;
	struct bmi_spsc_float max_range;
	int delay_us_max;
	signed char matrix[9];
	unsigned int float_significance;
	struct bmi_spsc_float scale;
};

/* SPSC constants */
#define BMI_SPSC_VAL_INT_PLUS_MICRO 1
#define BMI_SPSC_VAL_INT 1
#define BMI_SPSC_MOD_X 1
#define BMI_STS_SUSPEND (1 << 1)
#define BMI_STS_SHUTDOWN (1 << 0)
#define DECIMAL 10
/* SPSC function device structure */
struct bmi_spsc_fn_dev {
	int (*enable)(void *st, int snsr_id, int enable);
	int (*regs)(void *st, int snsr_id, char *buf);
	int (*temp)(void *st, int snsr_id, int *val, int *val2);
	int (*freq_read)(void *st, int snsr_id, int *val, int *val2);
	int (*freq_write)(void *st, int snsr_id, int val, int val2);
	int (*scale_write)(void *st, int snsr_id, int val, int val2);
	int (*max_range)(void *st, int snsr_id, int max_range);
	int (*read_err)(void *st, int snsr_id, char *buf);
	int (*get_data)(void *st, int snsr_id, int axis, int *val);
	int (*get_burst)(void *st, int snsr_id, u8 *buf, size_t len);
	int (*get_accel)(void *st, u8 *buf, size_t len);  /* Read accelerometer data only */
	int (*get_gyro)(void *st, u8 *buf, size_t len, bool read_fifo);  /* Read gyro: if read_fifo=false just length, if true read FIFO data */
	int (*i2c_bus)(void *st);  /* Get I2C bus number */
	int (*i2c_addr)(void *st, int hw_id);  /* Get I2C address for specific hardware (ACC/GYR) */
	int (*get_accel_freq)(void *st);  /* Get current accelerometer frequency */
	int (*get_gyro_freq)(void *st);   /* Get current gyroscope frequency */
	int (*get_accel_range)(void *st); /* Get current accelerometer range */
	int (*get_gyro_range)(void *st);  /* Get current gyroscope range */
	int (*flush_fifo)(void *st, int snsr_id); /* Flush/reset FIFO to clear accumulated data */
	unsigned int *sts;
};


/* SPSC Ring Buffer Constants */
#define BMI_SPSC_BUFFER_SIZE 1024  /* Must be power of 2 for efficient modulo */
#define BMI_SPSC_DATA_SIZE 20      /* packed u64 + six s16 */

/**
 * @brief BMI088 sensor data structure for SPSC ring buffer
 * 
 * Combined accelerometer and gyroscope data in single packet
 * Total size: 20 bytes (packed ABI; do not use a 22-byte stride).
 */
struct bmi_sensor_data {
    __u64 timestamp;      /* 8 bytes - nanosecond timestamp */
    __s16 gx, gy, gz;     /* 6 bytes - gyroscope data */
    __s16 ax, ay, az;     /* 6 bytes - accelerometer data */
} __packed;

/**
 * @brief Single Producer Single Consumer Ring Buffer
 * 
 * Lock-free ring buffer optimized for high-frequency sensor data
 * Uses separate cache lines for head/tail to avoid false sharing
 */
struct spsc_ring {
    volatile __u32 head;  /* Producer write index */
    volatile __u32 tail;  /* Consumer read index */ 
    struct bmi_sensor_data data[BMI_SPSC_BUFFER_SIZE];
};

/**
 * @brief BMI SPSC device state
 * 
 * Per-device state for BMI088 SPSC implementation
 */
struct bmi_spsc_device {
    int bmi_id;                         /* Sequential BMI device ID */
    int target_cpu;                     /* CPU affinity (bmi_id % nproc) */
    char device_name[32];               /* "spsc_bmi0", "spsc_bmi1" */
    /* SPSC Ring Buffer */
    struct spsc_ring *ring;             /* 1024-entry ring buffer */
    
    /* Back-reference to BMI state */
    struct bmi_state *st;           /* BMI state for I2C operations */
    struct bmi_spsc_fn_dev *fn_dev; /* Function pointers for I2C operations */
    
    /* High-priority I2C workqueue */
    struct workqueue_struct *i2c_workqueue;    /* Dedicated I2C workqueue */
    struct work_struct i2c_work;               /* I2C read work item */
    
    /* 200Hz Timer for data acquisition */
    struct hrtimer timer;                      /* High-resolution timer */
    
    /* FIFO history management for timestamp coordination */
    struct {
        s16 ax, ay, az;                        /* Accelerometer data */
        u64 timestamp;                         /* Read timestamp */
    } accel_history[8];                        /* Last 8 accelerometer samples (max FIFO=7) */
    int accel_head;                            /* Current write position in accel_history */
    
    /* Timestamp management with FIFO queue for timer callbacks */
    u64 last_poll_time;                        /* Last poll time (for drift calculation) */
    u64 last_assigned_timestamp;               /* Last timestamp assigned to a gyro sample */
    u64 gyro_sample_interval_ns;               /* Dynamic sample interval in ns (starts at 4.9ms) */
    u64 total_samples_received;                /* Total gyro samples received for interval adjustment */
    bool first_sample_written;                 /* Flag to track first sample after enable */
    
    u64 phase1_timestamps[4];                  /* Last 4 Phase1 read timestamps for delta_t calculation */
    u64 phase2_timestamps[4];                  /* Last 4 Phase2 read timestamps */
    
    struct {
        u64 poll_time;
        u8 sample_count;
    } batch_history[6];
    int batch_head;
    int batch_valid_count;
    
    s64 integral_error;
    s64 phase_error_prev;
    
    /* PID controller gains (tunable via sysfs) */
    s64 pid_kp;
    s64 pid_kd;
    s64 pid_ki;
    
    /* Device and resource pointers - opaque in header */    /* Device and resource pointers - opaque in header */
    struct cdev cdev;                     /* Character device */
    void *device_ptr;                   /* struct device * */
    void *wait_queue_ptr;               /* wait_queue_head_t * */
    
    /* Statistics */
    long long sample_count;             /* Total samples produced */
    long long i2c_errors;               /* I2C read error count */
    long long ring_drops;               /* Samples rejected by full ring */
    long long fifo_overruns;            /* Hardware FIFO overrun observations */
    int devno_major;                    /* Major device number */
    int devno_minor;                    /* Minor device number */
    
    /* PID timestamp validation statistics */
    long long total_batches;            /* Total number of batches processed */
    long long bounded_batches;          /* Batches where newest sample was in [-5ms, 0ms] of poll time */
    
    /* VMA-based ownership tracking */
    atomic_t mmap_count;           /* Number of active mmap VMAs (0 = free, >0 = owned) */
    
    /* Reference counting for safe cleanup */
    struct kref ref;               /* Reference count for device lifetime management */
};

/* Function prototypes */

/**
 * @brief Allocate and initialize SPSC ring buffer
 * @return Pointer to allocated ring buffer or NULL on failure
 */
struct spsc_ring *spsc_ring_alloc(void);

/**
 * @brief Free SPSC ring buffer
 * @param ring Pointer to ring buffer to free
 */
void spsc_ring_free(struct spsc_ring *ring);

/**
 * @brief Initialize BMI SPSC device
 * @param bmi_id Sequential BMI device ID
 * @param st Back-reference to BMI spsc state
 * @return Pointer to allocated device or NULL on failure
 */
struct bmi_spsc_device *bmi_spsc_device_init(int bmi_id, struct bmi_state *st, struct bmi_spsc_fn_dev *fn_dev);

/**
 * @brief Cleanup BMI SPSC device
 * @param spsc_dev Pointer to SPSC device to cleanup
 */
void bmi_spsc_device_cleanup(struct bmi_spsc_device *spsc_dev);

/**
 * @brief Start SPSC data acquisition
 * @param spsc_dev Pointer to SPSC device
 * @return 0 on success, negative error code on failure
 */
int bmi_spsc_start(struct bmi_spsc_device *spsc_dev);

/**
 * @brief Stop SPSC data acquisition
 * @param spsc_dev Pointer to SPSC device
 */
void bmi_spsc_stop(struct bmi_spsc_device *spsc_dev);

/**
 * @brief Check if SPSC ring buffer is empty
 * @param ring Pointer to SPSC ring buffer
 * @return true if empty, false otherwise
 */
bool spsc_ring_empty(struct spsc_ring *ring);

/**
 * @brief Get statistics from SPSC ring buffer
 * @param ring Pointer to SPSC ring buffer
 * @param head_out Output for head index
 * @param tail_out Output for tail index
 * @param count_out Output for count of elements
 * @return 0 on success, negative error code on failure
 */
int spsc_ring_get_stats(struct spsc_ring *ring, __u32 *head_out, 
                        __u32 *tail_out, __u32 *count_out);

/**
 * @brief Write sensor data to SPSC ring buffer
 * @param ring Pointer to SPSC ring buffer
 * @param data Pointer to sensor data to write
 * @return 0 on success, negative error code on failure
 */
int spsc_ring_write(struct spsc_ring *ring, const struct bmi_sensor_data *data);

/**
 * @brief Check if SPSC ring buffer is full
 * @param ring Pointer to SPSC ring buffer
 * @return true if full, false otherwise
 */
bool spsc_ring_full(struct spsc_ring *ring);

/**
 * @brief Get count of elements in SPSC ring buffer
 * @param ring Pointer to SPSC ring buffer
 * @return Number of elements in the ring buffer
 */
__u32 spsc_ring_count(struct spsc_ring *ring);

/**
 * @brief Read sensor data from SPSC ring buffer
 * @param ring Pointer to SPSC ring buffer
 * @param data Pointer to buffer for read data
 * @return 0 on success, negative error code on failure
 */
int spsc_ring_read(struct spsc_ring *ring, struct bmi_sensor_data *data);

/**
 * @brief Reset SPSC ring buffer
 * @param ring Pointer to SPSC ring buffer
 * @return 0 on success, negative error code on failure
 */
int spsc_ring_reset(struct spsc_ring *ring);



#endif /* _BMI_SPSC_H_ */
