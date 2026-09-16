#ifndef _BMI088_CORE_H_
#define _BMI088_CORE_H_

#include <linux/version.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/bitops.h>
#include <linux/ktime.h>     /* For ktime functions */
#include <linux/wait.h>      /* For wait queues */
#include <linux/device/bus.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/types.h>

/* Forward declarations */
struct bmi_state;

/* FIFO Frame Definitions */
#define BMI_FIFO_ACC_HEADER_PATTERN     (0x84)  /* 0b10000100 */
#define BMI_FIFO_ACC_FRAME_SIZE         (7)     /* 1 header + 6 data bytes */
#define BMI_FIFO_GYR_FRAME_SIZE         (6)     /* 6 data bytes, no header */
#define BMI_FIFO_ACC_DATA_SIZE          (6)     /* Actual data bytes per frame */
#define BMI_FIFO_MAX_FRAMES             (50)    /* Number of frames to read - 2000Hz * 20ms = 40 samples + margin */

/* FIFO Buffer sizes */
#define BMI_FIFO_ACC_BUFFER_SIZE        (BMI_FIFO_ACC_FRAME_SIZE * BMI_FIFO_MAX_FRAMES)
#define BMI_FIFO_GYR_BUFFER_SIZE        (BMI_FIFO_GYR_FRAME_SIZE * BMI_FIFO_MAX_FRAMES)
#define BMI_FIFO_TIMESTAMP_SIZE         (8)     /* Timestamp bytes */

/* Define frame header masks for different frame types */
#define ACC_FRAME_MASK            (0xFC)  /* 0b11111100 */
#define ACC_FRAME_HEADER          (0x84)  /* 0b10000100 */
#define SKIP_FRAME_HEADER         (0x40)  /* 0b01000000 */
#define SENSORTIME_FRAME_HEADER   (0x44)  /* 0b01000100 */
#define DROP_FRAME_HEADER         (0x50)  /* 0b01010000 */
#define CONFIG_FRAME_HEADER       (0x48)  /* 0b01001000 */

/* Delays */
#define BMI_ACC_SOFTRESET_DELAY_MS	(50)
#define BMI_GYR_SOFTRESET_DELAY_MS	(50)
#define BMI_ACC_PM_DELAY_MS		(5)
#define BMI_GYR_PM_DELAY_MS		(30)
#define BMI_HW_DELAY_POR_MS		(10)
#define BMI_HW_DELAY_DEV_ON_US		(2)
#define BMI_HW_DELAY_DEV_OFF_US		(1000)

/* Accelerometer Registers */
/* Register: Identification */
#define BMI_REG_ACC_CHIP_ID		(0x00)

/* Register: Error and Status */
#define BMI_REG_ACC_ERR_REG		(0x02)
#define BMI_REG_ACC_STATUS		(0x03)

/* Register: Data */
#define BMI_REG_ACC_DATA		(0x12)
#define BMI_REG_SENSORTIME_2		(0x1A)
#define BMI_REG_ACC_INT_STAT_1		(0x1D)
#define BMI_REG_TEMP_MSB		(0x22)
#define BMI_REG_TEMP_LSB		(0x23)
#define BMI_REG_FIFO_LENGTH		(0x24)

/* Register: FIFO */
#define BMI_REG_FIFO_ACC_DATA		(0x26)
#define BMI_REG_FIFO_DOWNS		(0x45)
#define BMI_FIFO_WTM_LEVEL_0          (0x46)
#define BMI_FIFO_WTM_LEVEL_1          (0x47)
#define BMI_REG_ACC_FIFO_CFG_0		(0x48)
#define BMI_REG_ACC_FIFO_CFG_1		(0x49)

/* Gyroscope FIFO-related Interrupt Controls */
#define BMI_REG_GYR_INT_CTRL_FIFO_EN    (0x40)  /* Enable FIFO interrupt in INT_CTRL register */
#define BMI_REG_INT_3_4_IO_CONF_3_PP    (0x02)  /* Configure INT3 pin as push-pull output */
#define BMI_REG_INT_3_4_IO_MAP_FIFO     (0x04)  /* Map FIFO watermark interrupt to INT3/4 pin */
#define BMI_REG_FIFO_WM_EN_BIT          (0x00)  /* Disable watermark interrupt */
#define BMI_GYR_WM_LEVEL          (0x03)  /* FIFO GYR watermark level  */
/* Gyroscope FIFO Modes */
#define BMI_GYR_FIFO_MODE_STREAM        (0x40)  /* STREAM mode: overwrites oldest data when full */
#define BMI_GYR_FIFO_MODE_FIFO          (0x40)  /* FIFO mode: stops writing when full */

/* Accelerometer FIFO Configuration Bits */
#define BMI_ACC_FIFO_CFG_1_RESERVED     (0x10)  /* Reserved bit that must be set to 1 */
#define BMI_ACC_FIFO_CFG_0_STREAM       (0x00)  /* STREAM mode for accelerometer FIFO */
#define BMI_ACC_FIFO_CFG_0_FIFO         (0x01)  /* FIFO mode for accelerometer FIFO */
#define BMI_ACC_FIFO_CFG_0_RESERVED     (0x02)  /* Reserved bit that must be set to 1 */

/* Accelerometer Interrupt Mapping */
#define BMI_REG_INT_MAP_DATA_FIFO_WM    (0x02)  /* Map FIFO watermark interrupt to INT1 pin */

/* Register: Configuration */
#define BMI_REG_ACC_CONF		(0x40)
#define BMI_REG_ACC_CONF_BWP_POR	(0xA0)
#define BMI_REG_ACC_CONF_BWP_MSK	(0xF0)
#define BMI_REG_ACC_RANGE		(0x41)

/* Register: Interrupt */
#define BMI_REG_INT1_IO_CTRL		(0x53)
#define BMI_REG_INT2_IO_CTRL		(0x54)
#define BMI_REG_ACCEL_INIT_CTRL		(0x59)
#define BMI_REG_INTX_IO_CTRL_OUT_EN	(0x08)
#define BMI_REG_INTX_IO_CTRL_ACTV_HI	(0x02)
#define BMI_REG_INT_MAP_DATA		(0x58)
#define BMI_INT1_OUT_ACTIVE_HIGH	(0x0A)
#define BMI_INT1_DTRDY			(0x04)

/* Register: Power */
#define BMI_REG_ACC_PWR_CONF		(0x7C)
#define BMI_REG_ACC_PWR_CONF_ACTV	(0x00)
#define BMI_REG_ACC_PWR_CONF_SUSP	(0x03)
#define BMI_REG_ACC_PWR_CTRL		(0x7D)
#define BMI_REG_ACC_PWR_CTRL_OFF	(0x00)
#define BMI_REG_ACC_PWR_CTRL_ON		(0x04)

/* Register: Soft Reset */
#define BMI_REG_ACC_SOFTRESET		(0x7E)
#define BMI_REG_ACC_SOFTRESET_FIFO	(0xB0)
#define BMI_REG_ACC_SOFTRESET_EXE	(0xB6)

/* Gyroscope Registers */

/* Register: Identification */
#define BMI_REG_GYR_CHIP_ID		(0x00)

/* Register: Error and Status */
#define BMI_REG_GYR_INT_STAT_1		(0x0A)

/* Register: Data */
#define BMI_REG_GYR_DATA		(0x02)
#define BMI_REG_GYR_Z_MSB		(0x07)

/* Register: FIFO */
#define BMI_REG_FIFO_GYR_DATA		(0x3F)
#define BMI_REG_FIFO_STATUS		(0x0E)
#define BMI_REG_GYR_FIFO_CFG_1		(0x3E)
#define BMI_FIFO_WM_EN        (0x1E)
// #define BMI_FIFO_WM_EN_BIT      (0x88)  /* Enable watermark interrupt (bit 7 set) */
#define BMI_FIFO_WM_EN_BIT      (0x00)  /* Disable watermark interrupt (bit 7 set) */

/* Register: Configuration */
#define BMI_REG_GYR_RANGE		(0x0F)
#define BMI_REG_GYR_BW			(0x10)
#define BMI_REG_GYR_LPM1		(0x11)
#define BMI_REG_GYR_LPM1_NORM		(0x00)
#define BMI_REG_GYR_LPM1_DEEP		(0x20)
#define BMI_REG_GYR_LPM1_SUSP		(0x80)

/* Register: Interrupt */
#define BMI_REG_GYR_INT_CTRL		(0x15)
#define BMI_REG_GYR_INT_CTRL_DIS	(0x00)
#define BMI_REG_GYR_INT_CTRL_DATA_EN	(0x80)
#define BMI_REG_INT_3_4_IO_CONF		(0x16)
#define BMI_REG_INT_3_4_IO_CONF_3_HI	(0x01)
#define BMI_REG_INT_3_4_IO_CONF_4_HI	(0x04)
#define BMI_REG_INT_3_4_IO_MAP		(0x18)
#define BMI_REG_INT_3_4_IO_MAP_INT3	(0x01)
#define BMI_REG_INT_3_ACTIVE_HIGH	(0x01)
#define BMI_REG_FIFO_EXT_INT_S		(0x34) /* FIFO sync register */
#define BMI_REG_GYR_FIFO_CFG_0		(0x3D)
/* Register: Self Test */
#define BMI_REG_GYR_SELF_TEST		(0x3C)

/* Register: Soft Reset */
#define BMI_REG_GYR_SOFTRESET		(0x14)
#define BMI_REG_GYR_SOFTRESET_EXE	(0xB6)

#define BMI_AXIS_N			(3)
#define BMI_IMU_DATA			(6)

#define BMI_NAME			"bmi088"
/* hardware devices */
#define BMI_HW_ACC			(0)
#define BMI_HW_GYR			(1)
#define BMI_HW_N			(2)

#define BMI_PART_BMI088			(0)


#define ERR_DESYNC         -100    /* Desynchronization error */
struct bmi_snsr {
	struct bmi_spsc_device *spsc_dev;  /* SPSC device for high-performance streaming */
	struct bmi_rrs *rrs;
	struct bmi_spsc_sensor_cfg cfg;
	unsigned int usr_cfg;
	unsigned int period_us;
	u64 irq_ts;
	u64 irq_ts_old;
	u64 seq;
	struct bmi_state *st;
};

/* FIFO Data Structures for burst read operations */
#define GYRO_HISTORY_SIZE 4
#define ACCEL_HISTORY_SIZE 4

/**
 * @brief Gyroscope data structure
 */
struct gyro_data {
    s16 x;
    s16 y;
    s16 z;
};

/**
 * @brief Accelerometer data structure with timestamp
 */
struct accel_data {
    s16 x;
    s16 y;
    s16 z;
    u64 timestamp;
};

/**
 * @brief Circular buffer for gyroscope history
 */
struct gyro_history {
    struct gyro_data buffer[GYRO_HISTORY_SIZE];
    int head;  /* Index for newest data */
    int tail;  /* Index for oldest data */
};

/**
 * @brief Circular buffer for accelerometer history
 */
struct accel_history {
    struct accel_data buffer[ACCEL_HISTORY_SIZE];
    int head;  /* Index for newest data */
    int tail;  /* Index for oldest data */
};

/**
 * @brief Data structure for ReadBurstLenRaw operation
 */
struct fifo_burst_len_data {
    s16 ax, ay, az;         /* Accelerometer data */
    u16 fifo_length;         /* FIFO length */
    u64 timestamp;          /* Read timestamp */
};

/**
 * @brief Data structure for ReadBurstFIFORaw operation
 */
struct fifo_burst_fifo_data {
    s16 ax, ay, az;                                  /* Accelerometer data */
    struct gyro_data gyro_samples[BMI_FIFO_MAX_FRAMES];   /* Gyroscope FIFO samples (max 50) */
    u16 sample_count;                                 /* Number of gyro samples */
    u64 timestamp;                                   /* Read timestamp */
    u64 fifo_status_lock_timestamp;                         /* Exact timestamp when FIFO status register was read */
};

/**
 * bmi_flush_fifo - Flush/reset the gyroscope FIFO to clear accumulated data
 * @client: BMI state structure
 * @snsr_id: Sensor identifier (should be BMI_HW_GYR)
 * 
 * This function resets the gyroscope FIFO by writing to the FIFO_CONFIG_1 register.
 * This is essential during startup to clear any accumulated samples.
 * 
 * Return: 0 on success, negative error code on failure
 */
int bmi_flush_fifo(void *client, int snsr_id);

/**
 * @brief Get burst data from BMI sensor
 * @param client Pointer to I2C client or device context
 * @param snsr_id Sensor identifier (accelerometer or gyroscope)
 * @param buf Buffer to store burst data
 * @param len Length of buffer in bytes
 * @return 0 on success, negative error code on failure
 */
int bmi_get_burst(void *client, int snsr_id, u8 *buf, size_t len);

/**
 * @brief Read only accelerometer data (called every 5ms)
 * @param client BMI state structure
 * @param buf Buffer to store fifo_burst_len_data (only ax, ay, az, timestamp used)
 * @param len Length of buffer in bytes
 * @return 0 on success, negative error code on failure
 */
int bmi_get_accel(void *client, u8 *buf, size_t len);

/**
 * @brief Read gyroscope FIFO with conditional FIFO data read (called every 20ms per camera)
 * @param client BMI state structure
 * @param buf Buffer to store fifo_burst_fifo_data
 * @param len Length of buffer in bytes
 * @param read_fifo If true, read FIFO data; if false, only check length
 * @return 0 on success, ERR_DESYNC if FIFO empty, negative error code on failure
 */
int bmi_get_gyro(void *client, u8 *buf, size_t len, bool read_fifo);

#endif /* _BMI088_CORE_H_ */