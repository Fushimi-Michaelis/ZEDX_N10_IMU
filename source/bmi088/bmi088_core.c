// SPDX-License-Identifier: GPL-2.0-only
// SPDX-FileCopyrightText: Copyright (c) 2023-2024, NVIDIA CORPORATION & AFFILIATES. All rights reserved.

/* Device tree example:
 *
 * bmi088@69 {
 *   compatible = "bmi,bmi088";
 *   reg = <0x69>; // <-- Must be gyroscope I2C address
 *   accel_i2c_addr = <0x19>; // Must be specified
 *   accel_irq-gpios = <&tegra_gpio TEGRA_GPIO(BB, 0) GPIO_ACTIVE_HIGH>;
 *   gyro_irq-gpios = <&tegra_gpio TEGRA_GPIO(BB, 1) GPIO_ACTIVE_HIGH>;
 *   accel_matrix    = [01 00 00 00 01 00 00 00 01];
 *   gyro_matrix        = [01 00 00 00 01 00 00 00 01];
 * };
 */

//#include <nvidia/conftest.h>

#include "bmi_spsc.h"
#include "bmi088_core.h"


#define BMI_RR_ACC_BMI088_SIZE 4
#define BMI_RR_GYR_SIZE 5

// Struct Definitions //

static const struct i2c_device_id bmi_i2c_device_ids[] = {
	{ BMI_NAME, BMI_PART_BMI088 },
	{},
};

struct bmi_reg_rd {
	u8 reg_lo;
	u8 reg_hi;
};

static struct bmi_reg_rd bmi_reg_rds_acc[] = {
	{
		.reg_lo			= BMI_REG_ACC_CHIP_ID,
		.reg_hi			= BMI_REG_ACC_STATUS,
	},
	{
		.reg_lo			= BMI_REG_ACC_DATA,
		.reg_hi			= BMI_REG_SENSORTIME_2,
	},
	{
		.reg_lo			= BMI_REG_ACC_INT_STAT_1,
		.reg_hi			= BMI_REG_ACC_INT_STAT_1,
	},
	{
		.reg_lo			= BMI_REG_TEMP_MSB,
		.reg_hi			= BMI_REG_FIFO_ACC_DATA,
	},
	{
		.reg_lo			= BMI_REG_ACC_CONF,
		.reg_hi			= BMI_REG_ACC_RANGE,
	},
	{
		.reg_lo			= BMI_REG_FIFO_DOWNS,
		.reg_hi			= BMI_REG_ACC_FIFO_CFG_1,
	},
	{
		.reg_lo			= BMI_REG_INT1_IO_CTRL,
		.reg_hi			= BMI_REG_INT2_IO_CTRL,
	},
	{
		.reg_lo			= BMI_REG_INT_MAP_DATA,
		.reg_hi			= BMI_REG_INT_MAP_DATA,
	},
	{
		.reg_lo			= BMI_REG_ACC_PWR_CONF,
		.reg_hi			= BMI_REG_ACC_SOFTRESET,
	}
};

static struct bmi_reg_rd bmi_reg_rds_gyr[] = {
	{
		.reg_lo			= BMI_REG_GYR_CHIP_ID,
		.reg_hi			= BMI_REG_GYR_Z_MSB,
	},
	{
		.reg_lo			= BMI_REG_GYR_INT_STAT_1,
		.reg_hi			= BMI_REG_GYR_INT_STAT_1,
	},
	{
		.reg_lo			= BMI_REG_FIFO_STATUS,
		.reg_hi			= BMI_REG_GYR_LPM1,
	},
	{
		.reg_lo			= BMI_REG_GYR_SOFTRESET,
		.reg_hi			= BMI_REG_INT_3_4_IO_CONF,
	},
	{
		.reg_lo			= BMI_REG_INT_3_4_IO_MAP,
		.reg_hi			= BMI_REG_INT_3_4_IO_MAP,
	},
	{
		.reg_lo			= BMI_REG_FIFO_EXT_INT_S,
		.reg_hi			= BMI_REG_FIFO_EXT_INT_S,
	},
	{
		.reg_lo			= BMI_REG_GYR_SELF_TEST,
		.reg_hi			= BMI_REG_GYR_FIFO_CFG_1,
	},
	// Add a new range specifically for FIFO configuration registers to ensure they're included
	{
		.reg_lo			= BMI_REG_GYR_FIFO_CFG_0,
		.reg_hi			= BMI_REG_GYR_FIFO_CFG_1,
	},
};

static struct bmi_spsc_sensor_cfg bmi_snsr_cfgs[] = {
	{
		.name			= "accelerometer",
		.snsr_id		= BMI_HW_ACC,
		.ch_n			= BMI_AXIS_N,
		.part			= BMI_NAME,
		.max_range		= {
			.ival		= 2, /* default = RANGE_12G (index 2) */
		},
		.delay_us_max		= 80000, // 12.5Hz min ODR
		/* default matrix to get the attribute */
		.matrix[0]		= 1,
		.matrix[4]		= 1,
		.matrix[8]		= 1,
		.float_significance	= BMI_SPSC_VAL_INT_PLUS_MICRO,
	},
	{
		.name			= "gyroscope",
		.snsr_id		= BMI_HW_GYR,
		.ch_n			= BMI_AXIS_N,
		.max_range		= {
			.ival		= 1, /* default = RANGE_1000_DPS (index 1) */
		},
		.delay_us_max		= 10000, // 100Hz min ODR 
		.matrix[0]		= 1,
		.matrix[4]		= 1,
		.matrix[8]		= 1,
		.float_significance	= BMI_SPSC_VAL_INT_PLUS_MICRO,
	},
};

struct bmi_rr {
	struct bmi_spsc_float max_range;
	struct bmi_spsc_float resolution;
};

static struct bmi_rr bmi_rr_acc_bmi088[] = {
	/* all accelerometer values are in g's (9.80665 m/s2) fval = NANO scale */
		{
			.max_range		= {
				.ival		= 29,
				.fval		= 419950000,
			},
			.resolution		= {
				.ival		= 0,
				.fval		= 897,
			},
		},
		{
			.max_range		= {
				.ival		= 58,
				.fval		= 839900000,
			},
			.resolution		= {
				.ival		= 0,
				.fval		= 1795,
			},
		},
		{
			.max_range		= {
				.ival		= 117,
				.fval		= 679800000,
			},
			.resolution		= {
				.ival		= 0,
				.fval		= 3591,
			},
		},
		{
			.max_range		 = {
				.ival		= 235,
				.fval		= 359600000,
			},
			.resolution		= {
				.ival		= 0,
				.fval		= 7182,
			},
		},
	};
	
static struct bmi_rr bmi_rr_gyr[] = {
		/* rad / sec  fval = NANO scale */
			{
				.max_range		= {
					.ival		= 34,
					.fval		= 906585040,
				},
				.resolution		= {
					.ival		= 0,
					.fval		= 1065,
				},
			},
			{
				.max_range		= {
					.ival		= 17,
					.fval		= 453292520,
				},
				.resolution		= {
					.ival		= 0,
					.fval		= 532,
				},
			},
			{
				.max_range		= {
					.ival		= 8,
					.fval		= 726646260,
				},
				.resolution		= {
					.ival		= 0,
					.fval		= 266,
				},
			},
			{
				.max_range		= {
					.ival		= 4,
					.fval		= 363323130,
				},
				.resolution		= {
					.ival		= 0,
					.fval		= 133,
				},
			},
			{
				.max_range		 = {
				.ival		= 2,
				.fval		= 181661565,
			},
			.resolution		= {
				.ival		= 0,
				.fval		= 66,
			},
		},
	};
		
struct bmi_rrs {
	struct bmi_rr *rr;
	unsigned int rr_0n;
};

static struct bmi_rrs bmi_rrs_acc[] = {
	{
		.rr			= bmi_rr_acc_bmi088,
		.rr_0n			= BMI_RR_ACC_BMI088_SIZE - 1,
	},
};

static struct bmi_rrs bmi_rrs_gyr[] = {
	{
		.rr			= bmi_rr_gyr,
		.rr_0n			= BMI_RR_GYR_SIZE - 1,
	},
};

struct bmi_state;

static int bmi_acc_able(struct bmi_state *st, int en, bool map_fifo);
static int bmi_acc_batch(struct bmi_state *st, unsigned int period_us,
			 bool range);
static int bmi_acc_softreset(struct bmi_state *st, unsigned int hw);
static int bmi_acc_pm(struct bmi_state *st, unsigned int hw, int able);
static int bmi_gyr_able(struct bmi_state *st, int en, bool map_fifo);
static int bmi_gyr_batch(struct bmi_state *st, unsigned int period_us,
			 bool range);
static int bmi_gyr_softreset(struct bmi_state *st, unsigned int hw);
static int bmi_gyr_pm(struct bmi_state *st, unsigned int hw, int able);
struct bmi_hw {
	struct bmi_reg_rd *reg_rds;
	struct bmi_rrs *rrs;
	unsigned int reg_rds_n;
	unsigned int rrs_0n;
	int (*fn_able)(struct bmi_state *st, int en, bool map_fifo);
	int (*fn_batch)(struct bmi_state *st, unsigned int period_us,
			bool range);
	int (*fn_softreset)(struct bmi_state *st, unsigned int hw);
	int (*fn_pm)(struct bmi_state *st, unsigned int hw, int able);
	unsigned long (*fn_irqflags)(struct bmi_state *st);
};

static struct bmi_hw bmi_hws[] = {
	{
		.reg_rds		= bmi_reg_rds_acc,
		.rrs			= bmi_rrs_acc,
		.reg_rds_n		= ARRAY_SIZE(bmi_reg_rds_acc),
		.rrs_0n			= ARRAY_SIZE(bmi_rrs_acc) - 1,
		.fn_able		= &bmi_acc_able,
		.fn_batch		= &bmi_acc_batch,
		.fn_softreset	= &bmi_acc_softreset,
		.fn_pm			= &bmi_acc_pm,
	},
	{
		.reg_rds		= bmi_reg_rds_gyr,
		.rrs			= bmi_rrs_gyr,
		.reg_rds_n		= ARRAY_SIZE(bmi_reg_rds_gyr),
		.rrs_0n			= ARRAY_SIZE(bmi_rrs_gyr) - 1,
		.fn_able		= &bmi_gyr_able,
		.fn_batch		= &bmi_gyr_batch,
		.fn_softreset	= &bmi_gyr_softreset,
		.fn_pm			= &bmi_gyr_pm,
	},
};
struct bmi_state {
	struct i2c_client *i2c;
	struct bmi_snsr snsrs[BMI_HW_N];
	bool spsc_init_done[BMI_HW_N];
	struct bmi_spsc_device *spsc_dev;  /* Single SPSC device for combined accel+gyro data */
	unsigned int part;
	unsigned int sts;
	unsigned int errs_bus[BMI_HW_N];
	unsigned int enabled;
	unsigned int suspend_en_st;
	unsigned int hw_n;
	unsigned int hw_en;
	s64 ts_hw[BMI_HW_N];
	u8 ra_0x53;
	u8 ra_0x54;
	u8 ra_0x58;
	u8 rg_0x16;
	u8 rg_0x18;
	u16 i2c_addrs[BMI_HW_N];
	
	/* FIFO state for new implementation */
	struct accel_history accel_queue;
	struct gyro_history gyro_queue;
	u64 fifo_timestamps[2];
	bool fifo_mode_active;
	unsigned int fifo_sample_count;  /* For prototype testing */

	/* Current sensor settings */
	int accel_freq_hz;
	int gyro_freq_hz;
	int accel_range;
	int gyro_range;
};

static inline s64 get_ktime_timestamp(void)
{
	struct timespec64 ts;

	ktime_get_ts64(&ts);

	return timespec64_to_ns(&ts);
}

static int bmi_i2c_rd(struct bmi_state *st, unsigned int hw,
		      u8 reg, u16 len, void *buf)
{
	struct i2c_msg msg[2];
	int ret = -ENODEV;

	if (st->i2c_addrs[hw]) {
		msg[0].addr = st->i2c_addrs[hw];
		msg[0].flags = 0;
		msg[0].len = 1;
		msg[0].buf = &reg;
		msg[1].addr = st->i2c_addrs[hw];
		msg[1].flags = I2C_M_RD;
		msg[1].len = len;
		msg[1].buf = buf;
		ret = i2c_transfer(st->i2c->adapter, msg, 2);
		if (ret != 2) {
			st->errs_bus[hw]++;
			ret = -EIO;
		} else {
			ret = 0;
		}
	}

	return ret;
}

/**
 * bmi_i2c_doublerd - Read data from both accelerometer and gyroscope in one I2C transfer
 * @st: BMI state structure
 * @acc_buf: Buffer to store accelerometer data (6 bytes)
 * @gyro_buf: Buffer to store gyroscope data (6 bytes)
 *
 * This function performs a burst read of 48 bits (6 bytes) from each sensor's
 * raw data register in a single I2C transfer for efficiency.
 *
 * Return: 0 on success, negative error code on failure
 */
static int bmi_i2c_doublerd(struct bmi_state *st, u8 *acc_buf, u8 *gyro_buf)
{
	struct i2c_msg msg[4];
	u8 acc_reg = BMI_REG_ACC_DATA;
	u8 gyro_reg = BMI_REG_GYR_DATA;
	int ret = -ENODEV;

	/* Check if both sensor addresses are available */
	if (!st->i2c_addrs[BMI_HW_ACC] || !st->i2c_addrs[BMI_HW_GYR])
		return -ENODEV;

	/* Message 0: Write accelerometer register address */
	msg[0].addr = st->i2c_addrs[BMI_HW_ACC];
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &acc_reg;

	/* Message 1: Read 6 bytes from accelerometer */
	msg[1].addr = st->i2c_addrs[BMI_HW_ACC];
	msg[1].flags = I2C_M_RD;
	msg[1].len = 6;
	msg[1].buf = acc_buf;

	/* Message 2: Write gyroscope register address */
	msg[2].addr = st->i2c_addrs[BMI_HW_GYR];
	msg[2].flags = 0;
	msg[2].len = 1;
	msg[2].buf = &gyro_reg;

	/* Message 3: Read 6 bytes from gyroscope */
	msg[3].addr = st->i2c_addrs[BMI_HW_GYR];
	msg[3].flags = I2C_M_RD;
	msg[3].len = 6;
	msg[3].buf = gyro_buf;

	ret = i2c_transfer(st->i2c->adapter, msg, 4);
	if (ret != 4) {
		/* Increment error counters for both sensors on failure */
		dev_err(&st->i2c->dev, "ERR: I2C transfer failed\n");
		st->errs_bus[BMI_HW_ACC]++;
		st->errs_bus[BMI_HW_GYR]++;
		ret = -EIO;
	} else {
		ret = 0;
	}

	return ret;
}

static int bmi_i2c_w(struct bmi_state *st, unsigned int hw,
		     u16 len, u8 *buf)
{
	struct i2c_msg msg;
	int ret;
	s64 ts;

	if (st->i2c_addrs[hw]) {
		msg.addr = st->i2c_addrs[hw];
		msg.flags = 0;
		msg.len = len;
		msg.buf = buf;
		ts = st->ts_hw[hw];
		if (st->hw_en & (1 << hw))
			ts += (BMI_HW_DELAY_DEV_ON_US * 1000);
		else
			ts += (BMI_HW_DELAY_DEV_OFF_US * 1000);
		ts -= get_ktime_timestamp();
		if (ts > 0) {
			ts /= 1000; /* ns => us */
			ts++;
			udelay(ts);
		}
		ret = i2c_transfer(st->i2c->adapter, &msg, 1);
		st->ts_hw[hw] = get_ktime_timestamp();
		if (ret != 1) {
			st->errs_bus[hw]++;
			ret = -EIO;
		} else {
			ret = 0;
		}
	} else {
		ret = -ENODEV;
	}

	return ret;
}

// Bmi I2c Write function
static int bmi_i2c_wr(struct bmi_state *st, unsigned int hw, u8 reg, u8 val) 
{
	int ret;
	u8 buf[2];

	buf[0] = reg;
	buf[1] = val;
	ret = bmi_i2c_w(st, hw, sizeof(buf), buf);
	if (ret)
		dev_dbg(&st->i2c->dev, "ERR: 0x%02X=>0x%02X\n", reg, val);
	return ret;
}



/* bmi_pm - Power management function for BMI sensors. */

static int bmi_pm(struct bmi_state *st, int snsr_id, bool en)
{
	unsigned int i;
	int ret = 0;

	if (en) {
		if (!st->hw_en)
			/* first time power on, wait for power on reset time */
			mdelay(BMI_HW_DELAY_POR_MS);

		if (snsr_id < 0) {
			for (i = 0; i < st->hw_n; i++)
				ret |= bmi_hws[i].fn_pm(st, i, 1);
		} else {
			if (snsr_id >= st->hw_n)
				return -ENODEV;

			ret = bmi_hws[snsr_id].fn_pm(st, snsr_id, 1);
		}
	} else {
		if (snsr_id < 0) {
			for (i = 0; i < st->hw_n; i++) {
				ret |= bmi_hws[i].fn_pm(st, i, 0);
				st->enabled &= ~(1 << i);
			}
		} else {
			if (snsr_id >= st->hw_n)
				return -ENODEV;

			dev_dbg(&st->i2c->dev, "turning off:%d\n", snsr_id);
			ret = bmi_hws[snsr_id].fn_pm(st, snsr_id, 0);
			st->enabled &= ~(1 << snsr_id);
		}
	}

	if (ret) {
		if (snsr_id < 0)
			dev_err(&st->i2c->dev, "ALL pm_en=%x  ERR=%d\n",
				en, ret);
		else
			dev_err(&st->i2c->dev, "%s pm_en=%x  ERR=%d\n",
				st->snsrs[snsr_id].cfg.name, en, ret);
	}

	return ret;
}

struct bmi_odr {
	unsigned int period_us;
	u8 hw;
	unsigned int odr_hz;
	unsigned int nodr_hz_mant;
};

// This function returns the index of the ODR that is less than or equal to
// the specified period in microseconds. 
static unsigned int bmi_odr_i(struct bmi_odr *odrs, unsigned int odrs_n,
			      unsigned int period_us)
{
	unsigned int i;

	for (i = 0; i < odrs_n; i++) {
		if (period_us >= odrs[i].period_us)
			break;
	}

	return i;
}

static struct bmi_odr bmi_odrs_acc[] = {
	{ 80000, 0x05, 12, 500000 },
	{ 40000, 0x06, 25, 0 },
	{ 20000, 0x07, 50, 0 },
	{ 10000, 0x08, 100, 0 },
	{ 5000,  0x09, 200, 0 },
	{ 2500,  0x0A, 400, 0 },
	{ 1250,  0x0B, 800, 0 },
	{ 625,   0x0C, 1600, 0 },
};

/* Configures ODR and range */
// TODO  - Add fixed range configuration
static int bmi_acc_batch(struct bmi_state *st, unsigned int period_us,
			 bool range)
{
	u8 val;
	unsigned int odr_i;
	int ret = 0;

	odr_i = bmi_odr_i(bmi_odrs_acc, ARRAY_SIZE(bmi_odrs_acc), period_us);

	val = bmi_odrs_acc[odr_i].hw; 
	val |= BMI_REG_ACC_CONF_BWP_POR;
	
	ret = bmi_i2c_wr(st, BMI_HW_ACC, BMI_REG_ACC_CONF, val);
	if (!ret)
		st->snsrs[BMI_HW_ACC].period_us = bmi_odrs_acc[odr_i].period_us;
		//st->snsrs[BMI_HW_ACC].period_us = 5000;
	if (range)
		ret |= bmi_i2c_wr(st, BMI_HW_ACC, BMI_REG_ACC_RANGE,
				  (u8)st->snsrs[BMI_HW_ACC].usr_cfg);

dev_dbg(&st->i2c->dev, "odr_i=%d  period_us=%d  val=0x%02X\n",
			odr_i, period_us, val);
	return ret;
}

/* Maps and set/reset the data ready interrupt */
static int bmi_acc_able(struct bmi_state *st, int en, bool map_fifo)
{
	int ret = 0;
dev_dbg(&st->i2c->dev, "bmi_acc_able: %d\n", en);
	if (!en)
		return bmi_i2c_wr(st, BMI_HW_ACC, BMI_REG_INT_MAP_DATA, 0x0);

	if (map_fifo) {
		dev_dbg(&st->i2c->dev, "map_fifo %d\n", en);
		ret = bmi_i2c_wr(st, BMI_HW_ACC, BMI_REG_INT1_IO_CTRL,
				 st->ra_0x53);
		ret |= bmi_i2c_wr(st, BMI_HW_ACC, BMI_REG_INT2_IO_CTRL,
				  st->ra_0x54);
				  
		// Configure FIFO watermark level (set to 21 bytes = 3 full xyz frames)
		ret |= bmi_i2c_wr(st, BMI_HW_ACC, BMI_FIFO_WTM_LEVEL_0, 0x00); 
		ret |= bmi_i2c_wr(st, BMI_HW_ACC, BMI_FIFO_WTM_LEVEL_1, 0x00);
		
		// Configure FIFO mode to stream mode
		ret |= bmi_i2c_wr(st, BMI_HW_ACC, BMI_REG_ACC_FIFO_CFG_0, 
				  0x02); // Stream mode with reserved bit set
		
		// Enable storing accelerometer data in FIFO
		ret |= bmi_i2c_wr(st, BMI_HW_ACC, BMI_REG_ACC_FIFO_CFG_1, 
				  0x50); // Enable accel data (0x40) + reserved bit (0x10)
	}

	// Map both data ready and FIFO watermark interrupt to INT1
	ret |= bmi_i2c_wr(st, BMI_HW_ACC, BMI_REG_INT_MAP_DATA, st->ra_0x58);

	return ret;
}

/* Execute accelerometer soft reset and wait for reset to complete */
static int bmi_acc_softreset(struct bmi_state *st, unsigned int hw)
{
	int ret;

	ret = bmi_i2c_wr(st, BMI_HW_ACC, BMI_REG_ACC_SOFTRESET,
			 BMI_REG_ACC_SOFTRESET_EXE);/* Gyroscope FIFO-related Interrupt Controls */
	mdelay(BMI_ACC_SOFTRESET_DELAY_MS);

	st->hw_en &= ~(1 << hw);
	st->enabled &= ~(1 << hw);

	return ret;
}

// Sets the accelerometer power mode to either active or suspend
static int bmi_acc_pm(struct bmi_state *st, unsigned int hw, int able)
{
	int ret;
	u8 pwr_conf;
	u8 pwr_on_off;

	if (able) {
		pwr_conf = BMI_REG_ACC_PWR_CONF_ACTV;
		pwr_on_off = BMI_REG_ACC_PWR_CTRL_ON;
		st->hw_en |= (1 << hw);
	} else {
		pwr_conf = BMI_REG_ACC_PWR_CONF_SUSP;
		pwr_on_off = BMI_REG_ACC_PWR_CTRL_OFF;
		st->hw_en &= ~(1 << hw);
	}

	ret = bmi_i2c_wr(st, BMI_HW_ACC, BMI_REG_ACC_PWR_CONF,
			 pwr_conf);

	mdelay(BMI_ACC_PM_DELAY_MS);


	ret |= bmi_i2c_wr(st, BMI_HW_ACC, BMI_REG_ACC_PWR_CTRL,
			  pwr_on_off);

	mdelay(BMI_ACC_PM_DELAY_MS);

	if (ret)
		st->hw_en &= ~(1 << hw);

	return ret;
}


static struct bmi_odr bmi_odrs_gyr[] = {
	{ 10000, 0x05, 100, 0 },
	{ 5000,  0x06, 200, 0 }, // Thracybulus: 0x06 instead of 0x04 Filter Reasons, Pierre asked for it.
	{ 2500,  0x03, 400, 0 },
	{ 1000,  0x02, 1000, 0 },
	{ 500,   0x01, 2000, 0 },
};

static int bmi_gyr_batch(struct bmi_state *st, unsigned int period_us,
			 bool range)
{
	u8 val;
	unsigned int odr_i;
	int ret = 0;

	odr_i = bmi_odr_i(bmi_odrs_gyr, ARRAY_SIZE(bmi_odrs_gyr), period_us);

	val = bmi_odrs_gyr[odr_i].hw; 
    dev_dbg(&st->i2c->dev, "DBG: odr_i=%d  period_us=%d  val=0x%02X\n",
			odr_i, period_us, val);
	ret = bmi_i2c_wr(st, BMI_HW_GYR, BMI_REG_GYR_BW, val);
	if (!ret)
		st->snsrs[BMI_HW_GYR].period_us = bmi_odrs_gyr[odr_i].period_us;
	if (range) {
		val = st->snsrs[BMI_HW_GYR].usr_cfg;
		ret |= bmi_i2c_wr(st, BMI_HW_GYR, BMI_REG_GYR_RANGE, val);
	}

	return ret;
}

/* Map and set/reset data ready interrupt */
static int bmi_gyr_able(struct bmi_state *st, int en, bool map_fifo)
{
	int ret = 0;

	if (!en)
		return bmi_i2c_wr(st, BMI_HW_GYR, BMI_REG_GYR_INT_CTRL,
				  0x00);

	if (map_fifo) {
		ret = bmi_i2c_wr(st, BMI_HW_GYR, BMI_REG_INT_3_4_IO_CONF,
				 st->rg_0x16);
		ret |= bmi_i2c_wr(st, BMI_HW_GYR, BMI_REG_INT_3_4_IO_MAP,
				  st->rg_0x18);			  
		// Enable FIFO watermark interrupt
		ret |= bmi_i2c_wr(st, BMI_HW_GYR, BMI_FIFO_WM_EN,
			BMI_FIFO_WM_EN_BIT);
		// Set FIFO external interrupt sync
		ret |= bmi_i2c_wr(st, BMI_HW_GYR, BMI_REG_FIFO_EXT_INT_S,
			0x20);
		// Set watermark level to 3 frames (18 bytes)
		ret |= bmi_i2c_wr(st, BMI_HW_GYR, BMI_REG_GYR_FIFO_CFG_0,
			BMI_GYR_WM_LEVEL);			
		// Set FIFO to stream mode
		ret |= bmi_i2c_wr(st, BMI_HW_GYR, BMI_REG_GYR_FIFO_CFG_1,
			BMI_GYR_FIFO_MODE_STREAM);
	}

	// Disable data ready interrupt
	ret |= bmi_i2c_wr(st, BMI_HW_GYR, BMI_REG_GYR_INT_CTRL,
			  0x00);

	return ret;
}

static int bmi_gyr_softreset(struct bmi_state *st, unsigned int hw)
{
	int ret;

	ret = bmi_i2c_wr(st, BMI_HW_GYR, BMI_REG_GYR_SOFTRESET,
			 BMI_REG_GYR_SOFTRESET_EXE);
	mdelay(BMI_GYR_SOFTRESET_DELAY_MS);

	st->hw_en &= ~(1 << hw);
	st->enabled &= ~(1 << hw);

	return ret;
}

static int bmi_gyr_pm(struct bmi_state *st, unsigned int hw, int able)
{
	int ret;
	u8 val;

	if (able) {
		val = BMI_REG_GYR_LPM1_NORM;
		st->hw_en |= (1 << hw);
	} else {
		val = BMI_REG_GYR_LPM1_SUSP;
		st->hw_en &= ~(1 << hw);
	}

	ret = bmi_i2c_wr(st, BMI_HW_GYR, BMI_REG_GYR_LPM1, val);
	if (ret)
		st->hw_en &= ~(1 << hw);

	mdelay(BMI_GYR_PM_DELAY_MS);

	return ret;
}

static int bmi_period(struct bmi_state *st, int snsr_id, bool range)
{
	if (snsr_id >= st->hw_n)
		return -ENODEV;


	return bmi_hws[snsr_id].fn_batch(st, st->snsrs[snsr_id].period_us,
					 range);
}

static int bmi_enable(void *client, int snsr_id, int enable)
{
	struct bmi_state *st = (struct bmi_state *)client;
	int ret;

	if (snsr_id >= st->hw_n) // check if sensor ID is valid
		return -ENODEV;

	if (enable < 0) // Check if enable is negative
		return (st->enabled & (1 << snsr_id)); // return current state

	if (enable) {
		enable = st->enabled | (1 << snsr_id);
		ret = bmi_pm(st, snsr_id, true); // enable power management
		if (ret < 0)
			return ret;

		ret = bmi_period(st, snsr_id, true); // set period (ODR)
		ret |= bmi_hws[snsr_id].fn_able(st, 1, true); // enable sensor 
		if (!ret) {
			st->enabled = enable;
			return ret;
		}
	}
	// dev_err(&st->i2c->dev, "Disabling snsr_id %d\n", snsr_id);
	ret = bmi_hws[snsr_id].fn_able(st, 0, false); // disable sensor
	ret |= bmi_pm(st, snsr_id, false);

	return ret;
}

static inline struct bmi_odr *bmi_find_odrs(struct bmi_state *st, int snsr_id,
					    unsigned int *sz)
{
	struct bmi_odr *odr;

	if (!st || !sz || (snsr_id >= st->hw_n))
		return NULL;

	if (snsr_id == BMI_HW_GYR) {
		odr = bmi_odrs_gyr;
		*sz = ARRAY_SIZE(bmi_odrs_gyr);
	} else if (snsr_id == BMI_HW_ACC) {
		odr = bmi_odrs_acc;
		*sz = ARRAY_SIZE(bmi_odrs_acc);
	} else {
		return NULL;
	}

	return odr;
}

static int bmi_read_odrs(struct bmi_state *st, int snsr_id, int *val,
			 int *val2)
{
	struct bmi_odr *odr;
	unsigned int o_sz;
	unsigned int index;

	if (!st || (snsr_id >= st->hw_n))
		return -EINVAL;

	odr = bmi_find_odrs(st, snsr_id, &o_sz);
	if (!odr)
		return -EINVAL;

	index = bmi_odr_i(odr, o_sz, st->snsrs[snsr_id].period_us);
	if (index >= o_sz)
		return -EINVAL;

	*val = odr[index].odr_hz;
	*val2 = odr[index].nodr_hz_mant;

	return index;
}

static int bmi_find_freq(struct bmi_odr *odr, unsigned int o_sz,
			 int val, int val2)
{
	unsigned int i;

	for (i = 0; i < o_sz; i++) {
		if (val == odr[i].odr_hz && val2 == odr[i].nodr_hz_mant)
			break;
	}

	if (i >= o_sz)
		return -EINVAL;

	return i;
}

static int bmi_freq_write(void *client, int snsr_id, int val, int val2)
{
	struct bmi_state *st = (struct bmi_state *)client;
	int odr_i;
	struct bmi_odr *odr;
	unsigned int o_sz, old_period;
	int ret = 0;

	if (!st || (snsr_id >= st->hw_n))
		return -EINVAL;

	odr = bmi_find_odrs(st, snsr_id, &o_sz);
	if (!odr)
		return -EINVAL;

	odr_i = bmi_find_freq(odr, o_sz, val, val2);
	if (odr_i < 0)
		return -EINVAL;

	old_period = st->snsrs[snsr_id].period_us;

	st->snsrs[snsr_id].period_us = odr[odr_i].period_us;

	ret = bmi_period(st, snsr_id, false);
	if (ret)
		st->snsrs[snsr_id].period_us = old_period;
	else {
		/* Store the frequency for sysfs readback */
		if (snsr_id == BMI_HW_ACC)
			st->accel_freq_hz = val;
		else if (snsr_id == BMI_HW_GYR)
			st->gyro_freq_hz = val;
	}

	return ret;
}

static int bmi_freq_read(void *client, int snsr_id, int *val, int *val2)
{
	struct bmi_state *st = (struct bmi_state *)client;
	int odr_i;
	int ret = 0;

	if (!val || !val2 || !st)
		return -EINVAL;

	odr_i = bmi_read_odrs(st, snsr_id, val, val2);

	if (unlikely(odr_i < 0))
		return -EINVAL;

	return ret;
}

static int bmi_max_range(void *client, int snsr_id, int max_range)
{
	struct bmi_state *st = (struct bmi_state *)client;
	unsigned int i = max_range;

	if (st->enabled & (1 << snsr_id))
		/* can't change settings on the fly (disable device first) */
		return -EBUSY;

	if (st->snsrs[snsr_id].rrs) {
		if (i > st->snsrs[snsr_id].rrs->rr_0n)
			/* clamp to highest setting */
			i = st->snsrs[snsr_id].rrs->rr_0n;
		st->snsrs[snsr_id].usr_cfg = i;
		st->snsrs[snsr_id].cfg.max_range.ival =
				  st->snsrs[snsr_id].rrs->rr[i].max_range.ival;
		st->snsrs[snsr_id].cfg.max_range.fval =
				  st->snsrs[snsr_id].rrs->rr[i].max_range.fval;
		st->snsrs[snsr_id].cfg.scale.ival =
				 st->snsrs[snsr_id].rrs->rr[i].resolution.ival;
		st->snsrs[snsr_id].cfg.scale.fval =
				 st->snsrs[snsr_id].rrs->rr[i].resolution.fval;
		
		/* Store the range index for sysfs readback */
		if (snsr_id == BMI_HW_ACC)
			st->accel_range = i;
		else if (snsr_id == BMI_HW_GYR)
			st->gyro_range = i;
	}

	return 0;
}

static int bmi_scale_write(void *client, int snsr_id, int val, int val2)
{
	struct bmi_state *st = (struct bmi_state *)client;
	int ret = 0;
	int i = 0;
	bool was_enabled = false;

	if (!st || (snsr_id >= st->hw_n))
		return -ENODEV;

	for (; i <= st->snsrs[snsr_id].rrs->rr_0n; i++) {
		if (st->snsrs[snsr_id].rrs->rr[i].resolution.ival == val &&
		    st->snsrs[snsr_id].rrs->rr[i].resolution.fval == val2)
			break;
	}
	if (i > st->snsrs[snsr_id].rrs->rr_0n)
		return -EINVAL;

	if (snsr_id == BMI_HW_GYR) {
		ret = bmi_i2c_wr(st, BMI_HW_GYR, BMI_REG_GYR_RANGE, i);
	} else if (snsr_id == BMI_HW_ACC) {
		/* For accelerometer, ensure device is powered on before writing range register */
		was_enabled = !!(st->hw_en & (1 << BMI_HW_ACC));
		
		/* If not already enabled, temporarily enable the accelerometer */
		if (!was_enabled) {
			dev_dbg(&st->i2c->dev, "Temporarily enabling accelerometer for range change\n");
			ret = bmi_acc_pm(st, BMI_HW_ACC, 1);
			if (ret) {
				dev_err(&st->i2c->dev, "Failed to enable accelerometer for range change: %d\n", ret);
				return ret;
			}
		}
		
		/* Write the range register */
		ret = bmi_i2c_wr(st, BMI_HW_ACC, BMI_REG_ACC_RANGE, i);
		
		/* If we temporarily enabled it, disable it again */
		if (!was_enabled) {
			int disable_ret = bmi_acc_pm(st, BMI_HW_ACC, 0);
			if (disable_ret) {
				dev_warn(&st->i2c->dev, "Failed to disable accelerometer after range change: %d\n", disable_ret);
			}
			dev_dbg(&st->i2c->dev, "Accelerometer disabled after range change\n");
		}
	} else {
		return -ENODEV;
	}

	if (!ret) {
		st->snsrs[snsr_id].usr_cfg = i;
		st->snsrs[snsr_id].cfg.max_range.ival =
				  st->snsrs[snsr_id].rrs->rr[i].max_range.ival;
		st->snsrs[snsr_id].cfg.max_range.fval =
				  st->snsrs[snsr_id].rrs->rr[i].max_range.fval;
		st->snsrs[snsr_id].cfg.scale.ival =
				 st->snsrs[snsr_id].rrs->rr[i].resolution.ival;
		st->snsrs[snsr_id].cfg.scale.fval =
				 st->snsrs[snsr_id].rrs->rr[i].resolution.fval;
	}

	return ret;
}

static int bmi_read_err(void *client, int snsr_id, char *buf)
{
	ssize_t t = 0;
	struct bmi_state *st = (struct bmi_state *)client;

	if (snsr_id >= st->hw_n)
		return -ENODEV;

	t += snprintf(buf, PAGE_SIZE, "%s:\n", st->snsrs[snsr_id].cfg.name);
	t += snprintf(buf + t, PAGE_SIZE - t,
		      "I2C Bus Errors:%u\n", st->errs_bus[snsr_id]);
	return t;
}

static int bmi_get_data(void *client, int snsr_id, int axis, int *val)
{
	struct bmi_state *st = (struct bmi_state *)client;
	u8 reg;
	__le16 sample;
	int ret = 0;

	if (snsr_id >= st->hw_n)
		return -ENODEV;

	if (snsr_id == BMI_HW_ACC)
		reg = BMI_REG_ACC_DATA;
	else
		reg = BMI_REG_GYR_DATA;

        reg += (axis - BMI_SPSC_MOD_X) * sizeof(__le16);	ret = bmi_i2c_rd(st, snsr_id, reg, sizeof(__le16), &sample);
	if (!ret)
		*val = sign_extend32(le16_to_cpu(sample), 15);

	return ret;
}

static int bmi_regs(void *client, int snsr_id, char *buf)
{
	struct bmi_state *st = (struct bmi_state *)client;
	struct bmi_reg_rd *reg_rd;
	ssize_t t;
	u8 reg;
	u8 val;
	unsigned int i;
	int ret;

	if (snsr_id >= st->hw_n)
		return -ENODEV;

	t = snprintf(buf, PAGE_SIZE, "register:value\n");
	reg_rd = bmi_hws[snsr_id].reg_rds;
	for (i = 0; i < bmi_hws[snsr_id].reg_rds_n; i++) {
		for (reg = reg_rd[i].reg_lo; reg <= reg_rd[i].reg_hi; reg++) {
			ret = bmi_i2c_rd(st, snsr_id, reg, 1, &val);
			if (ret)
				t += snprintf(buf + t, PAGE_SIZE - t,
						  "0x%02X=ERR\n", i);
			else
				t += snprintf(buf + t, PAGE_SIZE - t,
						  "0x%02X=0x%02X\n", reg, val);
		}
	}

	return t;
}

static int bmi_temp(void *client, int snsr_id, int *val, int *val2)
{
	struct bmi_state *st = (struct bmi_state *)client;
	u8 data[2] = { 0 };
	u16 msb, lsb;
	u16 msblsb;
	s16 temp;
	int sensor_temp;
	int ret;

	if (!st || !val || !val2)
		return -EINVAL;

	if (snsr_id != BMI_HW_ACC)
		return -ENODEV;

	ret = bmi_i2c_rd(st, BMI_HW_ACC, BMI_REG_TEMP_MSB, 2, data);
	if (ret)
		return ret;

	msb = (data[0] << 3);   /* MSB data */
	lsb = (data[1] >> 5);   /* LSB data */
	msblsb = (u16)(msb + lsb);
	if (msblsb > 1023) {
		temp = (s16)(msblsb - 2048);
	} else {
		temp = (s16)msblsb;
	}
	sensor_temp = (temp * 125) + 23000;
	*val = sensor_temp;
	*val2 = 0;
	return 0;
}

/**
 * bmi_get_burst - Read multiple bytes from a register
 * @client: Device client data
 * @snsr_id: Sensor ID (accelerometer or gyroscope)
 * @reg: Starting register address
 * @buf: Buffer to store read data
 * @len: Number of bytes to read
 *
 * This function reads multiple bytes from the specified register.
 * It's optimized for reading all axes data at once using double read.
 *
 * Return: 0 on success, negative error code on failure
 */
int bmi_get_burst(void *client, int snsr_id, u8 *buf, size_t len)
{
    struct bmi_state *st = (struct bmi_state *)client;
    u8 double_buf[12]; /* 96-bit buffer: GYR X,Y,Z (6 bytes) + ACC X,Y,Z (6 bytes) */
    int ret;

    if (snsr_id >= st->hw_n)
        return -ENODEV;

    /* Use double read for data registers */
	if (len == 12) {
        /* Read both sensors' data (12 bytes total) */
        ret = bmi_i2c_doublerd(st, &double_buf[6], &double_buf[0]);
        if (ret)
            return ret;
        memcpy(buf, double_buf, 12);
	}
	else {
		return -EINVAL; // Invalid length
	}
    return 0;
}

static int bmi_i2c_bus(void *client)
{
	struct bmi_state *st = (struct bmi_state *)client;
	
	if (!st || !st->i2c || !st->i2c->adapter)
		return -EINVAL;
	
	return st->i2c->adapter->nr;
}

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
int bmi_flush_fifo(void *client, int snsr_id)
{
	struct bmi_state *st = (struct bmi_state *)client;
	int ret;
	u8 flush_cmd = 0x03;  /* FIFO flush command for BMI088 */
	u8 stream_cmd = 0x40; /* Stop mode command for BMI088 */
	if (!st || snsr_id != BMI_HW_GYR)
		return -EINVAL;

	/* Check if gyroscope address is available */
	if (!st->i2c_addrs[BMI_HW_GYR])
		return -ENODEV;

	/* Write flush command to BMI_REG_GYR_FIFO_CFG_0 register (0x3D) */
	ret = bmi_i2c_wr(st, BMI_HW_GYR, BMI_REG_GYR_FIFO_CFG_0, flush_cmd);
	if (ret) {
		dev_err(&st->i2c->dev, "Failed to flush FIFO: %d\n", ret);
		return ret;
	}

	/* Send stream command to FIFO_CONFIG_1 register (0x3E) */
	ret = bmi_i2c_wr(st, BMI_HW_GYR, 0x3E, stream_cmd);
	if (ret) {
		dev_err(&st->i2c->dev, "Failed to set stop mode: %d\n", ret);
		return ret;
	}

	/* Reset stored FIFO state */
	st->fifo_sample_count = 0;

	dev_dbg(&st->i2c->dev, "FIFO flushed successfully\n");
	return 0;
}

/**
 * bmi_get_accel - Read only accelerometer data (called every 5ms)
 */
int bmi_get_accel(void *client, u8 *buf, size_t len)
{
	struct bmi_state *st = (struct bmi_state *)client;
	struct fifo_burst_len_data *data = (struct fifo_burst_len_data *)buf;
	struct i2c_msg msgs[2];
	u8 *acc_buf;
	u8 acc_reg = BMI_REG_ACC_DATA;
	u64 start_time, end_time;
	int ret;

	if (!st || !buf || len < sizeof(struct fifo_burst_len_data))
		return -EINVAL;

	if (!st->i2c_addrs[BMI_HW_ACC])
		return -ENODEV;

	/* Allocate buffer with GFP_KERNEL for DMA safety */
	acc_buf = kmalloc(6, GFP_KERNEL);
	if (!acc_buf)
		return -ENOMEM;

	/* First message: write register address */
	msgs[0].addr = st->i2c_addrs[BMI_HW_ACC];
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &acc_reg;

	/* Second message: read data */
	msgs[1].addr = st->i2c_addrs[BMI_HW_ACC];
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 6;
	msgs[1].buf = acc_buf;
	msgs[1].flags |= I2C_M_DMA_SAFE;

	start_time = get_ktime_timestamp();
	ret = i2c_transfer(st->i2c->adapter, msgs, 2);
	end_time = get_ktime_timestamp();
	
	if (ret != 2) {
		st->errs_bus[BMI_HW_ACC]++;
		kfree(acc_buf);
		return -EIO;
	}

	/* Parse accelerometer data (little endian) */
	data->ax = (s16)(acc_buf[1] << 8 | acc_buf[0]);
	data->ay = (s16)(acc_buf[3] << 8 | acc_buf[2]);
	data->az = (s16)(acc_buf[5] << 8 | acc_buf[4]);
	data->timestamp = start_time;
	data->fifo_length = 0;

	kfree(acc_buf);
	return 0;
}

/**
 * bmi_get_gyro - Read gyro FIFO (called every 20ms per camera, staggered)
 * 
 * If read_fifo=false: Only reads FIFO length, stores last 3 values
 * If read_fifo=true: Reads FIFO length, then reads FIFO data based on length
 */
int bmi_get_gyro(void *client, u8 *buf, size_t len, bool read_fifo)
{
	struct bmi_state *st = (struct bmi_state *)client;
	struct fifo_burst_fifo_data *data = (struct fifo_burst_fifo_data *)buf;
	struct i2c_msg msgs[2];
	u8 fifo_status;
	u8 *fifo_buffer = NULL;
	u8 fifo_status_reg = BMI_REG_FIFO_STATUS;
	u8 fifo_data_reg = BMI_REG_FIFO_GYR_DATA;
	u64 start_time, end_time;
	int ret, i;
	u16 fifo_count, fifo_bytes;

	if (!st || !buf || len < sizeof(struct fifo_burst_fifo_data))
		return -EINVAL;

	if (!st->i2c_addrs[BMI_HW_GYR])
		return -ENODEV;

	/* Step 1: Read FIFO status to get frame count */
	msgs[0].addr = st->i2c_addrs[BMI_HW_GYR];
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &fifo_status_reg;

	msgs[1].addr = st->i2c_addrs[BMI_HW_GYR];
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = &fifo_status;

	start_time = get_ktime_timestamp();
	ret = i2c_transfer(st->i2c->adapter, msgs, 2);
	
	if (ret != 2) {
		st->errs_bus[BMI_HW_GYR]++;
		return -EIO;
	}
	/* Capture the exact timestamp right after reading FIFO status register
	 * This is the precise moment we locked the FIFO count - the authoritative
	 * timestamp that corresponds to the sample count we just read.
	 * Subtract 200us to compensate for I2C latency and align with sample time. */
	data->fifo_status_lock_timestamp = get_ktime_timestamp() - 200000UL;

	/* Extract frame count (bits 6:0) */
	fifo_count = (u16)(fifo_status & 0x7F);
	if ((fifo_status & 0x80) && st->spsc_dev)
		st->spsc_dev->fifo_overruns++;
	data->sample_count = fifo_count;

	/* If FIFO is empty, return desync */
	if (fifo_count == 0) {
		data->timestamp = start_time;
		return ERR_DESYNC;
	}

	/* If not reading FIFO data, just return length */
	if (!read_fifo) {
		data->timestamp = start_time;
		return 0;
	}

	/* Step 2: Read FIFO data */
	if (fifo_count > BMI_FIFO_MAX_FRAMES) {
		fifo_count = BMI_FIFO_MAX_FRAMES;  /* Clamp to max allowed samples */
		data->sample_count = BMI_FIFO_MAX_FRAMES;
	}

	fifo_bytes = fifo_count * 6;

	/* Allocate buffer with GFP_KERNEL for DMA safety */
	fifo_buffer = kmalloc(fifo_bytes, GFP_KERNEL);
	if (!fifo_buffer)
		return -ENOMEM;

	msgs[0].buf = &fifo_data_reg;
	
	msgs[1].len = fifo_bytes;
	msgs[1].buf = fifo_buffer;
	msgs[1].flags = I2C_M_RD | I2C_M_DMA_SAFE;

	ret = i2c_transfer(st->i2c->adapter, msgs, 2);
	end_time = get_ktime_timestamp();

	if (ret != 2) {
		st->errs_bus[BMI_HW_GYR]++;
		kfree(fifo_buffer);
		return -EIO;
	}

	/* Parse gyro samples (little endian, 6 bytes each) */
	for (i = 0; i < fifo_count; i++) {
		int offset = i * 6;
		data->gyro_samples[i].x = (s16)(fifo_buffer[offset+1] << 8 | fifo_buffer[offset+0]);
		data->gyro_samples[i].y = (s16)(fifo_buffer[offset+3] << 8 | fifo_buffer[offset+2]);
		data->gyro_samples[i].z = (s16)(fifo_buffer[offset+5] << 8 | fifo_buffer[offset+4]);
	}

	/* Dummy accel data (will use accel_history) */
	data->ax = 0;
	data->ay = 0;
	data->az = 0;
	data->timestamp = start_time;

	st->ts_hw[BMI_HW_GYR] = end_time;

	kfree(fifo_buffer);
	return 0;
}

static int bmi_i2c_addr(void *client, int hw_id)
{
	struct bmi_state *st = (struct bmi_state *)client;
	
	if (!st || hw_id < 0 || hw_id >= BMI_HW_N)
		return -EINVAL;
	
	return st->i2c_addrs[hw_id];
}

static int bmi_get_accel_freq(void *client)
{
	struct bmi_state *st = (struct bmi_state *)client;
	
	if (!st)
		return -EINVAL;
	
	return st->accel_freq_hz;
}

static int bmi_get_gyro_freq(void *client)
{
	struct bmi_state *st = (struct bmi_state *)client;
	
	if (!st)
		return -EINVAL;
	
	return st->gyro_freq_hz;
}

static int bmi_get_accel_range(void *client)
{
	struct bmi_state *st = (struct bmi_state *)client;
	
	if (!st)
		return -EINVAL;
	
	return st->accel_range;
}

static int bmi_get_gyro_range(void *client)
{
	struct bmi_state *st = (struct bmi_state *)client;
	
	if (!st)
		return -EINVAL;
	
	return st->gyro_range;
}

static struct bmi_spsc_fn_dev bmi_fn_dev = {
    .enable = bmi_enable,
    .regs = bmi_regs,
	.temp = bmi_temp,
    .freq_read = bmi_freq_read,
    .freq_write = bmi_freq_write,
    .scale_write = bmi_scale_write,
    .max_range = bmi_max_range,
    .read_err = bmi_read_err,
    .get_data = bmi_get_data,
    .get_burst = bmi_get_burst,
    .get_accel = bmi_get_accel,
    .get_gyro = bmi_get_gyro,
    .i2c_bus = bmi_i2c_bus,
    .i2c_addr = bmi_i2c_addr,
    .get_accel_freq = bmi_get_accel_freq,
    .get_gyro_freq = bmi_get_gyro_freq,
    .get_accel_range = bmi_get_accel_range,
    .get_gyro_range = bmi_get_gyro_range,
    .flush_fifo = bmi_flush_fifo,
};

static int __maybe_unused bmi_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bmi_state *st = i2c_get_clientdata(client);
	unsigned int i;
	unsigned int old_en_st;
	int ret = 0;
	int temp_ret = 0;

	st->sts |= BMI_STS_SUSPEND;
	st->suspend_en_st = 0;

	for (i = 0; i < st->hw_n; i++) {
		/* No mutex needed for SPSC - lock-free design */
		/* check if sensor is enabled to begin with */
		old_en_st = bmi_enable(st, st->snsrs[i].cfg.snsr_id, -1);
		if (old_en_st) {
			temp_ret = bmi_enable(st, st->snsrs[i].cfg.snsr_id, 0);
			if (!temp_ret)
				st->suspend_en_st |= old_en_st;

			ret |= temp_ret;
		}
		/* No mutex unlock needed */
	}

	return ret;
}

static int __maybe_unused bmi_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bmi_state *st = i2c_get_clientdata(client);
	unsigned int i;
	int ret = 0;

	for (i = 0; i < st->hw_n; i++) {
		/* No mutex needed for SPSC - lock-free design */
		/* check if sensor is enabled to begin with */
		if (st->suspend_en_st & (1 << st->snsrs[i].cfg.snsr_id))
			ret |= bmi_enable(st, st->snsrs[i].cfg.snsr_id, 1);
		/* No mutex unlock needed */
	}

	st->sts &= ~BMI_STS_SUSPEND;

	return ret;
}

static SIMPLE_DEV_PM_OPS(bmi_pm_ops, bmi_suspend, bmi_resume);

static void bmi_shutdown(struct i2c_client *client)
{
	struct bmi_state *st = i2c_get_clientdata(client);
	unsigned int i;

	st->sts |= BMI_STS_SHUTDOWN;
	for (i = 0; i < st->hw_n; i++) {
		if (st->spsc_init_done[i]) {
			// No mutex needed for SPSC - lock-free design
		}

		if (bmi_enable(st, st->snsrs[i].cfg.snsr_id, -1))
			bmi_enable(st, st->snsrs[i].cfg.snsr_id, 0);

		if (st->spsc_init_done[i]) {
			// No mutex unlock needed
		}
	}
}

static void bmi_remove(void *data)
{
	struct i2c_client *client = data;
	struct bmi_state *st = i2c_get_clientdata(client);
	int i;

	if (st != NULL) {        
        
        bmi_shutdown(client);
        for (i = 0; i < st->hw_n; i++) {
            if (st->spsc_init_done[i]) {
                // SPSC cleanup is handled below
            }
        }
        
        /* Cleanup single SPSC device for this BMI chip */
        if (st->spsc_dev) {
            bmi_spsc_device_cleanup(st->spsc_dev);
            st->spsc_dev = NULL;
        }
    }
	/* Free resources for each sensor */
	for (i = 0; i < BMI_HW_N; i++) {
		/* Disable sensors if still enabled */
		if (st && (st->enabled & (1 << i))) {
			bmi_hws[i].fn_able(st, 0, false); // disable sensor
			bmi_pm(st, i, false); // power down
		}
	}
	
	/* Per-device workqueues are no longer needed - SPSC handles data streaming */
	/* No workqueue cleanup needed here*/
	
	dev_info(&client->dev, "removed\n");
}

// Get the device tree properties function
static int bmi_of_dt(struct bmi_state *st, struct device_node *dn) 
{
	u32 val32 = 0;
	const char *charp;
	int lenp;
	// Check if the device tree node is valid
	if (!dn){
		dev_err(&st->i2c->dev, "Device tree node is missing\n");
		return 0;
	}

	if (!st->i2c_addrs[BMI_HW_ACC]) { // Check if the accelerometer I2C address is not set
		if (!of_property_read_u32(dn, "accel_i2c_addr", &val32))
			st->i2c_addrs[BMI_HW_ACC] = val32;
		else
			return -ENODEV;
	}

	charp = of_get_property(dn, "accel_matrix", &lenp);
	if (charp && lenp == sizeof(st->snsrs[BMI_HW_ACC].cfg.matrix))
		memcpy(&st->snsrs[BMI_HW_ACC].cfg.matrix, charp, lenp);

	charp = of_get_property(dn, "gyro_matrix", &lenp);
	if (charp && lenp == sizeof(st->snsrs[BMI_HW_GYR].cfg.matrix))
		memcpy(&st->snsrs[BMI_HW_GYR].cfg.matrix, charp, lenp);

	return 0;
}

static int bmi_reset_all(struct bmi_state *st)
{
	int ret_acc, ret_gyr;
	int ret = 0;

	ret_acc = bmi_hws[BMI_HW_ACC].fn_softreset(st, BMI_HW_ACC);
	if (ret_acc == -ENODEV)
		return -ENODEV;

	ret_gyr = bmi_hws[BMI_HW_GYR].fn_softreset(st, BMI_HW_GYR);
	if (ret_gyr == -ENODEV)
		return -ENODEV;

	/* Preserve previous behavior for other error codes */
	ret = (ret_acc ? ret_acc : 0) | (ret_gyr ? ret_gyr : 0);

	return ret;
}

static int bmi_init(struct bmi_state *st, const struct i2c_device_id *id)
{
	//unsigned long irqflags;
	unsigned int i;
	int ret;
	u8 acc_id, gyro_id;
	static atomic_t bmi_device_counter = ATOMIC_INIT(0); // Global counter for BMI devices

	// Initialize the state structure, not writing to the device yet
	st->ra_0x53 = 0x02; 
	st->ra_0x54 = 0x00; 
	st->ra_0x58 = 0x04; 
	st->rg_0x16 = 0x0D;
	st->rg_0x18 = 0x00; 
	st->hw_n = BMI_HW_N;
	st->i2c_addrs[BMI_HW_ACC] = 0;
	st->hw_en = 0;
	st->enabled = 0;

	// Parse device tree properties and initialize the BMI state structure
	ret = bmi_of_dt(st, st->i2c->dev.of_node);
	if (ret) {
		dev_err(&st->i2c->dev, "of_dt ERR\n");
		return ret;
	}

	// Assigns the driver-specific data from the device ID structure to the sensor's state structure.
#if KERNEL_VERSION(6, 3, 0) <= LINUX_VERSION_CODE
	st->part = (uintptr_t)i2c_get_match_data(st->i2c);
#else
	st->part = id->driver_data;
#endif

	st->i2c_addrs[BMI_HW_GYR] = st->i2c->addr;
	ret = bmi_reset_all(st);
	if (ret) {
		dev_dbg(&st->i2c->dev, "imu not connected to this address\n");
		return ret;
	}

	bmi_fn_dev.sts = &st->sts; // Set the status of the driver

	/*
	 * Create single SPSC device for combined accel+gyro data
	 * Instead of separate devices for each sensor type
	 */
	
	/* Create one SPSC device for this BMI088 chip */
	st->spsc_dev = bmi_spsc_device_init(atomic_fetch_inc(&bmi_device_counter), st, &bmi_fn_dev);
	if (!st->spsc_dev) {
		dev_err(&st->i2c->dev, "Failed to initialize SPSC device for BMI device (atomic)\n");
		return -ENOMEM;
	}
	

	
	/* Initialize sensor configurations but don't create separate SPSC devices */
	for (i = 0; i < BMI_HW_N; i++) {
		memcpy(&st->snsrs[i].cfg, &bmi_snsr_cfgs[i],
		       sizeof(st->snsrs[i].cfg)); // Copy the sensor configuration

		/* Link both sensors to the same SPSC device */
		st->snsrs[i].spsc_dev = st->spsc_dev;

		st->snsrs[i].cfg.snsr_id = i; // Set the sensor ID
		st->snsrs[i].cfg.part = bmi_i2c_device_ids[st->part].name; // Set the part name
		st->snsrs[i].rrs = &bmi_hws[i].rrs[st->part]; // Set the range resolution structure
		bmi_max_range(st, i, st->snsrs[i].cfg.max_range.ival); // Set the default value (sw, hw will happen on enable)
		st->spsc_init_done[i] = true; // Set the SPSC initialization flag
		st->snsrs[i].st = st; // Set the state structure pointer
		
		/* Set default period to slowest speed, will be used during trigger setup */
		st->snsrs[i].period_us = st->snsrs[i].cfg.delay_us_max;
	}

	/* Initialize default frequency values for sysfs readback */
	st->accel_freq_hz = 25;  /* Default accelerometer frequency */
	st->gyro_freq_hz = 100;  /* Default gyroscope frequency */

	// Print device information after initialization
	dev_dbg(&st->i2c->dev, "BMI088 initialized:\n");
	dev_dbg(&st->i2c->dev, "Accelerometer I2C addr: 0x%02X\n", st->i2c_addrs[BMI_HW_ACC]);
	dev_dbg(&st->i2c->dev, "Gyroscope I2C addr: 0x%02X\n", st->i2c_addrs[BMI_HW_GYR]);
	// Read and print accelerometer chip ID
	if (bmi_i2c_rd(st, BMI_HW_ACC, BMI_REG_ACC_CHIP_ID, 1, &acc_id) == 0) {
		dev_dbg(&st->i2c->dev, "Accelerometer chip ID: 0x%02X\n", acc_id);
	} 
	else {
		dev_err(&st->i2c->dev, "Failed to read accelerometer chip ID\n");
	}
	// Read and print gyroscope chip ID
	if (bmi_i2c_rd(st, BMI_HW_GYR, BMI_REG_GYR_CHIP_ID, 1, &gyro_id) == 0) {
		dev_dbg(&st->i2c->dev, "Gyroscope chip ID: 0x%02X\n", gyro_id);
	} 
	else {
		dev_err(&st->i2c->dev, "Failed to read gyroscope chip ID\n");
	}
	return ret;
}

/**
 * @brief Probe function for BMI088 sensor driver initialization.
 *
 * This function is called by the I2C core when a device matching the driver's ID is detected.
 * It allocates and initializes the driver's private data structure, sets up the I2C client data,
 * and initializes the BMI088 sensor device.
 * 
 * The function handles kernel version differences for initialization parameters.
 * After successful initialization, a cleanup action is registered for device removal.
 *
 * @param client - Pointer to the I2C client structure for the detected device
 * @param id - Pointer to the I2C device ID that matched this device (not used in newer kernels)
 * 
 * @return 0 on success, negative error code on failure
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 0, 0)
static int bmi_probe(struct i2c_client *client, const struct i2c_device_id *id)
#else
static int bmi_probe(struct i2c_client *client)
#endif
{
	struct bmi_state *st;
	int ret;
	st = devm_kzalloc(&client->dev, sizeof(*st), GFP_KERNEL);
	if (st == NULL)
		return -ENOMEM;

	i2c_set_clientdata(client, st);
	st->i2c = client;
#if KERNEL_VERSION(6, 3, 0) <= LINUX_VERSION_CODE
	ret = bmi_init(st, NULL);
#else
	ret = bmi_init(st, id);
#endif
	if (ret) {
		bmi_remove(client);
		return ret;
	}
	// Add a cleanup action
	ret = devm_add_action_or_reset(&client->dev, bmi_remove, client); 
	if (ret)
		return ret;

	// Debug message indicating completion
	dev_info(&client->dev, "probe done\n"); 

	return ret;
}

MODULE_DEVICE_TABLE(i2c, bmi_i2c_device_ids);

static const struct of_device_id bmi_of_match[] = {
	{ .compatible = "bmi,bmi088", },
	{}
};

MODULE_DEVICE_TABLE(of, bmi_of_match);

static struct i2c_driver bmi_driver = {
	.class				= I2C_CLASS_HWMON,
	.probe				= bmi_probe,
	.driver				= {
		.name			= BMI_NAME,
		.owner			= THIS_MODULE,
		.of_match_table		= of_match_ptr(bmi_of_match),
		.pm			= &bmi_pm_ops,
	},
	.id_table			= bmi_i2c_device_ids,
};

module_i2c_driver(bmi_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Acknowledgement: Based NVIDIA's Driver for BMI088 I2C driver");
MODULE_AUTHOR("STEREOLABS <support@stereolabs.com>");
