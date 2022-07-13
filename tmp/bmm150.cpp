// by honey the codewitch
/* Code derived from Seed Studio code
The MIT License (MIT)

Copyright (c) 2013 Seeed Technology Inc.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/
#include <bmm150.hpp>
#include <math.h>
/**\name CHIP ID & SOFT RESET VALUES      */
#define BMM150_CHIP_ID              0x32
#define BMM150_SET_SOFT_RESET		0x82

/**\name POWER MODE DEFINTIONS      */
#define BMM150_NORMAL_MODE		0x00
#define BMM150_FORCED_MODE		0x01
#define BMM150_SLEEP_MODE		0x03
#define BMM150_SUSPEND_MODE		0x04

/**\name PRESET MODE DEFINITIONS  */
#define BMM150_PRESETMODE_LOWPOWER                 0x01
#define BMM150_PRESETMODE_REGULAR                  0x02
#define BMM150_PRESETMODE_HIGHACCURACY             0x03
#define BMM150_PRESETMODE_ENHANCED                 0x04

/**\name Power mode settings  */
#define	BMM150_POWER_CNTRL_DISABLE	0x00
#define	BMM150_POWER_CNTRL_ENABLE	0x01

/**\name Sensor delay time settings  */
#define BMM150_SOFT_RESET_DELAY		(1)
#define BMM150_NORMAL_SELF_TEST_DELAY	(2)
#define BMM150_START_UP_TIME		(3)
#define BMM150_ADV_SELF_TEST_DELAY	(4)

/**\name ENABLE/DISABLE DEFINITIONS  */
#define BMM150_XY_CHANNEL_ENABLE	0x00
#define BMM150_XY_CHANNEL_DISABLE	0x03

/**\name Register Address */
#define BMM150_CHIP_ID_ADDR		    0x40
#define BMM150_DATA_X_LSB		    0x42
#define BMM150_DATA_X_MSB		    0x43
#define BMM150_DATA_Y_LSB		    0x44
#define BMM150_DATA_Y_MSB		    0x45
#define BMM150_DATA_Z_LSB		    0x46
#define BMM150_DATA_Z_MSB		    0x47
#define BMM150_DATA_READY_STATUS	0x48
#define BMM150_INTERRUPT_STATUS		0x4A
#define BMM150_POWER_CONTROL_ADDR	0x4B
#define BMM150_OP_MODE_ADDR		    0x4C
#define BMM150_INT_CONFIG_ADDR		0x4D
#define BMM150_AXES_ENABLE_ADDR		0x4E
#define BMM150_LOW_THRESHOLD_ADDR	0x4F
#define BMM150_HIGH_THRESHOLD_ADDR	0x50
#define BMM150_REP_XY_ADDR		    0x51
#define BMM150_REP_Z_ADDR		    0x52

/**\name DATA RATE DEFINITIONS  */
#define BMM150_DATA_RATE_10HZ        (0x00)
#define BMM150_DATA_RATE_02HZ        (0x01)
#define BMM150_DATA_RATE_06HZ        (0x02)
#define BMM150_DATA_RATE_08HZ        (0x03)
#define BMM150_DATA_RATE_15HZ        (0x04)
#define BMM150_DATA_RATE_20HZ        (0x05)
#define BMM150_DATA_RATE_25HZ        (0x06)
#define BMM150_DATA_RATE_30HZ        (0x07)

/**\name TRIM REGISTERS      */
/* Trim Extended Registers */
#define BMM150_DIG_X1               UINT8_C(0x5D)
#define BMM150_DIG_Y1               UINT8_C(0x5E)
#define BMM150_DIG_Z4_LSB           UINT8_C(0x62)
#define BMM150_DIG_Z4_MSB           UINT8_C(0x63)
#define BMM150_DIG_X2               UINT8_C(0x64)
#define BMM150_DIG_Y2               UINT8_C(0x65)
#define BMM150_DIG_Z2_LSB           UINT8_C(0x68)
#define BMM150_DIG_Z2_MSB           UINT8_C(0x69)
#define BMM150_DIG_Z1_LSB           UINT8_C(0x6A)
#define BMM150_DIG_Z1_MSB           UINT8_C(0x6B)
#define BMM150_DIG_XYZ1_LSB         UINT8_C(0x6C)
#define BMM150_DIG_XYZ1_MSB         UINT8_C(0x6D)
#define BMM150_DIG_Z3_LSB           UINT8_C(0x6E)
#define BMM150_DIG_Z3_MSB           UINT8_C(0x6F)
#define BMM150_DIG_XY2              UINT8_C(0x70)
#define BMM150_DIG_XY1              UINT8_C(0x71)

/**\name PRESET MODES - REPETITIONS-XY RATES */
#define BMM150_LOWPOWER_REPXY                    (1)
#define BMM150_REGULAR_REPXY                     (4)
#define BMM150_ENHANCED_REPXY                    (7)
#define BMM150_HIGHACCURACY_REPXY                (23)

/**\name PRESET MODES - REPETITIONS-Z RATES */
#define BMM150_LOWPOWER_REPZ                     (2)
#define BMM150_REGULAR_REPZ                      (14)
#define BMM150_ENHANCED_REPZ                     (26)
#define BMM150_HIGHACCURACY_REPZ                 (82)

/**\name Macros for bit masking */
#define	BMM150_PWR_CNTRL_MSK		(0x01)

#define	BMM150_CONTROL_MEASURE_MSK	(0x38)
#define	BMM150_CONTROL_MEASURE_POS	(0x03)

#define BMM150_POWER_CONTROL_BIT_MSK	(0x01)
#define BMM150_POWER_CONTROL_BIT_POS	(0x00)

#define BMM150_OP_MODE_MSK		(0x06)
#define BMM150_OP_MODE_POS		(0x01)

#define BMM150_ODR_MSK			(0x38)
#define BMM150_ODR_POS			(0x03)

#define BMM150_DATA_X_MSK		(0xF8)
#define BMM150_DATA_X_POS		(0x03)

#define BMM150_DATA_Y_MSK		(0xF8)
#define BMM150_DATA_Y_POS		(0x03)

#define BMM150_DATA_Z_MSK		(0xFE)
#define BMM150_DATA_Z_POS		(0x01)

#define BMM150_DATA_RHALL_MSK		(0xFC)
#define BMM150_DATA_RHALL_POS		(0x02)

#define	BMM150_SELF_TEST_MSK		(0x01)

#define	BMM150_ADV_SELF_TEST_MSK	(0xC0)
#define	BMM150_ADV_SELF_TEST_POS	(0x06)

#define	BMM150_DRDY_EN_MSK		(0x80)
#define	BMM150_DRDY_EN_POS		(0x07)

#define	BMM150_INT_PIN_EN_MSK		(0x40)
#define	BMM150_INT_PIN_EN_POS		(0x06)

#define	BMM150_DRDY_POLARITY_MSK	(0x04)
#define	BMM150_DRDY_POLARITY_POS	(0x02)

#define	BMM150_INT_LATCH_MSK		(0x02)
#define	BMM150_INT_LATCH_POS		(0x01)

#define	BMM150_INT_POLARITY_MSK		(0x01)

#define	BMM150_DATA_OVERRUN_INT_MSK	(0x80)
#define	BMM150_DATA_OVERRUN_INT_POS	(0x07)

#define	BMM150_OVERFLOW_INT_MSK		(0x40)
#define	BMM150_OVERFLOW_INT_POS		(0x06)

#define	BMM150_HIGH_THRESHOLD_INT_MSK	(0x38)
#define	BMM150_HIGH_THRESHOLD_INT_POS	(0x03)

#define	BMM150_LOW_THRESHOLD_INT_MSK	(0x07)

#define	BMM150_DRDY_STATUS_MSK		(0x01)

/**\name OVERFLOW DEFINITIONS  */
#define BMM150_XYAXES_FLIP_OVERFLOW_ADCVAL	(-4096)
#define BMM150_ZAXIS_HALL_OVERFLOW_ADCVAL	  (-16384)
#define BMM150_OVERFLOW_OUTPUT			        (-32768)
#define BMM150_NEGATIVE_SATURATION_Z        (-32767)
#define BMM150_POSITIVE_SATURATION_Z        (32767)
#ifdef BMM150_USE_FLOATING_POINT
    #define BMM150_OVERFLOW_OUTPUT_FLOAT		0.0f
#endif

/**\name Register read lengths	*/
#define BMM150_SELF_TEST_LEN			(5)
#define BMM150_SETTING_DATA_LEN			(8)
#define BMM150_XYZR_DATA_LEN			(8)

/**\name Self test selection macros */
#define BMM150_NORMAL_SELF_TEST			(0)
#define BMM150_ADVANCED_SELF_TEST		(1)

/**\name Self test settings */
#define BMM150_DISABLE_XY_AXIS			(0x03)
#define BMM150_SELF_TEST_REP_Z			(0x04)

/**\name Advanced self-test current settings */
#define BMM150_DISABLE_SELF_TEST_CURRENT	(0x00)
#define BMM150_ENABLE_NEGATIVE_CURRENT		(0x02)
#define BMM150_ENABLE_POSITIVE_CURRENT		(0x03)

/**\name Normal self-test status */
#define BMM150_SELF_TEST_STATUS_XYZ_FAIL	(0x00)
#define BMM150_SELF_TEST_STATUS_SUCCESS		(0x07)

/**\name Macro to SET and GET BITS of a register*/
#define BMM150_SET_BITS(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | \
     ((data << bitname##_POS) & bitname##_MSK))

#define BMM150_GET_BITS(reg_data, bitname)  ((reg_data & (bitname##_MSK)) >> \
        (bitname##_POS))

#define BMM150_SET_BITS_POS_0(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | \
     (data & bitname##_MSK))

#define BMM150_GET_BITS_POS_0(reg_data, bitname)  (reg_data & (bitname##_MSK))

namespace arduino {
void bmm150_i2c_write(TwoWire& i2c, short address, short data) {
    i2c.beginTransmission(bmm150::address);
    i2c.write(address);
    i2c.write(data);
    i2c.endTransmission();
}

void bmm150_i2c_read(TwoWire& i2c, short address, uint8_t* buffer, short length) {
    i2c.beginTransmission(bmm150::address);
    i2c.write(address);
    i2c.endTransmission();
    i2c.requestFrom(bmm150::address, length);

    if (i2c.available() == length) {
        for (uint8_t i = 0; i < length; i++) {
            buffer[i] = i2c.read();
        }
    }
}

void bmm150_i2c_read(TwoWire& i2c, short address, int8_t* buffer, short length) {
    i2c.beginTransmission(bmm150::address);
    i2c.write(address);
    i2c.endTransmission();
    i2c.requestFrom(bmm150::address, length);

    if (i2c.available() == length) {
        for (uint8_t i = 0; i < length; i++) {
            buffer[i] = i2c.read();
        }
    }
}

uint8_t bmm150_i2c_read(TwoWire& i2c, short address) {
    uint8_t byte;

    i2c.beginTransmission(bmm150::address);
    i2c.write(address);
    i2c.endTransmission();
    i2c.requestFrom(bmm150::address, 1);
    byte = i2c.read();
    return byte;
}

void bmm150_set_power_control_bit(TwoWire& i2c, uint8_t pwrcntrl_bit) {
    uint8_t reg_data = 0;
    /* Power control register 0x4B is read */
    reg_data = bmm150_i2c_read(i2c,BMM150_POWER_CONTROL_ADDR);
    /* Sets the value of power control bit */
    reg_data = BMM150_SET_BITS_POS_0(reg_data, BMM150_PWR_CNTRL, pwrcntrl_bit);
    bmm150_i2c_write(i2c,BMM150_POWER_CONTROL_ADDR, reg_data);
}
void bmm150_write_op_mode(TwoWire& i2c,uint8_t op_mode) {
    uint8_t reg_data = 0;
    reg_data = bmm150_i2c_read(i2c,BMM150_OP_MODE_ADDR);
    /* Set the op_mode value in Opmode bits of 0x4C */
    reg_data = BMM150_SET_BITS(reg_data, BMM150_OP_MODE, op_mode);
    bmm150_i2c_write(i2c,BMM150_OP_MODE_ADDR, reg_data);
}
/*
    @brief This internal API is used to obtain the compensated
    magnetometer X axis data(micro-tesla) in int16_t.
*/
int16_t bmm150::adjust_x(int16_t mag_data_x, uint16_t data_rhall) {
    int16_t retval;
    uint16_t process_comp_x0 = 0;
    int32_t process_comp_x1;
    uint16_t process_comp_x2;
    int32_t process_comp_x3;
    int32_t process_comp_x4;
    int32_t process_comp_x5;
    int32_t process_comp_x6;
    int32_t process_comp_x7;
    int32_t process_comp_x8;
    int32_t process_comp_x9;
    int32_t process_comp_x10;

    /* Overflow condition check */
    if (mag_data_x != BMM150_XYAXES_FLIP_OVERFLOW_ADCVAL) {
        if (data_rhall != 0) {
            /* Availability of valid data*/
            process_comp_x0 = data_rhall;
        } else if (m_trim_data.dig_xyz1 != 0) {
            process_comp_x0 = m_trim_data.dig_xyz1;
        } else {
            process_comp_x0 = 0;
        }
        if (process_comp_x0 != 0) {
            /* Processing compensation equations*/
            process_comp_x1 = ((int32_t)m_trim_data.dig_xyz1) * 16384;
            process_comp_x2 = ((uint16_t)(process_comp_x1 / process_comp_x0)) - ((uint16_t)0x4000);
            retval = ((int16_t)process_comp_x2);
            process_comp_x3 = (((int32_t)retval) * ((int32_t)retval));
            process_comp_x4 = (((int32_t)m_trim_data.dig_xy2) * (process_comp_x3 / 128));
            process_comp_x5 = (int32_t)(((int16_t)m_trim_data.dig_xy1) * 128);
            process_comp_x6 = ((int32_t)retval) * process_comp_x5;
            process_comp_x7 = (((process_comp_x4 + process_comp_x6) / 512) + ((int32_t)0x100000));
            process_comp_x8 = ((int32_t)(((int16_t)m_trim_data.dig_x2) + ((int16_t)0xA0)));
            process_comp_x9 = ((process_comp_x7 * process_comp_x8) / 4096);
            process_comp_x10 = ((int32_t)mag_data_x) * process_comp_x9;
            retval = ((int16_t)(process_comp_x10 / 8192));
            retval = (retval + (((int16_t)m_trim_data.dig_x1) * 8)) / 16;
        } else {
            retval = BMM150_OVERFLOW_OUTPUT;
        }
    } else {
        /* Overflow condition */
        retval = BMM150_OVERFLOW_OUTPUT;
    }

    return retval;
}

/*
    @brief This internal API is used to obtain the compensated
    magnetometer Y axis data(micro-tesla) in int16_t.
*/
int16_t bmm150::adjust_y(int16_t mag_data_y, uint16_t data_rhall) {
    int16_t retval;
    uint16_t process_comp_y0 = 0;
    int32_t process_comp_y1;
    uint16_t process_comp_y2;
    int32_t process_comp_y3;
    int32_t process_comp_y4;
    int32_t process_comp_y5;
    int32_t process_comp_y6;
    int32_t process_comp_y7;
    int32_t process_comp_y8;
    int32_t process_comp_y9;

    /* Overflow condition check */
    if (mag_data_y != BMM150_XYAXES_FLIP_OVERFLOW_ADCVAL) {
        if (data_rhall != 0) {
            /* Availability of valid data*/
            process_comp_y0 = data_rhall;
        } else if (m_trim_data.dig_xyz1 != 0) {
            process_comp_y0 = m_trim_data.dig_xyz1;
        } else {
            process_comp_y0 = 0;
        }
        if (process_comp_y0 != 0) {
            /*Processing compensation equations*/
            process_comp_y1 = (((int32_t)m_trim_data.dig_xyz1) * 16384) / process_comp_y0;
            process_comp_y2 = ((uint16_t)process_comp_y1) - ((uint16_t)0x4000);
            retval = ((int16_t)process_comp_y2);
            process_comp_y3 = ((int32_t) retval) * ((int32_t)retval);
            process_comp_y4 = ((int32_t)m_trim_data.dig_xy2) * (process_comp_y3 / 128);
            process_comp_y5 = ((int32_t)(((int16_t)m_trim_data.dig_xy1) * 128));
            process_comp_y6 = ((process_comp_y4 + (((int32_t)retval) * process_comp_y5)) / 512);
            process_comp_y7 = ((int32_t)(((int16_t)m_trim_data.dig_y2) + ((int16_t)0xA0)));
            process_comp_y8 = (((process_comp_y6 + ((int32_t)0x100000)) * process_comp_y7) / 4096);
            process_comp_y9 = (((int32_t)mag_data_y) * process_comp_y8);
            retval = (int16_t)(process_comp_y9 / 8192);
            retval = (retval + (((int16_t)m_trim_data.dig_y1) * 8)) / 16;
        } else {
            retval = BMM150_OVERFLOW_OUTPUT;
        }
    } else {
        /* Overflow condition*/
        retval = BMM150_OVERFLOW_OUTPUT;
    }

    return retval;
}

/*
    @brief This internal API is used to obtain the compensated
    magnetometer Z axis data(micro-tesla) in int16_t.
*/
int16_t bmm150::adjust_z(int16_t mag_data_z, uint16_t data_rhall) {
    int32_t retval;
    int16_t process_comp_z0;
    int32_t process_comp_z1;
    int32_t process_comp_z2;
    int32_t process_comp_z3;
    int16_t process_comp_z4;

    if (mag_data_z != BMM150_ZAXIS_HALL_OVERFLOW_ADCVAL) {
        if ((m_trim_data.dig_z2 != 0) && (m_trim_data.dig_z1 != 0)
                && (data_rhall != 0) && (m_trim_data.dig_xyz1 != 0)) {
            /*Processing compensation equations*/
            process_comp_z0 = ((int16_t)data_rhall) - ((int16_t) m_trim_data.dig_xyz1);
            process_comp_z1 = (((int32_t)m_trim_data.dig_z3) * ((int32_t)(process_comp_z0))) / 4;
            process_comp_z2 = (((int32_t)(mag_data_z - m_trim_data.dig_z4)) * 32768);
            process_comp_z3 = ((int32_t)m_trim_data.dig_z1) * (((int16_t)data_rhall) * 2);
            process_comp_z4 = (int16_t)((process_comp_z3 + (32768)) / 65536);
            retval = ((process_comp_z2 - process_comp_z1) / (m_trim_data.dig_z2 + process_comp_z4));

            /* saturate result to +/- 2 micro-tesla */
            if (retval > BMM150_POSITIVE_SATURATION_Z) {
                retval =  BMM150_POSITIVE_SATURATION_Z;
            } else {
                if (retval < BMM150_NEGATIVE_SATURATION_Z) {
                    retval = BMM150_NEGATIVE_SATURATION_Z;
                }
            }
            /* Conversion of LSB to micro-tesla*/
            retval = retval / 16;
        } else {
            retval = BMM150_OVERFLOW_OUTPUT;

        }
    } else {
        /* Overflow condition*/
        retval = BMM150_OVERFLOW_OUTPUT;
    }

    return (int16_t)retval;
}

bool bmm150::initialize() {
    if(!m_initialized) {
        m_i2c.begin();
        power_mode(BMM150_SLEEP_MODE);
        delay(BMM150_START_UP_TIME);
        uint8_t id = bmm150_i2c_read(m_i2c,BMM150_CHIP_ID_ADDR);
        if(id!=BMM150_CHIP_ID) {
            Serial.println("Could not initialize BMM150");
            return false;
        }
        update_trim();
        power_mode(BMM150_NORMAL_MODE);
        m_initialized = true;
    }
    return true;
}
void bmm150::odr(const settings& s) {
    uint8_t reg_data;

    reg_data = bmm150_i2c_read(m_i2c,BMM150_OP_MODE_ADDR);
    /*Set the ODR value */
    reg_data = BMM150_SET_BITS(reg_data, BMM150_ODR, s.data_rate);
    bmm150_i2c_write(m_i2c,BMM150_OP_MODE_ADDR, reg_data);
}

void bmm150::odr_xyz_rep(const settings& s) {
    /* Set the ODR */
    odr(s);
    /* Set the XY-repetitions number */
    xy_rep(s);
    /* Set the Z-repetitions number */
    z_rep(s);
}

void bmm150::xy_rep(const settings& s) {
    uint8_t rep_xy;
    rep_xy = s.xy_rep;
    bmm150_i2c_write(m_i2c,BMM150_REP_XY_ADDR, rep_xy);

}

void bmm150::z_rep(const settings& s) {
    uint8_t rep_z;
    rep_z = s.z_rep;
    bmm150_i2c_write(m_i2c,BMM150_REP_Z_ADDR, rep_z);
}
void bmm150::update_trim() {
    uint8_t trim_x1y1[2] = {0};
    uint8_t trim_xyz_data[4] = {0};
    uint8_t trim_xy1xy2[10] = {0};
    uint16_t temp_msb = 0;

    /* Trim register value is read */
    bmm150_i2c_read(m_i2c,BMM150_DIG_X1, trim_x1y1, 2);
    bmm150_i2c_read(m_i2c,BMM150_DIG_Z4_LSB, trim_xyz_data, 4);
    bmm150_i2c_read(m_i2c,BMM150_DIG_Z2_LSB, trim_xy1xy2, 10);
    /*  Trim data which is read is updated
        in the device structure */
    m_trim_data.dig_x1 = (int8_t)trim_x1y1[0];
    m_trim_data.dig_y1 = (int8_t)trim_x1y1[1];
    m_trim_data.dig_x2 = (int8_t)trim_xyz_data[2];
    m_trim_data.dig_y2 = (int8_t)trim_xyz_data[3];
    temp_msb = ((uint16_t)trim_xy1xy2[3]) << 8;
    m_trim_data.dig_z1 = (uint16_t)(temp_msb | trim_xy1xy2[2]);
    temp_msb = ((uint16_t)trim_xy1xy2[1]) << 8;
    m_trim_data.dig_z2 = (int16_t)(temp_msb | trim_xy1xy2[0]);
    temp_msb = ((uint16_t)trim_xy1xy2[7]) << 8;
    m_trim_data.dig_z3 = (int16_t)(temp_msb | trim_xy1xy2[6]);
    temp_msb = ((uint16_t)trim_xyz_data[1]) << 8;
    m_trim_data.dig_z4 = (int16_t)(temp_msb | trim_xyz_data[0]);
    m_trim_data.dig_xy1 = trim_xy1xy2[9];
    m_trim_data.dig_xy2 = (int8_t)trim_xy1xy2[8];
    temp_msb = ((uint16_t)(trim_xy1xy2[5] & 0x7F)) << 8;
    m_trim_data.dig_xyz1 = (uint16_t)(temp_msb | trim_xy1xy2[4]);
}
void bmm150::preset_mode(uint8_t mode) {
    switch (mode) {
        case BMM150_PRESETMODE_LOWPOWER:
            /*  Set the data rate x,y,z repetition
                for Low Power mode */
            m_settings.data_rate = BMM150_DATA_RATE_10HZ;
            m_settings.xy_rep = BMM150_LOWPOWER_REPXY;
            m_settings.z_rep = BMM150_LOWPOWER_REPZ;
            odr_xyz_rep(m_settings);
            break;
        case BMM150_PRESETMODE_REGULAR:
            /*  Set the data rate x,y,z repetition
                for Regular mode */
            m_settings.data_rate = BMM150_DATA_RATE_10HZ;
            m_settings.xy_rep = BMM150_REGULAR_REPXY;
            m_settings.z_rep = BMM150_REGULAR_REPZ;
            odr_xyz_rep(m_settings);
            break;
        case BMM150_PRESETMODE_HIGHACCURACY:
            /*  Set the data rate x,y,z repetition
                for High Accuracy mode */
            m_settings.data_rate = BMM150_DATA_RATE_20HZ;
            m_settings.xy_rep = BMM150_HIGHACCURACY_REPXY;
            m_settings.z_rep = BMM150_HIGHACCURACY_REPZ;
            odr_xyz_rep(m_settings);
            break;
        case BMM150_PRESETMODE_ENHANCED:
            /*  Set the data rate x,y,z repetition
                for Enhanced Accuracy mode */
            m_settings.data_rate = BMM150_DATA_RATE_10HZ;
            m_settings.xy_rep = BMM150_ENHANCED_REPXY;
            m_settings.z_rep = BMM150_ENHANCED_REPZ;
            odr_xyz_rep(m_settings);
            break;
        default:
            break;
    }
}
void bmm150::power_mode(uint8_t pwr_mode) {
    /* Select the power mode to set */
    switch (pwr_mode) {
        case BMM150_NORMAL_MODE:
            /*  If the sensor is in suspend mode
                put the device to sleep mode */
            bmm150_set_power_control_bit(m_i2c,BMM150_POWER_CNTRL_ENABLE);
            delay(3);
            /* write the op mode */
            bmm150_write_op_mode(m_i2c,pwr_mode);
            break;
        case BMM150_FORCED_MODE:
            /*  If the sensor is in suspend mode
                put the device to sleep mode */
            bmm150_set_power_control_bit(m_i2c,BMM150_POWER_CNTRL_ENABLE);
            delay(3);
            /* write the op mode */
            bmm150_write_op_mode(m_i2c,pwr_mode);
            break;
        case BMM150_SLEEP_MODE:
            /*  If the sensor is in suspend mode
                put the device to sleep mode */
            bmm150_set_power_control_bit(m_i2c,BMM150_POWER_CNTRL_ENABLE);
            delay(3);
            /* write the op mode */
            bmm150_write_op_mode(m_i2c,pwr_mode);
            break;
        case BMM150_SUSPEND_MODE:
            /* Set the power control bit to zero */
            bmm150_set_power_control_bit(m_i2c,BMM150_POWER_CNTRL_DISABLE);
            break;
        default:
            break;
    }
}
void bmm150::reset() {
    initialize();
    uint8_t reg_data;

    reg_data = bmm150_i2c_read(m_i2c,BMM150_POWER_CONTROL_ADDR);
    reg_data = reg_data | BMM150_SET_SOFT_RESET;
    bmm150_i2c_write(m_i2c,BMM150_POWER_CONTROL_ADDR, reg_data);
    delay(BMM150_SOFT_RESET_DELAY);
}

void bmm150::update() {
    initialize();
    int16_t msb_data;
    int8_t reg_data[BMM150_XYZR_DATA_LEN] = {0};

    bmm150_i2c_read(m_i2c,BMM150_DATA_X_LSB, reg_data, BMM150_XYZR_DATA_LEN);

    /* Mag X axis data */
    reg_data[0] = BMM150_GET_BITS(reg_data[0], BMM150_DATA_X);
    /* Shift the MSB data to left by 5 bits */
    /* Multiply by 32 to get the shift left by 5 value */
    msb_data = ((int16_t)((int8_t)reg_data[1])) * 32;
    /* Raw mag X axis data */
    m_raw_mag_data.x = (int16_t)(msb_data | reg_data[0]);
    /* Mag Y axis data */
    reg_data[2] = BMM150_GET_BITS(reg_data[2], BMM150_DATA_Y);
    /* Shift the MSB data to left by 5 bits */
    /* Multiply by 32 to get the shift left by 5 value */
    msb_data = ((int16_t)((int8_t)reg_data[3])) * 32;
    /* Raw mag Y axis data */
    m_raw_mag_data.y = (int16_t)(msb_data | reg_data[2]);
    /* Mag Z axis data */
    reg_data[4] = BMM150_GET_BITS(reg_data[4], BMM150_DATA_Z);
    /* Shift the MSB data to left by 7 bits */
    /* Multiply by 128 to get the shift left by 7 value */
    msb_data = ((int16_t)((int8_t)reg_data[5])) * 128;
    /* Raw mag Z axis data */
    m_raw_mag_data.z = (int16_t)(msb_data | reg_data[4]);
    /* Mag R-HALL data */
    reg_data[6] = BMM150_GET_BITS(reg_data[6], BMM150_DATA_RHALL);
    m_raw_mag_data.r = (uint16_t)(((uint16_t)reg_data[7] << 6) | reg_data[6]);

    /* Compensated Mag X data in int16_t format */
    m_mag_data.x = adjust_x(m_raw_mag_data.x, m_raw_mag_data.r);
    /* Compensated Mag Y data in int16_t format */
    m_mag_data.y = adjust_y(m_raw_mag_data.y, m_raw_mag_data.r);
    /* Compensated Mag Z data in int16_t format */
    m_mag_data.z = adjust_z(m_raw_mag_data.z, m_raw_mag_data.r);

    m_raw_heading.heading=m_raw_heading.xy_heading = atan2(m_raw_mag_data.x,m_raw_mag_data.y);
    m_raw_heading.zx_heading = atan2(m_raw_mag_data.z,m_raw_mag_data.x);
    if(m_raw_heading.heading<0) {
        m_raw_heading.heading+=2*PI;
    }
    if(m_raw_heading.heading>2*PI) {
        m_raw_heading.heading -= 2*PI;
    }
    m_raw_heading.heading *= 180/PI;
    m_raw_heading.xy_heading *= 180/PI;
    m_raw_heading.zx_heading *= 180/PI;

    m_heading.heading=m_heading.xy_heading = atan2(m_mag_data.x,m_mag_data.y);
    m_heading.zx_heading = atan2(m_mag_data.z,m_mag_data.x);
    if(m_heading.heading<0) {
        m_heading.heading+=2*PI;
    }
    if(m_heading.heading>2*PI) {
        m_heading.heading -= 2*PI;
    }
    m_heading.heading *= 180/PI;
    m_heading.xy_heading *= 180/PI;
    m_heading.zx_heading *= 180/PI;
}

}