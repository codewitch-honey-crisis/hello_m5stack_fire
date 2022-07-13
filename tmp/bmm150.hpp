#pragma once
#include <Arduino.h>
#include <Wire.h>
namespace arduino {
class bmm150 {
    bool m_initialized;
    TwoWire& m_i2c;
    struct trim_registers {
        /*! trim x1 data */
        int8_t dig_x1;
        /*! trim y1 data */
        int8_t dig_y1;
        /*! trim x2 data */
        int8_t dig_x2;
        /*! trim y2 data */
        int8_t dig_y2;
        /*! trim z1 data */
        uint16_t dig_z1;
        /*! trim z2 data */
        int16_t dig_z2;
        /*! trim z3 data */
        int16_t dig_z3;
        /*! trim z4 data */
        int16_t dig_z4;
        /*! trim xy1 data */
        uint8_t dig_xy1;
        /*! trim xy2 data */
        int8_t dig_xy2;
        /*! trim xyz1 data */
        uint16_t dig_xyz1;
    };
    trim_registers m_trim_data;
    struct raw_mag_data {
        /*! Raw mag X data */
        int16_t x;
        /*! Raw mag Y data */
        int16_t y;
        /*! Raw mag Z data */
        int16_t z;
        /*! Raw mag resistance value */
        uint16_t r;
    };
    raw_mag_data m_raw_mag_data;
    struct mag_data {
        int16_t x;
        int16_t y;
        int16_t z;
    };
    mag_data m_mag_data;
    struct heading_data {
        float heading;
        float xy_heading;
        float zx_heading;
    };
    heading_data m_raw_heading;
    heading_data m_heading;
    
    struct settings {
        /*! Control measurement of XYZ axes */
        uint8_t xyz_axes_control;
        /*! Power control bit value */
        uint8_t pwr_cntrl_bit;
        /*! Power control bit value */
        uint8_t pwr_mode;
        /*! Data rate value (ODR) */
        uint8_t data_rate;
        /*! XY Repetitions */
        uint8_t xy_rep;
        /*! Z Repetitions */
        uint8_t z_rep;
        /*! Preset mode of sensor */
        uint8_t preset_mode;
        /*! Interrupt configuration settings */
        // struct bmm150_int_ctrl_settings int_settings;
    };
    settings m_settings;
    void update_trim();
    void power_mode(uint8_t mode);
    void preset_mode(uint8_t mode);
    void odr_xyz_rep(const settings& s);
    void odr(const settings& s);
    void xy_rep(const settings& s);
    void z_rep(const settings& s);
    int16_t adjust_x(int16_t max_data_x, uint16_t data_rhall);
    int16_t adjust_y(int16_t max_data_y, uint16_t data_rhall);
    int16_t adjust_z(int16_t max_data_z, uint16_t data_rhall);
public:
    constexpr static const int8_t address = 0x13;
    inline bmm150(TwoWire& i2c) : m_initialized(false), m_i2c(i2c) {

    }
    inline bool initialized() const { return m_initialized; }
    bool initialize();
    void reset();
    void update();
    inline int16_t raw_x() const {return m_raw_mag_data.x;}
    inline int16_t raw_y() const {return m_raw_mag_data.y;}
    inline int16_t raw_z() const {return m_raw_mag_data.z;}
    inline uint16_t raw_r() const {return m_raw_mag_data.r;}
    inline float raw_heading() const {return m_raw_heading.heading;};
    inline float raw_xy_heading() const {return m_raw_heading.xy_heading;};
    inline float raw_zx_heading() const {return m_raw_heading.zx_heading;};
    inline int16_t x() const {return m_mag_data.x;}
    inline int16_t y() const {return m_mag_data.y;}
    inline int16_t z() const {return m_mag_data.z;}
    inline float heading() const {return m_heading.heading;};
    inline float xy_heading() const {return m_heading.xy_heading;};
    inline float zx_heading() const {return m_heading.zx_heading;};
};
}