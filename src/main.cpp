#include <Arduino.h>
#include <SPIFFS.h>
#include <SD.h>
#include <mpu6886.hpp>
#include <ili9341.hpp>
#include <tft_io.hpp>
#include <htcw_button.hpp>
#include <gfx.hpp>
#include <w2812.hpp>
#include "Robinette.hpp"
using namespace arduino;
using namespace gfx;

constexpr static const uint8_t spi_host = VSPI;
constexpr static const int8_t lcd_pin_bl = 32;
constexpr static const int8_t lcd_pin_dc = 27;
constexpr static const int8_t lcd_pin_rst = 33;
constexpr static const int8_t lcd_pin_cs = 14;
constexpr static const int8_t sd_pin_cs = 4;
constexpr static const int8_t speaker_pin_cs = 25;
constexpr static const int8_t mic_pin_cs = 34;
constexpr static const int8_t button_a_pin = 39;
constexpr static const int8_t button_b_pin = 38;
constexpr static const int8_t button_c_pin = 37;
constexpr static const int8_t led_pin = 15;
constexpr static const int8_t spi_pin_mosi = 23;
constexpr static const int8_t spi_pin_clk = 18;
constexpr static const int8_t spi_pin_miso = 19;

using bus_t = tft_spi_ex<spi_host, 
                        lcd_pin_cs, 
                        spi_pin_mosi, 
                        -1, 
                        spi_pin_clk, 
                        SPI_MODE0,
                        true, 
                        320 * 240 * 2 + 8, 2>;

using lcd_t = ili9342c<lcd_pin_dc, 
                      lcd_pin_rst, 
                      lcd_pin_bl, 
                      bus_t, 
                      1, 
                      true, 
                      400,200>;

using lscolor_t = color<typename w2812::pixel_type>;
using color_t = color<typename lcd_t::pixel_type>;

lcd_t lcd;

// declare the MPU6886 that's attached
// to the first I2C host
mpu6886 gyro(i2c_container<0>::instance());
// the following is equiv at least on the ESP32
// mpu6886 mpu(Wire);

w2812 led_strips({5,2},led_pin,NEO_GBR);

button<button_a_pin,10,true> button_a;
button<button_b_pin,10,true> button_b;
button<button_c_pin,10,true> button_c;

// initialize M5 Stack Fire peripherals/features
void initialize_m5stack_fire() {
    Serial.begin(115200);
    SPIFFS.begin(false);
    SD.begin(4,spi_container<spi_host>::instance());
    led_strips.fill(led_strips.bounds(),lscolor_t::purple);
    lcd.fill(lcd.bounds(),color_t::purple);
    rect16 rect(0,0,64,64);
    rect.center_inplace(lcd.bounds());
    lcd.fill(rect,color_t::white);
    lcd.fill(rect.inflate(-8,-8),color_t::purple);
    // the following takes awhile
    gyro.initialize();
    // see https://github.com/m5stack/m5-docs/blob/master/docs/en/core/fire.md
    pinMode(led_pin, OUTPUT_OPEN_DRAIN);
    led_strips.initialize();
    button_a.initialize();
    button_b.initialize();
    button_c.initialize();
}

const char* text = "hello!";
const open_font &text_font = Robinette;
constexpr static const uint16_t text_height = 125;
// The above line is because some fonts are kinda small 
// in lowercase. Use the above for that. Otherwise
// use the following:
//constexpr static const uint16_t text_height = 100;
srect16 text_rect;
open_text_info text_draw_info;

const rgb_pixel<16> colors_a[] = {
    color_t::red,
    color_t::orange,
    color_t::yellow,
    color_t::green,
    color_t::blue,
    color_t::purple
};
constexpr const size_t color_count_a = sizeof(colors_a)/sizeof(rgb_pixel<16>);
const rgb_pixel<16> colors_b[] = {
    color_t::cyan,
    color_t::pink,
    color_t::white,
    color_t::pink
};
constexpr const size_t color_count_b = sizeof(colors_b)/sizeof(rgb_pixel<16>);
const rgb_pixel<16> colors_c[] = {
    color_t::red,
    color_t::white,
    color_t::blue,
};
constexpr const size_t color_count_c = sizeof(colors_c)/sizeof(rgb_pixel<16>);

int color_height;
unsigned int color_offset;
size_t color_count;
const rgb_pixel<16>* colors;
 
uint32_t led_strip_ts;
uint32_t led_strip_offset;

// the frame buffer type is based on the LCD's pixel format
using frame_buffer_t = bitmap_type_from<lcd_t>;
uint8_t* frame_buffer_data;
frame_buffer_t frame_buffer;

void set_colors(int i) {
    switch(i) {
        case 0:
            colors = colors_a;
            color_count = color_count_a;
            break;
        case 1:
            colors = colors_b;
            color_count = color_count_b;
            break;
        default:
            colors = colors_c;
            color_count = color_count_c;
            break;
    }
    
    color_height = lcd.dimensions().height/color_count;
}

void setup() {
    initialize_m5stack_fire();

    led_strip_ts=0;
    led_strip_offset = 0;
    color_offset = 0;
    // framebuffer is in PSRAM
    frame_buffer_data = (uint8_t*)ps_malloc(frame_buffer_t::sizeof_buffer(lcd.dimensions()));
    if(frame_buffer_data==nullptr) {
        // shouldn't happen
        Serial.println("Out of memory.");
        while(true) {delay(10000);}
    }
    
    button_a.callback([](bool value,void*state){
        if(value) {
            Serial.println("a");
            set_colors(0);
        }
    },nullptr);

    button_b.callback([](bool value,void*state){
        if(value) {
            Serial.println("b");
            set_colors(1);
        }
    },nullptr);

    button_c.callback([](bool value,void*state){
        if(value) {
            Serial.println("c");
            set_colors(2);
        }
    },nullptr);

    // create a bitmap with the same format as the LCD
    frame_buffer = create_bitmap_from(lcd,lcd.dimensions(),frame_buffer_data);

    // choose the first "palette"
    set_colors(0);

    // draw the stripes    
    rect16 r(0,0,lcd.bounds().x2,color_height-1);
    frame_buffer.fill(r,colors[0]);
    for(size_t i = 1;i<color_count;++i) {
        r.offset_inplace(0,color_height);
        frame_buffer.fill(r,colors[i]);
    }
    
    // blt to the LCD
    draw::bitmap(lcd,lcd.bounds(),frame_buffer,frame_buffer.bounds());
    
    // precompute our text info
    text_draw_info.text = text;
    text_draw_info.font = &text_font;
    text_draw_info.scale = text_font.scale(text_height);
    text_rect = text_font.measure_text(ssize16::max(),spoint16::zero(),text,text_draw_info.scale).bounds().center((srect16)lcd.bounds());
}
void loop() {
    button_a.update();
    button_b.update();
    button_c.update();
    // draw a trailing line color for each horizontal bar, expanding it downward by one pixel
    rect16 r(0,(color_height-1+color_offset)%lcd.dimensions().height,lcd.bounds().x2,(color_height-1+color_offset)%lcd.dimensions().height);
    frame_buffer.fill(r,colors[0]);
    for(size_t i = 1;i<color_count;++i) {
        r.offset_inplace(0,color_height);
        r.y1=r.y2=(r.y1%lcd.dimensions().height);
        frame_buffer.fill(r,colors[i]);
    }
    // draw the font
    draw::text(frame_buffer,text_rect,text_draw_info,color_t::black);
    // now blt the frame buffer to the display
    draw::bitmap(lcd,lcd.bounds(),frame_buffer,frame_buffer.bounds());   
    uint32_t ms = millis();
    if(ms>=led_strip_ts+250) {
        led_strip_ts = ms;
        // suspend so we update all at once on resume
        draw::suspend(led_strips);
        // update each color
        for(int y = 0;y<led_strips.dimensions().height;++y) {
            for(int x = 0;x<led_strips.dimensions().width;++x) {
                draw::point(led_strips,point16(x,y),colors[(led_strip_offset+(x+(y*led_strips.dimensions().width)))%color_count]);
            }
        }
        // finally, refresh the strips
        draw::resume(led_strips);
        ++led_strip_offset;
    }
    ++color_offset;
    
}