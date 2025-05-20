#include "st7701.hpp"
#include "libraries/pico_graphics/pico_graphics.hpp"
#include "micropython/modules/util.hpp"
#include "ws2812.hpp"
#include <cstdio>
#include <cfloat>


#include "hardware/structs/ioqspi.h"
#include "hardware/structs/qmi.h"
#include "hardware/structs/xip_ctrl.h"


using namespace pimoroni;
using namespace plasma;

extern "C" {
#include "presto.h"
#include "py/builtin.h"
#include <stdarg.h>

// MicroPython's GC heap will automatically resize, so we should just
// statically allocate these in C++ to avoid fragmentation.
__attribute__((section(".uninitialized_data"))) static uint16_t presto_buffer[WIDTH * HEIGHT];
__attribute__((section(".uninitialized_data"), aligned(1024))) static uint32_t presto_palette[256];

void __printf_debug_flush() {
    for(auto i = 0u; i < 10; i++) {
        sleep_ms(2);
        mp_event_handle_nowait();
    }
}

int mp_vprintf(const mp_print_t *print, const char *fmt, va_list args);

#if DEBUG
void presto_debug(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    int ret = mp_vprintf(&mp_plat_print, fmt, ap);
    va_end(ap);
    __printf_debug_flush();
    (void)ret;
}
#else
#define presto_debug(fmt, ...)
#endif

typedef struct _Presto_led_values_t {
    float h, s, v;
} _Presto_led_values_t;

typedef struct _Presto_led_pulsating_t {
    int fade_time = 0, on_time = 0, off_time = 0;
    float min_value = 0.0f;
} _Presto_led_pulsating_t;

/***** Variables Struct *****/
typedef struct _Presto_obj_t {
    mp_obj_base_t base;
    ST7701* presto;
    uint16_t width;
    uint16_t height;
    bool using_palette;
    volatile bool exit_core1;

    WS2812* ws2812;
    int led_pulsating_counter = 0;
    uint led_pulsating_counter_direction = LED_COUNTER_UP;
    _Presto_led_values_t led_values[7];
    _Presto_led_pulsating_t led_pulsating;
} _Presto_obj_t;

typedef struct _ModPicoGraphics_obj_t {
    mp_obj_base_t base;
    PicoGraphics *graphics;
    DisplayDriver *display;
} ModPicoGraphics_obj_t;

// There can only be one presto display, so have a global pointer
// so that core1 can access it.  Note it also needs to be in the
// Micropython object to prevent GC freeing it.
static _Presto_obj_t *presto_obj;

#define NUM_LEDS 7

static void __no_inline_not_in_flash_func(update_backlight_leds)() {
    while (!presto_obj->exit_core1) {
        presto_obj->presto->wait_for_vsync();

        if (presto_obj->exit_core1) break;

        float b = 1.0f;
        
        if (presto_obj->led_pulsating.fade_time > 0 || presto_obj->led_pulsating.on_time > 0 || presto_obj->led_pulsating.off_time > 0) {
            const int top = presto_obj->led_pulsating.fade_time + presto_obj->led_pulsating.on_time;
            const int bottom = -presto_obj->led_pulsating.off_time;
            
            if (presto_obj->led_pulsating_counter_direction == LED_COUNTER_UP) {
                if (presto_obj->led_pulsating_counter < top) {
                    presto_obj->led_pulsating_counter++;
                } else {
                    presto_obj->led_pulsating_counter_direction = LED_COUNTER_DOWN;
                }
            } else {
                if (presto_obj->led_pulsating_counter > bottom) {
                    presto_obj->led_pulsating_counter--;
                } else {
                    presto_obj->led_pulsating_counter_direction = LED_COUNTER_UP;
                }
            }

            if (presto_obj->led_pulsating_counter > presto_obj->led_pulsating.fade_time) {
                b = 1.0f;
            } else if (presto_obj->led_pulsating_counter < 0) {
                b = presto_obj->led_pulsating.min_value;
            } else {
                const float range = 1.0f - presto_obj->led_pulsating.min_value;
                b = (((float) presto_obj->led_pulsating_counter / (float) presto_obj->led_pulsating.fade_time) * range) + presto_obj->led_pulsating.min_value;
            }
        }

        for (int i = 0; i < NUM_LEDS; ++i) {
            const float h = presto_obj->led_values[i].h;
            const float s = presto_obj->led_values[i].s;
            const float v = presto_obj->led_values[i].v;
            presto_obj->ws2812->set_hsv(i, h, s, v * b);
        }

        presto_obj->ws2812->update();
    }

    multicore_fifo_push_blocking(1);
}

void presto_core1_entry() {
    // The multicore lockout uses the FIFO, so we use just use sev and volatile flags to signal this core
    multicore_lockout_victim_init();

    presto_obj->presto->init();

    multicore_fifo_push_blocking(0); // Todo handle issues here?*/

    // Presto is now running the display using interrupts on this core.
    // We can also drive the backlight if requested.
    while (!presto_obj->exit_core1) {
        if (!presto_obj->exit_core1) {
            update_backlight_leds();
        }
    }

    presto_obj->presto->cleanup();

    multicore_fifo_push_blocking(0);
}

#define stack_size 512u
static uint32_t core1_stack[stack_size] = {0};

mp_obj_t Presto_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    _Presto_obj_t *self = nullptr;

    enum { ARG_full_res, ARG_palette };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_full_res, MP_ARG_BOOL, {.u_bool = false} },
        { MP_QSTR_palette, MP_ARG_BOOL, {.u_bool = false} },
    };

    // Parse args.
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    presto_debug("malloc self\n");
    self = mp_obj_malloc_with_finaliser(_Presto_obj_t, &Presto_type);
    presto_obj = self;

    presto_debug("set fb pointers\n");

    if (!args[ARG_full_res].u_bool) {
        self->width = WIDTH / 2;
        self->height = HEIGHT / 2;
    }
    else {
        self->width = WIDTH;
        self->height = HEIGHT;
    }

    self->using_palette = args[ARG_palette].u_bool;

    presto_debug("m_new_class(ST7701...\n");
    self->presto = m_new_class(ST7701, self->width, self->height, ROTATE_0,
        SPIPins{spi1, LCD_CS, LCD_CLK, LCD_DAT, PIN_UNUSED, LCD_DC, BACKLIGHT},
        presto_buffer, self->using_palette ? presto_palette : nullptr,
        LCD_D0);

    presto_debug("launch core1\n");
    multicore_reset_core1();
    self->exit_core1 = false;

    WS2812::RGB* buffer = m_new(WS2812::RGB, NUM_LEDS);
    self->ws2812 = m_new_class(WS2812, NUM_LEDS, pio0, 3, LED_DAT, WS2812::DEFAULT_SERIAL_FREQ, false, WS2812::COLOR_ORDER::GRB, buffer);
    memset(self->led_values, 0, sizeof(self->led_values));

    // Micropython uses all of both scratch memory (and more!) for core0 stack, 
    // so we must supply our own small stack for core1 here.
    multicore_launch_core1_with_stack(presto_core1_entry, core1_stack, stack_size);
    presto_debug("launched core1\n");

    int res = multicore_fifo_pop_blocking();
    presto_debug("core1 returned\n");

    if(res != 0) {
        mp_raise_msg(&mp_type_RuntimeError, "Presto: failed to start ST7701 on Core1.");
    }

    return MP_OBJ_FROM_PTR(self);
}

mp_int_t Presto_get_framebuffer(mp_obj_t self_in, mp_buffer_info_t *bufinfo, mp_uint_t flags) {
    _Presto_obj_t *self = MP_OBJ_TO_PTR2(self_in, _Presto_obj_t);
    (void)flags;
    if(self->width == WIDTH / 2) {
        // Skip the first region, since it's used as the front-buffer
        bufinfo->buf = presto_buffer + (self->width * self->height);
        // Return the remaining space, enough for three layers at 16bpp
        bufinfo->len = self->width * self->height * 2 * 3;
    } else if (self->using_palette) {
        // Full res palette mode, there is enough space for a single layer
        bufinfo->buf = (uint8_t*)presto_buffer + (self->width * self->height);
        bufinfo->len = self->width * self->height;
    } else {
        // Just return the buffer as-is, this is not really useful for much
        // other than doing fast writes *directly* to the front buffer
        bufinfo->buf = presto_buffer;
        bufinfo->len = self->width * self->height * 2;
    }
    bufinfo->typecode = 'B';
    return 0;
}

extern mp_obj_t Presto_update(mp_obj_t self_in, mp_obj_t graphics_in) {
    _Presto_obj_t *self = MP_OBJ_TO_PTR2(self_in, _Presto_obj_t);
    ModPicoGraphics_obj_t *picographics = MP_OBJ_TO_PTR2(graphics_in, ModPicoGraphics_obj_t);

    self->presto->update(picographics->graphics);

    return mp_const_none;
}

extern mp_obj_t Presto_partial_update(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_self, ARG_graphics, ARG_x, ARG_y, ARG_w, ARG_h };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_, MP_ARG_REQUIRED | MP_ARG_OBJ },
        { MP_QSTR_graphics, MP_ARG_REQUIRED | MP_ARG_OBJ },
        { MP_QSTR_x, MP_ARG_REQUIRED | MP_ARG_INT },
        { MP_QSTR_y, MP_ARG_REQUIRED | MP_ARG_INT },
        { MP_QSTR_w, MP_ARG_REQUIRED | MP_ARG_INT },
        { MP_QSTR_h, MP_ARG_REQUIRED | MP_ARG_INT }
    };

    // Parse args.
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    _Presto_obj_t *self = MP_OBJ_TO_PTR2(args[ARG_self].u_obj, _Presto_obj_t);
    ModPicoGraphics_obj_t *picographics = MP_OBJ_TO_PTR2(args[ARG_graphics].u_obj, ModPicoGraphics_obj_t);
    int x = args[ARG_x].u_int;
    int y = args[ARG_y].u_int;
    int w = args[ARG_w].u_int;
    int h = args[ARG_h].u_int;

    self->presto->partial_update(picographics->graphics, {x, y, w, h});

    return mp_const_none;
}

mp_obj_t Presto_set_backlight(mp_obj_t self_in, mp_obj_t brightness) {
    _Presto_obj_t *self = MP_OBJ_TO_PTR2(self_in, _Presto_obj_t);

    float b = mp_obj_get_float(brightness);

    if(b < 0 || b > 1.0f) mp_raise_ValueError("brightness out of range. Expected 0.0 to 1.0");

    self->presto->set_backlight((uint8_t)(b * 255.0f));

    return mp_const_none;
}

static void cleanup_leds() {
    void* buffer = presto_obj->ws2812->buffer;
    presto_obj->ws2812->stop();
    presto_obj->ws2812->clear();
    sleep_ms(1);
    presto_obj->ws2812->update(true);
    sleep_ms(1);
    m_del_class(WS2812, presto_obj->ws2812);
    m_del(WS2812::RGB, buffer, NUM_LEDS);
    presto_obj->ws2812 = nullptr;
}

typedef struct _mp_obj_float_t {
    mp_obj_base_t base;
    mp_float_t value;
} mp_obj_float_t;

const mp_obj_float_t const_float_1 = {{&mp_type_float}, 1.0f};

mp_obj_t Presto_set_led_hsv(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_self, ARG_index, ARG_h, ARG_s, ARG_v };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_, MP_ARG_REQUIRED | MP_ARG_OBJ },
        { MP_QSTR_index, MP_ARG_REQUIRED | MP_ARG_INT },
        { MP_QSTR_hue, MP_ARG_REQUIRED | MP_ARG_OBJ },
        { MP_QSTR_sat, MP_ARG_OBJ, {.u_rom_obj = MP_ROM_PTR(&const_float_1)} },
        { MP_QSTR_val, MP_ARG_OBJ, {.u_rom_obj = MP_ROM_PTR(&const_float_1)} },
    };

    // Parse args.
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    _Presto_obj_t *self = MP_OBJ_TO_PTR2(args[ARG_self].u_obj, _Presto_obj_t);

    int index = args[ARG_index].u_int;
    float h = mp_obj_get_float(args[ARG_h].u_obj);
    float s = mp_obj_get_float(args[ARG_s].u_obj);
    float v = mp_obj_get_float(args[ARG_v].u_obj);

    self->led_values[index] = {h, s, v};

    return mp_const_none;
}

mp_obj_t Presto_set_led_pulsating(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_self, ARG_fade_time, ARG_on_time, ARG_off_time, ARG_min_value};
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_, MP_ARG_REQUIRED | MP_ARG_OBJ },
        { MP_QSTR_fade_time, MP_ARG_REQUIRED | MP_ARG_INT },        
        { MP_QSTR_on_time, MP_ARG_REQUIRED | MP_ARG_INT },        
        { MP_QSTR_off_time, MP_ARG_REQUIRED | MP_ARG_INT },        
        { MP_QSTR_min_value, MP_ARG_REQUIRED | MP_ARG_OBJ },
    };

    // Parse args.
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    _Presto_obj_t *self = MP_OBJ_TO_PTR2(args[ARG_self].u_obj, _Presto_obj_t);

    const int fade_time = args[ARG_fade_time].u_int;
    const int on_time = args[ARG_on_time].u_int;
    const int off_time = args[ARG_off_time].u_int;
    const float min_value = mp_obj_get_float(args[ARG_min_value].u_obj);

    self->led_pulsating = {fade_time, on_time, off_time, min_value};
    self->led_pulsating_counter = 0;
    self->led_pulsating_counter_direction = LED_COUNTER_UP;

    return mp_const_none;
}

mp_obj_t Presto___del__(mp_obj_t self_in) {
    (void)self_in;
    //_Presto_obj_t *self = MP_OBJ_TO_PTR2(self_in, _Presto_obj_t);
    presto_debug("signal core1\n");
    presto_obj->exit_core1 = true;
    __sev();

    int fifo_code;
    do {
        fifo_code = multicore_fifo_pop_blocking();
        if (fifo_code == 1) {
            cleanup_leds();
        }
    } while (fifo_code != 0);

    presto_debug("core1 returned\n");

    m_del_class(ST7701, presto_obj->presto);
    presto_obj->presto = nullptr;
    presto_obj = nullptr;
    
    return mp_const_none;
}

}