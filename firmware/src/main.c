/*
 * Controller Main
 * WHowe <github.com/whowechina>
 */

#include <stdint.h>
#include <stdbool.h>

#include "bsp/board.h"
#include "pico/multicore.h"
#include "pico/bootrom.h"
#include "pico/stdio.h"
#include "hardware/watchdog.h"

#include "tusb.h"
#include "usb_descriptors.h"

#include "setup.h"

#include "buttons.h"
#include "hebtn.h"
#include "rgb.h"
#include "turntable.h"

#include "tt_blade.h"
#include "tt_rainbow.h"
#include "tt_heatbar.h"

#include "savedata.h"
#include "config.h"
#include "cli.h"
#include "commands.h"

#include "board_defs.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"

bool joy_mode_check = true;

#define AUX_1_BIT 12
#define AUX_2_BIT 11
#define AUX_PS_BIT 10

void boot_check()
{
    uint16_t key1 = (1 << AUX_1_BIT);
    uint16_t key2 = (1 << AUX_2_BIT);
    uint16_t buttons = button_read();
    if (!watchdog_caused_reboot() && (buttons & key1) && (buttons & key2)) {
        reset_usb_boot(0, 2);
    }
}
 
void mode_check()
{
    uint16_t key1 = (1 << AUX_1_BIT);
    uint16_t key2 = (1 << AUX_2_BIT);
	uint16_t key3 = (1 << AUX_PS_BIT);
    uint16_t buttons = button_read();
    if (buttons & key1) {
        iidx_cfg->hid.konami = true;
		iidx_cfg->hid.ps = false;
        savedata_save(false);
    } else if (buttons & key2) {
        iidx_cfg->hid.konami = false;
		iidx_cfg->hid.ps = false;
        savedata_save(false);
    } else if (buttons & key3) {
        iidx_cfg->hid.konami = false;
		iidx_cfg->hid.ps = true;
        savedata_save(false);
    }

	joy_mode_check = true;
    if (iidx_cfg->hid.konami) {
        switch_to_konami_mode();
    } else if (iidx_cfg->hid.ps) {
		joy_mode_check = false;
        switch_to_ps_mode();
    }
}

struct __attribute__((packed)) {
    uint16_t buttons;
    uint8_t axis[2];
} hid_joy, hid_joy_sent;

struct __attribute__((packed)) {
    uint8_t lx;
    uint8_t ly;
    uint8_t rx;
    uint8_t ry;

    // Byte 5 — DPad + 4 face buttons
    uint8_t dpad_buttons;

    // Byte 6 — L1 R1 L2 R2 Share Options L3 R3
    uint8_t shoulder_buttons;

    // Byte 7 — PS + Touchpad
    uint8_t special_buttons;

    // Bytes 8-63 — empty (should be 0)
    uint8_t rest[56];
} hid_joy_ps, hid_joy_sent_ps;

void report_usb_hid()
{
    static uint64_t last_report_time = 0;
    if (!tud_hid_ready()) return;

    uint64_t now = time_us_64();
    if (iidx_cfg->hid.ps) { // PlayStation mode
        if ((memcmp(&hid_joy_ps, &hid_joy_sent_ps, sizeof(hid_joy_ps)) == 0) &&
            (now - last_report_time < 10000)) return;

        last_report_time = now;
        if (tud_hid_report(REPORT_ID_JOYSTICK, &hid_joy_ps, sizeof(hid_joy_ps))) {
            hid_joy_sent_ps = hid_joy_ps;
        }
    } else {
        if ((memcmp(&hid_joy, &hid_joy_sent, sizeof(hid_joy)) == 0) &&
            (now - last_report_time < 10000)) return;

        last_report_time = now;
        if (tud_hid_report(REPORT_ID_JOYSTICK, &hid_joy, sizeof(hid_joy))) {
            hid_joy_sent = hid_joy;
        }
    }
}

static uint8_t latest_angle;
static uint16_t latest_buttons;

#define TT_HOLD_TIME_MS 200

static uint8_t gen_binary_tt()
{
    static uint8_t previous_angle = 0;
    static bool tt_active = false;
    static bool tt_dir_cw = false;
    static uint32_t tt_timeout = 0;

    int8_t delta = latest_angle - previous_angle;
    previous_angle = latest_angle;

    uint64_t now = time_us_32();

    if (delta != 0) {
        tt_active = true;
        tt_dir_cw = (delta > 0);
        tt_timeout = now + TT_HOLD_TIME_MS * 1000;
    } else if (tt_active && (now > tt_timeout)) {
        tt_active = false;
    }

	if (iidx_cfg->hid.ps) {
		return tt_active ? (tt_dir_cw ? 0xff : 0x00) : 0x80;
	} else {
		hid_joy.axis[0] = tt_active ? (tt_dir_cw ? 0xff : 0x00) : 0x80;
		hid_joy.axis[1] = 0x80;
		return 0;
	}
}

static void gen_hid_report()
{
    uint16_t buttons = latest_buttons;
    if (iidx_cfg->hid.konami) {
        uint16_t aux_buttons = buttons & 0xff80;
        buttons = (buttons & 0x7f) | (aux_buttons << 1); // skips button 8
    }
	if (iidx_cfg->hid.ps) {
		uint8_t out = 0;
		uint8_t hat = 8; // default neutral
		uint8_t tt_val = gen_binary_tt();
		bool tt_up = (tt_val == 0x00);
		bool tt_down = (tt_val == 0xFF);
		bool left = (buttons & (1 << 6));
		if (tt_up && left)       hat = 7; // Up-Left
		else if (tt_down && left) hat = 5; // Down-Left
		else if (tt_up)           hat = 0; // Up
		else if (tt_down)         hat = 4; // Down
		else if (left)            hat = 6; // Left
		else                      hat = 8; // Neutral
		if (buttons & (1 << 0)) out |= (1 << 0); // 1 → Square (DS bit 0)
		if (buttons & (1 << 1)) out |= (1 << 4); // 2 → L1 (DS bit 4)
		if (buttons & (1 << 2)) out |= (1 << 1); // 3 → Cross (DS bit 1)
		if (buttons & (1 << 3)) out |= (1 << 5); // 4 → R1 (DS bit 5)
		if (buttons & (1 << 4)) out |= (1 << 2); // 5 → Circle (DS bit 2)
		if (buttons & (1 << 5)) out |= (1 << 6); // 6 → L2 (DS bit 6)
		
		uint8_t dpad_buttons = 0;
		dpad_buttons |= (hat & 0x0F);
		uint8_t face4 = (out & 0x0F) << 4;
		dpad_buttons |= face4;
		hid_joy_ps.dpad_buttons = dpad_buttons;
		
		uint8_t misc = 0;
		if (out & (1 << 4)) misc |= (1 << 0); // L1
		if (out & (1 << 5)) misc |= (1 << 1); // R1
		if (out & (1 << 6)) misc |= (1 << 2); // L2
		if (buttons & (1 << 7)) misc |= (1 << 5); // 8 → E1 = Start (DS bit 9)
		if (buttons & (1 << 8)) misc |= (1 << 4); // 9 → E2 = Select (DS bit 8)
		if (buttons & (1 << 9)) misc |= (1 << 6); // 10 → E3 = L3 (DS bit 10)
		if (buttons & (1 << 10)) misc |= (1 << 7); // 11 → E4 = R3 (DS bit 11)
		hid_joy_ps.shoulder_buttons = misc;
		
		uint8_t sp = 0;
		if (buttons & (1 << 11)) sp |= (1 << 0); // PS
		if (buttons & (1 << 12)) sp |= (1 << 1); // Touch
		hid_joy_ps.special_buttons = sp;
		
		hid_joy_ps.lx = 0x80;
		hid_joy_ps.ly = 0x80;
		hid_joy_ps.rx = 0x80;
		hid_joy_ps.ry = 0x80;
		memset(hid_joy_ps.rest, 0, sizeof(hid_joy_ps.rest));
		
	} else {
		hid_joy.buttons = buttons;

		if (iidx_cfg->sensor.binary) {
			gen_binary_tt();
		} else {
			hid_joy.axis[0] = latest_angle;
			hid_joy.axis[1] = 127;
		}
    }
}

/*
 * ESP32 Link Part
 * Silver Sword <github.com/SilverSword95>
 */

uint8_t esp_packet[4];
const uint8_t START_BYTE = 0xAA;

static void esplink_init()
{
	gpio_init(ESPLINK_TX);
    gpio_init(ESPLINK_RX);
	gpio_pull_up(ESPLINK_RX);    // for more stability
	gpio_disable_pulls(ESPLINK_TX);
	uart_init(uart0, 115200);
	gpio_set_function(ESPLINK_TX, GPIO_FUNC_UART);
    gpio_set_function(ESPLINK_RX, GPIO_FUNC_UART);
	uart_set_format(uart0, 8, 1, UART_PARITY_NONE);
	uart_set_fifo_enabled(uart0, true);
	uart_set_hw_flow(uart0, false, false);
}

static void esplink_update()
{
	uint8_t btn_low7  = hid_joy.buttons & 0x7F;        // buttons 0–6
	uint8_t btn_high7 = (hid_joy.buttons >> 7) & 0x7F; // buttons 7–13
	esp_packet[0] = START_BYTE;
	esp_packet[1] = hid_joy.axis[0];
	esp_packet[2] = btn_low7;
	esp_packet[3] = btn_high7;
	uart_write_blocking(uart0, esp_packet, 4);
}


static mutex_t core1_io_lock;
static void core1_loop()
{
    while (true) {
        uint32_t raw_angle = turntable_raw();
        latest_angle = turntable_read();

        if (mutex_try_enter(&core1_io_lock, NULL)) {
            rgb_update(raw_angle, latest_buttons);
            mutex_exit(&core1_io_lock);
        }

        cli_fps_count(1);
        sleep_us(500);
    }
}

static uint16_t hybrid_button_read()
{
    uint16_t buttons = button_read();
    for (int i = 0; i < hebtn_keynum(); i++) {
        if (hebtn_present(i)) {
            buttons |= (hebtn_actuated(i) << i);
        }
    }
    return buttons;
}

static bool hall_version = false;

static void core0_loop()
{
    absolute_time_t next_frame = {0};

    while (true)
    {
        tud_task();
        cli_run();

        turntable_update();

        if (hall_version) {
            hebtn_update();
        }

        latest_buttons = hybrid_button_read();
        uint16_t angle = turntable_raw() >> 4;
        setup_run(latest_buttons, angle);

        bool ov_tt = setup_needs_tt_led();
        bool ov_btn = setup_needs_button_led();

        if (ov_tt) {
            rgb_override_tt(setup_led_tt);
        }
        if (ov_btn) {
            rgb_override_button(setup_led_button);
        } else {
            rgb_set_button_light(latest_buttons);
        }

        hid_joy.buttons = 0;
        if (!ov_tt && !ov_btn) {
            gen_hid_report();
            savedata_loop();
			if (iidx_cfg->hid.konami) {
				esplink_update();
			}
        }

        report_usb_hid();
        cli_fps_count(0);

        sleep_until(next_frame);
        next_frame += 1001;
    }
}

void init()
{
    board_init();
    tusb_init();
    
    button_init();

    bool tt_present = turntable_init();

    if ((tt_present) && (turntable_is_alternative())) {
        // identify hall version by sensor's i2c port
        hall_version = true;
        hebtn_init();
    }

    if (!tt_present) {
        // even if tt not available, we still try to detect hall sensor
        hebtn_init();
        hall_version = (hebtn_presence_map() > 0);
    }
	
	if (!turntable_is_alternative()) {
		// hall version cannot work with ESP32 Link
		esplink_init();
	}

    rgb_init(hall_version);

    tt_rainbow_init();
    tt_blade_init();
    tt_heatbar_init();

    boot_check();
    stdio_init_all();

    setup_init();
    config_init();
    mutex_init(&core1_io_lock);
    savedata_init(0xca341125, &core1_io_lock);

    cli_init("iidx_pico>", "\n   << SilverIIDX Pico Controller >>\n"
                            " https://github.com/SilverSword95\n\n");
    commands_init();
    mode_check();
}

void main(void)
{
    init();
    multicore_launch_core1(core1_loop);
    core0_loop();
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id,
                               hid_report_type_t report_type, uint8_t *buffer,
                               uint16_t reqlen)
{
    return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id,
                           hid_report_type_t report_type, uint8_t const *buffer,
                           uint16_t bufsize)
{
    if ((report_id == REPORT_ID_LIGHTS) &&
        (report_type == HID_REPORT_TYPE_OUTPUT)) {
        rgb_set_hid_light(buffer, bufsize);
    }
}
