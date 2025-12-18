#ifndef USB_DESCRIPTORS_H_
#define USB_DESCRIPTORS_H_

#include "common/tusb_common.h"
#include "device/usbd.h"

enum {
    REPORT_ID_JOYSTICK = 1,
    REPORT_ID_LIGHTS,
};

#define HID_STRING_MINIMUM(x) HID_REPORT_ITEM(x, 8, RI_TYPE_LOCAL, 1)
#define HID_STRING_MAXIMUM(x) HID_REPORT_ITEM(x, 9, RI_TYPE_LOCAL, 1)

#define GAMECON_REPORT_DESC_JOYSTICK                           \
    HID_USAGE_PAGE(HID_USAGE_PAGE_DESKTOP),                    \
    HID_USAGE(HID_USAGE_DESKTOP_JOYSTICK),                     \
    HID_COLLECTION(HID_COLLECTION_APPLICATION),                \
        HID_REPORT_ID(REPORT_ID_JOYSTICK)                      \
        HID_USAGE_PAGE(HID_USAGE_PAGE_BUTTON),                 \
        HID_USAGE_MIN(1), HID_USAGE_MAX(14),                   \
        HID_LOGICAL_MIN(0), HID_LOGICAL_MAX(1),                \
        HID_REPORT_COUNT(14), HID_REPORT_SIZE(1),              \
        HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),     \
        HID_REPORT_COUNT(1), HID_REPORT_SIZE(16 - 14),         \
        HID_INPUT(HID_CONSTANT | HID_VARIABLE | HID_ABSOLUTE), \
                                                               \
        HID_USAGE_PAGE(HID_USAGE_PAGE_DESKTOP),                \
        HID_LOGICAL_MIN(0x00), HID_LOGICAL_MAX_N(0xff, 2),     \
        HID_USAGE(HID_USAGE_DESKTOP_X),                        \
        HID_USAGE(HID_USAGE_DESKTOP_Y),                        \
        HID_REPORT_COUNT(2), HID_REPORT_SIZE(8),               \
        HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),     \
    HID_COLLECTION_END
	
#define GAMECON_REPORT_DESC_JOYSTICK_PS                        \
    HID_USAGE_PAGE(HID_USAGE_PAGE_DESKTOP),                    \
    HID_USAGE(HID_USAGE_DESKTOP_GAMEPAD),                     \
    HID_COLLECTION(HID_COLLECTION_APPLICATION),                \
        HID_REPORT_ID(REPORT_ID_JOYSTICK)                      \
        /* 0..3: LeftStick X, LeftStick Y, Z, Rz (bytes 1..4) */\
        HID_USAGE_PAGE(HID_USAGE_PAGE_DESKTOP),                \
        HID_USAGE(HID_USAGE_DESKTOP_X),                        \
        HID_USAGE(HID_USAGE_DESKTOP_Y),                        \
        HID_USAGE(HID_USAGE_DESKTOP_Z),                        \
        HID_USAGE(HID_USAGE_DESKTOP_RZ),                       \
        HID_LOGICAL_MIN(0x00), HID_LOGICAL_MAX_N(0xff, 2),     \
        HID_REPORT_SIZE(8), HID_REPORT_COUNT(4),               \
        HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),     \
                                                               \
        /* 4 bits: Hat Switch (bits 32..35) */                 \
        HID_USAGE_PAGE(HID_USAGE_PAGE_DESKTOP),                \
        HID_USAGE(HID_USAGE_DESKTOP_HAT_SWITCH),               \
        HID_LOGICAL_MIN(0), HID_LOGICAL_MAX(7),                \
		HID_PHYSICAL_MAX_N(315, 2),                            \
		HID_UNIT(0x14),								           \
        HID_REPORT_SIZE(4), HID_REPORT_COUNT(1),               \
        HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE | HID_NULL_STATE), \
		                                                       \
		/* 14 x 1-bit buttons (bits 36..49) */                 \
        HID_USAGE_PAGE(HID_USAGE_PAGE_BUTTON),                 \
        HID_USAGE_MIN(1), HID_USAGE_MAX(14),                   \
        HID_LOGICAL_MIN(0), HID_LOGICAL_MAX(1),                \
		HID_UNIT(0x00),								           \
        HID_REPORT_SIZE(1), HID_REPORT_COUNT(14),             \
        HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),     \
                                                               \
        /* 6 bits vendor-defined usage (bits 50..55) */        \
        HID_USAGE_PAGE_N(HID_USAGE_PAGE_VENDOR, 2),            \
        HID_USAGE_N(0x0020, 2), /* vendor:0xFF00:0x0020 */    \
        HID_LOGICAL_MIN(0), HID_LOGICAL_MAX(127),              \
        HID_REPORT_SIZE(6), HID_REPORT_COUNT(1),               \
        HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),     \
                                                               \
        /* 8 bits: Rx (bits 56..63) */                         \
        HID_USAGE_PAGE(HID_USAGE_PAGE_DESKTOP),                \
        HID_USAGE(HID_USAGE_DESKTOP_RX),                       \
        HID_LOGICAL_MIN(0), HID_LOGICAL_MAX(255),              \
        HID_REPORT_SIZE(8), HID_REPORT_COUNT(1),               \
		HID_PHYSICAL_MAX_N(315, 2),                            \
        HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),     \
                                                               \
        /* 8 bits: Ry (bits 64..71) */                         \
        HID_USAGE(HID_USAGE_DESKTOP_RY),                       \
        HID_LOGICAL_MIN(0), HID_LOGICAL_MAX(255),              \
        HID_REPORT_SIZE(8), HID_REPORT_COUNT(1),               \
		HID_PHYSICAL_MAX_N(315, 2),                            \
        HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),     \
                                                               \
        /* 54 bytes vendor-defined data (bits 72..503) */      \
        HID_USAGE_PAGE_N(HID_USAGE_PAGE_VENDOR, 2),            \
        HID_USAGE_N(0x0021, 2), /* vendor:0xFF00:0x0021 */    \
        HID_REPORT_SIZE(8), HID_REPORT_COUNT(54),              \
        HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),     \
    HID_COLLECTION_END

#define GAMECON_REPORT_DESC_LIGHTS                             \
    HID_USAGE_PAGE(HID_USAGE_PAGE_DESKTOP),                    \
    HID_USAGE(0x00),                                           \
    HID_COLLECTION(HID_COLLECTION_APPLICATION),                \
        HID_REPORT_ID(REPORT_ID_LIGHTS)                        \
        HID_REPORT_COUNT(14), HID_REPORT_SIZE(8),              \
        HID_LOGICAL_MIN(0x00), HID_LOGICAL_MAX_N(0x00ff, 2),   \
        HID_USAGE_PAGE(HID_USAGE_PAGE_ORDINAL),                \
        HID_STRING_MINIMUM(5), HID_STRING_MAXIMUM(17),         \
        HID_USAGE_MIN(1), HID_USAGE_MAX(16),                   \
        HID_OUTPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),    \
        HID_REPORT_COUNT(1), HID_REPORT_SIZE(8),               \
        HID_INPUT(HID_CONSTANT | HID_VARIABLE | HID_ABSOLUTE), \
    HID_COLLECTION_END
	
extern bool joy_mode_check;

/* Enable Konami or PlayStation spoof mode */
void switch_to_konami_mode();
void switch_to_ps_mode();

#endif /* USB_DESCRIPTORS_H_ */