/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * gpio_v2_compat.h -- GPIO character-device uAPI v2 definitions, supplied when
 * the toolchain's kernel headers predate them.
 *
 * WHY THIS EXISTS
 * ---------------
 * maburplay's record button (gs/player/src/rec_button.cpp) speaks the v2 GPIO
 * uAPI directly -- gpio_v2_line_request / GPIO_V2_GET_LINE_IOCTL and friends --
 * which landed in Linux 5.10. The external toolchains Buildroot offers for
 * aarch64 ship much older sysroot headers on purpose, so that binaries stay
 * runnable on old kernels: the ARM AArch64 toolchain these boards default to
 * declares BR2_TOOLCHAIN_HEADERS_AT_LEAST_4_20, and even Bootlin's aarch64
 * glibc toolchain only guarantees 5.4. Neither has v2, so rec_button.cpp does
 * not compile against them.
 *
 * The RUNTIME is fine: these boards run a 6.1 kernel, which implements v2. Only
 * the compile-time declarations are missing, and this is stable ABI, so
 * supplying them here yields byte-identical ioctls.
 *
 * package/mabur/mabur.mk force-includes this file into every mabur translation
 * unit via -include. Everything below is guarded on GPIO_V2_GET_LINE_IOCTL, so
 * the moment the toolchain's own headers gain v2 this file goes inert and can
 * be deleted along with the -include flag.
 *
 * The v2 block below is copied verbatim from the target kernel's own
 * include/uapi/linux/gpio.h (linux 6.1, radxa BSP), stopping at the deprecated
 * v1 ABI -- the old sysroot header already defines that half, and redefining it
 * here would clash. Regenerate from the same file if it ever needs refreshing.
 *
 * The durable fix belongs upstream in mabur: an equivalent #ifndef fallback in
 * rec_button.cpp would make it build against any toolchain and let this file go
 * away entirely.
 */

#ifndef MABUR_BR_GPIO_V2_COMPAT_H_
#define MABUR_BR_GPIO_V2_COMPAT_H_

#include <linux/gpio.h>

#ifndef GPIO_V2_GET_LINE_IOCTL

#include <linux/const.h>   /* _BITULL, used by enum gpio_v2_line_flag */
#include <linux/ioctl.h>
#include <linux/types.h>

#ifndef _BITULL
#define _BITULL(x) (1ULL << (x))
#endif

#ifndef GPIO_MAX_NAME_SIZE
#define GPIO_MAX_NAME_SIZE 32
#endif

#define GPIO_V2_LINES_MAX 64

/*
 * The maximum number of configuration attributes associated with a line
 * request.
 */
#define GPIO_V2_LINE_NUM_ATTRS_MAX 10

/**
 * enum gpio_v2_line_flag - &struct gpio_v2_line_attribute.flags values
 * @GPIO_V2_LINE_FLAG_USED: line is not available for request
 * @GPIO_V2_LINE_FLAG_ACTIVE_LOW: line active state is physical low
 * @GPIO_V2_LINE_FLAG_INPUT: line is an input
 * @GPIO_V2_LINE_FLAG_OUTPUT: line is an output
 * @GPIO_V2_LINE_FLAG_EDGE_RISING: line detects rising (inactive to active)
 * edges
 * @GPIO_V2_LINE_FLAG_EDGE_FALLING: line detects falling (active to
 * inactive) edges
 * @GPIO_V2_LINE_FLAG_OPEN_DRAIN: line is an open drain output
 * @GPIO_V2_LINE_FLAG_OPEN_SOURCE: line is an open source output
 * @GPIO_V2_LINE_FLAG_BIAS_PULL_UP: line has pull-up bias enabled
 * @GPIO_V2_LINE_FLAG_BIAS_PULL_DOWN: line has pull-down bias enabled
 * @GPIO_V2_LINE_FLAG_BIAS_DISABLED: line has bias disabled
 * @GPIO_V2_LINE_FLAG_EVENT_CLOCK_REALTIME: line events contain REALTIME timestamps
 * @GPIO_V2_LINE_FLAG_EVENT_CLOCK_HTE: line events contain timestamps from
 * hardware timestamp engine
 */
enum gpio_v2_line_flag {
	GPIO_V2_LINE_FLAG_USED			= _BITULL(0),
	GPIO_V2_LINE_FLAG_ACTIVE_LOW		= _BITULL(1),
	GPIO_V2_LINE_FLAG_INPUT			= _BITULL(2),
	GPIO_V2_LINE_FLAG_OUTPUT		= _BITULL(3),
	GPIO_V2_LINE_FLAG_EDGE_RISING		= _BITULL(4),
	GPIO_V2_LINE_FLAG_EDGE_FALLING		= _BITULL(5),
	GPIO_V2_LINE_FLAG_OPEN_DRAIN		= _BITULL(6),
	GPIO_V2_LINE_FLAG_OPEN_SOURCE		= _BITULL(7),
	GPIO_V2_LINE_FLAG_BIAS_PULL_UP		= _BITULL(8),
	GPIO_V2_LINE_FLAG_BIAS_PULL_DOWN	= _BITULL(9),
	GPIO_V2_LINE_FLAG_BIAS_DISABLED		= _BITULL(10),
	GPIO_V2_LINE_FLAG_EVENT_CLOCK_REALTIME	= _BITULL(11),
	GPIO_V2_LINE_FLAG_EVENT_CLOCK_HTE	= _BITULL(12),
};

/**
 * struct gpio_v2_line_values - Values of GPIO lines
 * @bits: a bitmap containing the value of the lines, set to 1 for active
 * and 0 for inactive.
 * @mask: a bitmap identifying the lines to get or set, with each bit
 * number corresponding to the index into &struct
 * gpio_v2_line_request.offsets.
 */
struct gpio_v2_line_values {
	__aligned_u64 bits;
	__aligned_u64 mask;
};

/**
 * enum gpio_v2_line_attr_id - &struct gpio_v2_line_attribute.id values
 * identifying which field of the attribute union is in use.
 * @GPIO_V2_LINE_ATTR_ID_FLAGS: flags field is in use
 * @GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES: values field is in use
 * @GPIO_V2_LINE_ATTR_ID_DEBOUNCE: debounce_period_us field is in use
 */
enum gpio_v2_line_attr_id {
	GPIO_V2_LINE_ATTR_ID_FLAGS		= 1,
	GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES	= 2,
	GPIO_V2_LINE_ATTR_ID_DEBOUNCE		= 3,
};

/**
 * struct gpio_v2_line_attribute - a configurable attribute of a line
 * @id: attribute identifier with value from &enum gpio_v2_line_attr_id
 * @padding: reserved for future use and must be zero filled
 * @flags: if id is %GPIO_V2_LINE_ATTR_ID_FLAGS, the flags for the GPIO
 * line, with values from &enum gpio_v2_line_flag, such as
 * %GPIO_V2_LINE_FLAG_ACTIVE_LOW, %GPIO_V2_LINE_FLAG_OUTPUT etc, added
 * together.  This overrides the default flags contained in the &struct
 * gpio_v2_line_config for the associated line.
 * @values: if id is %GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES, a bitmap
 * containing the values to which the lines will be set, with each bit
 * number corresponding to the index into &struct
 * gpio_v2_line_request.offsets.
 * @debounce_period_us: if id is %GPIO_V2_LINE_ATTR_ID_DEBOUNCE, the
 * desired debounce period, in microseconds
 */
struct gpio_v2_line_attribute {
	__u32 id;
	__u32 padding;
	union {
		__aligned_u64 flags;
		__aligned_u64 values;
		__u32 debounce_period_us;
	};
};

/**
 * struct gpio_v2_line_config_attribute - a configuration attribute
 * associated with one or more of the requested lines.
 * @attr: the configurable attribute
 * @mask: a bitmap identifying the lines to which the attribute applies,
 * with each bit number corresponding to the index into &struct
 * gpio_v2_line_request.offsets.
 */
struct gpio_v2_line_config_attribute {
	struct gpio_v2_line_attribute attr;
	__aligned_u64 mask;
};

/**
 * struct gpio_v2_line_config - Configuration for GPIO lines
 * @flags: flags for the GPIO lines, with values from &enum
 * gpio_v2_line_flag, such as %GPIO_V2_LINE_FLAG_ACTIVE_LOW,
 * %GPIO_V2_LINE_FLAG_OUTPUT etc, added together.  This is the default for
 * all requested lines but may be overridden for particular lines using
 * @attrs.
 * @num_attrs: the number of attributes in @attrs
 * @padding: reserved for future use and must be zero filled
 * @attrs: the configuration attributes associated with the requested
 * lines.  Any attribute should only be associated with a particular line
 * once.  If an attribute is associated with a line multiple times then the
 * first occurrence (i.e. lowest index) has precedence.
 */
struct gpio_v2_line_config {
	__aligned_u64 flags;
	__u32 num_attrs;
	/* Pad to fill implicit padding and reserve space for future use. */
	__u32 padding[5];
	struct gpio_v2_line_config_attribute attrs[GPIO_V2_LINE_NUM_ATTRS_MAX];
};

/**
 * struct gpio_v2_line_request - Information about a request for GPIO lines
 * @offsets: an array of desired lines, specified by offset index for the
 * associated GPIO chip
 * @consumer: a desired consumer label for the selected GPIO lines such as
 * "my-bitbanged-relay"
 * @config: requested configuration for the lines.
 * @num_lines: number of lines requested in this request, i.e. the number
 * of valid fields in the %GPIO_V2_LINES_MAX sized arrays, set to 1 to
 * request a single line
 * @event_buffer_size: a suggested minimum number of line events that the
 * kernel should buffer.  This is only relevant if edge detection is
 * enabled in the configuration. Note that this is only a suggested value
 * and the kernel may allocate a larger buffer or cap the size of the
 * buffer. If this field is zero then the buffer size defaults to a minimum
 * of @num_lines * 16.
 * @padding: reserved for future use and must be zero filled
 * @fd: if successful this field will contain a valid anonymous file handle
 * after a %GPIO_GET_LINE_IOCTL operation, zero or negative value means
 * error
 */
struct gpio_v2_line_request {
	__u32 offsets[GPIO_V2_LINES_MAX];
	char consumer[GPIO_MAX_NAME_SIZE];
	struct gpio_v2_line_config config;
	__u32 num_lines;
	__u32 event_buffer_size;
	/* Pad to fill implicit padding and reserve space for future use. */
	__u32 padding[5];
	__s32 fd;
};

/**
 * struct gpio_v2_line_info - Information about a certain GPIO line
 * @name: the name of this GPIO line, such as the output pin of the line on
 * the chip, a rail or a pin header name on a board, as specified by the
 * GPIO chip, may be empty (i.e. name[0] == '\0')
 * @consumer: a functional name for the consumer of this GPIO line as set
 * by whatever is using it, will be empty if there is no current user but
 * may also be empty if the consumer doesn't set this up
 * @offset: the local offset on this GPIO chip, fill this in when
 * requesting the line information from the kernel
 * @num_attrs: the number of attributes in @attrs
 * @flags: flags for this GPIO line, with values from &enum
 * gpio_v2_line_flag, such as %GPIO_V2_LINE_FLAG_ACTIVE_LOW,
 * %GPIO_V2_LINE_FLAG_OUTPUT etc, added together.
 * @attrs: the configuration attributes associated with the line
 * @padding: reserved for future use
 */
struct gpio_v2_line_info {
	char name[GPIO_MAX_NAME_SIZE];
	char consumer[GPIO_MAX_NAME_SIZE];
	__u32 offset;
	__u32 num_attrs;
	__aligned_u64 flags;
	struct gpio_v2_line_attribute attrs[GPIO_V2_LINE_NUM_ATTRS_MAX];
	/* Space reserved for future use. */
	__u32 padding[4];
};

/**
 * enum gpio_v2_line_changed_type - &struct gpio_v2_line_changed.event_type
 * values
 * @GPIO_V2_LINE_CHANGED_REQUESTED: line has been requested
 * @GPIO_V2_LINE_CHANGED_RELEASED: line has been released
 * @GPIO_V2_LINE_CHANGED_CONFIG: line has been reconfigured
 */
enum gpio_v2_line_changed_type {
	GPIO_V2_LINE_CHANGED_REQUESTED	= 1,
	GPIO_V2_LINE_CHANGED_RELEASED	= 2,
	GPIO_V2_LINE_CHANGED_CONFIG	= 3,
};

/**
 * struct gpio_v2_line_info_changed - Information about a change in status
 * of a GPIO line
 * @info: updated line information
 * @timestamp_ns: estimate of time of status change occurrence, in nanoseconds
 * @event_type: the type of change with a value from &enum
 * gpio_v2_line_changed_type
 * @padding: reserved for future use
 */
struct gpio_v2_line_info_changed {
	struct gpio_v2_line_info info;
	__aligned_u64 timestamp_ns;
	__u32 event_type;
	/* Pad struct to 64-bit boundary and reserve space for future use. */
	__u32 padding[5];
};

/**
 * enum gpio_v2_line_event_id - &struct gpio_v2_line_event.id values
 * @GPIO_V2_LINE_EVENT_RISING_EDGE: event triggered by a rising edge
 * @GPIO_V2_LINE_EVENT_FALLING_EDGE: event triggered by a falling edge
 */
enum gpio_v2_line_event_id {
	GPIO_V2_LINE_EVENT_RISING_EDGE	= 1,
	GPIO_V2_LINE_EVENT_FALLING_EDGE	= 2,
};

/**
 * struct gpio_v2_line_event - The actual event being pushed to userspace
 * @timestamp_ns: best estimate of time of event occurrence, in nanoseconds.
 * @id: event identifier with value from &enum gpio_v2_line_event_id
 * @offset: the offset of the line that triggered the event
 * @seqno: the sequence number for this event in the sequence of events for
 * all the lines in this line request
 * @line_seqno: the sequence number for this event in the sequence of
 * events on this particular line
 * @padding: reserved for future use
 *
 * By default the @timestamp_ns is read from %CLOCK_MONOTONIC and is
 * intended to allow the accurate measurement of the time between events.
 * It does not provide the wall-clock time.
 *
 * If the %GPIO_V2_LINE_FLAG_EVENT_CLOCK_REALTIME flag is set then the
 * @timestamp_ns is read from %CLOCK_REALTIME.
 */
struct gpio_v2_line_event {
	__aligned_u64 timestamp_ns;
	__u32 id;
	__u32 offset;
	__u32 seqno;
	__u32 line_seqno;
	/* Space reserved for future use. */
	__u32 padding[6];
};

#define GPIO_V2_GET_LINEINFO_IOCTL _IOWR(0xB4, 0x05, struct gpio_v2_line_info)
#define GPIO_V2_GET_LINEINFO_WATCH_IOCTL _IOWR(0xB4, 0x06, struct gpio_v2_line_info)
#define GPIO_V2_GET_LINE_IOCTL _IOWR(0xB4, 0x07, struct gpio_v2_line_request)
#define GPIO_V2_LINE_SET_CONFIG_IOCTL _IOWR(0xB4, 0x0D, struct gpio_v2_line_config)
#define GPIO_V2_LINE_GET_VALUES_IOCTL _IOWR(0xB4, 0x0E, struct gpio_v2_line_values)
#define GPIO_V2_LINE_SET_VALUES_IOCTL _IOWR(0xB4, 0x0F, struct gpio_v2_line_values)

#endif /* GPIO_V2_GET_LINE_IOCTL */
#endif /* MABUR_BR_GPIO_V2_COMPAT_H_ */
