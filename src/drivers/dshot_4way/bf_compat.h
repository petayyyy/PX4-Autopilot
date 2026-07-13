/****************************************************************************
 * dshot_4way / bf_compat.h
 *
 * Single compatibility header that replaces the Betaflight platform layer
 * (platform.h, drivers, io/beeper.h, flight/mixer.h) for the ported
 * serial_4way.c / serial_4way_avrootloader.c files.
 *
 * Everything Betaflight expected from its HAL is provided here, mapped onto
 * PX4/NuttX (STM32H7). The actual implementations live in
 * px4_serial_4way_io.c.
 ****************************************************************************/
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <px4_arch/micro_hal.h>   /* px4_arch_configgpio / gpioread / gpiowrite */
#include <px4_arch/io_timer.h>    /* io_timer_channel_get_gpio_output */
#include <nuttx/irq.h>            /* enter_critical_section for suart timing */

__BEGIN_DECLS

/* ---- Feature selection (compile the BLHeli/SiLabs bootloader path only) ---- */
#define USE_SERIAL_4WAY_BLHELI_INTERFACE
#define USE_SERIAL_4WAY_BLHELI_BOOTLOADER
/* NOT defined: USE_SERIAL_4WAY_SK_BOOTLOADER, USE_VIRTUAL_ESC */

#define MAX_SUPPORTED_MOTORS 8

/* ---- LED / beeper: no-ops on PX4 (optional: wire to a board LED later) ---- */
#define LED0_ON      do {} while (0)
#define LED0_OFF     do {} while (0)
/* LED1 intentionally left undefined -> TX uses LED0 macros in serial_4way.c */
#define beeperSilence() do {} while (0)

/* ---- Time (implemented over hrt in px4_serial_4way_io.c) ---- */
typedef uint32_t timeUs_t;
typedef uint32_t timeMs_t;
typedef int32_t  timeDelta_t;

uint32_t micros(void);
uint32_t millis(void);
#define cmpTimeUs(a, b) ((timeDelta_t)((a) - (b)))

/* ---- Serial port abstraction over a file descriptor (USB CDC) ---- */
typedef struct serialPort_s {
	int fd;
} serialPort_t;

uint8_t  serialRead(serialPort_t *port);
void     serialWrite(serialPort_t *port, uint8_t ch);
uint32_t serialRxBytesWaiting(const serialPort_t *port);
uint32_t serialTxBytesFree(const serialPort_t *port);

/* buffered-write bracketing: no-ops for our unbuffered fd-backed port */
#define serialBeginWrite(port) do { (void)(port); } while (0)
#define serialEndWrite(port)   do { (void)(port); } while (0)

/* ---- Motor hooks (implemented in px4_serial_4way_io.c) ----
 * esc4wayInit()/Release() in serial_4way.c drive these to stop/restore DShot
 * and to obtain the per-ESC signal pin.
 */
typedef uint32_t IO_t;              /* we store the DShot channel's GPIO pinset */
#define IO_NONE ((IO_t)0)

void     motorDisable(void);
void     motorEnable(void);
bool     motorIsMotorEnabled(uint8_t index);
IO_t     motorGetIo(uint8_t index);

/* ---- GPIO primitives mapped onto px4_arch (STM32H7) ----
 * IO_t carries a full pinset; we mask to port+pin identity for read/write and
 * OR in the requested mode for (re)configuration.
 */
#define IO_PINONLY(io)          ((io) & (GPIO_PORT_MASK | GPIO_PIN_MASK))
#define GPIO_PIN_RESET          false

#define IORead(io)              px4_arch_gpioread(IO_PINONLY(io))
#define IOHi(io)                px4_arch_gpiowrite(IO_PINONLY(io), true)
#define IOLo(io)                px4_arch_gpiowrite(IO_PINONLY(io), false)
#define IOConfigGPIO(io, cfg)   px4_arch_configgpio(IO_PINONLY(io) | (cfg))

/* one-wire half-duplex: input = pull-up, output = push-pull (idle high) */
#define IOCFG_IPU     (GPIO_INPUT  | GPIO_PULLUP)
#define IOCFG_OUT_PP  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET)
/* AF restore is handled by re-running dshot init on exit; keep pin benign here */
#define IOCFG_AF_PP   (GPIO_INPUT  | GPIO_PULLUP)

__END_DECLS
