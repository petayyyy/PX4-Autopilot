/****************************************************************************
 * dshot_4way / px4_serial_4way_io.c
 *
 * PX4/NuttX implementations of the platform hooks declared in bf_compat.h:
 *   - micros() / millis()        via hrt
 *   - serial* glue               over a file descriptor (USB CDC)
 *   - motor* hooks               map ESC index -> DShot channel GPIO pinset
 *
 * The DShot PX4 driver must be stopped ("dshot stop") before passthrough so
 * the timer/DMA releases the output pins; it is restarted ("dshot start")
 * afterwards. motorDisable()/motorEnable() are therefore no-ops here.
 ****************************************************************************/

#include "bf_compat.h"

#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#include <drivers/drv_hrt.h>

/* ---- time -------------------------------------------------------------- */

uint32_t micros(void)
{
	return (uint32_t)hrt_absolute_time();
}

uint32_t millis(void)
{
	return (uint32_t)(hrt_absolute_time() / 1000);
}

/* ---- serial over fd (USB CDC), 1-byte read look-ahead ------------------ *
 * The 4way protocol polls serialRxBytesWaiting() before every serialRead(),
 * so we implement "bytes waiting" as a single-byte non-blocking peek. The fd
 * is opened O_NONBLOCK by the caller.
 */

static bool    s_have_byte = false;
static uint8_t s_peek_byte = 0;

uint32_t serialRxBytesWaiting(const serialPort_t *port)
{
	if (s_have_byte) {
		return 1;
	}

	uint8_t b;
	int r = read(port->fd, &b, 1);

	if (r == 1) {
		s_peek_byte = b;
		s_have_byte = true;
		return 1;
	}

	return 0;
}

uint8_t serialRead(serialPort_t *port)
{
	if (s_have_byte) {
		s_have_byte = false;
		return s_peek_byte;
	}

	/* Only reached if a caller skipped the RxBytesWaiting() poll. */
	uint8_t b = 0;

	for (;;) {
		int r = read(port->fd, &b, 1);

		if (r == 1) {
			break;
		}

		if (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
			break;
		}
	}

	return b;
}

void serialWrite(serialPort_t *port, uint8_t ch)
{
	for (;;) {
		int r = write(port->fd, &ch, 1);

		if (r == 1) {
			break;
		}

		if (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
			break;
		}
	}
}

uint32_t serialTxBytesFree(const serialPort_t *port)
{
	(void)port;
	return 1; /* CDC write() blocks/retries as needed; always claim space */
}

/* ---- motor / ESC pin mapping ------------------------------------------ *
 * ESC index maps directly to DShot io_timer channel index. A channel exists
 * (is "motor enabled") when the board provides a GPIO output config for it.
 */

bool motorIsMotorEnabled(uint8_t index)
{
	if (index >= MAX_SUPPORTED_MOTORS) {
		return false;
	}

	return io_timer_channel_get_gpio_output(index) != 0;
}

IO_t motorGetIo(uint8_t index)
{
	return (IO_t)io_timer_channel_get_gpio_output(index);
}

void motorDisable(void)
{
	/* DShot driver is stopped externally ("dshot stop"); nothing to do. */
}

void motorEnable(void)
{
	/* DShot driver is restarted externally ("dshot start"); nothing to do. */
}
