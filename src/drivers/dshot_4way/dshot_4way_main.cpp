/****************************************************************************
 * dshot_4way / dshot_4way_main.cpp
 *
 * nsh command that exposes the BLHeli/SiLabs bootloader on the DShot output
 * pins over a serial port, so esc-configurator / BLHeliSuite can read & write
 * ESC settings (e.g. Beacon Delay = Infinite).
 *
 * Usage:
 *   dshot stop                     # release the DShot timer/pins first
 *   dshot_4way start [device]      # default device: /dev/ttyACM0
 *   ... connect esc-configurator to <device>, do the work, then in its UI
 *       send "Exit interface" (esc4way exit) ...
 *   dshot start                    # restore normal DShot output
 *
 * Props MUST be removed. See PORTING.md.
 ****************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>

#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/* C linkage to the ported Betaflight 4way interface (serial_4way.c). */
extern "C" {
	typedef struct serialPort_s {
		int fd;
	} serialPort_t;

	uint8_t esc4wayInit(void);
	void    esc4wayProcess(serialPort_t *mspPort);
	void    esc4wayRelease(void);
	int     dshot_4way_dump(int num_esc);
	int     dshot_4way_probe(int num_esc);
	int     dshot_4way_set_beacon(int esc_index, int value);
}

/* Safety guard: every subcommand bit-bangs the ESC signal lines directly with
 * interrupts disabled for milliseconds at a time. That is only ever safe on the
 * bench with motors stopped ("dshot stop") and props removed. Running it while
 * the vehicle is armed would fight the control loop and drive the motor pins by
 * hand -> refuse. actuator_armed is published continuously by commander; if it
 * has never been advertised the vehicle cannot be armed, so copy()==false (which
 * leaves armed=false) is the correct fail-safe result. */
static bool vehicle_is_armed(void)
{
	uORB::Subscription armed_sub{ORB_ID(actuator_armed)};
	actuator_armed_s armed{};
	armed_sub.copy(&armed);
	return armed.armed || armed.prearmed;
}

static void usage(void)
{
	PX4_INFO("usage:");
	PX4_INFO("  dshot_4way probe [n]           read-only: does the ESC bootloader answer?");
	PX4_INFO("  dshot_4way dump [n]            read-only: dump ESC settings (default n=4)");
	PX4_INFO("  dshot_4way beacon <esc> <v>    write Beacon Delay (1=1m..4=10m,5=Infinite)");
	PX4_INFO("  dshot_4way start [device]      4way passthrough (default /dev/ttyACM0)");
	PX4_INFO("  run 'dshot stop' first, and 'dshot start' after. REMOVE PROPS.");
}

extern "C" __EXPORT int dshot_4way_main(int argc, char *argv[])
{
	/* Refuse any hardware-touching subcommand while armed (see vehicle_is_armed). */
	if (argc >= 2 && (strcmp(argv[1], "dump") == 0 || strcmp(argv[1], "probe") == 0
			  || strcmp(argv[1], "beacon") == 0 || strcmp(argv[1], "start") == 0)) {
		if (vehicle_is_armed()) {
			PX4_ERR("refusing '%s': vehicle is ARMED. Disarm, run 'dshot stop', REMOVE PROPS.", argv[1]);
			return 1;
		}
	}

	if (argc >= 2 && strcmp(argv[1], "dump") == 0) {
		int n = (argc >= 3) ? atoi(argv[2]) : 4;
		return dshot_4way_dump(n);
	}

	if (argc >= 2 && strcmp(argv[1], "probe") == 0) {
		int n = (argc >= 3) ? atoi(argv[2]) : 4;
		return dshot_4way_probe(n);
	}

	if (argc >= 2 && strcmp(argv[1], "beacon") == 0) {
		if (argc < 4) {
			PX4_ERR("usage: dshot_4way beacon <esc_index> <value 1..5>");
			return 1;
		}

		return dshot_4way_set_beacon(atoi(argv[2]), atoi(argv[3]));
	}

	if (argc < 2 || strcmp(argv[1], "start") != 0) {
		usage();
		return 1;
	}

	const char *device = (argc >= 3) ? argv[2] : "/dev/ttyACM0";

	int fd = open(device, O_RDWR | O_NONBLOCK | O_NOCTTY);

	if (fd < 0) {
		PX4_ERR("failed to open %s", device);
		return 1;
	}

	uint8_t esc_count = esc4wayInit();

	if (esc_count == 0) {
		PX4_ERR("no DShot channels found (is 'dshot' stopped and configured?)");
		close(fd);
		return 1;
	}

	PX4_INFO("4way passthrough on %s, %u ESC(s). Connect esc-configurator now.",
		 device, (unsigned)esc_count);
	PX4_INFO("REMOVE PROPS. This blocks until the tool sends 'exit interface'.");

	serialPort_t port;
	port.fd = fd;

	/* Runs the full 4way session; returns when the host sends cmd_InterfaceExit
	 * (esc4wayRelease() is invoked internally on exit). */
	esc4wayProcess(&port);

	esc4wayRelease();
	close(fd);

	PX4_INFO("4way passthrough ended. Run 'dshot start' to restore motors.");
	return 0;
}
