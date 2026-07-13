/****************************************************************************
 * dshot_4way / dshot_4way_ops.c
 *
 * High-level operations on top of the ported BLHeli bootloader layer:
 *   - dshot_4way_dump():  READ-ONLY. Connect to each ESC's bootloader and
 *     hexdump its BLHeli_S settings block. Used to validate the bit-bang
 *     stack on real hardware and to locate the Beacon Delay byte before any
 *     write is attempted.
 *
 * REMOVE PROPS. Run "dshot stop" first so the timer releases the pins.
 ****************************************************************************/

#include "serial_4way.h"
#include "serial_4way_impl.h"
#include "serial_4way_avrootloader.h"

#include <stdio.h>
#include <unistd.h>

/* BLHeli_S (SiLabs) settings live in flash at 0x1A00; 112-byte layout. */
#define BLHELI_S_EEPROM_ADDR_H 0x1A
#define BLHELI_S_EEPROM_ADDR_L 0x00
#define BLHELI_S_SETTINGS_LEN  0x70   /* 112 bytes */

extern uint8_t selected_esc;

/* Global from serial_4way.c: isMcuConnected() returns (DeviceInfo.bytes[0]>0),
 * which BL_ReadBuf uses to decide the SiLabs framing (data + CRC + ACK). We
 * bypass the MSP Connect() path, so we must set it ourselves after connecting
 * or the settings read decodes the CRC bytes as data and fails. */
extern uint8_32_u DeviceInfo;

/* soft-UART diagnostics from serial_4way_avrootloader.c */
extern volatile uint32_t g_suart_startbits;
extern volatile uint32_t g_suart_timeouts;
extern volatile uint32_t g_suart_framing_err;
extern volatile uint8_t  g_suart_rx[32];
extern volatile uint32_t g_suart_rx_n;

/* Connect to the selected ESC's bootloader; returns 1 on success and fills
 * *devinfo with the 4-byte device signature/info. Uses BL_ConnectBootloader
 * which first holds the line idle-high so the running app hands off. */
static int connect_selected(uint8_32_u *devinfo)
{
	devinfo->dword = 0;
	return BL_ConnectBootloader(devinfo) ? 1 : 0;
}

/* SiLabs EFM8BB2x flash page containing the BLHeli_S/Bluejay settings. */
#define BLHELI_S_PAGE_ADDR  0x1A00
#define BLHELI_S_PAGE_LEN   512
#define BEACON_DELAY_OFFSET 0x1D   /* 1=1m 2=2m 3=5m 4=10m 5=Infinite */

/* Read n bytes from flash address addr into dst using the connected bootloader,
 * in small IRQ-protected chunks (clean back-to-back bytes, short IRQ-off). */
static int bl_read_chunked(uint16_t addr, uint8_t *dst, int n)
{
	const int CHUNK = 8;

	for (int off = 0; off < n; off += CHUNK) {
		int c = (n - off < CHUNK) ? (n - off) : CHUNK;
		uint16_t a = addr + off;

		ioMem_t mem;
		mem.D_FLASH_ADDR_H = (uint8_t)(a >> 8);
		mem.D_FLASH_ADDR_L = (uint8_t)(a & 0xFF);
		mem.D_NUM_BYTES    = (uint8_t)c;
		mem.D_PTR_I        = dst + off;

		irqstate_t flags = enter_critical_section();
		uint8_t ok = BL_ReadFlash(imSIL_BLB, &mem);
		leave_critical_section(flags);

		if (!ok) {
			return 0;
		}
	}

	return 1;
}

/* Write n bytes to flash address addr from src in IRQ-protected chunks, each
 * chunk retried a few times. The page MUST be erased first. */
static int bl_write_chunked(uint16_t addr, uint8_t *src, int n)
{
	const int CHUNK = 16;

	for (int off = 0; off < n; off += CHUNK) {
		int c = (n - off < CHUNK) ? (n - off) : CHUNK;
		uint16_t a = addr + off;

		int wrote = 0;

		for (int attempt = 0; attempt < 3 && !wrote; attempt++) {
			ioMem_t mem;
			mem.D_FLASH_ADDR_H = (uint8_t)(a >> 8);
			mem.D_FLASH_ADDR_L = (uint8_t)(a & 0xFF);
			mem.D_NUM_BYTES    = (uint8_t)c;
			mem.D_PTR_I        = src + off;

			irqstate_t flags = enter_critical_section();
			uint8_t ok = BL_WriteFlash(&mem);
			leave_critical_section(flags);

			wrote = ok ? 1 : 0;
		}

		if (!wrote) {
			printf("  write FAILED at 0x%04X\n", (unsigned)a);
			fflush(stdout);
			return 0;
		}
	}

	return 1;
}

int dshot_4way_set_beacon(int esc_index, int value);

int dshot_4way_set_beacon(int esc_index, int value)
{
	if (value < 1 || value > 5) {
		printf("dshot_4way: beacon value must be 1..5 (5=Infinite)\n");
		return 1;
	}

	uint8_t detected = esc4wayInit();

	if (detected == 0) {
		printf("dshot_4way: no DShot channels (run 'dshot stop' first?)\n");
		return 1;
	}

	if (esc_index < 0 || esc_index >= detected) {
		printf("dshot_4way: esc index %d out of range (0..%d)\n", esc_index, detected - 1);
		esc4wayRelease();
		return 1;
	}

	selected_esc = (uint8_t)esc_index;
	DeviceInfo.dword = 0;

	uint8_32_u devinfo;

	if (!connect_selected(&devinfo)) {
		printf("ESC %d: no bootloader response (retry, or wrong channel)\n", esc_index);
		esc4wayRelease();
		return 1;
	}

	DeviceInfo = devinfo;
	printf("ESC %d: connected (sig %02X%02X)\n", esc_index, devinfo.bytes[1], devinfo.bytes[0]);
	fflush(stdout);

	static uint8_t page[BLHELI_S_PAGE_LEN];
	memset(page, 0xFF, sizeof(page));

	if (!bl_read_chunked(BLHELI_S_PAGE_ADDR, page, BLHELI_S_PAGE_LEN)) {
		printf("ESC %d: settings read FAILED, aborting (no change made)\n", esc_index);
		esc4wayRelease();
		return 1;
	}

	/* Sanity: the BLHeli_S "initialized" marker must be present, else we would
	 * be writing into a page we don't understand. */
	if (page[0x0D] != 0x55 || page[0x0E] != 0xAA) {
		printf("ESC %d: settings marker 55 AA not found (got %02X %02X), aborting\n",
		       esc_index, page[0x0D], page[0x0E]);
		esc4wayRelease();
		return 1;
	}

	printf("ESC %d: Beacon Delay currently 0x%02X -> setting 0x%02X\n",
	       esc_index, page[BEACON_DELAY_OFFSET], (unsigned)value);
	fflush(stdout);

	if (page[BEACON_DELAY_OFFSET] == (uint8_t)value) {
		printf("ESC %d: already set, no write needed\n", esc_index);
		esc4wayRelease();
		return 0;
	}

	page[BEACON_DELAY_OFFSET] = (uint8_t)value;

	/* Erase the 512-byte settings page, then write the whole page back. */
	ioMem_t emem;
	emem.D_FLASH_ADDR_H = (uint8_t)(BLHELI_S_PAGE_ADDR >> 8);
	emem.D_FLASH_ADDR_L = (uint8_t)(BLHELI_S_PAGE_ADDR & 0xFF);
	emem.D_NUM_BYTES    = 0;
	emem.D_PTR_I        = page;

	irqstate_t flags = enter_critical_section();
	uint8_t erased = BL_PageErase(&emem);
	leave_critical_section(flags);

	if (!erased) {
		printf("ESC %d: page erase FAILED (settings may be default on reboot)\n", esc_index);
		esc4wayRelease();
		return 1;
	}

	if (!bl_write_chunked(BLHELI_S_PAGE_ADDR, page, BLHELI_S_PAGE_LEN)) {
		printf("ESC %d: WRITE FAILED (settings may be default on reboot)\n", esc_index);
		esc4wayRelease();
		return 1;
	}

	/* Verify */
	static uint8_t verify[BLHELI_S_PAGE_LEN];
	memset(verify, 0x00, sizeof(verify));

	if (!bl_read_chunked(BLHELI_S_PAGE_ADDR, verify, BLHELI_S_PAGE_LEN)) {
		printf("ESC %d: verify read FAILED\n", esc_index);
		esc4wayRelease();
		return 1;
	}

	if (verify[BEACON_DELAY_OFFSET] != (uint8_t)value) {
		printf("ESC %d: VERIFY MISMATCH: Beacon Delay = 0x%02X (expected 0x%02X)\n",
		       esc_index, verify[BEACON_DELAY_OFFSET], (unsigned)value);
		esc4wayRelease();
		return 1;
	}

	printf("ESC %d: OK, Beacon Delay = 0x%02X (5=Infinite) written & verified\n",
	       esc_index, verify[BEACON_DELAY_OFFSET]);
	fflush(stdout);

	esc4wayRelease();
	return 0;
}

int dshot_4way_dump(int num_esc);

int dshot_4way_probe(int num_esc);

int dshot_4way_probe(int num_esc)
{
	uint8_t detected = esc4wayInit();

	if (detected == 0) {
		printf("dshot_4way: no DShot channels (run 'dshot stop' first?)\n");
		return 1;
	}

	if (num_esc <= 0 || num_esc > detected) {
		num_esc = detected;
	}

	printf("dshot_4way probe: %d channel(s), listening 30ms/ESC after wake string\n",
	       (int)detected);

	for (int i = 0; i < num_esc; i++) {
		selected_esc = (uint8_t)i;

		uint32_t pinset = io_timer_channel_get_gpio_output(i);
		uint32_t pin = pinset & (GPIO_PORT_MASK | GPIO_PIN_MASK);

		/* idle the line high (input + pull-up) before the wake string */
		px4_arch_configgpio(pin | GPIO_INPUT | GPIO_PULLUP);
		usleep(1000);

		uint32_t first_low_us = 0;
		uint32_t samples = 0;
		uint32_t lows = BL_DiagProbe(30000, &first_low_us, &samples);

		printf("ESC %d: pin=0x%08lX  lows=%lu/%lu  first_low=%luus  => %s\n",
		       i, (unsigned long)pinset,
		       (unsigned long)lows, (unsigned long)samples,
		       (unsigned long)first_low_us,
		       lows ? "ESC RESPONDS (bootloader active)" : "silent (no bootloader entry)");
	}

	esc4wayRelease();
	printf("dshot_4way: probe done. Run 'dshot start' to restore motors.\n");
	return 0;
}

int dshot_4way_dump(int num_esc)
{
	uint8_t detected = esc4wayInit();

	if (detected == 0) {
		printf("dshot_4way: no DShot channels (run 'dshot stop' first?)\n");
		return 1;
	}

	if (num_esc <= 0 || num_esc > detected) {
		num_esc = (num_esc <= 0) ? 4 : detected;

		if (num_esc > detected) {
			num_esc = detected;
		}
	}

	printf("dshot_4way: %d channel(s) available, dumping first %d ESC(s)\n",
	       (int)detected, num_esc);
	fflush(stdout);

	for (int i = 0; i < num_esc; i++) {
		selected_esc = (uint8_t)i;

		printf("ESC %d: entering bootloader (hold-high + wake, up to ~1.3s)...\n", i);
		fflush(stdout);

		g_suart_startbits = 0;
		g_suart_timeouts = 0;
		g_suart_framing_err = 0;
		g_suart_rx_n = 0;

		/* Connect handshake needs isMcuConnected()==false (reads 8+ACK, no CRC);
		 * we only set DeviceInfo true *after* connecting, for the flash read. */
		DeviceInfo.dword = 0;

		uint8_32_u devinfo;
		int ok = connect_selected(&devinfo);

		printf("ESC %d: startbits=%lu timeouts=%lu framing_err=%lu\n", i,
		       (unsigned long)g_suart_startbits, (unsigned long)g_suart_timeouts,
		       (unsigned long)g_suart_framing_err);

		printf("ESC %d: first RX bytes:", i);

		for (uint32_t k = 0; k < g_suart_rx_n && k < 32; k++) {
			printf(" %02X", g_suart_rx[k]);
		}

		printf("\n");
		fflush(stdout);

		if (!ok) {
			printf("ESC %d: no bootloader response (no ESC on this channel?)\n", i);
			fflush(stdout);
			continue;
		}

		printf("ESC %d: connected, signature %02X %02X %02X %02X\n",
		       i, devinfo.bytes[0], devinfo.bytes[1],
		       devinfo.bytes[2], devinfo.bytes[3]);
		fflush(stdout);

		/* Mark connected so BL_ReadBuf reads the SiLabs CRC+ACK framing. */
		DeviceInfo = devinfo;

		uint8_t buf[BLHELI_S_SETTINGS_LEN];
		memset(buf, 0, sizeof(buf));

		/* Read the 112-byte settings block in small chunks. Each chunk is one
		 * SET_ADDRESS + READ held with IRQs off (clean back-to-back bytes); the
		 * chunking keeps each IRQ-off window short (~17ms for 32 bytes). */
		int read_ok = 1;
		const int CHUNK = 8;

		for (int off = 0; off < BLHELI_S_SETTINGS_LEN; off += CHUNK) {
			int n = BLHELI_S_SETTINGS_LEN - off;

			if (n > CHUNK) {
				n = CHUNK;
			}

			uint16_t addr = (uint16_t)((BLHELI_S_EEPROM_ADDR_H << 8) | BLHELI_S_EEPROM_ADDR_L) + off;

			ioMem_t mem;
			mem.D_FLASH_ADDR_H = (uint8_t)(addr >> 8);
			mem.D_FLASH_ADDR_L = (uint8_t)(addr & 0xFF);
			mem.D_NUM_BYTES    = (uint8_t)n;
			mem.D_PTR_I        = buf + off;

			irqstate_t flags = enter_critical_section();
			uint8_t chunk_ok = BL_ReadFlash(imSIL_BLB, &mem);
			leave_critical_section(flags);

			if (!chunk_ok) {
				printf("ESC %d: settings read FAILED at offset 0x%02X\n", i, off);
				fflush(stdout);
				read_ok = 0;
				break;
			}
		}

		if (!read_ok) {
			continue;
		}

		for (int row = 0; row < BLHELI_S_SETTINGS_LEN; row += 16) {
			printf("  %02X:", row);

			for (int col = 0; col < 16 && (row + col) < BLHELI_S_SETTINGS_LEN; col++) {
				printf(" %02X", buf[row + col]);
			}

			printf("\n");
		}

		fflush(stdout);
	}

	esc4wayRelease();
	printf("dshot_4way: dump done. Run 'dshot start' to restore motors.\n");
	fflush(stdout);
	return 0;
}
