#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

// control Crescent GPS and parse binary packet to print signal strength info


// cf 7.1.10  Crescent Integrators Manual

#define STATUS99_CODE_LOCK	0x01
#define STATUS99_BIT_LOCK	0x02
#define STATUS99_FRAME_LOCK	0x04
#define STATUS99_FRAME_SYNC	0x08
#define STATUS99_FAME_SYNC_NEW_EPOCH	0x10
#define STATUS99_CHANNEL_RESET	0x20
#define STATUS99_PHASE_LOCK	0x40

struct chan99 {
	uint8_t		channel;
	uint8_t		sat;			// 0 not tracked
	uint8_t		status;
	uint8_t		last_sub_frame;
	uint8_t		ephem_valid;
	uint8_t		ephem_health;
	uint8_t		almanac_valid;
	uint8_t		almanac_health;
	int8_t		elevation;
	uint8_t		half_azimuth;	// 0-180 for 0-360 degrees
	uint8_t		user_range_error;
	uint8_t		spare1;			// unused
	uint16_t	cli_no;			// SNR = 10 * 4096 * cli_no / 80000  (noise floor)
	int16_t		diff_corr;		// 100 * differential correction
	int16_t		pos_residual;	// 10 * position residual
	int16_t		vel_residual;	// 10 * velocity residual
	int16_t		doppler_hz;		// expected doppler
	int16_t		carrier_track_offset;
};

struct msg99 {
	uint8_t		nav_mode;		// 0 invalid, 1 no fix, 2 2D fix, 3 3D fix
	uint8_t		time_diff;		// seconds between GPS and UTC
	uint16_t	gps_week;
	double		time_of_week;	// 8 bytes  (but at a 4 byte boundary, so packed)
	struct chan99 c[12];
	int16_t		clock_err_L1;
	int16_t		spare1;
} __attribute__((packed));

// cf 7.1  Crescent Integrators Manual

struct msg {
	char 		magic[4];		// "$BIN"
	uint16_t	type;			//  1,  2, 80, 93-99
	uint16_t	len;			// 52, 16, 40, 56, 96, 128, 300, 28, 68, 304 
	union {
		char raw[304];
		struct msg99 m99;
		// define more of them if we ever need any
	};
};

// C hack to confirm sizes and alignments of structures at compile time
char *nc99 = sizeof(struct chan99) - 24;
char *nm99 = sizeof(struct msg99) - 304;
char *nm   = sizeof(struct msg) - 312;

int handle_msg99(struct msg99 *m)
{
	// we care about overall fix, and each satellite's SNR
	int i, sats = 0;
	switch (m->nav_mode & 0x7)
	{
	case 0:		printf("==> invalid"); break;
	case 1:		printf("==> no fix"); break;
	case 2:		printf("==> 2D fix"); break;
	case 3:		printf("==> 3D fix"); break;
	default:	printf(" bad nav mode: %d", m->nav_mode);
	}
	for (i=0; i<12; i++)
		if ((m->c[i].status & STATUS99_CHANNEL_RESET) == 0)
			sats++;
	printf("  %d satellites visible:\n", sats);

	struct chan99 *c;
	for (c = m->c; c < m->c+12; c++) {
		if (c->status & STATUS99_CHANNEL_RESET) continue;
		printf("  sat%02u (%02x): SNR %.2lf\n", c->sat, c->status,
			40960.0 * c->cli_no / 80000.0);
	}
	return 0;
}

int handle_packet(struct msg *m)
{
	switch (m->type)
	{
	case 99:
		if (m->len == 304)
			return handle_msg99(&m->m99);
		fprintf(stderr, "bad length %d != 304\n", m->len);
		return -3;
	default:
		fprintf(stderr, "unhandled packet type: %d length %d\n", m->type, m->len);
		return -4;
	}
}

uint16_t sum(uint8_t *packet, int len)
{
	uint16_t s = 0;
	while (len--)
		s += *packet++;
	return s;
}

int get_packet(FILE *gps, struct msg *m)
{
	static char buf[8];
	char *p;
	uint16_t checksum;

	// sync stream to "$BIN"
	do {
		fread(buf, 1, 8, gps);
		p = memchr(buf, '$', 8);
		if (p == NULL)
			continue;
		if (p > buf) {
			memmove(buf, p, 8-(p-buf));
			fread(buf + 8-(p-buf), 1, p-buf, gps);
		}
	} while (memcmp(buf, "$BIN", 4) != 0);
	memcpy(m, buf, 8);

	if (m->type > 99 || m->len > 304) {
		fprintf(stderr, "bad header\n");
		return -1;
	}

	fread(&m->raw, 1, m->len, gps);
	// check whether packet was truncated
	if ((p = strnstr(m->raw, "$BIN", m->len)) != NULL) {
		// TODO: resync on found header
		fprintf(stderr, "packet truncated: got %ld wanted %d\n", p-m->raw, m->len);
		return -2;
	}
	fread(&checksum, 2, 1, gps);
	fread(buf, 1, 2, gps);			// \r\n

	if (sum(m->raw, m->len) != checksum) {
		fprintf(stderr, "bad checksum %u (expected %u)\n", sum(m->raw, m->len), checksum);
		return -3;
	}
	return 0;
}

int main(int argc, char *argv[])
{
	if (argc < 2) {
		fprintf(stderr, "usage: %s gps-device\n", argv[0]);
		exit(1);
	}

	FILE *gps = fopen(argv[1], "r");
	if (gps == NULL) {
		fprintf(stderr, "can't open %s\n", argv[1]);
		exit(1);
	}
#if 0
	// turn on binary packet 99
	char buf[16];
	fprintf(gps, "$JBIN,99,1\r\n");	fflush(gps);
	fread(buf, 1, 2, gps);
	if (memcmp(buf, "$>", 2) != 0) {
		fprintf(stderr, "bad response from device, starting with '%c%c'\n", buf[0], buf[1]);
		exit(2);
	}
#endif

	// suck packets forever
	while (1) {
		struct msg m;

		if (get_packet(gps, &m) < 0)
			continue;

		handle_packet(&m);
	}
	return 0;
}
