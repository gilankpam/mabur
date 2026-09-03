/* vencprobe — how long does a bitrate command take to reach the wire?
 *
 * Answers the encoder half of that question on hardware: it steps maburd's
 * encoder bitrate through the debug endpoint (POST /venc/set?bitrate=N,
 * which calls venc_set_bitrate_kbps() synchronously on the HTTP thread) and
 * watches every frame the encoder publishes, timestamping both on ONE
 * CLOCK_MONOTONIC. The onset lag is then a subtraction, with no clock
 * domain to bridge and no GS in the loop.
 *
 * The frame side is a PASSIVE observer of the venc frame ring: it mmaps
 * /dev/shm/<shm> PROT_READ and polls hdr->write_idx, reading only each
 * committed slot's length prefix and its 8-byte VencFrameMeta. It never
 * writes read_idx, so maburd keeps consuming exactly as it would with the
 * probe absent — safe to run against a live daemon. A slot is only reused
 * slot_count frames later (8 frames = ~133 ms at 60 fps), far longer than
 * the poll period, so what it reads is what was committed.
 *
 * Build (OpenIPC glibc toolchain, no cmake needed):
 *   ../openipc-builder/openipc/output/host/bin/arm-openipc-linux-gnueabihf-gcc \
 *       -O2 -Idrone/vendor -o out/arm/vencprobe tools/bench/vencprobe.c
 *
 * Run on the drone (stdout is CSV; see tools/bench/vencprobe_analyze.py):
 *   vencprobe --low 3000 --high 10000 --dwell 2500 --cycles 6 > /tmp/vp.csv
 *
 * PASSIVE mode (2026-09-03, venc-overshoot bench): --cycles 0 --duration MS
 * never POSTs; it only logs frames and the 25 ms /venc polls (req bitrate +
 * encoder qp) for MS milliseconds. Stimulus is external (cover/uncover the
 * lens, pan) and tools/bench/vencburst_analyze.py finds the cycles:
 *   vencprobe --cycles 0 --duration 120000 > /tmp/vb.csv
 *
 * NOTE RcAgent re-asserts its own computed bitrate every 5 s
 * (kReassertMs, rc_agent.h), so an override here is bounded and self-heals
 * — that is also why dwell should stay under 5 s and why cycles are
 * repeated: a re-assert landing inside a dwell truncates it, and the
 * analyzer drops steps whose window is not clean.
 */
#define _GNU_SOURCE
#include <arpa/inet.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "venc_frame_ring.h"

static uint64_t mono_us(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)ts.tv_nsec / 1000ull;
}

/* Same arithmetic as venc_frame_ring.c's calc_slot_stride. */
static uint32_t slot_stride(uint32_t slot_data_size)
{
	uint32_t raw = (uint32_t)sizeof(uint32_t) + slot_data_size;
	return (raw + 7u) & ~7u;
}

/* GET /venc, returning req_bitrate_kbps (venc_core's g_cur_bitrate_kbps:
 * the last value ANY writer applied) or -1. Polled a few times per second so
 * the analyzer can see RcAgent's 5 s re-assert land on top of an override
 * and truncate that window instead of reporting the collision as encoder
 * behaviour. Reads atomics only — no verb lock, so it cannot perturb the
 * thing being measured. */
static int get_req_bitrate(int port, int *qp)
{
	struct sockaddr_in sa;
	const char *req = "GET /venc HTTP/1.1\r\nHost: 127.0.0.1\r\n"
			  "Connection: close\r\n\r\n";
	char resp[512], *p;
	int fd, n, val = -1;

	if (qp)
		*qp = -1;

	fd = socket(AF_INET, SOCK_STREAM, 0);
	if (fd < 0)
		return -1;
	memset(&sa, 0, sizeof(sa));
	sa.sin_family = AF_INET;
	sa.sin_port = htons((uint16_t)port);
	sa.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	if (connect(fd, (struct sockaddr *)&sa, sizeof(sa)) != 0) {
		close(fd);
		return -1;
	}
	if (write(fd, req, strlen(req)) > 0) {
		n = (int)read(fd, resp, sizeof(resp) - 1);
		if (n > 0) {
			resp[n] = 0;
			p = strstr(resp, "\"req_bitrate_kbps\":");
			if (p)
				val = atoi(p + strlen("\"req_bitrate_kbps\":"));
			/* Encoder QP of the last published frame (VencStats
			 * last_qp, 2026-09-03). The only per-frame-ish QP
			 * readback without touching the ring meta (which is
			 * the FrameHdr wire format, kFrameHdrLen == 8). */
			p = strstr(resp, "\"qp\":");
			if (p && qp)
				*qp = atoi(p + strlen("\"qp\":"));
		}
	}
	close(fd);
	return val;
}

/* POST /venc/set?bitrate=<kbps> to 127.0.0.1:<port>. Returns 0 if the
 * daemon answered 200, -1 otherwise. Blocking and short: the whole
 * exchange is one loopback round trip plus the MI_VENC_SetChnAttr the
 * handler makes before it replies. */
static int post_bitrate(int port, int kbps, uint64_t *t_before, uint64_t *t_after)
{
	struct sockaddr_in sa;
	char req[256], resp[256];
	int fd, n, rc = -1;

	fd = socket(AF_INET, SOCK_STREAM, 0);
	if (fd < 0)
		return -1;
	memset(&sa, 0, sizeof(sa));
	sa.sin_family = AF_INET;
	sa.sin_port = htons((uint16_t)port);
	sa.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	if (connect(fd, (struct sockaddr *)&sa, sizeof(sa)) != 0) {
		close(fd);
		return -1;
	}
	n = snprintf(req, sizeof(req),
		"POST /venc/set?bitrate=%d HTTP/1.1\r\nHost: 127.0.0.1\r\n"
		"Content-Length: 0\r\nConnection: close\r\n\r\n", kbps);

	*t_before = mono_us();
	if (write(fd, req, (size_t)n) == n) {
		n = (int)read(fd, resp, sizeof(resp) - 1);
		*t_after = mono_us();
		if (n > 12) {
			resp[n] = 0;
			if (strstr(resp, " 200 "))
				rc = 0;
		}
	} else {
		*t_after = mono_us();
	}
	close(fd);
	return rc;
}

int main(int argc, char **argv)
{
	const char *shm = "mabur_f";
	int port = 8301, low = 3000, high = 10000, dwell_ms = 2500, cycles = 6;
	int settle_ms = 1500, duration_ms = 0;

	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "--shm") && i + 1 < argc) shm = argv[++i];
		else if (!strcmp(argv[i], "--port") && i + 1 < argc) port = atoi(argv[++i]);
		else if (!strcmp(argv[i], "--low") && i + 1 < argc) low = atoi(argv[++i]);
		else if (!strcmp(argv[i], "--high") && i + 1 < argc) high = atoi(argv[++i]);
		else if (!strcmp(argv[i], "--dwell") && i + 1 < argc) dwell_ms = atoi(argv[++i]);
		else if (!strcmp(argv[i], "--cycles") && i + 1 < argc) cycles = atoi(argv[++i]);
		else if (!strcmp(argv[i], "--settle") && i + 1 < argc) settle_ms = atoi(argv[++i]);
		else if (!strcmp(argv[i], "--duration") && i + 1 < argc) duration_ms = atoi(argv[++i]);
		else {
			fprintf(stderr, "usage: %s [--shm N] [--port P] [--low K] "
				"[--high K] [--dwell MS] [--cycles N] [--settle MS]\n"
				"       %s --cycles 0 --duration MS   (passive: no commands)\n",
				argv[0], argv[0]);
			return 2;
		}
	}
	if (cycles <= 0 && duration_ms <= 0) {
		fprintf(stderr, "--cycles 0 needs --duration MS\n");
		return 2;
	}

	char path[300];
	snprintf(path, sizeof(path), "/dev/shm/%s", shm);
	int fd = open(path, O_RDONLY);
	if (fd < 0) {
		fprintf(stderr, "open %s: %m\n", path);
		return 1;
	}
	struct stat st;
	if (fstat(fd, &st) != 0 || st.st_size < (off_t)sizeof(venc_frame_ring_hdr_t)) {
		fprintf(stderr, "%s: too small\n", path);
		return 1;
	}
	void *map = mmap(NULL, (size_t)st.st_size, PROT_READ, MAP_SHARED, fd, 0);
	if (map == MAP_FAILED) {
		fprintf(stderr, "mmap: %m\n");
		return 1;
	}
	const venc_frame_ring_hdr_t *hdr = (const venc_frame_ring_hdr_t *)map;
	if (hdr->magic != VENC_FRAME_RING_MAGIC) {
		fprintf(stderr, "bad magic 0x%08x\n", hdr->magic);
		return 1;
	}
	const uint8_t *slots = (const uint8_t *)map + sizeof(venc_frame_ring_hdr_t);
	const uint32_t stride = slot_stride(hdr->slot_data_size);
	const uint32_t count = hdr->slot_count;

	printf("# vencprobe shm=%s slots=%u slot_data=%u low=%d high=%d "
	       "dwell_ms=%d cycles=%d duration_ms=%d\n", shm, count,
	       hdr->slot_data_size, low, high, dwell_ms, cycles, duration_ms);
	printf("# f,mono_us,write_idx,len,pts_us,flags,enc_us\n");
	printf("# c,mono_us_before,mono_us_after,kbps,ok\n");
	printf("# s,mono_us,req_bitrate_kbps,qp  (25 ms poll; a change we did not command is RcAgent; qp = encoder startQual of the last frame, -1 if the daemon predates it)\n");
	fflush(stdout);

	uint64_t last_w = __atomic_load_n(&hdr->write_idx, __ATOMIC_ACQUIRE);
	uint64_t t0 = mono_us();
	/* Passive: one deadline, no steps. Active: first step after settle. */
	uint64_t next_step_us = cycles > 0
		? t0 + (uint64_t)settle_ms * 1000ull
		: t0 + (uint64_t)duration_ms * 1000ull;
	uint64_t next_poll_us = t0;
	int steps_left = cycles * 2;
	int want_low = 1;

	for (;;) {
		uint64_t w = __atomic_load_n(&hdr->write_idx, __ATOMIC_ACQUIRE);
		while (last_w < w) {
			uint64_t now = mono_us();
			const uint8_t *slot = slots + (uint64_t)(last_w & (count - 1)) * stride;
			uint32_t len;
			VencFrameMeta meta;
			memcpy(&len, slot, sizeof(len));
			memcpy(&meta, slot + sizeof(uint32_t), sizeof(meta));
			/* len covers meta + payload; report the payload bytes,
			 * i.e. exactly what the encoder emitted for the frame. */
			printf("f,%llu,%llu,%u,%u,%u,%u\n",
			       (unsigned long long)now, (unsigned long long)last_w,
			       len > VENC_FRAME_META_SIZE ? len - VENC_FRAME_META_SIZE : 0,
			       meta.pts, meta.flags, meta.enc_us);
			last_w++;
		}

		uint64_t now = mono_us();
		if (now >= next_poll_us) {
			int qp = -1;
			int req = get_req_bitrate(port, &qp);
			uint64_t tp = mono_us();
			printf("s,%llu,%d,%d\n", (unsigned long long)tp, req, qp);
			next_poll_us = tp + 25000ull;
		}

		if (steps_left > 0 && now >= next_step_us) {
			uint64_t tb = 0, ta = 0;
			int kbps = want_low ? low : high;
			int ok = post_bitrate(port, kbps, &tb, &ta) == 0;
			printf("c,%llu,%llu,%d,%d\n", (unsigned long long)tb,
			       (unsigned long long)ta, kbps, ok);
			fflush(stdout);
			want_low = !want_low;
			steps_left--;
			next_step_us = now + (uint64_t)dwell_ms * 1000ull;
		}
		if (steps_left == 0 && now >= next_step_us)
			break;

		struct timespec ts = {0, 100000};  /* 100 µs */
		nanosleep(&ts, NULL);
	}

	fflush(stdout);
	return 0;
}
