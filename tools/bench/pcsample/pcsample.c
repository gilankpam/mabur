// pcsample: sample the program counter of one thread with perf_event_open
// (software cpu-clock, no PMU needed) and dump "count dso offset" rows.
//   pcsample <tid> <seconds> <hz> <out.txt>
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <linux/perf_event.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

typedef struct { uint64_t lo, hi, off; char name[256]; } Map;
static Map maps[512]; static int nmaps;
static void load_maps(pid_t tid) {
  char p[64]; snprintf(p, sizeof p, "/proc/%d/maps", tid);
  FILE* f = fopen(p, "r"); if (!f) { perror("maps"); exit(1); }
  char line[512];
  while (fgets(line, sizeof line, f) && nmaps < 512) {
    Map* m = &maps[nmaps]; char perm[8]; unsigned long lo, hi, off;
    m->name[0] = 0;
    if (sscanf(line, "%lx-%lx %7s %lx %*s %*s %255s", &lo, &hi, perm, &off, m->name) < 4) continue;
    if (perm[2] != 'x') continue;
    m->lo = lo; m->hi = hi; m->off = off; nmaps++;
  }
  fclose(f);
}
typedef struct { uint64_t ip; uint32_t n; } Hist;
static Hist* hist; static size_t hn, hcap;
static void bump(uint64_t ip) {
  for (size_t i = 0; i < hn; ++i) if (hist[i].ip == ip) { hist[i].n++; return; }
  if (hn == hcap) { hcap = hcap ? hcap * 2 : 4096; hist = realloc(hist, hcap * sizeof *hist); }
  hist[hn].ip = ip; hist[hn].n = 1; hn++;
}
static int cmp(const void* a, const void* b) { return ((const Hist*)b)->n - ((const Hist*)a)->n; }

int main(int argc, char** argv) {
  if (argc < 5) { fprintf(stderr, "usage: pcsample tid seconds hz out\n"); return 2; }
  pid_t tid = atoi(argv[1]); int secs = atoi(argv[2]); int hz = atoi(argv[3]);
  load_maps(tid);
  struct perf_event_attr a; memset(&a, 0, sizeof a);
  a.type = PERF_TYPE_SOFTWARE; a.size = sizeof a; a.config = PERF_COUNT_SW_CPU_CLOCK;
  a.sample_freq = hz; a.freq = 1; a.sample_type = PERF_SAMPLE_IP | PERF_SAMPLE_TID;
  a.disabled = 1; a.exclude_hv = 1; a.wakeup_events = 64;
  int fd = syscall(__NR_perf_event_open, &a, tid, -1, -1, 0);
  if (fd < 0) { perror("perf_event_open"); return 1; }
  size_t pg = sysconf(_SC_PAGESIZE), npg = 64; size_t len = (1 + npg) * pg;
  void* mm = mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  if (mm == MAP_FAILED) { perror("mmap"); return 1; }
  struct perf_event_mmap_page* meta = mm; char* data = (char*)mm + pg; size_t mask = npg * pg - 1;
  ioctl(fd, PERF_EVENT_IOC_ENABLE, 0);
  uint64_t total = 0, kern = 0, tail = 0; time_t t0 = time(NULL);
  while (time(NULL) - t0 < secs) {
    usleep(20000);
    uint64_t head = __atomic_load_n(&meta->data_head, __ATOMIC_ACQUIRE);
    while (tail < head) {
      struct perf_event_header h; 
      for (size_t i = 0; i < sizeof h; ++i) ((char*)&h)[i] = data[(tail + i) & mask];
      if (h.type == PERF_RECORD_SAMPLE) {
        uint64_t ip = 0; for (size_t i = 0; i < 8; ++i) ((char*)&ip)[i] = data[(tail + sizeof h + i) & mask];
        total++; if (ip >= 0xc0000000ULL) kern++; else bump(ip);
      }
      tail += h.size;
    }
    __atomic_store_n(&meta->data_tail, tail, __ATOMIC_RELEASE);
  }
  ioctl(fd, PERF_EVENT_IOC_DISABLE, 0);
  qsort(hist, hn, sizeof *hist, cmp);
  FILE* o = fopen(argv[4], "w"); if (!o) { perror("out"); return 1; }
  fprintf(o, "# pcsample tid=%d secs=%d hz=%d total=%" PRIu64 " kernel=%" PRIu64 " user_unique=%zu\n", tid, secs, hz, total, kern, hn);
  for (size_t i = 0; i < hn; ++i) {
    uint64_t ip = hist[i].ip; const char* dso = "?"; uint64_t off = ip;
    for (int m = 0; m < nmaps; ++m) if (ip >= maps[m].lo && ip < maps[m].hi) { dso = maps[m].name; off = ip - maps[m].lo + maps[m].off; break; }
    fprintf(o, "%u 0x%" PRIx64 " %s 0x%" PRIx64 "\n", hist[i].n, ip, dso, off);
  }
  fclose(o); fprintf(stderr, "samples %" PRIu64 " (kernel %" PRIu64 "), %zu unique user IPs\n", total, kern, hn);
  return 0;
}
