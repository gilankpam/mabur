#include "threads.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#if defined(__linux__)
#include <dirent.h>
#include <pthread.h>
#include <sched.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

namespace mabur::maburd {

// An empty <dir>/<name> marker for the update loop (written as .new, then
// renamed), creating dir if it is missing.
void touch_update_file(const std::string& dir, const char* name) {
  ::mkdir(dir.c_str(), 0755);
  const std::string path = dir + "/" + name, tmp = path + ".new";
  FILE* f = std::fopen(tmp.c_str(), "w");
  if (!f || std::fclose(f) != 0 || std::rename(tmp.c_str(), path.c_str()) != 0) {
    std::remove(tmp.c_str());
    std::fprintf(stderr, "maburd trial: could not write %s\n", path.c_str());
    return;
  }
  std::fprintf(stderr, "maburd trial: %s\n", name);
}

// Names show in /proc/<pid>/task/<tid>/comm (16-char cap) — without them
// every maburd thread reads "maburd" next to the vendor SDK's unnamed
// threads, and the fec-compute forensics once spent an hour pinning
// the wrong tid. Names also let an operator taskset/renice by name.
void name_thread(const char* n) {
#if defined(__linux__)
  pthread_setname_np(pthread_self(), n);
#else
  (void)n;
#endif
}

// Moves every OTHER thread of this process whose comm equals `comm` onto
// `cpu`. A thread's children inherit its affinity AND its name, so a thread
// that pins itself somewhere unusual before calling into a library that
// spawns workers (devourer's InitWrite starts a periodic ticker) leaves
// those workers on that core with its name on them -- which is exactly how
// they are found again here. Returns the number moved.
int repin_inherited_threads(const char* comm, int cpu) {
#if defined(__linux__)
  const long self = syscall(SYS_gettid);
  int moved = 0;
  DIR* d = opendir("/proc/self/task");
  if (!d) return 0;
  while (dirent* e = readdir(d)) {
    if (e->d_name[0] == '.') continue;
    const long tid = std::atol(e->d_name);
    if (tid == self) continue;
    char path[64], name[32] = {0};
    std::snprintf(path, sizeof path, "/proc/self/task/%ld/comm", tid);
    FILE* f = std::fopen(path, "r");
    if (!f) continue;
    if (!std::fgets(name, sizeof name, f)) name[0] = '\0';
    std::fclose(f);
    name[strcspn(name, "\n")] = '\0';
    if (std::strcmp(name, comm) != 0) continue;
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(cpu, &set);
    if (sched_setaffinity(static_cast<pid_t>(tid), sizeof set, &set) == 0) ++moved;
  }
  closedir(d);
  return moved;
#else
  (void)comm; (void)cpu;
  return 0;
#endif
}
bool two_core_target() {
#if defined(__linux__)
  return sysconf(_SC_NPROCESSORS_ONLN) == 2;
#else
  return false;
#endif
}

void pin_self_to(int cpu) {
#if defined(__linux__)
  cpu_set_t set;
  CPU_ZERO(&set);
  CPU_SET(cpu, &set);
  if (pthread_setaffinity_np(pthread_self(), sizeof set, &set) != 0)
    std::fprintf(stderr, "maburd: affinity pin to cpu%d failed\n", cpu);
#else
  (void)cpu;
#endif
}

}  // namespace mabur::maburd
