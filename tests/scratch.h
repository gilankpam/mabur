#pragma once
// A writable throwaway file for the handful of tests that must hand a
// loader something the build can't hold for them: a malformed header, a
// truncated body, a font shape only one test wants.
//
// It lives under the BUILD tree, never in /tmp. Nothing reclaims a /tmp
// file promptly -- systemd's tmpfiles.d sweeps it on an age rule measured
// in days, and where /tmp is a tmpfs the file is RAM until then -- so a
// suite that leaves tens of MB behind per run fills a CI runner's disk, and
// the failure surfaces as some unrelated test failing to write. Under the
// build tree the files go with the build directory, and this type unlinks
// them as soon as its owner goes out of scope anyway. (std::tmpnam, which
// this replaces, is deprecated and races by construction.)
//
// MABUR_TEST_SCRATCH_DIR is injected by tests/CMakeLists.txt.
#include <sys/stat.h>
#include <unistd.h>

#include <cstdio>
#include <string>

struct ScratchFile {
  std::string path;

  // `tag` names the test binary, so a directory listing during a parallel
  // ctest says which test left something behind; pid + sequence make the
  // name unique across concurrent test processes without a temp-name race.
  ScratchFile(const char* tag, const char* suffix) {
    static int seq = 0;
    ::mkdir(MABUR_TEST_SCRATCH_DIR, 0777);
    char buf[512];
    std::snprintf(buf, sizeof(buf), "%s/%s-%d-%d%s", MABUR_TEST_SCRATCH_DIR, tag,
                  (int)::getpid(), seq++, suffix);
    path = buf;
  }
  ~ScratchFile() { std::remove(path.c_str()); }
  ScratchFile(const ScratchFile&) = delete;
  ScratchFile& operator=(const ScratchFile&) = delete;

  const char* c_str() const { return path.c_str(); }
};
