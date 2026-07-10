#pragma once
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <vector>
struct MTest { const char* name; std::function<void()> fn; };
inline std::vector<MTest>& mtests() { static std::vector<MTest> v; return v; }
inline int mtest_failures = 0;
struct MReg { MReg(const char* n, void (*f)()) { mtests().push_back({n, f}); } };
#define TEST(name) static void name(); static MReg reg_##name(#name, name); static void name()
#define CHECK(cond) do { if (!(cond)) { std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); ++mtest_failures; } } while (0)
#define REQUIRE(cond) do { if (!(cond)) { std::printf("FATAL %s:%d: %s\n", __FILE__, __LINE__, #cond); std::exit(1); } } while (0)
#define MTEST_MAIN int main() { for (auto& t : mtests()) { std::printf("RUN  %s\n", t.name); t.fn(); } \
  if (mtest_failures) { std::printf("%d FAILURE(S)\n", mtest_failures); return 1; } \
  std::printf("OK — %zu test(s)\n", mtests().size()); return 0; }
