// encosd -- MPP encoder-side OSD spike bench (see
// docs/superpowers/specs/2026-08-04-mpp-enc-osd-spike.md).
//
// Creates an MPP H.265 encoder at 1080p, feeds it synthetic NV12 frames with
// an obvious moving pattern, optionally attaches a palette + indexed bitmap
// OSD region carrying a REAL maburplay MSP screen (same font atlas, same
// rasterizer -- mabur_player_core), and writes the elementary stream out.
//
// It exists to answer five questions with evidence, not to be a component:
//   1. does an MPP HEVC encoder run here at all
//   2. does the encoder-side OSD burn in
//   3. what region geometry is accepted (--region / --regions)
//   4. how does the 256-colour quantization look on antialiased glyphs
//   5. what does it cost (--no-osd A/B, printed fps + CPU)
//
// The OSD is attached by hanging a MppEncOSDData (generation 1) or
// MppEncOSDData2 (generation 2) off the input frame's meta under
// KEY_OSD_DATA / KEY_OSD_DATA2. Both land in the same place: the vepu54x HAL
// (mpp/hal/rkenc/common/vepu541_common.c, vepu540_set_osd for this SoC) calls
// copy2osd2() which widens gen-1 into gen-2 -- gen-1 wins if both are set.
#include <fcntl.h>
#include <time.h>
#include <unistd.h>

#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <rockchip/rk_mpi.h>

#include "mabur/msp_dp.h"
#include "osd_font.h"
#include "osd_palette.h"
#include "osd_raster.h"

namespace {

constexpr int kMbSize = 16;

struct Args {
  int width = 1920;
  int height = 1080;
  int hor_stride = 1920;
  int ver_stride = 1088;
  int frames = 120;
  int fps = 60;
  int bps = 8000000;
  int gop = 60;
  bool osd = true;
  int rx = 0, ry = 0, rw = 120, rh = 67;  // region, macroblocks
  int regions = 1;
  int palette_size = 255;
  double alpha_weight = 2.0;
  bool gen2 = false;      // use MppEncOSDData2 instead of MppEncOSDData
  bool mb_tile = false;   // macroblock-tiled bitmap layout instead of raster
  bool alpha_ramp = false;
  bool inverse = false;
  bool plt_atlas = false;  // palette from the whole atlas, not from this screen
  int bps2 = 0;           // mid-stream bitrate change (0 = none)
  int bps_at = 0;
  std::string out = "/tmp/encosd.265";
  std::string font = "/usr/local/share/mabur/font_btfl.mfont";
  std::string dump_osd;   // PPM of the palettized OSD reconstruction
  std::string msp_file;   // raw MSP DisplayPort byte capture
};

void usage() {
  std::fprintf(stderr,
               "usage: encosd [opts]\n"
               "  --frames N        frames to encode (default 120)\n"
               "  --fps N           encoder output fps (default 60)\n"
               "  --bps N           target bitrate (default 8000000)\n"
               "  --gop N           gop length / IDR interval (default 60)\n"
               "  --size WxH        picture size (default 1920x1080)\n"
               "  --no-osd          encode without any OSD (the A/B baseline)\n"
               "  --region x,y,w,h  ONE region, macroblock units (default 0,0,120,67)\n"
               "  --regions N       split the region into N stacked bands (max 8)\n"
               "  --gen2            attach via MppEncOSDData2/KEY_OSD_DATA2\n"
               "  --mb-tile         write the bitmap macroblock-tiled, not raster\n"
               "  --inverse         set region->inverse\n"
               "  --palette N       quantizer colours, 1..255 (default 255)\n"
               "  --alpha-weight F  alpha axis weight in the metric (default 2.0)\n"
               "  --alpha-ramp      overlay an alpha staircase (probes hw blending)\n"
               "  --plt-atlas       palette from the WHOLE glyph atlas, not this screen\n"
               "  --bps2 N --bps-at F   change bitrate to N at frame F\n"
               "  --out PATH        elementary stream (default /tmp/encosd.265)\n"
               "  --font PATH       .mfont atlas\n"
               "  --dump-osd PATH   write the palettized OSD back as a PPM\n"
               "  --msp-file PATH   raw MSP DisplayPort bytes instead of the built-in screen\n");
}

bool parse(int argc, char** argv, Args* a) {
  for (int i = 1; i < argc; ++i) {
    const std::string k = argv[i];
    auto next = [&]() -> const char* { return (i + 1 < argc) ? argv[++i] : nullptr; };
    if (k == "--frames") a->frames = atoi(next());
    else if (k == "--fps") a->fps = atoi(next());
    else if (k == "--bps") a->bps = atoi(next());
    else if (k == "--gop") a->gop = atoi(next());
    else if (k == "--size") {
      const char* v = next();
      if (!v || sscanf(v, "%dx%d", &a->width, &a->height) != 2) return false;
      a->hor_stride = (a->width + 15) & ~15;
      a->ver_stride = (a->height + 15) & ~15;
    } else if (k == "--no-osd") a->osd = false;
    else if (k == "--region") {
      const char* v = next();
      if (!v || sscanf(v, "%d,%d,%d,%d", &a->rx, &a->ry, &a->rw, &a->rh) != 4) return false;
    } else if (k == "--regions") a->regions = atoi(next());
    else if (k == "--gen2") a->gen2 = true;
    else if (k == "--mb-tile") a->mb_tile = true;
    else if (k == "--inverse") a->inverse = true;
    else if (k == "--palette") a->palette_size = atoi(next());
    else if (k == "--alpha-weight") a->alpha_weight = atof(next());
    else if (k == "--alpha-ramp") a->alpha_ramp = true;
    else if (k == "--plt-atlas") a->plt_atlas = true;
    else if (k == "--bps2") a->bps2 = atoi(next());
    else if (k == "--bps-at") a->bps_at = atoi(next());
    else if (k == "--out") a->out = next();
    else if (k == "--font") a->font = next();
    else if (k == "--dump-osd") a->dump_osd = next();
    else if (k == "--msp-file") a->msp_file = next();
    else if (k == "-h" || k == "--help") { usage(); return false; }
    else { std::fprintf(stderr, "unknown option %s\n", k.c_str()); usage(); return false; }
  }
  return true;
}

double now_s() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

// Process CPU time (user+sys) in seconds, for the cost question.
double cpu_s() {
  timespec ts{};
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts);
  return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

// ---------------------------------------------------------------- NV12 source
// Moving diagonal bars plus a slow chroma sweep: high contrast so a burned-in
// OSD is unmistakable, and enough motion that P-frames are not trivial.
void fill_nv12(uint8_t* buf, int w, int h, int hs, int vs, int n) {
  uint8_t* yp = buf;
  uint8_t* cp = buf + (size_t)hs * vs;
  const int phase = n * 6;
  for (int y = 0; y < h; ++y) {
    uint8_t* row = yp + (size_t)y * hs;
    for (int x = 0; x < w; ++x) {
      const int d = (x + y + phase) & 127;
      row[x] = (uint8_t)(d < 64 ? 32 : 220);
    }
  }
  for (int y = 0; y < h / 2; ++y) {
    uint8_t* row = cp + (size_t)y * hs;
    for (int x = 0; x < w / 2; ++x) {
      const int band = ((x * 8) / (w / 2)) & 7;
      row[x * 2 + 0] = (uint8_t)(40 + band * 24 + (phase & 31));
      row[x * 2 + 1] = (uint8_t)(220 - band * 24);
    }
  }
}

// ------------------------------------------------------------- MSP OSD screen
// A realistic betaflight/INAV screen: mixed ASCII and symbol glyphs, the
// artificial-horizon row, a crosshair, and the two sidebars -- i.e. the same
// glyph population the shipping OSD draws, so the quantizer is judged on real
// antialiased edges rather than on block text.
void append_str(std::vector<uint8_t>& s, int row, int col, const std::string& t) {
  std::vector<uint8_t> p = {3, (uint8_t)row, (uint8_t)col, 0};
  for (unsigned char c : t) p.push_back(c);
  mabur::msp_append_message(s, mabur::MSP_CMD_DISPLAYPORT, p.data(), p.size());
}

std::vector<uint8_t> builtin_screen() {
  std::vector<uint8_t> s;
  const uint8_t clr = mabur::MSP_DP_CLEAR;
  mabur::msp_append_message(s, mabur::MSP_CMD_DISPLAYPORT, &clr, 1);

  append_str(s, 0, 1, "\x97" "16.42\x06");         // battery symbol, volts
  append_str(s, 0, 10, "\x9a" "23.7\x9b");         // amps
  append_str(s, 0, 20, "\x07" "1284");             // mAh
  append_str(s, 0, 32, "\x01" "97");               // rssi
  append_str(s, 0, 40, "\x03" "14");               // sats
  append_str(s, 1, 1, "MABUR FPV");
  append_str(s, 1, 32, "LQ 100");
  // Left sidebar: speed ladder.  Right sidebar: altitude ladder.
  for (int r = 3; r <= 14; ++r) {
    append_str(s, r, 2, "\x14");
    append_str(s, r, 47, "\x15");
  }
  append_str(s, 3, 0, "\x9e" "042");
  append_str(s, 3, 44, "128\x0c");
  // Artificial horizon: the AH bar glyphs are 0x80..0x88, the crosshair
  // 0x72/0x73/0x74 at centre.
  for (int c = 17; c <= 32; ++c) {
    std::string g(1, (char)(0x80 + ((c * 3) % 9)));
    append_str(s, 8 + ((c - 17) % 3) - 1, c, g);
  }
  append_str(s, 8, 23, "\x72\x73\x74");
  append_str(s, 12, 18, "\x05" "HOME 0.24km");
  append_str(s, 15, 1, "\x9f" "45.123456 \x9f" "-93.234567");
  append_str(s, 16, 1, "ARMED  ANGLE   03:41");
  append_str(s, 16, 30, "\x0b" "24\x0a" "C");
  append_str(s, 17, 1, "abcdefghijklmnopqrstuvwxyz0123456789 .,:/-+%");

  const uint8_t scr = mabur::MSP_DP_DRAW_SCREEN;
  mabur::msp_append_message(s, mabur::MSP_CMD_DISPLAYPORT, &scr, 1);
  return s;
}

// An alpha staircase across the top of the OSD surface: 16 steps of pure
// white at alpha 0,17,...,255. If the hardware honours the palette's alpha
// byte as a blend weight the decoded picture shows a smooth ramp over the
// moving bars; if it treats alpha as a 0/non-0 mask every step but the first
// is solid white.
void draw_alpha_ramp(uint32_t* px, int w, int h, int stride) {
  const int band_h = 40;
  if (h < band_h) return;
  for (int i = 0; i < 16; ++i) {
    const int x0 = i * (w / 16);
    const int x1 = (i + 1) * (w / 16);
    const uint8_t a = (uint8_t)(i * 17);
    const uint32_t p = ((uint32_t)a << 24) | ((uint32_t)a << 16) | ((uint32_t)a << 8) | a;
    for (int y = h - band_h; y < h; ++y)
      for (int x = x0; x < x1; ++x) px[(size_t)y * stride + x] = p;
  }
}

// PPM dump of the palettized reconstruction, composited over mid-grey so
// partial alpha is visible. Pure quantizer evidence, no encoder involved.
bool dump_ppm(const std::string& path, const encosd::QuantResult& q, int w, int h) {
  std::FILE* f = std::fopen(path.c_str(), "wb");
  if (!f) return false;
  std::fprintf(f, "P6\n%d %d\n255\n", w, h);
  std::vector<uint8_t> row((size_t)w * 3);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const uint8_t idx = q.index[(size_t)y * w + x];
      const encosd::Yuva c = encosd::unpack_yuva(q.plt.entries[idx]);
      const uint32_t argb = encosd::yuva_to_argb_premul(c);
      const int a = (int)((argb >> 24) & 0xFF);
      // premultiplied over 0x80 grey
      const int bg = 128 * (255 - a) / 255;
      row[(size_t)x * 3 + 0] = (uint8_t)(((argb >> 16) & 0xFF) + bg);
      row[(size_t)x * 3 + 1] = (uint8_t)(((argb >> 8) & 0xFF) + bg);
      row[(size_t)x * 3 + 2] = (uint8_t)((argb & 0xFF) + bg);
    }
    std::fwrite(row.data(), 1, row.size(), f);
  }
  std::fclose(f);
  return true;
}

struct Region {
  int mb_x = 0, mb_y = 0, mb_w = 0, mb_h = 0;
  uint32_t buf_offset = 0;
};

}  // namespace

int main(int argc, char** argv) {
  Args a;
  if (!parse(argc, argv, &a)) return 2;

  // ------------------------------------------------------- OSD preparation
  // Render the OSD once at FULL picture size, then crop each region out of
  // it, so changing --region/--regions probes geometry without changing the
  // content under test.
  std::vector<Region> regions;
  encosd::QuantResult quant;
  std::vector<uint8_t> osd_bits;  // concatenated per-region bitmaps
  MppEncOSDPlt osd_plt;
  std::memset(&osd_plt, 0, sizeof(osd_plt));

  if (a.osd) {
    maburplay::OsdFont font;
    std::string err;
    if (!font.load(a.font, &err)) {
      std::fprintf(stderr, "encosd: font load failed: %s\n", err.c_str());
      return 1;
    }
    std::vector<uint8_t> msp;
    if (!a.msp_file.empty()) {
      std::FILE* f = std::fopen(a.msp_file.c_str(), "rb");
      if (!f) { std::fprintf(stderr, "encosd: cannot open %s\n", a.msp_file.c_str()); return 1; }
      uint8_t b[65536];
      size_t n;
      while ((n = std::fread(b, 1, sizeof(b), f)) > 0) msp.insert(msp.end(), b, b + n);
      std::fclose(f);
    } else {
      msp = builtin_screen();
    }
    mabur::MspParser parser;
    mabur::MspScreen screen;
    for (const auto& m : parser.feed(msp.data(), msp.size())) screen.apply(m);

    std::vector<uint32_t> surf((size_t)a.width * a.height, 0);
    maburplay::Surface s{surf.data(), a.width, a.height, a.width};
    maburplay::OsdRaster raster(font, maburplay::ScaleMode::kSharp);
    const int drawn = raster.draw(screen, s, nullptr);
    if (a.alpha_ramp) draw_alpha_ramp(surf.data(), a.width, a.height, a.width);
    const maburplay::OsdLayout& lay = raster.layout();
    std::printf("osd: screen %dx%d cells, drew %d, glyph %dx%d at (%d,%d)\n", screen.cols(),
                screen.rows(), drawn, lay.draw_w, lay.draw_h, lay.origin_x, lay.origin_y);

    if (a.plt_atlas) {
      // Quantize the ENTIRE atlas -- every glyph the OSD could ever draw --
      // then index this screen against that fixed palette. Phase 2's real
      // shape: one constant palette, uploaded once, never re-uploaded when
      // the glyph mix on screen changes.
      const maburplay::GlyphAtlas& at = font.native();
      const int an = at.n_glyphs * at.glyph_w * at.glyph_h;
      const encosd::QuantResult aq =
          encosd::quantize(at.pixels, an, 1, an, a.palette_size, a.alpha_weight);
      std::printf("osd: atlas %d glyphs, distinct yuva %d -> fixed palette %d entries\n",
                  at.n_glyphs, aq.distinct_colors, aq.plt.used);
      quant = encosd::map_to_palette(surf.data(), a.width, a.height, a.width, aq.plt,
                                     a.alpha_weight);
    } else {
      quant = encosd::quantize(surf.data(), a.width, a.height, a.width, a.palette_size,
                               a.alpha_weight);
    }
    std::printf("osd: distinct yuva %d -> palette %d entries, mean err %.2f max %.2f (%s)\n",
                quant.distinct_colors, quant.plt.used, quant.mean_err, quant.max_err,
                a.plt_atlas ? "atlas-wide palette" : "per-screen palette");
    for (int i = 0; i < 256; ++i) osd_plt.data[i].val = quant.plt.entries[i];

    if (!a.dump_osd.empty()) {
      if (dump_ppm(a.dump_osd, quant, a.width, a.height))
        std::printf("osd: wrote %s\n", a.dump_osd.c_str());
      else
        std::fprintf(stderr, "encosd: dump-osd write failed\n");
    }

    // Split the requested rectangle into --regions stacked bands.
    if (a.regions < 1) a.regions = 1;
    if (a.regions > 8) { std::fprintf(stderr, "encosd: max 8 regions\n"); return 2; }
    const int band = a.rh / a.regions;
    if (band < 1) { std::fprintf(stderr, "encosd: region too short to split\n"); return 2; }
    uint32_t off = 0;
    for (int i = 0; i < a.regions; ++i) {
      Region r;
      r.mb_x = a.rx;
      r.mb_y = a.ry + i * band;
      r.mb_w = a.rw;
      r.mb_h = (i == a.regions - 1) ? (a.rh - i * band) : band;
      r.buf_offset = off;
      off += (uint32_t)((r.mb_w * r.mb_h * 256 + 15) & ~15);  // 16B-aligned
      regions.push_back(r);
    }
    osd_bits.assign(off, 0);
    for (const auto& r : regions) {
      uint8_t* dst = osd_bits.data() + r.buf_offset;
      const int rw = r.mb_w * kMbSize, rh = r.mb_h * kMbSize;
      const int sx = r.mb_x * kMbSize, sy = r.mb_y * kMbSize;
      for (int y = 0; y < rh; ++y) {
        const int py = sy + y;
        for (int x = 0; x < rw; ++x) {
          const int px = sx + x;
          const uint8_t v = (py < a.height && px < a.width)
                                ? quant.index[(size_t)py * a.width + px]
                                : 0;
          size_t o;
          if (a.mb_tile) {
            // 16x16 blocks laid out block-row-major, 256 bytes each.
            const int bx = x / kMbSize, by = y / kMbSize;
            o = ((size_t)by * r.mb_w + bx) * 256 + (size_t)(y % kMbSize) * kMbSize +
                (x % kMbSize);
          } else {
            o = (size_t)y * rw + x;  // raster, stride = mb_w * 16
          }
          dst[o] = v;
        }
      }
    }
    std::printf("osd: %d region(s), bitmap %zu bytes, layout %s, api %s\n", (int)regions.size(),
                osd_bits.size(), a.mb_tile ? "mb-tile" : "raster", a.gen2 ? "gen2" : "gen1");
    for (const auto& r : regions)
      std::printf("osd:   region mb x=%d y=%d w=%d h=%d (px %d,%d %dx%d) off=%u\n", r.mb_x,
                  r.mb_y, r.mb_w, r.mb_h, r.mb_x * kMbSize, r.mb_y * kMbSize, r.mb_w * kMbSize,
                  r.mb_h * kMbSize, r.buf_offset);
  }

  // ------------------------------------------------------------- MPP encoder
  MppCtx ctx = nullptr;
  MppApi* mpi = nullptr;
  MppEncCfg cfg = nullptr;
  MppBufferGroup grp = nullptr;
  MppBuffer frm_buf = nullptr, pkt_buf = nullptr, osd_buf = nullptr;
  std::FILE* fout = nullptr;
  int rc = 1;
  const size_t frame_size = (size_t)a.hor_stride * a.ver_stride * 3 / 2;

  MPP_RET ret = mpp_create(&ctx, &mpi);
  if (ret) { std::fprintf(stderr, "encosd: mpp_create %d\n", ret); goto out; }
  ret = mpp_init(ctx, MPP_CTX_ENC, MPP_VIDEO_CodingHEVC);
  if (ret) { std::fprintf(stderr, "encosd: mpp_init %d\n", ret); goto out; }

  ret = mpp_enc_cfg_init(&cfg);
  if (ret) { std::fprintf(stderr, "encosd: cfg_init %d\n", ret); goto out; }
  ret = mpi->control(ctx, MPP_ENC_GET_CFG, cfg);
  if (ret) { std::fprintf(stderr, "encosd: GET_CFG %d\n", ret); goto out; }

  mpp_enc_cfg_set_s32(cfg, "codec:type", MPP_VIDEO_CodingHEVC);
  mpp_enc_cfg_set_s32(cfg, "prep:width", a.width);
  mpp_enc_cfg_set_s32(cfg, "prep:height", a.height);
  mpp_enc_cfg_set_s32(cfg, "prep:hor_stride", a.hor_stride);
  mpp_enc_cfg_set_s32(cfg, "prep:ver_stride", a.ver_stride);
  mpp_enc_cfg_set_s32(cfg, "prep:format", MPP_FMT_YUV420SP);
  mpp_enc_cfg_set_s32(cfg, "rc:mode", MPP_ENC_RC_MODE_CBR);
  mpp_enc_cfg_set_s32(cfg, "rc:bps_target", a.bps);
  mpp_enc_cfg_set_s32(cfg, "rc:bps_max", a.bps * 17 / 16);
  mpp_enc_cfg_set_s32(cfg, "rc:bps_min", a.bps * 15 / 16);
  mpp_enc_cfg_set_s32(cfg, "rc:fps_in_flex", 0);
  mpp_enc_cfg_set_s32(cfg, "rc:fps_in_num", a.fps);
  mpp_enc_cfg_set_s32(cfg, "rc:fps_in_denom", 1);
  mpp_enc_cfg_set_s32(cfg, "rc:fps_out_flex", 0);
  mpp_enc_cfg_set_s32(cfg, "rc:fps_out_num", a.fps);
  mpp_enc_cfg_set_s32(cfg, "rc:fps_out_denom", 1);
  mpp_enc_cfg_set_s32(cfg, "rc:gop", a.gop);
  mpp_enc_cfg_set_s32(cfg, "rc:qp_init", -1);
  mpp_enc_cfg_set_s32(cfg, "rc:qp_max", 51);
  mpp_enc_cfg_set_s32(cfg, "rc:qp_min", 10);
  mpp_enc_cfg_set_s32(cfg, "rc:qp_max_i", 51);
  mpp_enc_cfg_set_s32(cfg, "rc:qp_min_i", 10);
  mpp_enc_cfg_set_s32(cfg, "rc:qp_ip", 2);
  mpp_enc_cfg_set_s32(cfg, "h265:diff_cu_qp_delta_depth", 0);
  ret = mpi->control(ctx, MPP_ENC_SET_CFG, cfg);
  if (ret) { std::fprintf(stderr, "encosd: SET_CFG %d\n", ret); goto out; }

  {
    MppEncHeaderMode hm = MPP_ENC_HEADER_MODE_EACH_IDR;
    ret = mpi->control(ctx, MPP_ENC_SET_HEADER_MODE, &hm);
    if (ret) std::fprintf(stderr, "encosd: SET_HEADER_MODE %d (non-fatal)\n", ret);
  }

  ret = mpp_buffer_group_get_internal(&grp, MPP_BUFFER_TYPE_DRM | MPP_BUFFER_FLAGS_CACHABLE);
  if (ret) { std::fprintf(stderr, "encosd: buffer_group %d\n", ret); goto out; }
  ret = mpp_buffer_get(grp, &frm_buf, frame_size);
  if (ret) { std::fprintf(stderr, "encosd: frm_buf %d\n", ret); goto out; }
  ret = mpp_buffer_get(grp, &pkt_buf, frame_size);
  if (ret) { std::fprintf(stderr, "encosd: pkt_buf %d\n", ret); goto out; }

  if (a.osd) {
    ret = mpp_buffer_get(grp, &osd_buf, osd_bits.size());
    if (ret || !osd_buf) {
      std::fprintf(stderr, "encosd: osd_buf %zu bytes failed ret %d\n", osd_bits.size(), ret);
      goto out;
    }
    std::memcpy(mpp_buffer_get_ptr(osd_buf), osd_bits.data(), osd_bits.size());
    mpp_buffer_sync_end(osd_buf);
    std::printf("osd: buffer fd %d size %zu\n", mpp_buffer_get_fd(osd_buf),
                mpp_buffer_get_size(osd_buf));

    MppEncOSDPltCfg plt_cfg;
    std::memset(&plt_cfg, 0, sizeof(plt_cfg));
    plt_cfg.change = MPP_ENC_OSD_PLT_CFG_CHANGE_ALL;
    plt_cfg.type = MPP_ENC_OSD_PLT_TYPE_USERDEF;
    plt_cfg.plt = &osd_plt;
    ret = mpi->control(ctx, MPP_ENC_SET_OSD_PLT_CFG, &plt_cfg);
    if (ret) { std::fprintf(stderr, "encosd: SET_OSD_PLT_CFG %d\n", ret); goto out; }
    std::printf("osd: palette installed (userdef)\n");
  }

  fout = std::fopen(a.out.c_str(), "wb");
  if (!fout) { std::fprintf(stderr, "encosd: cannot open %s\n", a.out.c_str()); goto out; }

  // SPS/PPS/VPS up front.
  {
    MppPacket hdr = nullptr;
    mpp_packet_init_with_buffer(&hdr, pkt_buf);
    mpp_packet_set_length(hdr, 0);
    ret = mpi->control(ctx, MPP_ENC_GET_HDR_SYNC, hdr);
    if (ret) {
      std::fprintf(stderr, "encosd: GET_HDR_SYNC %d\n", ret);
    } else {
      std::fwrite(mpp_packet_get_pos(hdr), 1, mpp_packet_get_length(hdr), fout);
      std::printf("enc: header %zu bytes\n", mpp_packet_get_length(hdr));
    }
    mpp_packet_deinit(&hdr);
  }

  {
    // MppEncOSDData* must outlive encode_put_frame: the HAL reads the pointer
    // off the frame meta and dereferences it at gen_regs time, inside the
    // encoder thread. Keeping them on the stack of main() (not of the loop
    // body) is the cheap correct thing here.
    MppEncOSDData osd1;
    MppEncOSDData2 osd2;
    std::memset(&osd1, 0, sizeof(osd1));
    std::memset(&osd2, 0, sizeof(osd2));
    if (a.osd) {
      osd1.buf = osd_buf;
      osd1.num_region = (RK_U32)regions.size();
      osd2.num_region = (RK_U32)regions.size();
      for (size_t i = 0; i < regions.size(); ++i) {
        osd1.region[i].enable = 1;
        osd1.region[i].inverse = a.inverse ? 1 : 0;
        osd1.region[i].start_mb_x = (RK_U32)regions[i].mb_x;
        osd1.region[i].start_mb_y = (RK_U32)regions[i].mb_y;
        osd1.region[i].num_mb_x = (RK_U32)regions[i].mb_w;
        osd1.region[i].num_mb_y = (RK_U32)regions[i].mb_h;
        osd1.region[i].buf_offset = regions[i].buf_offset;
        osd2.region[i].enable = 1;
        osd2.region[i].inverse = a.inverse ? 1 : 0;
        osd2.region[i].start_mb_x = (RK_U32)regions[i].mb_x;
        osd2.region[i].start_mb_y = (RK_U32)regions[i].mb_y;
        osd2.region[i].num_mb_x = (RK_U32)regions[i].mb_w;
        osd2.region[i].num_mb_y = (RK_U32)regions[i].mb_h;
        osd2.region[i].buf_offset = regions[i].buf_offset;
        osd2.region[i].buf = osd_buf;
      }
    }

    int frames_out = 0, errors = 0;
    uint64_t bytes = 0;
    double fill_s = 0.0;
    const double t0 = now_s();
    const double c0 = cpu_s();
    double t_first_pkt = 0;

    for (int n = 0; n < a.frames; ++n) {
      if (a.bps2 && n == a.bps_at) {
        mpp_enc_cfg_set_s32(cfg, "rc:bps_target", a.bps2);
        mpp_enc_cfg_set_s32(cfg, "rc:bps_max", a.bps2 * 17 / 16);
        mpp_enc_cfg_set_s32(cfg, "rc:bps_min", a.bps2 * 15 / 16);
        const MPP_RET r2 = mpi->control(ctx, MPP_ENC_SET_CFG, cfg);
        std::printf("enc: bitrate -> %d at frame %d (ret %d)\n", a.bps2, n, r2);
      }

      // The synthetic frame fill is pure CPU and would otherwise be charged
      // to the encoder in the fps number -- time it separately.
      const double tf0 = now_s();
      uint8_t* p = (uint8_t*)mpp_buffer_get_ptr(frm_buf);
      mpp_buffer_sync_begin(frm_buf);
      fill_nv12(p, a.width, a.height, a.hor_stride, a.ver_stride, n);
      mpp_buffer_sync_end(frm_buf);
      fill_s += now_s() - tf0;

      MppFrame frame = nullptr;
      if (mpp_frame_init(&frame)) { ++errors; break; }
      mpp_frame_set_width(frame, a.width);
      mpp_frame_set_height(frame, a.height);
      mpp_frame_set_hor_stride(frame, a.hor_stride);
      mpp_frame_set_ver_stride(frame, a.ver_stride);
      mpp_frame_set_fmt(frame, MPP_FMT_YUV420SP);
      mpp_frame_set_eos(frame, n == a.frames - 1);
      mpp_frame_set_buffer(frame, frm_buf);

      MppMeta meta = mpp_frame_get_meta(frame);
      MppPacket packet = nullptr;
      mpp_packet_init_with_buffer(&packet, pkt_buf);
      mpp_packet_set_length(packet, 0);
      mpp_meta_set_packet(meta, KEY_OUTPUT_PACKET, packet);
      if (a.osd) {
        if (a.gen2)
          mpp_meta_set_ptr(meta, KEY_OSD_DATA2, (void*)&osd2);
        else
          mpp_meta_set_ptr(meta, KEY_OSD_DATA, (void*)&osd1);
      }

      const MPP_RET pr = mpi->encode_put_frame(ctx, frame);
      mpp_frame_deinit(&frame);
      if (pr) {
        std::fprintf(stderr, "encosd: put_frame %d at %d\n", pr, n);
        ++errors;
        mpp_packet_deinit(&packet);
        break;
      }

      MppPacket outp = nullptr;
      const MPP_RET gr = mpi->encode_get_packet(ctx, &outp);
      if (gr || !outp) {
        std::fprintf(stderr, "encosd: get_packet %d at %d\n", gr, n);
        ++errors;
        mpp_packet_deinit(&packet);
        break;
      }
      const size_t len = mpp_packet_get_length(outp);
      std::fwrite(mpp_packet_get_pos(outp), 1, len, fout);
      bytes += len;
      ++frames_out;
      if (!t_first_pkt) t_first_pkt = now_s();
      mpp_packet_deinit(&outp);
    }

    const double t1 = now_s();
    const double c1 = cpu_s();
    const double el = t1 - t0;
    std::fclose(fout);
    fout = nullptr;
    const double enc_el = el - fill_s;
    std::printf(
        "result: osd=%s regions=%d frames=%d errors=%d elapsed=%.3fs fps=%.2f "
        "fill=%.3fs enc_only=%.3fs enc_fps=%.2f enc_ms=%.2f "
        "bytes=%" PRIu64 " avg_frame=%.0fB bitrate=%.2fMbps cpu=%.3fs cpu_per_frame=%.2fms "
        "first_pkt_latency=%.1fms\n",
        a.osd ? "on" : "off", (int)regions.size(), frames_out, errors, el,
        frames_out ? frames_out / el : 0.0, fill_s, enc_el,
        (frames_out && enc_el > 0) ? frames_out / enc_el : 0.0,
        frames_out ? enc_el * 1000.0 / frames_out : 0.0, bytes,
        frames_out ? (double)bytes / frames_out : 0.0,
        el > 0 ? (double)bytes * 8 / el / 1e6 : 0.0, c1 - c0,
        frames_out ? (c1 - c0) * 1000.0 / frames_out : 0.0,
        t_first_pkt ? (t_first_pkt - t0) * 1000.0 : 0.0);
    rc = (frames_out == a.frames && errors == 0) ? 0 : 1;
    std::printf("result: wrote %s\n", a.out.c_str());
  }

out:
  if (fout) std::fclose(fout);
  if (ctx) mpp_destroy(ctx);
  if (cfg) mpp_enc_cfg_deinit(cfg);
  if (osd_buf) mpp_buffer_put(osd_buf);
  if (pkt_buf) mpp_buffer_put(pkt_buf);
  if (frm_buf) mpp_buffer_put(frm_buf);
  if (grp) mpp_buffer_group_put(grp);
  return rc;
}
