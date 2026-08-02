#include "dvr_mux.h"

#include <cstring>

#include "hevc_params.h"

namespace maburplay {
namespace {

// Small big-endian box builder. Mirrors the writer style used elsewhere
// in this codebase (explicit byte packing, no external mp4 library) so
// the on-disk layout is fully controlled and easy to verify with a box
// walker in the test.
struct BoxWriter {
  std::vector<uint8_t> buf;

  // Writes a placeholder size + the fourcc; returns the offset of the
  // size field so end() can patch it once the box's contents are known.
  size_t begin(const char* tag) {
    size_t at = buf.size();
    u32(0);
    bytes(reinterpret_cast<const uint8_t*>(tag), 4);
    return at;
  }
  void end(size_t at) {
    uint32_t size = static_cast<uint32_t>(buf.size() - at);
    buf[at + 0] = static_cast<uint8_t>((size >> 24) & 0xFF);
    buf[at + 1] = static_cast<uint8_t>((size >> 16) & 0xFF);
    buf[at + 2] = static_cast<uint8_t>((size >> 8) & 0xFF);
    buf[at + 3] = static_cast<uint8_t>(size & 0xFF);
  }
  void u8(uint8_t v) { buf.push_back(v); }
  void u16(uint16_t v) {
    buf.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
    buf.push_back(static_cast<uint8_t>(v & 0xFF));
  }
  void u32(uint32_t v) {
    buf.push_back(static_cast<uint8_t>((v >> 24) & 0xFF));
    buf.push_back(static_cast<uint8_t>((v >> 16) & 0xFF));
    buf.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
    buf.push_back(static_cast<uint8_t>(v & 0xFF));
  }
  void u64(uint64_t v) {
    u32(static_cast<uint32_t>((v >> 32) & 0xFFFFFFFFu));
    u32(static_cast<uint32_t>(v & 0xFFFFFFFFu));
  }
  void bytes(const uint8_t* p, size_t n) { buf.insert(buf.end(), p, p + n); }
  void bytes(const std::vector<uint8_t>& v) { bytes(v.data(), v.size()); }
  void fourcc(const char* tag) { bytes(reinterpret_cast<const uint8_t*>(tag), 4); }
};

void put_unity_matrix(BoxWriter& b) {
  b.u32(0x00010000);
  b.u32(0);
  b.u32(0);
  b.u32(0);
  b.u32(0x00010000);
  b.u32(0);
  b.u32(0);
  b.u32(0);
  b.u32(0x40000000);
}

}  // namespace

bool DvrMux::open(const std::string& path, const std::vector<uint8_t>& hvcc, int width,
                   int height, int fragment_ms) {
  f_ = std::fopen(path.c_str(), "wb");
  if (!f_) return false;

  width_ = width;
  height_ = height;
  hvcc_ = hvcc;
  fragment_ms_ = fragment_ms;

  BoxWriter b;

  // ftyp
  {
    size_t at = b.begin("ftyp");
    b.fourcc("iso5");    // major_brand
    b.u32(0x00000200);   // minor_version
    b.fourcc("iso5");    // compatible_brands[0]
    b.fourcc("iso6");    // compatible_brands[1]
    b.fourcc("mp41");    // compatible_brands[2]
    b.end(at);
  }

  // moov
  {
    size_t moov_at = b.begin("moov");

    // mvhd
    {
      size_t at = b.begin("mvhd");
      b.u32(0);          // version(0)/flags(0)
      b.u32(0);          // creation_time
      b.u32(0);          // modification_time
      b.u32(1000000);    // timescale
      b.u32(0);          // duration
      b.u32(0x00010000); // rate = 1.0
      b.u16(0x0100);     // volume = 1.0
      b.u16(0);          // reserved
      b.u32(0);
      b.u32(0);  // reserved[2]
      put_unity_matrix(b);
      for (int i = 0; i < 6; ++i) b.u32(0);  // pre_defined
      b.u32(2);                              // next_track_ID
      b.end(at);
    }

    // trak
    {
      size_t trak_at = b.begin("trak");

      // tkhd
      {
        size_t at = b.begin("tkhd");
        b.u32(0x00000003);  // version(0)/flags(track_enabled|track_in_movie)
        b.u32(0);           // creation_time
        b.u32(0);           // modification_time
        b.u32(1);           // track_ID
        b.u32(0);           // reserved
        b.u32(0);           // duration
        b.u32(0);
        b.u32(0);  // reserved[2]
        b.u16(0);  // layer
        b.u16(0);  // alternate_group
        b.u16(0);  // volume (0 for video track)
        b.u16(0);  // reserved
        put_unity_matrix(b);
        b.u32(static_cast<uint32_t>(width) << 16);   // width, 16.16 fixed
        b.u32(static_cast<uint32_t>(height) << 16);  // height, 16.16 fixed
        b.end(at);
      }

      // mdia
      {
        size_t mdia_at = b.begin("mdia");

        // mdhd
        {
          size_t at = b.begin("mdhd");
          b.u32(0);         // version/flags
          b.u32(0);         // creation_time
          b.u32(0);         // modification_time
          b.u32(1000000);   // timescale
          b.u32(0);         // duration
          b.u16(0x55C4);    // language = 'und' (packed ISO-639-2)
          b.u16(0);         // pre_defined
          b.end(at);
        }

        // hdlr
        {
          size_t at = b.begin("hdlr");
          b.u32(0);          // version/flags
          b.u32(0);          // pre_defined
          b.fourcc("vide");  // handler_type
          b.u32(0);
          b.u32(0);
          b.u32(0);  // reserved[3]
          const char* name = "mabur DVR";
          b.bytes(reinterpret_cast<const uint8_t*>(name), std::strlen(name) + 1);
          b.end(at);
        }

        // minf
        {
          size_t minf_at = b.begin("minf");

          // vmhd
          {
            size_t at = b.begin("vmhd");
            b.u32(0x00000001);  // version(0)/flags(1)
            b.u16(0);           // graphicsmode
            b.u16(0);
            b.u16(0);
            b.u16(0);  // opcolor
            b.end(at);
          }

          // dinf { dref { url } }
          {
            size_t dinf_at = b.begin("dinf");
            {
              size_t at = b.begin("dref");
              b.u32(0);  // version/flags
              b.u32(1);  // entry_count
              {
                size_t uat = b.begin("url ");
                b.u32(0x00000001);  // version(0)/flags(self-contained)
                b.end(uat);
              }
              b.end(at);
            }
            b.end(dinf_at);
          }

          // stbl
          {
            size_t stbl_at = b.begin("stbl");

            // stsd { hvc1 { hvcC } }
            {
              size_t at = b.begin("stsd");
              b.u32(0);  // version/flags
              b.u32(1);  // entry_count
              {
                size_t hvc1_at = b.begin("hvc1");
                // SampleEntry
                b.u32(0);
                b.u16(0);  // reserved[6]
                b.u16(1);  // data_reference_index
                // VisualSampleEntry
                b.u16(0);  // pre_defined
                b.u16(0);  // reserved
                b.u32(0);
                b.u32(0);
                b.u32(0);  // pre_defined[3]
                b.u16(static_cast<uint16_t>(width));
                b.u16(static_cast<uint16_t>(height));
                b.u32(0x00480000);  // horizresolution = 72 dpi
                b.u32(0x00480000);  // vertresolution = 72 dpi
                b.u32(0);           // reserved
                b.u16(1);           // frame_count
                for (int i = 0; i < 32; ++i) b.u8(0);  // compressorname (zeroed)
                b.u16(24);      // depth
                b.u16(0xFFFF);  // pre_defined
                {
                  size_t hat = b.begin("hvcC");
                  b.bytes(hvcc_);
                  b.end(hat);
                }
                b.end(hvc1_at);
              }
              b.end(at);
            }

            // empty stts/stsc/stco, zero-count stsz
            {
              size_t at = b.begin("stts");
              b.u32(0);
              b.u32(0);
              b.end(at);
            }
            {
              size_t at = b.begin("stsc");
              b.u32(0);
              b.u32(0);
              b.end(at);
            }
            {
              size_t at = b.begin("stsz");
              b.u32(0);  // version/flags
              b.u32(0);  // sample_size
              b.u32(0);  // sample_count
              b.end(at);
            }
            {
              size_t at = b.begin("stco");
              b.u32(0);
              b.u32(0);
              b.end(at);
            }

            b.end(stbl_at);
          }

          b.end(minf_at);
        }

        b.end(mdia_at);
      }

      b.end(trak_at);
    }

    // mvex { trex }
    {
      size_t mvex_at = b.begin("mvex");
      {
        size_t at = b.begin("trex");
        b.u32(0);  // version/flags
        b.u32(1);  // track_ID
        b.u32(1);  // default_sample_description_index
        b.u32(0);  // default_sample_duration
        b.u32(0);  // default_sample_size
        b.u32(0);  // default_sample_flags
        b.end(at);
      }
      b.end(mvex_at);
    }

    b.end(moov_at);
  }

  size_t n = std::fwrite(b.buf.data(), 1, b.buf.size(), f_);
  std::fflush(f_);
  return n == b.buf.size();
}

uint64_t DvrMux::unwrap_pts(uint32_t pts_us) {
  if (!have_pts_) {
    have_pts_ = true;
    last_pts_raw_ = pts_us;
    // Rebase the recording timeline to zero at the first sample. The
    // capture pts is encoder-session-relative; writing it absolute into
    // tfdt made players front-pad the seekbar with the session's age
    // (observed: ffprobe duration 403 s for 68 s of content recorded
    // ~7 min after an encoder restart).
    last_pts64_ = 0;
    return 0;
  }
  // Same delta-unwrap technique as AuRingWriter::finish's frame_id
  // unwrap (gs/src/au_ring.cpp): the u32-wrapped forward delta is
  // well-defined via unsigned subtraction as long as consecutive
  // samples are within 2^31 us of each other, which holds for a
  // real-time capture stream.
  uint32_t d = pts_us - last_pts_raw_;
  last_pts64_ += static_cast<uint64_t>(d);
  last_pts_raw_ = pts_us;
  return last_pts64_;
}

void DvrMux::write_sample(const uint8_t* au, size_t n, uint32_t pts_us, bool key) {
  uint64_t pts64 = unwrap_pts(pts_us);

  bool cut = false;
  if (!pending_.empty()) {
    if (key) {
      cut = true;
    } else if (pts64 - fragment_start_pts_ >= static_cast<uint64_t>(fragment_ms_) * 1000) {
      cut = true;
    }
  }
  if (cut) flush_fragment();

  if (pending_.empty()) fragment_start_pts_ = pts64;

  Sample s;
  s.data = annexb_to_length_prefixed(au, n);
  s.pts64 = pts64;
  s.key = key;
  pending_.push_back(std::move(s));
  ++samples_;
}

void DvrMux::flush_fragment() {
  if (pending_.empty()) return;
  ++fragments_;

  BoxWriter b;
  size_t moof_at = b.begin("moof");

  {
    size_t at = b.begin("mfhd");
    b.u32(0);  // version/flags
    b.u32(static_cast<uint32_t>(fragments_));
    b.end(at);
  }

  size_t data_offset_pos = 0;
  {
    size_t traf_at = b.begin("traf");

    {
      size_t at = b.begin("tfhd");
      b.u32(0x00020000);  // version(0)/flags(default-base-is-moof)
      b.u32(1);           // track_ID
      b.end(at);
    }

    {
      size_t at = b.begin("tfdt");
      b.u32(0x01000000);  // version(1)/flags(0)
      b.u64(fragment_start_pts_);
      b.end(at);
    }

    {
      size_t at = b.begin("trun");
      b.u32(0x00000701);  // data-offset | sample-duration | sample-size | sample-flags
      b.u32(static_cast<uint32_t>(pending_.size()));
      data_offset_pos = b.buf.size();
      b.u32(0);  // data_offset placeholder, patched below

      for (size_t i = 0; i < pending_.size(); ++i) {
        uint32_t dur;
        if (i + 1 < pending_.size()) {
          // Real delta to the next sample in this fragment — becomes the
          // new running estimate for the next lone-sample fragment.
          dur = static_cast<uint32_t>(pending_[i + 1].pts64 - pending_[i].pts64);
          last_dur_us_ = dur;
        } else {
          // Last sample of the fragment (possibly the only one): no next
          // sample to measure against, so reuse the running estimate.
          // Never 0 — some players compute playback rate from duration.
          dur = last_dur_us_;
        }
        b.u32(dur);
        b.u32(static_cast<uint32_t>(pending_[i].data.size()));
        if (pending_[i].key) {
          b.u8(0x02);
          b.u8(0x00);
          b.u8(0x00);
          b.u8(0x00);
        } else {
          b.u8(0x01);
          b.u8(0x01);
          b.u8(0x00);
          b.u8(0x00);
        }
      }
      b.end(at);
    }

    b.end(traf_at);
  }

  b.end(moof_at);

  uint32_t moof_size = static_cast<uint32_t>(b.buf.size());
  uint32_t data_offset = moof_size + 8;  // moof end + mdat's own 8-byte header
  b.buf[data_offset_pos + 0] = static_cast<uint8_t>((data_offset >> 24) & 0xFF);
  b.buf[data_offset_pos + 1] = static_cast<uint8_t>((data_offset >> 16) & 0xFF);
  b.buf[data_offset_pos + 2] = static_cast<uint8_t>((data_offset >> 8) & 0xFF);
  b.buf[data_offset_pos + 3] = static_cast<uint8_t>(data_offset & 0xFF);

  size_t mdat_at = b.begin("mdat");
  for (auto& s : pending_) b.bytes(s.data);
  b.end(mdat_at);

  std::fwrite(b.buf.data(), 1, b.buf.size(), f_);
  std::fflush(f_);

  pending_.clear();
}

void DvrMux::close() {
  flush_fragment();
  if (f_) {
    std::fclose(f_);
    f_ = nullptr;
  }
}

}  // namespace maburplay
