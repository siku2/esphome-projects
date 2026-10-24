#pragma once

#include <cstddef>
#include <cstdint>

// Pure decoding logic for the SureFlap cat flap radio link, with no ESPHome or
// IDF dependencies so it can run in a host test. The device layer feeds raw
// 802.15.4 frames in and reads the parsed result out.
//
// The link is plain 802.15.4 on PAN 0x3421 with extended addresses and PAN ID
// compression. Application messages of type 0x01 are encrypted with a stream
// cipher whose keystream is fixed per session and reused for every message.
// See scratch/sureflap-protocol.md.

namespace sureflap_protocol {

constexpr uint16_t PAN = 0x3421;

// First three bytes of the extended addresses (most significant first). The hub
// prefix is how we tell the two sides apart. The flap prefix is the SureFlap OUI.
constexpr uint8_t HUB_PREFIX[3] = {0x98, 0x27, 0x82};
constexpr uint8_t FLAP_PREFIX[3] = {0x70, 0xb3, 0xd5};

enum MsgType : uint8_t {
  MSG_DATA = 0x01,
  MSG_ACK_DATA = 0x02,
  MSG_ACK_SETUP = 0x03,
  MSG_HEARTBEAT = 0x08,
  MSG_DATA_REQUEST = 0x09,
  MSG_HEARTBEAT_REPLY = 0x0a,
  MSG_SESSION_SETUP = 0x0e,
  MSG_HELLO = 0x13,
  MSG_HELLO_REPLY = 0x14,
};

struct Frame {
  bool valid = false;
  bool src_is_hub = false;
  uint8_t type = 0;
  uint8_t seq = 0;
  const uint8_t *payload = nullptr;  // points into the input buffer
  size_t payload_len = 0;
};

inline bool prefix_matches(const uint8_t *addr_be, const uint8_t *prefix) {
  return addr_be[0] == prefix[0] && addr_be[1] == prefix[1] && addr_be[2] == prefix[2];
}

// Parse a raw 802.15.4 MAC frame (PSDU without the FCS). Returns a Frame with
// valid=false if it is not a SureFlap application frame on our PAN.
inline Frame parse_frame(const uint8_t *f, size_t len) {
  Frame out;
  if (len < 3)
    return out;

  uint16_t fcf = static_cast<uint16_t>(f[0] | (f[1] << 8));
  uint8_t dst_mode = (fcf >> 10) & 0x3;
  uint8_t src_mode = (fcf >> 14) & 0x3;
  bool pan_compressed = (fcf >> 6) & 0x1;

  size_t off = 3;  // skip FCF and MAC sequence number
  bool have_pan = false;
  uint16_t pan = 0;

  auto read_u16 = [&](uint16_t &value) -> bool {
    if (off + 2 > len)
      return false;
    value = static_cast<uint16_t>(f[off] | (f[off + 1] << 8));
    off += 2;
    return true;
  };

  // Addresses are little endian on the wire. Reverse the extended address into
  // big-endian order so the prefix bytes read as written in the notes.
  auto skip_or_read_addr = [&](uint8_t mode, uint8_t *addr_be) -> bool {
    if (mode == 2) {  // short address
      if (off + 2 > len)
        return false;
      off += 2;
      return true;
    }
    if (mode == 3) {  // extended address
      if (off + 8 > len)
        return false;
      if (addr_be != nullptr)
        for (size_t i = 0; i < 8; i++)
          addr_be[i] = f[off + 7 - i];
      off += 8;
      return true;
    }
    return addr_be == nullptr;  // no address present
  };

  if (dst_mode) {
    if (!read_u16(pan))
      return out;
    have_pan = true;
    if (!skip_or_read_addr(dst_mode, nullptr))
      return out;
  }

  uint8_t src_addr_be[8] = {};
  bool have_src_ext = false;
  if (src_mode) {
    if (!pan_compressed) {
      if (!read_u16(pan))
        return out;
      have_pan = true;
    }
    have_src_ext = src_mode == 3;
    if (!skip_or_read_addr(src_mode, have_src_ext ? src_addr_be : nullptr))
      return out;
  }

  if (!have_pan || pan != PAN)
    return out;
  if (off + 2 > len)
    return out;

  out.valid = true;
  out.src_is_hub = have_src_ext && prefix_matches(src_addr_be, HUB_PREFIX);
  out.type = f[off];
  out.seq = f[off + 1];
  out.payload = f + off;
  out.payload_len = len - off;
  return out;
}

// Per-session keystream for type 0x01 payloads. Bytes 0 and 1 (type and sender
// sequence) are cleartext, so the keystream applies from offset 2 on.
struct Keystream {
  static constexpr size_t START = 2;
  static constexpr size_t CAP = 128;

  uint8_t data[CAP] = {};
  bool known[CAP] = {};

  void reset() {
    for (size_t i = 0; i < CAP; i++) {
      data[i] = 0;
      known[i] = false;
    }
  }

  // Learn one keystream byte from a known plaintext byte at an offset.
  void learn(size_t offset, uint8_t cipher, uint8_t plain) {
    if (offset >= CAP)
      return;
    data[offset] = cipher ^ plain;
    known[offset] = true;
  }

  // Decrypt in place from START on. Returns false if any needed byte is unknown.
  bool decrypt(const uint8_t *payload, size_t len, uint8_t *out) const {
    for (size_t i = 0; i < len && i < START; i++)
      out[i] = payload[i];
    for (size_t i = START; i < len; i++) {
      if (i >= CAP || !known[i])
        return false;
      out[i] = payload[i] ^ data[i];
    }
    return true;
  }
};

// FDX-B pet microchip tag. value = (country << 38) | national_id, stored as 6
// little-endian bytes. Country 756 and the shared high bits identify the format,
// the low bits identify the individual chip.
struct Fdxb {
  uint16_t country = 0;
  uint64_t national_id = 0;
};

inline Fdxb fdxb_decode(const uint8_t tag[6]) {
  uint64_t value = 0;
  for (size_t i = 0; i < 6; i++)
    value |= static_cast<uint64_t>(tag[i]) << (8 * i);
  Fdxb out;
  out.national_id = value & ((static_cast<uint64_t>(1) << 38) - 1);
  out.country = static_cast<uint16_t>(value >> 38);
  return out;
}

inline void fdxb_encode(uint16_t country, uint64_t national_id, uint8_t tag[6]) {
  uint64_t value = (static_cast<uint64_t>(country) << 38) | (national_id & ((static_cast<uint64_t>(1) << 38) - 1));
  for (size_t i = 0; i < 6; i++)
    tag[i] = static_cast<uint8_t>(value >> (8 * i));
}

// Movement frame layout (confirmed against camera ground truth): a 38 byte flap
// type 0x01 frame carries the direction at offsets 23-24 and the 6 byte tag at
// offsets 25-30. A longer status frame decodes to direction 0x0202.
constexpr size_t MOVEMENT_LEN = 38;
constexpr size_t DIRECTION_OFFSET = 23;
constexpr size_t TAG_OFFSET = 25;
constexpr size_t TAG_LEN = 6;

enum Direction : uint8_t {
  DIR_OUT,
  DIR_IN,
  DIR_STATUS,
  DIR_UNKNOWN,
};

// Read the direction from a decrypted payload. The caller provides plaintext.
inline Direction movement_direction(const uint8_t *decrypted, size_t len) {
  if (len < DIRECTION_OFFSET + 2)
    return DIR_UNKNOWN;
  uint8_t a = decrypted[DIRECTION_OFFSET];
  uint8_t b = decrypted[DIRECTION_OFFSET + 1];
  if (a == 0x00 && b == 0x00)
    return DIR_OUT;
  if (a == 0x01 && b == 0x01)
    return DIR_IN;
  if (a == 0x02 && b == 0x02)
    return DIR_STATUS;
  return DIR_UNKNOWN;
}

}  // namespace sureflap_protocol
