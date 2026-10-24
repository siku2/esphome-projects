#include "../protocol.h"

#include <cassert>
#include <cstdio>
#include <cstring>
#include <vector>

using namespace sureflap_protocol;

namespace {

// Build a data frame with extended source and destination addresses and PAN ID
// compression, matching what the flap and hub send.
std::vector<uint8_t> make_frame(const uint8_t src_be[8], const std::vector<uint8_t> &payload) {
  std::vector<uint8_t> f;
  uint16_t fcf = 0x0001 | (1 << 6) | (3 << 10) | (3 << 14);  // data, PAN compressed, ext dst+src
  f.push_back(fcf & 0xff);
  f.push_back(fcf >> 8);
  f.push_back(0x00);  // MAC sequence number
  f.push_back(PAN & 0xff);
  f.push_back(PAN >> 8);
  for (int i = 0; i < 8; i++)  // destination address, skipped by the parser
    f.push_back(0xaa);
  for (int i = 7; i >= 0; i--)  // source address, little endian on the wire
    f.push_back(src_be[i]);
  for (uint8_t b : payload)
    f.push_back(b);
  return f;
}

void test_parse_hub() {
  uint8_t hub_be[8] = {0x98, 0x27, 0x82, 0xff, 0xfe, 0xe1, 0x00, 0x6f};
  auto f = make_frame(hub_be, {MSG_HEARTBEAT_REPLY, 0x50, 0x3e});
  Frame m = parse_frame(f.data(), f.size());
  assert(m.valid);
  assert(m.src_is_hub);
  assert(m.type == MSG_HEARTBEAT_REPLY);
  assert(m.seq == 0x50);
  assert(m.payload_len == 3);
}

void test_parse_flap() {
  uint8_t flap_be[8] = {0x70, 0xb3, 0xd5, 0xf9, 0xcf, 0x05, 0x9d, 0xf5};
  auto f = make_frame(flap_be, {MSG_HEARTBEAT, 0x0c, 0x00});
  Frame m = parse_frame(f.data(), f.size());
  assert(m.valid);
  assert(!m.src_is_hub);
  assert(m.type == MSG_HEARTBEAT);
  assert(m.seq == 0x0c);
}

void test_parse_rejects_other_pan() {
  uint8_t hub_be[8] = {0x98, 0x27, 0x82, 0xff, 0xfe, 0xe1, 0x00, 0x6f};
  auto f = make_frame(hub_be, {MSG_DATA, 0x01});
  f[3] = 0x35;  // PAN 0x8435, the unrelated Thread network
  f[4] = 0x84;
  Frame m = parse_frame(f.data(), f.size());
  assert(!m.valid);
}

void test_keystream_lock() {
  // Known plaintext from the lock session: keystream[21] solves to 0xab.
  Keystream ks;
  ks.learn(21, 0xa8, 3);  // lock inside, KEEPIN=3
  assert(ks.data[21] == 0xab);
  // The same keystream byte decrypts the other two modes.
  assert((static_cast<uint8_t>(0xad) ^ ks.data[21]) == 6);  // unlock, UNLOCKED=6
  assert((static_cast<uint8_t>(0xae) ^ ks.data[21]) == 5);  // lock outside, KEEPOUT=5
}

void test_keystream_decrypt_needs_all_bytes() {
  Keystream ks;
  uint8_t cipher[5] = {MSG_DATA, 0x10, 0x11, 0x22, 0x33};
  uint8_t out[5] = {};
  assert(!ks.decrypt(cipher, 5, out));  // nothing learned yet
  ks.learn(2, 0x11, 0xaa);
  ks.learn(3, 0x22, 0xbb);
  assert(!ks.decrypt(cipher, 5, out));  // offset 4 still unknown
  ks.learn(4, 0x33, 0xcc);
  assert(ks.decrypt(cipher, 5, out));
  assert(out[0] == MSG_DATA && out[1] == 0x10);  // header stays cleartext
  assert(out[2] == 0xaa && out[3] == 0xbb && out[4] == 0xcc);
}

void test_fdxb_roundtrip() {
  uint8_t tag[6] = {};
  fdxb_encode(756, 12345678901ULL, tag);
  Fdxb d = fdxb_decode(tag);
  assert(d.country == 756);
  assert(d.national_id == 12345678901ULL);
}

void test_is_movement() {
  uint8_t flap_be[8] = {0x70, 0xb3, 0xd5, 0xf9, 0xcf, 0x05, 0x9d, 0xf5};
  uint8_t hub_be[8] = {0x98, 0x27, 0x82, 0xff, 0xfe, 0xe1, 0x00, 0x6f};
  // A 38 byte flap type 0x01 frame is a movement.
  std::vector<uint8_t> payload(MOVEMENT_LEN, 0);
  payload[0] = MSG_DATA;
  payload[1] = 0x20;
  auto f = make_frame(flap_be, payload);
  assert(is_movement(parse_frame(f.data(), f.size())));
  // Same payload from the hub is not a movement.
  auto fh = make_frame(hub_be, payload);
  assert(!is_movement(parse_frame(fh.data(), fh.size())));
  // Wrong length is not a movement.
  payload.push_back(0);
  auto fl = make_frame(flap_be, payload);
  assert(!is_movement(parse_frame(fl.data(), fl.size())));
}

void test_movement_direction() {
  uint8_t buf[MOVEMENT_LEN] = {};
  buf[DIRECTION_OFFSET] = 0x00;
  buf[DIRECTION_OFFSET + 1] = 0x00;
  assert(movement_direction(buf, MOVEMENT_LEN) == DIR_OUT);
  buf[DIRECTION_OFFSET] = 0x01;
  buf[DIRECTION_OFFSET + 1] = 0x01;
  assert(movement_direction(buf, MOVEMENT_LEN) == DIR_IN);
  buf[DIRECTION_OFFSET] = 0x02;
  buf[DIRECTION_OFFSET + 1] = 0x02;
  assert(movement_direction(buf, MOVEMENT_LEN) == DIR_STATUS);
  uint8_t tag[6] = {};  // unused, kept for clarity of the frame layout
  (void) tag;
  assert(movement_direction(buf, 10) == DIR_UNKNOWN);  // too short
}

}  // namespace

int main() {
  test_parse_hub();
  test_parse_flap();
  test_parse_rejects_other_pan();
  test_keystream_lock();
  test_keystream_decrypt_needs_all_bytes();
  test_fdxb_roundtrip();
  test_is_movement();
  test_movement_direction();
  printf("all sureflap protocol tests passed\n");
  return 0;
}
