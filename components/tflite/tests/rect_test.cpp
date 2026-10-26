#include "../rect.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

using esphome::esphome_tflite::format_crop;
using esphome::esphome_tflite::parse_crop;
using esphome::esphome_tflite::Rect;

#define CHECK(cond) \
  do { \
    if (!(cond)) { \
      std::fprintf(stderr, "%s:%d: check failed: %s\n", __FILE__, __LINE__, #cond); \
      std::exit(1); \
    } \
  } while (0)

static bool parse(const char *text, Rect &rect) { return parse_crop(text, 640, 480, rect); }

int main() {
  Rect rect{};
  CHECK(parse("10,20,30,40", rect));
  CHECK(rect.top_left.x == 10 && rect.top_left.y == 20);
  CHECK(rect.bottom_right.x == 40 && rect.bottom_right.y == 60);
  CHECK(rect.width() == 30 && rect.height() == 40);

  char buffer[32];
  CHECK(format_crop(rect, buffer, sizeof(buffer)) == 11);
  CHECK(std::strcmp(buffer, "10,20,30,40") == 0);

  CHECK(parse("0,0,640,480", rect));
  CHECK(!parse("0,0,641,480", rect));
  CHECK(!parse("1,0,640,480", rect));
  CHECK(!parse("0,1,640,480", rect));

  CHECK(!parse("", rect));
  CHECK(!parse("1,2,3", rect));
  CHECK(!parse("1,2,3,4,5", rect));
  CHECK(!parse("1,2,3,4 ", rect));
  CHECK(!parse("1;2;3;4", rect));
  CHECK(!parse("a,2,3,4", rect));
  CHECK(!parse("1,2,3,", rect));
  CHECK(!parse("-1,2,3,4", rect));
  CHECK(!parse("1,2,0,4", rect));
  CHECK(!parse("1,2,3,0", rect));
  CHECK(!parse("1,2,70000,4", rect));
  CHECK(!parse("99999999999999999999,2,3,4", rect));

  rect = Rect{{5, 6}, {7, 8}};
  CHECK(!parse("0,0,0,0", rect));
  CHECK(rect.top_left.x == 5 && rect.bottom_right.y == 8);
  return 0;
}
