#include "Mode.h"

namespace bitluni {
// int hFront, int hSync, int hBack, int hRes, int vFront, int vSync, int vBack, int vRes, int frequency, int hPol = 1, int vPol = 1, int vClones = 1
const Mode Mode::MODE_640x400x70(16, 96, 48, 640, 12, 2, 35, 400, 25175000);
const Mode Mode::MODE_320x200x70(8, 48, 24, 320, 12, 2, 35, 200, 12587500, 0, 0, 2);
const Mode Mode::MODE_640x480x60(16, 96, 48, 640, 10, 2, 33, 480, 25175000);
const Mode Mode::MODE_320x240x60(8, 48, 24, 320, 10, 2, 33, 240, 12587500, 0, 0, 2);
const Mode Mode::MODE_800x600x56(24, 72, 128, 800, 1, 2, 22, 600, 36000000);
const Mode Mode::MODE_800x600x60(40, 128, 88, 800, 1, 4, 23, 600, 40000000);
const Mode Mode::MODE_400x300x60(20, 64, 44, 400, 1, 4, 23, 300, 20000000, 0, 0, 2);
const Mode Mode::MODE_1024x768x43(8, 176, 56, 1024, 0, 4, 20, 768, 44900000);
const Mode Mode::MODE_1024x768x60(24, 136, 160, 1024, 3, 6, 29, 768, 65000000);
const Mode Mode::MODE_1280x720x60(110, 40, 220, 1280, 5, 5, 20, 720, 74250000);
} // namespace bitluni