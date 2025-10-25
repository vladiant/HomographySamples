#include "imgconversion.h"

void rgb24_to_rgb24(unsigned char* src, unsigned char* dst, int width,
                    int height, int ppl_src, int ppl_dst) {
  int x, y;
  for (y = 0; y < height; ++y) {
    for (x = 0; x < width; ++x) {
      *dst++ = *src++;
      *dst++ = *src++;
      *dst++ = *src++;
    }
    src += (ppl_src - width) * 3;
    dst += (ppl_dst - width) * 3;
  }
}

void rgb24_to_rgb32(unsigned char* src, unsigned char* dst, int width,
                    int height, int ppl_src, int ppl_dst) {
  int x, y;
  for (y = 0; y < height; ++y) {
    for (x = 0; x < width; ++x) {
      *dst++ = *src++;
      *dst++ = *src++;
      *dst++ = *src++;
      *dst++ = 0;
    }
    src += (ppl_src - width) * 3;
    dst += (ppl_dst - width) * 4;
  }
}

void rgb32_to_rgb24(unsigned char* src, unsigned char* dst, int width,
                    int height, int ppl_src, int ppl_dst) {
  int x, y;
  for (y = 0; y < height; ++y) {
    for (x = 0; x < width; ++x) {
      *dst++ = *src++;
      *dst++ = *src++;
      *dst++ = *src++;
      src++;
    }
    src += (ppl_src - width) * 4;
    dst += (ppl_dst - width) * 3;
  }
}

void rgb24_to_gray(unsigned char* src, unsigned char* dst, int width,
                   int height, int ppl_src, int ppl_dst) {
  src--;
  int x, y;
  for (y = 0; y < height; ++y) {
    for (x = 0; x < width; ++x) {
      *dst++ = ((*++src) * 213 + (*++src) * 715 + (*++src) * 72) / 1000;
    }
    src += (ppl_src - width) * 3;
    dst += (ppl_dst - width) * 1;
  }
}

void gray_to_rgb24(unsigned char* src, unsigned char* dst, int width,
                   int height, int ppl_src, int ppl_dst) {
  int x, y;
  for (y = 0; y < height; ++y) {
    for (x = 0; x < width; ++x) {
      *dst++ = *src;
      *dst++ = *src;
      *dst++ = *src;
      src++;
    }
    src += (ppl_src - width) * 1;
    dst += (ppl_dst - width) * 3;
  }
}

// *****************************************************************************
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

void yuv422_to_rgb24(unsigned char* src, unsigned char* dst, int width,
                     int height, int ppl_src, int ppl_dst) {
  int x, y;
  int r, g, b;
  int y1, y2, u, v;

  int width2 = width / 2;
  for (y = 0; y < height; ++y) {
    for (x = 0; x < width2; ++x) {
      // y1 = *src++;	u = *src++;		y2 = *src++;		v =
      // *src++;
      u = *src++;
      y1 = *src++;
      v = *src++;
      y2 = *src++;

      r = (256 * y1 + 359 * (v - 128) + 128) >> 8;
      g = (256 * y1 - 88 * (u - 128) - 183 * (v - 128) + 128) >> 8;
      b = (256 * y1 + 454 * (u - 128) + 128) >> 8;
      r = MAX(0, MIN(r, 255));
      g = MAX(0, MIN(g, 255));
      b = MAX(0, MIN(b, 255));
      *dst++ = r;
      *dst++ = g;
      *dst++ = b;

      r = (256 * y2 + 359 * (v - 128) + 128) >> 8;
      g = (256 * y2 - 88 * (u - 128) - 183 * (v - 128) + 128) >> 8;
      b = (256 * y2 + 454 * (u - 128) + 128) >> 8;
      r = MAX(0, MIN(r, 255));
      g = MAX(0, MIN(g, 255));
      b = MAX(0, MIN(b, 255));
      *dst++ = r;
      *dst++ = g;
      *dst++ = b;
    }
    src += (ppl_src - width) * 2;
    dst += (ppl_dst - width) * 3;
  }
}

void yuv422_to_gray(unsigned char* src, unsigned char* dst, int width,
                    int height, int ppl_src, int ppl_dst) {
  int x, y;
  int r, g, b;
  int y1, y2, u, v;

  for (y = 0; y < height; ++y) {
    for (x = 0; x < width; ++x) {
      *dst++ = *src++;
      src++;
    }
    src += (ppl_src - width) * 1;
    dst += (ppl_dst - width) * 3;
  }
}

void rgb24_to_yuv422(unsigned char* src, unsigned char* dst, int width,
                     int height, int ppl_src, int ppl_dst) {
  int x, y;
  unsigned char r, g, b;
  int y1, u1, v1, y2, u2, v2;

  int width2 = width / 2;
  for (y = 0; y < height; ++y) {
    for (x = 0; x < width2; ++x) {
      r = *src++;
      g = *src++;
      b = *src++;
      y1 = ((77 * (int)r + 150 * (int)g + 29 * (int)b + 128) >> 8) + 0;
      u1 = ((-43 * (int)r - 85 * (int)g + 128 * (int)b + 128) >> 8) + 128;
      v1 = ((128 * (int)r - 107 * (int)g - 21 * (int)b + 128) >> 8) + 128;

      r = *src++;
      g = *src++;
      b = *src++;
      y2 = ((77 * (int)r + 150 * (int)g + 29 * (int)b + 128) >> 8) + 0;
      u2 = ((-43 * (int)r - 85 * (int)g + 128 * (int)b + 128) >> 8) + 128;
      v2 = ((128 * (int)r - 107 * (int)g - 21 * (int)b + 128) >> 8) + 128;

      y1 = MAX(0, MIN(y1, 255));
      y2 = MAX(0, MIN(y2, 255));
      u1 = MAX(0, MIN(u1, 255));
      u2 = MAX(0, MIN(u2, 255));
      v1 = MAX(0, MIN(v1, 255));
      v2 = MAX(0, MIN(v2, 255));

      // *dst++ = y1;	*dst++ = (u1 + u2) / 2;
      // *dst++ = y2;	*dst++ = (v1 + v2) / 2;
      *dst++ = (u1 + u2) / 2;
      *dst++ = y1;
      *dst++ = (v1 + v2) / 2;
      *dst++ = y2;
    }
    src += (ppl_src - width) * 3;
    dst += (ppl_dst - width) * 2;
  }
}
