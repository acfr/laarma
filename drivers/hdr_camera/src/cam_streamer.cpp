#include "Arena/ArenaApi.h"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <chrono>
#include <cstdio>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#if defined __aarch64__
#include <opencv2/xphoto/tonemap.hpp>
#endif
#include "camera_info_manager/camera_info_manager.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>

// debayer code
void ClearBorders_uint32(uint32_t *rgb, int sx, int sy, int w) {
  int i, j;

  // black edges:
  i = 3 * sx * w - 1;
  j = 3 * sx * sy - 1;
  while (i >= 0) {
    rgb[i--] = 0;
    rgb[j--] = 0;
  }

  int low = sx * (w - 1) * 3 - 1 + w * 3;
  i = low + sx * (sy - w * 2 + 1) * 3;
  while (i > low) {
    j = 6 * w;
    while (j > 0) {
      rgb[i--] = 0;
      j--;
    }
    i -= (sx - 2 * w) * 3;
  }
}

// Bilinear algorithm reworked from
// https://github.com/jdthomas/bayer2rgb/blob/master/bayer.c
void bayer_Bilinear_uint32(const uint32_t *bayer, uint32_t *rgb, int sx, int sy,
                           std::string tile = "BG", int bits = 32) {
  int bayerStep = sx;
  int rgbStep = 3 * sx;
  int width = sx;
  int height = sy;

  int blue = tile == "BG" || tile == "GB" ? -1 : 1;
  int start_with_green = tile == "GB" || tile == "GR";

  ClearBorders_uint32(rgb, sx, sy, 1);
  rgb += rgbStep + 3 + 1;
  height -= 2;
  width -= 2;

  for (; height--; bayer += bayerStep, rgb += rgbStep) {
    int t0, t1;
    const uint32_t *bayerEnd = bayer + width;

    if (start_with_green) {
      /* OpenCV has a bug in the next line, which was
      t0 = (bayer[0] + bayer[bayerStep * 2] + 1) >> 1; */
      t0 = (bayer[1] + bayer[bayerStep * 2 + 1] + 1) >> 1;
      t1 = (bayer[bayerStep] + bayer[bayerStep + 2] + 1) >> 1;
      rgb[-blue] = (uint32_t)t0;
      rgb[0] = bayer[bayerStep + 1];
      rgb[blue] = (uint32_t)t1;
      bayer++;
      rgb += 3;
    }

    if (blue > 0) {
      for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
        t0 = (bayer[0] + bayer[2] + bayer[bayerStep * 2] +
              bayer[bayerStep * 2 + 2] + 2) >>
             2;
        t1 = (bayer[1] + bayer[bayerStep] + bayer[bayerStep + 2] +
              bayer[bayerStep * 2 + 1] + 2) >>
             2;
        rgb[-1] = (uint32_t)t0;
        rgb[0] = (uint32_t)t1;
        rgb[1] = bayer[bayerStep + 1];

        t0 = (bayer[2] + bayer[bayerStep * 2 + 2] + 1) >> 1;
        t1 = (bayer[bayerStep + 1] + bayer[bayerStep + 3] + 1) >> 1;
        rgb[2] = (uint32_t)t0;
        rgb[3] = bayer[bayerStep + 2];
        rgb[4] = (uint32_t)t1;
      }
    } else {
      for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
        t0 = (bayer[0] + bayer[2] + bayer[bayerStep * 2] +
              bayer[bayerStep * 2 + 2] + 2) >>
             2;
        t1 = (bayer[1] + bayer[bayerStep] + bayer[bayerStep + 2] +
              bayer[bayerStep * 2 + 1] + 2) >>
             2;
        rgb[1] = (uint32_t)t0;
        rgb[0] = (uint32_t)t1;
        rgb[-1] = bayer[bayerStep + 1];

        t0 = (bayer[2] + bayer[bayerStep * 2 + 2] + 1) >> 1;
        t1 = (bayer[bayerStep + 1] + bayer[bayerStep + 3] + 1) >> 1;
        rgb[4] = (uint32_t)t0;
        rgb[3] = bayer[bayerStep + 2];
        rgb[2] = (uint32_t)t1;
      }
    }

    if (bayer < bayerEnd) {
      t0 = (bayer[0] + bayer[2] + bayer[bayerStep * 2] +
            bayer[bayerStep * 2 + 2] + 2) >>
           2;
      t1 = (bayer[1] + bayer[bayerStep] + bayer[bayerStep + 2] +
            bayer[bayerStep * 2 + 1] + 2) >>
           2;
      rgb[-blue] = (uint32_t)t0;
      rgb[0] = (uint32_t)t1;
      rgb[blue] = bayer[bayerStep + 1];
      bayer++;
      rgb += 3;
    }

    bayer -= width;
    rgb -= width * 3;

    blue = -blue;
    start_with_green = !start_with_green;
  }
}

/* Variable Number of Gradients, from dcraw
 * <http://www.cybercom.net/~dcoffin/dcraw/> */
/* Ported to libdc1394 by Frederic Devernay */

#define FORC3 for (c = 0; c < 3; c++)

#define SQR(x) ((x) * (x))
#define ABS(x) (((int)(x) ^ ((int)(x) >> 31)) - ((int)(x) >> 31))
#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif
#define LIM(x, min, max) MAX(min, MIN(x, max))
#define ULIM(x, y, z) ((y) < (z) ? LIM(x, y, z) : LIM(x, z, y))
/*
   In order to inline this calculation, I make the risky
   assumption that all filter patterns can be described
   by a repeating pattern of eight rows and two columns

   Return values are either 0/1/2/3 = G/M/C/Y or 0/1/2/3 = R/G1/B/G2
 */
#define FC(row, col) (filters >> ((((row) << 1 & 14) + ((col)&1)) << 1) & 3)

#define CLIP32(in, out, bits)                                                  \
  in = in < 0 ? 0 : in;                                                        \
  in = in > ((1 << bits) - 1) ? ((1 << bits) - 1) : in;                        \
  out = in;

/*
   This algorithm is officially called:

   "Interpolation using a Threshold-based variable number of gradients"

   described in http://www-ise.stanford.edu/~tingchen/algodep/vargra.html

   I've extended the basic idea to work with non-Bayer filter arrays.
   Gradients are numbered clockwise from NW=0 to W=7.
 */
static const int8_t bayervng_terms[] =
    {-2, -2, +0, -1, 0, (int8_t)0x01, -2, -2, +0, +0, 1, (int8_t)0x01,
     -2, -1, -1, +0, 0, (int8_t)0x01, -2, -1, +0, -1, 0, (int8_t)0x02,
     -2, -1, +0, +0, 0, (int8_t)0x03, -2, -1, +0, +1, 1, (int8_t)0x01,
     -2, +0, +0, -1, 0, (int8_t)0x06, -2, +0, +0, +0, 1, (int8_t)0x02,
     -2, +0, +0, +1, 0, (int8_t)0x03, -2, +1, -1, +0, 0, (int8_t)0x04,
     -2, +1, +0, -1, 1, (int8_t)0x04, -2, +1, +0, +0, 0, (int8_t)0x06,
     -2, +1, +0, +1, 0, (int8_t)0x02, -2, +2, +0, +0, 1, (int8_t)0x04,
     -2, +2, +0, +1, 0, (int8_t)0x04, -1, -2, -1, +0, 0, (int8_t)0x80,
     -1, -2, +0, -1, 0, (int8_t)0x01, -1, -2, +1, -1, 0, (int8_t)0x01,
     -1, -2, +1, +0, 1, (int8_t)0x01, -1, -1, -1, +1, 0, (int8_t)0x88,
     -1, -1, +1, -2, 0, (int8_t)0x40, -1, -1, +1, -1, 0, (int8_t)0x22,
     -1, -1, +1, +0, 0, (int8_t)0x33, -1, -1, +1, +1, 1, (int8_t)0x11,
     -1, +0, -1, +2, 0, (int8_t)0x08, -1, +0, +0, -1, 0, (int8_t)0x44,
     -1, +0, +0, +1, 0, (int8_t)0x11, -1, +0, +1, -2, 1, (int8_t)0x40,
     -1, +0, +1, -1, 0, (int8_t)0x66, -1, +0, +1, +0, 1, (int8_t)0x22,
     -1, +0, +1, +1, 0, (int8_t)0x33, -1, +0, +1, +2, 1, (int8_t)0x10,
     -1, +1, +1, -1, 1, (int8_t)0x44, -1, +1, +1, +0, 0, (int8_t)0x66,
     -1, +1, +1, +1, 0, (int8_t)0x22, -1, +1, +1, +2, 0, (int8_t)0x10,
     -1, +2, +0, +1, 0, (int8_t)0x04, -1, +2, +1, +0, 1, (int8_t)0x04,
     -1, +2, +1, +1, 0, (int8_t)0x04, +0, -2, +0, +0, 1, (int8_t)0x80,
     +0, -1, +0, +1, 1, (int8_t)0x88, +0, -1, +1, -2, 0, (int8_t)0x40,
     +0, -1, +1, +0, 0, (int8_t)0x11, +0, -1, +2, -2, 0, (int8_t)0x40,
     +0, -1, +2, -1, 0, (int8_t)0x20, +0, -1, +2, +0, 0, (int8_t)0x30,
     +0, -1, +2, +1, 1, (int8_t)0x10, +0, +0, +0, +2, 1, (int8_t)0x08,
     +0, +0, +2, -2, 1, (int8_t)0x40, +0, +0, +2, -1, 0, (int8_t)0x60,
     +0, +0, +2, +0, 1, (int8_t)0x20, +0, +0, +2, +1, 0, (int8_t)0x30,
     +0, +0, +2, +2, 1, (int8_t)0x10, +0, +1, +1, +0, 0, (int8_t)0x44,
     +0, +1, +1, +2, 0, (int8_t)0x10, +0, +1, +2, -1, 1, (int8_t)0x40,
     +0, +1, +2, +0, 0, (int8_t)0x60, +0, +1, +2, +1, 0, (int8_t)0x20,
     +0, +1, +2, +2, 0, (int8_t)0x10, +1, -2, +1, +0, 0, (int8_t)0x80,
     +1, -1, +1, +1, 0, (int8_t)0x88, +1, +0, +1, +2, 0, (int8_t)0x08,
     +1, +0, +2, -1, 0, (int8_t)0x40, +1, +0, +2, +1, 0, (int8_t)0x10},
                    bayervng_chood[] = {-1, -1, -1, 0, -1, +1, 0, +1,
                                        +1, +1, +1, 0, +1, -1, 0, -1};

void bayer_VNG_uint32(const uint32_t *bayer, uint32_t *rgb, int sx, int sy,
                      std::string tile = "BG", int bits = 32) {
  const int height = sy, width = sx;
  static const signed char *cp;
  /* the following has the same type as the image */
  uint32_t(*brow[5])[3], *pix; /* [FD] */
  int code[8][2][320], *ip, gval[8], gmin, gmax, sum[4];
  int row, col, x, y, x1, x2, y1, y2, t, weight, grads, color, diag;
  int g, diff, thold, num, c;
  uint32_t filters; /* [FD] */

  /* first, use bilinear bayer decoding */

  bayer_Bilinear_uint32(bayer, rgb, sx, sy, tile);

  if (tile == "BG")
    filters = 0x16161616;
  else if (tile == "GR")
    filters = 0x61616161;
  else if (tile == "RG")
    filters = 0x94949494;
  else if (tile == "GB")
    filters = 0x49494949;
  else
    filters = 0x16161616; // should not reach here

  for (row = 0; row < 8; row++) { /* Precalculate for VNG */
    for (col = 0; col < 2; col++) {
      ip = code[row][col];
      for (cp = bayervng_terms, t = 0; t < 64; t++) {
        y1 = *cp++;
        x1 = *cp++;
        y2 = *cp++;
        x2 = *cp++;
        weight = *cp++;
        grads = *cp++;
        color = FC(row + y1, col + x1);
        if (FC(row + y2, col + x2) != color)
          continue;
        diag = (FC(row, col + 1) == color && FC(row + 1, col) == color) ? 2 : 1;
        if (abs(y1 - y2) == diag && abs(x1 - x2) == diag)
          continue;
        *ip++ = (y1 * width + x1) * 3 + color; /* [FD] */
        *ip++ = (y2 * width + x2) * 3 + color; /* [FD] */
        *ip++ = weight;
        for (g = 0; g < 8; g++)
          if (grads & 1 << g)
            *ip++ = g;
        *ip++ = -1;
      }
      *ip++ = INT_MAX;
      for (cp = bayervng_chood, g = 0; g < 8; g++) {
        y = *cp++;
        x = *cp++;
        *ip++ = (y * width + x) * 3; /* [FD] */
        color = FC(row, col);
        if (FC(row + y, col + x) != color &&
            FC(row + y * 2, col + x * 2) == color)
          *ip++ = (y * width + x) * 6 + color; /* [FD] */
        else
          *ip++ = 0;
      }
    }
  }
  brow[4] = (uint32_t(*)[3])calloc((size_t)width * 3, sizeof **brow);
  // merror (brow[4], "vng_interpolate()");
  for (row = 0; row < 3; row++)
    brow[row] = brow[4] + row * width;
  for (row = 2; row < height - 2; row++) { /* Do VNG interpolation */
    for (col = 2; col < width - 2; col++) {
      pix = rgb + (row * width + col) * 3; /* [FD] */
      ip = code[row & 7][col & 1];
      memset(gval, 0, sizeof gval);
      while ((g = ip[0]) != INT_MAX) { /* Calculate gradients */
        diff = ABS(pix[g] - pix[ip[1]]) << ip[2];
        gval[ip[3]] += diff;
        ip += 5;
        if ((g = ip[-1]) == -1)
          continue;
        gval[g] += diff;
        while ((g = *ip++) != -1)
          gval[g] += diff;
      }
      ip++;
      gmin = gmax = gval[0]; /* Choose a threshold */
      for (g = 1; g < 8; g++) {
        if (gmin > gval[g])
          gmin = gval[g];
        if (gmax < gval[g])
          gmax = gval[g];
      }
      if (gmax == 0) {
        memcpy(brow[2][col], pix, 3 * sizeof *rgb); /* [FD] */
        continue;
      }
      thold = gmin + (gmax >> 1);
      memset(sum, 0, sizeof sum);
      color = FC(row, col);
      for (num = g = 0; g < 8; g++, ip += 2) { /* Average the neighbors */
        if (gval[g] <= thold) {
          for (c = 0; c < 3; c++) /* [FD] */
            if (c == color && ip[1])
              sum[c] += (pix[c] + pix[ip[1]]) >> 1;
            else
              sum[c] += pix[ip[0] + c];
          num++;
        }
      }
      for (c = 0; c < 3; c++) { /* [FD] Save to buffer */
        t = pix[color];
        if (c != color)
          t += (sum[c] - sum[color]) / num;
        CLIP32(t, brow[2][col][c], bits); /* [FD] */
      }
    }
    if (row > 3) /* Write buffer to image */
      memcpy(rgb + 3 * ((row - 2) * width + 2), brow[0] + 2,
             (width - 4) * 3 * sizeof *rgb); /* [FD] */
    for (g = 0; g < 4; g++)
      brow[(g - 1) & 3] = brow[g];
  }
  memcpy(rgb + 3 * ((row - 2) * width + 2), brow[0] + 2,
         (width - 4) * 3 * sizeof *rgb);
  memcpy(rgb + 3 * ((row - 1) * width + 2), brow[1] + 2,
         (width - 4) * 3 * sizeof *rgb);
  free(brow[4]);

  return;
}

void bayer_HQLinear_uint32(const uint32_t *bayer, uint32_t *rgb, int sx, int sy,
                           std::string tile = "BG", int bits = 32) {
  const int bayerStep = sx;
  const int rgbStep = 3 * sx;
  int width = sx;
  int height = sy;
  /*
     the two letters  of the OpenCV name are respectively
     the 4th and 3rd letters from the blinky name,
     and we also have to switch R and B (OpenCV is BGR)

     CV_BayerBG2BGR <-> DC1394_COLOR_FILTER_BGGR
     CV_BayerGB2BGR <-> DC1394_COLOR_FILTER_GBRG
     CV_BayerGR2BGR <-> DC1394_COLOR_FILTER_GRBG

     int blue = tile == CV_BayerBG2BGR || tile == CV_BayerGB2BGR ? -1 : 1;
     int start_with_green = tile == CV_BayerGB2BGR || tile == CV_BayerGR2BGR;
   */
  int blue = tile == "BG" || tile == "GB" ? -1 : 1;
  int start_with_green = tile == "GB" || tile == "GR";

  ClearBorders_uint32(rgb, sx, sy, 2);
  rgb += 2 * rgbStep + 6 + 1;
  height -= 4;
  width -= 4;

  /* We begin with a (+1 line,+1 column) offset with respect to bilinear
   * decoding, so start_with_green is the same, but blue is opposite */
  blue = -blue;

  for (; height--; bayer += bayerStep, rgb += rgbStep) {
    int t0, t1;
    const uint32_t *bayerEnd = bayer + width;
    const int bayerStep2 = bayerStep * 2;
    const int bayerStep3 = bayerStep * 3;
    const int bayerStep4 = bayerStep * 4;

    if (start_with_green) {
      /* at green pixel */
      rgb[0] = bayer[bayerStep2 + 2];
      t0 = rgb[0] * 5 + ((bayer[bayerStep + 2] + bayer[bayerStep3 + 2]) << 2) -
           bayer[2] - bayer[bayerStep + 1] - bayer[bayerStep + 3] -
           bayer[bayerStep3 + 1] - bayer[bayerStep3 + 3] -
           bayer[bayerStep4 + 2] +
           ((bayer[bayerStep2] + bayer[bayerStep2 + 4] + 1) >> 1);
      t1 = rgb[0] * 5 + ((bayer[bayerStep2 + 1] + bayer[bayerStep2 + 3]) << 2) -
           bayer[bayerStep2] - bayer[bayerStep + 1] - bayer[bayerStep + 3] -
           bayer[bayerStep3 + 1] - bayer[bayerStep3 + 3] -
           bayer[bayerStep2 + 4] +
           ((bayer[2] + bayer[bayerStep4 + 2] + 1) >> 1);
      t0 = (t0 + 4) >> 3;
      CLIP32(t0, rgb[-blue], bits);
      t1 = (t1 + 4) >> 3;
      CLIP32(t1, rgb[blue], bits);
      bayer++;
      rgb += 3;
    }

    if (blue > 0) {
      for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
        /* B at B */
        rgb[1] = bayer[bayerStep2 + 2];
        /* R at B */
        t0 = ((bayer[bayerStep + 1] + bayer[bayerStep + 3] +
               bayer[bayerStep3 + 1] + bayer[bayerStep3 + 3])
              << 1) -
             (((bayer[2] + bayer[bayerStep2] + bayer[bayerStep2 + 4] +
                bayer[bayerStep4 + 2]) *
                   3 +
               1) >>
              1) +
             rgb[1] * 6;
        /* G at B */
        t1 = ((bayer[bayerStep + 2] + bayer[bayerStep2 + 1] +
               bayer[bayerStep2 + 3] + bayer[bayerStep * 3 + 2])
              << 1) -
             (bayer[2] + bayer[bayerStep2] + bayer[bayerStep2 + 4] +
              bayer[bayerStep4 + 2]) +
             (rgb[1] << 2);
        t0 = (t0 + 4) >> 3;
        CLIP32(t0, rgb[-1], bits);
        t1 = (t1 + 4) >> 3;
        CLIP32(t1, rgb[0], bits);
        /* at green pixel */
        rgb[3] = bayer[bayerStep2 + 3];
        t0 = rgb[3] * 5 +
             ((bayer[bayerStep + 3] + bayer[bayerStep3 + 3]) << 2) - bayer[3] -
             bayer[bayerStep + 2] - bayer[bayerStep + 4] -
             bayer[bayerStep3 + 2] - bayer[bayerStep3 + 4] -
             bayer[bayerStep4 + 3] +
             ((bayer[bayerStep2 + 1] + bayer[bayerStep2 + 5] + 1) >> 1);
        t1 = rgb[3] * 5 +
             ((bayer[bayerStep2 + 2] + bayer[bayerStep2 + 4]) << 2) -
             bayer[bayerStep2 + 1] - bayer[bayerStep + 2] -
             bayer[bayerStep + 4] - bayer[bayerStep3 + 2] -
             bayer[bayerStep3 + 4] - bayer[bayerStep2 + 5] +
             ((bayer[3] + bayer[bayerStep4 + 3] + 1) >> 1);
        t0 = (t0 + 4) >> 3;
        CLIP32(t0, rgb[2], bits);
        t1 = (t1 + 4) >> 3;
        CLIP32(t1, rgb[4], bits);
      }
    } else {
      for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
        /* R at R */
        rgb[-1] = bayer[bayerStep2 + 2];
        /* B at R */
        t0 = ((bayer[bayerStep + 1] + bayer[bayerStep + 3] +
               bayer[bayerStep * 3 + 1] + bayer[bayerStep3 + 3])
              << 1) -
             (((bayer[2] + bayer[bayerStep2] + bayer[bayerStep2 + 4] +
                bayer[bayerStep4 + 2]) *
                   3 +
               1) >>
              1) +
             rgb[-1] * 6;
        /* G at R */
        t1 = ((bayer[bayerStep + 2] + bayer[bayerStep2 + 1] +
               bayer[bayerStep2 + 3] + bayer[bayerStep3 + 2])
              << 1) -
             (bayer[2] + bayer[bayerStep2] + bayer[bayerStep2 + 4] +
              bayer[bayerStep4 + 2]) +
             (rgb[-1] << 2);
        t0 = (t0 + 4) >> 3;
        CLIP32(t0, rgb[1], bits);
        t1 = (t1 + 4) >> 3;
        CLIP32(t1, rgb[0], bits);

        /* at green pixel */
        rgb[3] = bayer[bayerStep2 + 3];
        t0 = rgb[3] * 5 +
             ((bayer[bayerStep + 3] + bayer[bayerStep3 + 3]) << 2) - bayer[3] -
             bayer[bayerStep + 2] - bayer[bayerStep + 4] -
             bayer[bayerStep3 + 2] - bayer[bayerStep3 + 4] -
             bayer[bayerStep4 + 3] +
             ((bayer[bayerStep2 + 1] + bayer[bayerStep2 + 5] + 1) >> 1);
        t1 = rgb[3] * 5 +
             ((bayer[bayerStep2 + 2] + bayer[bayerStep2 + 4]) << 2) -
             bayer[bayerStep2 + 1] - bayer[bayerStep + 2] -
             bayer[bayerStep + 4] - bayer[bayerStep3 + 2] -
             bayer[bayerStep3 + 4] - bayer[bayerStep2 + 5] +
             ((bayer[3] + bayer[bayerStep4 + 3] + 1) >> 1);
        t0 = (t0 + 4) >> 3;
        CLIP32(t0, rgb[4], bits);
        t1 = (t1 + 4) >> 3;
        CLIP32(t1, rgb[2], bits);
      }
    }

    if (bayer < bayerEnd) {
      /* B at B */
      rgb[blue] = bayer[bayerStep2 + 2];
      /* R at B */
      t0 = ((bayer[bayerStep + 1] + bayer[bayerStep + 3] +
             bayer[bayerStep3 + 1] + bayer[bayerStep3 + 3])
            << 1) -
           (((bayer[2] + bayer[bayerStep2] + bayer[bayerStep2 + 4] +
              bayer[bayerStep4 + 2]) *
                 3 +
             1) >>
            1) +
           rgb[blue] * 6;
      /* G at B */
      t1 = (((bayer[bayerStep + 2] + bayer[bayerStep2 + 1] +
              bayer[bayerStep2 + 3] + bayer[bayerStep3 + 2]))
            << 1) -
           (bayer[2] + bayer[bayerStep2] + bayer[bayerStep2 + 4] +
            bayer[bayerStep4 + 2]) +
           (rgb[blue] << 2);
      t0 = (t0 + 4) >> 3;
      CLIP32(t0, rgb[-blue], bits);
      t1 = (t1 + 4) >> 3;
      CLIP32(t1, rgb[0], bits);
      bayer++;
      rgb += 3;
    }

    bayer -= width;
    rgb -= width * 3;

    blue = -blue;
    start_with_green = !start_with_green;
  }

  return;
}

void bayer_EdgeSense_uint32(const uint32_t *bayer, uint32_t *rgb, int sx,
                            int sy, std::string tile = "BG", int bits = 32) {
  uint32_t *outR, *outG, *outB;
  int i3, j3, base;
  int i, j;
  int dh, dv;
  int tmp;
  int sx3 = sx * 3;

  // sx and sy should be even
  if (tile == "GR" or tile == "BG") {
    outR = &rgb[0];
    outG = &rgb[1];
    outB = &rgb[2];
  } else if (tile == "GB" or tile == "RG") {
    outR = &rgb[2];
    outG = &rgb[1];
    outB = &rgb[0];
  }

  if (tile == "GR" or tile == "GB") {
    // copy original RGB data to output images
    for (i = 0, i3 = 0; i < sy * sx; i += (sx << 1), i3 += (sx3 << 1)) {
      for (j = 0, j3 = 0; j < sx; j += 2, j3 += 6) {
        base = i3 + j3;
        outG[base] = bayer[i + j];
        outG[base + sx3 + 3] = bayer[i + j + sx + 1];
        outR[base + 3] = bayer[i + j + 1];
        outB[base + sx3] = bayer[i + j + sx];
      }
    }
    // process GREEN channel
    for (i3 = 3 * sx3; i3 < (sy - 2) * sx3; i3 += (sx3 << 1)) {
      for (j3 = 6; j3 < sx3 - 9; j3 += 6) {
        base = i3 + j3;
        dh = abs<long>(((outB[base - 6] + outB[base + 6]) >> 1) - outB[base]);
        dv = abs<long>(
            ((outB[base - (sx3 << 1)] + outB[base + (sx3 << 1)]) >> 1) -
            outB[base]);
        tmp = (((outG[base - 3] + outG[base + 3]) >> 1) * (dh <= dv) +
               ((outG[base - sx3] + outG[base + sx3]) >> 1) * (dh > dv));
        // tmp = (dh==dv) ? tmp>>1 : tmp;
        CLIP32(tmp, outG[base], bits);
      }
    }

    for (i3 = 2 * sx3; i3 < (sy - 3) * sx3; i3 += (sx3 << 1)) {
      for (j3 = 9; j3 < sx3 - 6; j3 += 6) {
        base = i3 + j3;
        dh = abs<long>(((outR[base - 6] + outR[base + 6]) >> 1) - outR[base]);
        dv = abs<long>(
            ((outR[base - (sx3 << 1)] + outR[base + (sx3 << 1)]) >> 1) -
            outR[base]);
        tmp = (((outG[base - 3] + outG[base + 3]) >> 1) * (dh <= dv) +
               ((outG[base - sx3] + outG[base + sx3]) >> 1) * (dh > dv));
        // tmp = (dh==dv) ? tmp>>1 : tmp;
        CLIP32(tmp, outG[base], bits);
      }
    }
    // process RED channel
    for (i3 = 0; i3 < (sy - 1) * sx3; i3 += (sx3 << 1)) {
      for (j3 = 6; j3 < sx3 - 3; j3 += 6) {
        base = i3 + j3;
        tmp = outG[base] + ((outR[base - 3] - outG[base - 3] + outR[base + 3] -
                             outG[base + 3]) >>
                            1);
        CLIP32(tmp, outR[base], bits);
      }
    }
    for (i3 = sx3; i3 < (sy - 2) * sx3; i3 += (sx3 << 1)) {
      for (j3 = 3; j3 < sx3; j3 += 6) {
        base = i3 + j3;
        tmp = outG[base] + ((outR[base - sx3] - outG[base - sx3] +
                             outR[base + sx3] - outG[base + sx3]) >>
                            1);
        CLIP32(tmp, outR[base], bits);
      }
      for (j3 = 6; j3 < sx3 - 3; j3 += 6) {
        base = i3 + j3;
        tmp = outG[base] + ((outR[base - sx3 - 3] - outG[base - sx3 - 3] +
                             outR[base - sx3 + 3] - outG[base - sx3 + 3] +
                             outR[base + sx3 - 3] - outG[base + sx3 - 3] +
                             outR[base + sx3 + 3] - outG[base + sx3 + 3]) >>
                            2);
        CLIP32(tmp, outR[base], bits);
      }
    }

    // process BLUE channel
    for (i3 = sx3; i3 < sy * sx3; i3 += (sx3 << 1)) {
      for (j3 = 3; j3 < sx3 - 6; j3 += 6) {
        base = i3 + j3;
        tmp = outG[base] + ((outB[base - 3] - outG[base - 3] + outB[base + 3] -
                             outG[base + 3]) >>
                            1);
        CLIP32(tmp, outB[base], bits);
      }
    }
    for (i3 = 2 * sx3; i3 < (sy - 1) * sx3; i3 += (sx3 << 1)) {
      for (j3 = 0; j3 < sx3 - 3; j3 += 6) {
        base = i3 + j3;
        tmp = outG[base] + ((outB[base - sx3] - outG[base - sx3] +
                             outB[base + sx3] - outG[base + sx3]) >>
                            1);
        CLIP32(tmp, outB[base], bits);
      }
      for (j3 = 3; j3 < sx3 - 6; j3 += 6) {
        base = i3 + j3;
        tmp = outG[base] + ((outB[base - sx3 - 3] - outG[base - sx3 - 3] +
                             outB[base - sx3 + 3] - outG[base - sx3 + 3] +
                             outB[base + sx3 - 3] - outG[base + sx3 - 3] +
                             outB[base + sx3 + 3] - outG[base + sx3 + 3]) >>
                            2);
        CLIP32(tmp, outB[base], bits);
      }
    }
  } else if (tile == "BG" or tile == "RG") {
    // copy original RGB data to output images
    for (i = 0, i3 = 0; i < sy * sx; i += (sx << 1), i3 += (sx3 << 1)) {
      for (j = 0, j3 = 0; j < sx; j += 2, j3 += 6) {
        base = i3 + j3;
        outB[base] = bayer[i + j];
        outR[base + sx3 + 3] = bayer[i + sx + (j + 1)];
        outG[base + 3] = bayer[i + j + 1];
        outG[base + sx3] = bayer[i + sx + j];
      }
    }
    // process GREEN channel
    for (i3 = 2 * sx3; i3 < (sy - 2) * sx3; i3 += (sx3 << 1)) {
      for (j3 = 6; j3 < sx3 - 9; j3 += 6) {
        base = i3 + j3;
        dh = abs<long>(((outB[base - 6] + outB[base + 6]) >> 1) - outB[base]);
        dv = abs<long>(
            ((outB[base - (sx3 << 1)] + outB[base + (sx3 << 1)]) >> 1) -
            outB[base]);
        tmp = (((outG[base - 3] + outG[base + 3]) >> 1) * (dh <= dv) +
               ((outG[base - sx3] + outG[base + sx3]) >> 1) * (dh > dv));
        // tmp = (dh==dv) ? tmp>>1 : tmp;
        CLIP32(tmp, outG[base], bits);
      }
    }
    for (i3 = 3 * sx3; i3 < (sy - 3) * sx3; i3 += (sx3 << 1)) {
      for (j3 = 9; j3 < sx3 - 6; j3 += 6) {
        base = i3 + j3;
        dh = abs<long>(((outR[base - 6] + outR[base + 6]) >> 1) - outR[base]);
        dv = abs<long>(
            ((outR[base - (sx3 << 1)] + outR[base + (sx3 << 1)]) >> 1) -
            outR[base]);
        tmp = (((outG[base - 3] + outG[base + 3]) >> 1) * (dh <= dv) +
               ((outG[base - sx3] + outG[base + sx3]) >> 1) * (dh > dv));
        // tmp = (dh==dv) ? tmp>>1 : tmp;
        CLIP32(tmp, outG[base], bits);
      }
    }
    // process RED channel
    for (i3 = sx3; i3 < (sy - 1) * sx3; i3 += (sx3 << 1)) { // G-points (1/2)
      for (j3 = 6; j3 < sx3 - 3; j3 += 6) {
        base = i3 + j3;
        tmp = outG[base] + ((outR[base - 3] - outG[base - 3] + outR[base + 3] -
                             outG[base + 3]) >>
                            1);
        CLIP32(tmp, outR[base], bits);
      }
    }
    for (i3 = 2 * sx3; i3 < (sy - 2) * sx3; i3 += (sx3 << 1)) {
      for (j3 = 3; j3 < sx3; j3 += 6) { // G-points (2/2)
        base = i3 + j3;
        tmp = outG[base] + ((outR[base - sx3] - outG[base - sx3] +
                             outR[base + sx3] - outG[base + sx3]) >>
                            1);
        CLIP32(tmp, outR[base], bits);
      }
      for (j3 = 6; j3 < sx3 - 3; j3 += 6) { // B-points
        base = i3 + j3;
        tmp = outG[base] + ((outR[base - sx3 - 3] - outG[base - sx3 - 3] +
                             outR[base - sx3 + 3] - outG[base - sx3 + 3] +
                             outR[base + sx3 - 3] - outG[base + sx3 - 3] +
                             outR[base + sx3 + 3] - outG[base + sx3 + 3]) >>
                            2);
        CLIP32(tmp, outR[base], bits);
      }
    }

    // process BLUE channel
    for (i = 0, i3 = 0; i < sy * sx; i += (sx << 1), i3 += (sx3 << 1)) {
      for (j = 1, j3 = 3; j < sx - 2; j += 2, j3 += 6) {
        base = i3 + j3;
        tmp = outG[base] + ((outB[base - 3] - outG[base - 3] + outB[base + 3] -
                             outG[base + 3]) >>
                            1);
        CLIP32(tmp, outB[base], bits);
      }
    }
    for (i3 = sx3; i3 < (sy - 1) * sx3; i3 += (sx3 << 1)) {
      for (j3 = 0; j3 < sx3 - 3; j3 += 6) {
        base = i3 + j3;
        tmp = outG[base] + ((outB[base - sx3] - outG[base - sx3] +
                             outB[base + sx3] - outG[base + sx3]) >>
                            1);
        CLIP32(tmp, outB[base], bits);
      }
      for (j3 = 3; j3 < sx3 - 6; j3 += 6) {
        base = i3 + j3;
        tmp = outG[base] + ((outB[base - sx3 - 3] - outG[base - sx3 - 3] +
                             outB[base - sx3 + 3] - outG[base - sx3 + 3] +
                             outB[base + sx3 - 3] - outG[base + sx3 - 3] +
                             outB[base + sx3 + 3] - outG[base + sx3 + 3]) >>
                            2);
        CLIP32(tmp, outB[base], bits);
      }
    }
  }

  ClearBorders_uint32(rgb, sx, sy, 3);

  return;
}

// blocking queue
template <typename T> class BlockingQueue {
public:
  BlockingQueue(const BlockingQueue &) = delete; // make the class noncopyable
  BlockingQueue &
  operator=(const BlockingQueue &) = delete; // make the class noncopyable
  BlockingQueue(int queue_limit) { m_limit = queue_limit; }
  void push(const T &val) {
    std::unique_lock<std::mutex> lock(m_mutex);
    m_queue.push(val);
    while (m_queue.size() > m_limit)
      m_queue.pop();
    m_cond.notify_one();
  }
  void pop(T &val) {
    std::unique_lock<std::mutex> lock(m_mutex);
    while (m_queue.empty()) {
      m_cond.wait(lock);
    }
    val = m_queue.front();
    m_queue.pop();
  }
  bool try_pop(T &val) {
    std::unique_lock<std::mutex> lock(m_mutex);
    if (!m_queue.empty()) {
      val = m_queue.front();
      m_queue.pop();
      return true;
    }
    return false;
  }
  size_t size() const {
    std::unique_lock<std::mutex> lock(m_mutex);
    return m_queue.size();
  }
  bool empty() const {
    std::unique_lock<std::mutex> lock(m_mutex);
    return m_queue.empty();
  }
  bool full() const {
    std::unique_lock<std::mutex> lock(m_mutex);
    return m_queue.size() >= m_limit;
  }

private:
  std::queue<T> m_queue;
  unsigned int m_limit;
  std::condition_variable m_cond;
  mutable std::mutex m_mutex;
};

struct HdrImage {
  uint32_t seq;
  rclcpp::Time stamp;
  cv::Mat img;
};

// main node

cv::Mat LoadBayerHdrImageIntoOpenCV(Arena::IImage *pImage, size_t imgWidth,
                                    size_t imgHeight, size_t bytesPerPixel,
                                    std::string tile, int debayer_method) {
  const uint8_t *input_data;
  size_t n_pixels = imgWidth * imgHeight;

  input_data = pImage->GetData();
  uint32_t *bayer_data = new uint32_t[imgWidth * imgHeight];

  // std::cout << "Unpack 3 bytes into 1 output pixel" << std::endl;
  for (size_t i = 0; i < n_pixels; i++) {
    if (bytesPerPixel == 3) {
      // Unpack 3 bytes into 1 output pixel
      bayer_data[i] = ((uint32_t)input_data[0]) +
                      (((uint32_t)input_data[1]) << 8) +
                      (((uint32_t)input_data[2]) << 16);
      input_data += 3;
    } else if (bytesPerPixel == 2) {
      bayer_data[i] =
          ((uint32_t)input_data[0]) + (((uint32_t)input_data[1]) << 8);
      input_data += 2;
    } else if (bytesPerPixel == 1) {
      bayer_data[i] = ((uint32_t)input_data[0]);
      input_data += 3;
    }
  }

  uint32_t *rgb_data = new uint32_t[imgWidth * imgHeight * 3];
  if (debayer_method == 0)
    bayer_Bilinear_uint32(bayer_data, rgb_data, static_cast<int>(imgWidth),
                          static_cast<int>(imgHeight), tile, bytesPerPixel * 8);
  else if (debayer_method == 1)
    bayer_HQLinear_uint32(bayer_data, rgb_data, static_cast<int>(imgWidth),
                          static_cast<int>(imgHeight), tile, bytesPerPixel * 8);

  cv::Mat image_s32_rgb(static_cast<int>(imgHeight), static_cast<int>(imgWidth),
                        CV_32SC3);
  image_s32_rgb.data = (uchar *)rgb_data;

  if (debayer_method == 0) {
    // bilinear debayer leaves one-pixel borders undecoded
    // need to copy border pixels from neighboring pixels
    image_s32_rgb.col(1).copyTo(image_s32_rgb.col(0));
    image_s32_rgb.col(imgWidth - 2).copyTo(image_s32_rgb.col(imgWidth - 1));
    image_s32_rgb.row(1).copyTo(image_s32_rgb.row(0));
    image_s32_rgb.row(imgHeight - 2).copyTo(image_s32_rgb.row(imgHeight - 1));
  } else if (debayer_method == 1) {
    // hdlinear debayer leaves two-pixel borders undecoded
    image_s32_rgb.col(2).copyTo(image_s32_rgb.col(1));
    image_s32_rgb.col(2).copyTo(image_s32_rgb.col(0));
    image_s32_rgb.col(imgWidth - 3).copyTo(image_s32_rgb.col(imgWidth - 2));
    image_s32_rgb.col(imgWidth - 3).copyTo(image_s32_rgb.col(imgWidth - 1));
    image_s32_rgb.row(2).copyTo(image_s32_rgb.row(1));
    image_s32_rgb.row(2).copyTo(image_s32_rgb.row(0));
    image_s32_rgb.row(imgHeight - 3).copyTo(image_s32_rgb.row(imgHeight - 2));
    image_s32_rgb.row(imgHeight - 3).copyTo(image_s32_rgb.row(imgHeight - 1));
  }

  // Convert into 32-bit float for OpenCV
  cv::Mat image_f32_rgb(static_cast<int>(imgHeight), static_cast<int>(imgWidth),
                        CV_32FC3, cv::Scalar::all(0));
  image_s32_rgb.convertTo(image_f32_rgb, CV_32FC3);

  image_s32_rgb.release();
  delete[] bayer_data;
  delete[] rgb_data;

  return image_f32_rgb;
}

cv::Mat LoadRgbImageIntoOpenCV(Arena::IImage *pImage, size_t imgWidth,
                               size_t imgHeight) {
  cv::Mat image_u8 =
      cv::Mat(static_cast<int>(imgHeight), static_cast<int>(imgWidth), CV_8UC3,
              (void *)pImage->GetData());

  return image_u8;
}

cv::Mat ProcessImageInOpenCV(cv::Mat input, int tonemap_method = 1,
                             float gamma = 1.0f) {
  cv::Mat tonemap_output;

  std::string newfilename;

  switch (tonemap_method) {
  case 0: {
    cv::Ptr<cv::Tonemap> tonemap = cv::createTonemap(gamma);
    tonemap->process(input, tonemap_output);
    break;
  }
  case 1: {
    cv::Ptr<cv::TonemapDrago> tonemap_drago = cv::createTonemapDrago(gamma);
    tonemap_drago->process(input, tonemap_output);
    break;
  }
#if defined __aarch64__
  case 2: // dusty-nv container has opencv built with nonfree option enabled
  {
    cv::Ptr<cv::xphoto::TonemapDurand> tonemap_durand =
        cv::xphoto::createTonemapDurand(gamma);
    tonemap_durand->process(input, tonemap_output);
    break;
  }
#endif
  case 3: // very slow
  {
    cv::Ptr<cv::TonemapMantiuk> tonemap_mantiuk = cv::createTonemapMantiuk();
    tonemap_mantiuk->process(input, tonemap_output);
    break;
  }
  case 4: {
    cv::Ptr<cv::TonemapReinhard> tonemap_reinhard = cv::createTonemapReinhard();
    tonemap_reinhard->process(input, tonemap_output);
    break;
  }
  }

  return tonemap_output;
}

class CamStreamer : public ::rclcpp::Node {
public:
  explicit CamStreamer(rclcpp::NodeOptions node_options);
  ~CamStreamer(void);

private:
  int obtain_meta(GenICam::gcstring, double &, double &, size_t &,
                  cv::ColorConversionCodes &, std::string &);
  int configure_camera(void);
  int connect_camera(void);
  int disconnect_camera(void);

  void TriggerImages(void);
  void AcquireImages(void);
  void ProcessImages(void);
  void PublishImages(void);

  void print_diagnosis(void);
  void publish_image(rclcpp::Time stamp, const cv::Mat &image,
                     std::string encoding = sensor_msgs::image_encodings::BGR8);
  void init_camera_info(std::string camera_name, std::string camera_info_url);

  std::shared_ptr<image_transport::CameraPublisher> m_p_camera_publisher{};
  std::shared_ptr<camera_info_manager::CameraInfoManager> m_p_camera_info{};

  std::unique_ptr<BlockingQueue<HdrImage>> m_p_hdr_image_queue;
  std::unique_ptr<BlockingQueue<HdrImage>> m_p_processed_image_queue;

  std::unique_ptr<std::thread> m_p_image_trigger;
  std::unique_ptr<std::thread> m_p_image_producer;
  std::unique_ptr<std::thread> m_p_image_consumer;
  std::unique_ptr<std::thread> m_p_image_publisher;

  int m_param_timeout_ms; // m_param_timeout_ms for grabbing images (in
                          // milliseconds)
  std::string m_param_pixel_format; // pixel format
  int m_param_tonemap_method;       // tonemap
  float m_param_tonemap_gamma;   // tonemap parameters for the OpenCV controls,
                                 // not used for the moment
  int m_param_scaled_bit_offset; // scaled image bit offset from 24 bit
  int m_param_trigger_mode; // 0: free running, 1: hardware triggering, 2: PTP
                            // Scheduled Action, 3: PTP Sync
  int m_param_frame_rate; // only for free running, PTP triggering, and PTP Sync
                          // modes
  int m_param_exposure_time; // exposure time
  int m_param_debayer_method;
  bool m_param_use_image_crop;
  int m_param_image_width;
  int m_param_image_height;
  int m_param_image_offset_x;
  int m_param_image_offset_y;
  bool m_param_use_max_packet_size; // whether to use max packet size
  bool m_param_use_binning;
  bool m_param_flip_horizontal; // flip
  bool m_param_flip_vertical;   // flip
  bool m_param_use_sys_default_qos;
  bool m_param_use_ptp_time;
  bool m_param_viz; // if to show opencv image

  std::string m_param_camera_ns;       // camera namespace
  int m_param_camera_serial;           // camera serial number
  std::string m_param_camera_info_url; // camera info url
  // image_geometry::PinholeCameraModel m_param_camera_model;
  std::string m_param_frame_id;

  Arena::ISystem *m_pSystem;
  Arena::IDevice *m_pDevice;
  bool m_isStreaming;
};

int CamStreamer::configure_camera(void) {
  if (!m_pDevice) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Camera not connected yet");
    return -1;
  }

  GenICam::gcstring modelName = Arena::GetNodeValue<GenICam::gcstring>(
      m_pDevice->GetNodeMap(), "DeviceModelName");
  RCLCPP_INFO_STREAM(this->get_logger(), "Camera Model Name: " << modelName);

  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Set pixel format = " << m_param_pixel_format);
  Arena::SetNodeValue<GenICam::gcstring>(
      m_pDevice->GetNodeMap(), "PixelFormat",
      static_cast<GenICam::gcstring>(m_param_pixel_format.c_str()));
  GenICam::gcstring pixel_format_readback =
      Arena::GetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(),
                                             "PixelFormat");
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Confirming pixel format: " << pixel_format_readback);

  Arena::SetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(),
                                         "AcquisitionStartMode", "Normal");

  // debugging
  if (m_param_use_binning) {
    // Set binning mode
    //    Sets binning mode to sensor, so that processing is done before
    //    transport to software.
    RCLCPP_INFO_STREAM(this->get_logger(), "Set binning mode to Sensor");
    Arena::SetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(),
                                           "BinningSelector", "Sensor");
    Arena::SetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(),
                                           "BinningVerticalMode", "Average");
    Arena::SetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(),
                                           "BinningHorizontalMode", "Average");

    // Set BinningHorizontal and BinningVertical to their maxes.
    //    This sets width and height of the bins: the number of pixels along
    //    each axis.
    int binWidth = 2;
    int binHeight = 2;
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Set binning horizontal and vertical to "
                           << binWidth << " and " << binHeight
                           << " respectively");
    Arena::SetNodeValue<int64_t>(m_pDevice->GetNodeMap(), "BinningVertical",
                                 binHeight);
    Arena::SetNodeValue<int64_t>(m_pDevice->GetNodeMap(), "BinningHorizontal",
                                 binWidth);
  }

  if (m_param_use_image_crop) {
    Arena::SetNodeValue<int64_t>(m_pDevice->GetNodeMap(), "OffsetX",
                                 0); // reset offsets first
    Arena::SetNodeValue<int64_t>(m_pDevice->GetNodeMap(), "OffsetY", 0);
    Arena::SetNodeValue<int64_t>(m_pDevice->GetNodeMap(), "Width",
                                 m_param_image_width);
    Arena::SetNodeValue<int64_t>(m_pDevice->GetNodeMap(), "Height",
                                 m_param_image_height);
    Arena::SetNodeValue<int64_t>(m_pDevice->GetNodeMap(), "OffsetX",
                                 m_param_image_offset_x);
    Arena::SetNodeValue<int64_t>(m_pDevice->GetNodeMap(), "OffsetY",
                                 m_param_image_offset_y);
  }

  int64_t width =
      Arena::GetNodeValue<int64_t>(m_pDevice->GetNodeMap(), "Width");
  int64_t height =
      Arena::GetNodeValue<int64_t>(m_pDevice->GetNodeMap(), "Height");
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Image size (w,h) = (" << width << "," << height << ") ");

  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Set ReverseX = " << m_param_flip_horizontal);
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Set ReverseY = " << m_param_flip_vertical);
  Arena::SetNodeValue<bool>(m_pDevice->GetNodeMap(), "ReverseX",
                            m_param_flip_horizontal);
  Arena::SetNodeValue<bool>(m_pDevice->GetNodeMap(), "ReverseY",
                            m_param_flip_vertical);

  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Set ExposureTime = " << m_param_exposure_time);
  Arena::SetNodeValue<double>(m_pDevice->GetNodeMap(), "ExposureTime",
                              m_param_exposure_time);

  if (m_param_use_ptp_time or m_param_trigger_mode == 2 or
      m_param_trigger_mode == 3) {
    Arena::SetNodeValue<bool>(m_pDevice->GetNodeMap(), "PtpEnable", true);

    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Waiting for the camera to become PTP slave");
    Arena::SetNodeValue<bool>(m_pDevice->GetNodeMap(), "PtpSlaveOnly", true);
    GenICam::gcstring currPtpStatus;
    while ((currPtpStatus = Arena::GetNodeValue<GenICam::gcstring>(
                m_pDevice->GetNodeMap(), "PtpStatus")) != "Slave") {
      rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(5e3)));
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Success: " << currPtpStatus);
  }

  // Set acquisition mode
  //    Set acquisition mode before starting the stream. Starting the stream
  //    requires the acquisition mode to be set beforehand. The acquisition
  //    mode controls the number of images a device acquires once the stream
  //    has been started. Setting the acquisition mode to 'Continuous' keeps
  //    the stream from stopping. This example returns the camera to its
  //    initial acquisition mode near the end of the example.
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Set acquisition mode to 'Continuous'");
  Arena::SetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(),
                                         "AcquisitionMode", "Continuous");

  if (m_param_trigger_mode == 0) { // free running
    RCLCPP_INFO_STREAM(this->get_logger(), "Camera trigger mode: Free running");

    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Set AcquisitionFrameRate = " << m_param_frame_rate);
    Arena::SetNodeValue<bool>(m_pDevice->GetNodeMap(),
                              "AcquisitionFrameRateEnable", true);
    Arena::SetNodeValue<double>(m_pDevice->GetNodeMap(), "AcquisitionFrameRate",
                                m_param_frame_rate);

    Arena::SetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(),
                                           "TriggerMode", "Off");
  } else if (m_param_trigger_mode == 1) { // hardware triggering
    RCLCPP_INFO_STREAM(this->get_logger(), "Camera trigger mode: Hardware");
    Arena::SetNodeValue<bool>(m_pDevice->GetNodeMap(),
                              "AcquisitionFrameRateEnable", false);

    // Set trigger selector
    //    Set the trigger selector to FrameStart. When triggered, the device
    //    will start acquiring a single frame. This can also be set to
    //    AcquisitionStart or FrameBurstStart.
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Set trigger selector to FrameStart");
    Arena::SetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(),
                                           "TriggerSelector", "FrameStart");

    // Set trigger mode
    //    Enable trigger mode before setting the source and selector and before
    //    starting the stream. Trigger mode cannot be turned on and off while
    //    the device is streaming.
    RCLCPP_INFO_STREAM(this->get_logger(), "Enable trigger mode");
    Arena::SetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(),
                                           "TriggerMode", "On");
    Arena::SetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(),
                                           "TriggerSource", "Line0");
    Arena::SetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(),
                                           "TriggerActivation", "RisingEdge");

    // Set trigger overlap
    //    Trigger overlap defines when a trigger can start accepting a new
    //    frame. Setting trigger to overlap with the previous frame allows the
    //    camera to being exposing the new frame while the camera is still
    //    reading out the sensor data acquired from the previous frame.
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Set trigger overlap to PreviousFrame");
    Arena::SetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(),
                                           "TriggerOverlap", "PreviousFrame");
  } else if (m_param_trigger_mode == 2) { // PTP scheduled action triggering
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Camera trigger mode: PTP Scheduled Action");
    Arena::SetNodeValue<bool>(m_pDevice->GetNodeMap(),
                              "AcquisitionFrameRateEnable", false);

    // Set trigger selector
    //    Set the trigger selector to FrameStart. When triggered, the device
    //    will start acquiring a single frame. This can also be set to
    //    AcquisitionStart or FrameBurstStart.
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Set trigger selector to FrameStart");
    Arena::SetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(),
                                           "TriggerSelector", "FrameStart");

    // Set trigger mode
    //    Enable trigger mode before setting the source and selector and before
    //    starting the stream. Trigger mode cannot be turned on and off while
    //    the device is streaming.
    RCLCPP_INFO_STREAM(this->get_logger(), "Enable trigger mode");
    Arena::SetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(),
                                           "TriggerMode", "On");

    // Set trigger source
    Arena::SetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(),
                                           "TriggerSource", "Action0");

    // Set scheduled action
    Arena::SetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(),
                                           "ActionUnconditionalMode", "On");
    Arena::SetNodeValue<int64_t>(m_pDevice->GetNodeMap(), "ActionSelector", 0);
    Arena::SetNodeValue<int64_t>(m_pDevice->GetNodeMap(), "ActionDeviceKey",
                                 m_param_camera_serial);
    Arena::SetNodeValue<int64_t>(m_pDevice->GetNodeMap(), "ActionGroupKey", 1);
    Arena::SetNodeValue<int64_t>(m_pDevice->GetNodeMap(), "ActionGroupMask", 1);
    Arena::SetNodeValue<int64_t>(m_pSystem->GetTLSystemNodeMap(),
                                 "ActionCommandDeviceKey",
                                 m_param_camera_serial);
    Arena::SetNodeValue<int64_t>(m_pSystem->GetTLSystemNodeMap(),
                                 "ActionCommandGroupKey", 1);
    Arena::SetNodeValue<int64_t>(m_pSystem->GetTLSystemNodeMap(),
                                 "ActionCommandGroupMask", 1);
    Arena::SetNodeValue<int64_t>(m_pSystem->GetTLSystemNodeMap(),
                                 "ActionCommandTargetIP", 0xFFFFFFFF);

    // Set trigger overlap
    //    Trigger overlap defines when a trigger can start accepting a new
    //    frame. Setting trigger to overlap with the previous frame allows the
    //    camera to being exposing the new frame while the camera is still
    //    reading out the sensor data acquired from the previous frame.
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Set trigger overlap to PreviousFrame");
    Arena::SetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(),
                                           "TriggerOverlap", "PreviousFrame");
  } else if (m_param_trigger_mode ==
             3) { // PTP triggering without scheduled action
    RCLCPP_INFO_STREAM(this->get_logger(), "Camera trigger mode: PTP Sync");

    Arena::SetNodeValue<bool>(m_pDevice->GetNodeMap(),
                              "AcquisitionFrameRateEnable", true);
    GenApi::CFloatPtr pAcquisitionFrameRate =
        m_pDevice->GetNodeMap()->GetNode("AcquisitionFrameRate");
    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Set AcquisitionFrameRate = " << pAcquisitionFrameRate->GetMax());
    pAcquisitionFrameRate->SetValue(
        pAcquisitionFrameRate->GetMax()); // make sure AcquisitionFrameRate is
                                          // not capping the FPS
    Arena::SetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(),
                                           "TriggerMode", "Off");

    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Set AcquisitionStartMode = PTPSync");
    Arena::SetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(),
                                           "AcquisitionStartMode", "PTPSync");
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Set PTPSyncFrameRate = " << m_param_frame_rate);
    Arena::SetNodeValue<double>(m_pDevice->GetNodeMap(), "PTPSyncFrameRate",
                                m_param_frame_rate);
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "Trigger mode not supported: " << m_param_trigger_mode);
    throw std::runtime_error("Trigger mode not supported: " +
                             std::to_string(m_param_trigger_mode));
  }

  // Set buffer handling mode
  //    Set buffer handling mode before starting the stream. Starting the
  //    stream requires the buffer handling mode to be set beforehand. The
  //    buffer handling mode determines the order and behavior of buffers in
  //    the underlying stream engine. Setting the buffer handling mode to
  //    'NewestOnly' ensures the most recent image is delivered, even if it
  //    means skipping frames.
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Set buffer handling mode to 'NewestOnly'");
  Arena::SetNodeValue<GenICam::gcstring>(m_pDevice->GetTLStreamNodeMap(),
                                         "StreamBufferHandlingMode",
                                         "NewestOnly");

  // Enable stream auto negotiate packet size
  //    Setting the stream packet size is done before starting the stream.
  //    Setting the stream to automatically negotiate packet size instructs
  //    the camera to receive the largest packet size that the system will
  //    allow. This generally increases frame rate and results in fewer
  //    interrupts per image, thereby reducing CPU load on the host system.
  //    Ethernet settings may also be manually changed to allow for a
  //    larger packet size.
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Enable stream to auto negotiate packet size");
  Arena::SetNodeValue<bool>(m_pDevice->GetTLStreamNodeMap(),
                            "StreamAutoNegotiatePacketSize", true);

  // Enable stream packet resend
  //    Enable stream packet resend before starting the stream. Images are
  //    sent from the camera to the host in packets using UDP protocol,
  //    which includes a header image number, packet number, and timestamp
  //    information. If a packet is missed while receiving an image, a
  //    packet resend is requested and this information is used to retrieve
  //    and redeliver the missing packet in the correct order.
  RCLCPP_INFO_STREAM(this->get_logger(), "Enable stream packet resend");
  Arena::SetNodeValue<bool>(m_pDevice->GetTLStreamNodeMap(),
                            "StreamPacketResendEnable", true);

  // set packet size and enable auto negotiate following tips from
  // https://support.thinklucid.com/knowledgebase/tips-for-reaching-maximum-frame-rate/

  // Set maximum stream channel packet size
  //    Maximizing packet size increases frame rate by reducing the amount of
  //    overhead required between images. This includes both extra
  //    header/trailer data per packet as well as extra time from intra-packet
  //    spacing (the time between packets). In order to grab images at the
  //    maximum packet size, the ethernet adapter must be configured
  //    appropriately: 'Jumbo packet' must be set to its maximum, 'UDP checksum
  //    offload' must be set to 'Rx & Tx Enabled', and 'Received Buffers' must
  //    be set to its maximum.
  if (m_param_use_max_packet_size) {
    GenApi::CIntegerPtr pDeviceStreamChannelPacketSize =
        m_pDevice->GetNodeMap()->GetNode("DeviceStreamChannelPacketSize");
    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Set maximum device stream channel packet size to max allowable: "
            << pDeviceStreamChannelPacketSize->GetMax() << " "
            << pDeviceStreamChannelPacketSize->GetUnit());
    pDeviceStreamChannelPacketSize->SetValue(
        pDeviceStreamChannelPacketSize->GetMax());
  }
  // readout to double check
  int64_t deviceLinkSpeed =
      Arena::GetNodeValue<int64_t>(m_pDevice->GetNodeMap(), "DeviceLinkSpeed");
  int64_t deviceStreamChannelPacketSize = Arena::GetNodeValue<int64_t>(
      m_pDevice->GetNodeMap(), "DeviceStreamChannelPacketSize");
  RCLCPP_INFO_STREAM(this->get_logger(), "Current device link speed = "
                                             << deviceLinkSpeed
                                             << ", packet size = "
                                             << deviceStreamChannelPacketSize);

  return 0;
}

int CamStreamer::disconnect_camera(void) {
  // close device
  if (m_pDevice != nullptr)
    m_pSystem->DestroyDevice(m_pDevice);
  m_pDevice = nullptr;

  return 0;
}

int CamStreamer::connect_camera(void) {
  if (!m_pSystem) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "System not initialised yet");
    return -1;
  }

  m_pSystem->UpdateDevices(100);

  std::vector<Arena::DeviceInfo> deviceInfos = m_pSystem->GetDevices();
  if (deviceInfos.size() == 0) {
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "There is no connected cameras devices. Please connect "
                        "a device and try again");
    return -1;
  }

  auto it = std::find_if(deviceInfos.begin(), deviceInfos.end(),
                         [&](Arena::DeviceInfo &d_info) {
                           return std::to_string(m_param_camera_serial) ==
                                  d_info.SerialNumber().c_str();
                         });
  if (it == deviceInfos.end()) {
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "There is no connected camera with serial no. "
                            << m_param_camera_serial);
    return -2;
  }

  GenICam::gcstring vendor = it->VendorName();
  GenICam::gcstring model = it->ModelName();
  GenICam::gcstring serial = it->SerialNumber();
  GenICam::gcstring macStr = it->MacAddressStr();
  GenICam::gcstring ipStr = it->IpAddressStr();
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Trying to connect to camera ("
                         << vendor << "; " << model << "; serial " << serial
                         << "; MAC " << macStr << "; IP " << ipStr << ")");
  m_pDevice = m_pSystem->CreateDevice(*it);
  RCLCPP_INFO_STREAM(this->get_logger(), "Success");

  // static key used to aquire control between applicaions
  static int64_t switchoverKey = 0x1234;
  // check if we were able to get control of application
  GenICam::gcstring deviceAccessStatus = Arena::GetNodeValue<GenICam::gcstring>(
      m_pDevice->GetTLDeviceNodeMap(), "DeviceAccessStatus");
  if (deviceAccessStatus == "ReadWrite") {
    // this mean we are running with control
    // lets set a unique key in case someone wants to gain control
    Arena::SetNodeValue<int64_t>(m_pDevice->GetTLDeviceNodeMap(),
                                 "CcpSwitchoverKey", switchoverKey);
  } else {
    // lets set a unique key in case someone wants to gain control
    Arena::SetNodeValue<int64_t>(m_pDevice->GetTLDeviceNodeMap(),
                                 "CcpSwitchoverKey", switchoverKey);
    // Now try to set the access status to read/write
    Arena::SetNodeValue<GenICam::gcstring>(m_pDevice->GetTLDeviceNodeMap(),
                                           "DeviceAccessStatus", "ReadWrite");
    GenICam::gcstring readback = Arena::GetNodeValue<GenICam::gcstring>(
        m_pDevice->GetTLDeviceNodeMap(), "DeviceAccessStatus");
    if (readback == "ReadWrite") {
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Create device succeeded with aquiring control");
      RCLCPP_INFO_STREAM(
          this->get_logger(),
          "Needs to disconnect and reconnect to revive the camera");
      disconnect_camera();
      return -3; // to reconnect
    } else {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "Create device failed to aquire control");
      return -4;
    }
  }

  return 0;
}

int CamStreamer::obtain_meta(GenICam::gcstring pixel_format, double &bpp,
                             double &offset, size_t &imgBytesPerPixel,
                             cv::ColorConversionCodes &cvBayerPattern,
                             std::string &cvTile) {
  double bitRangeMax = 23.0;
  bpp = 23.0;
  offset = 23.0;
  double bitRangeHigh = 23.0;
  double bitRangeLow = 16.0;

  if (pixel_format == "BayerRG8" || pixel_format == "BayerGR8" ||
      pixel_format == "BayerGB8" || pixel_format == "BayerBG8" ||
      pixel_format == "Mono8") {
    bitRangeMax = 7.0;
    bpp = 7.0;
    offset = 7.0;
    bitRangeHigh = 7.0;
    bitRangeLow = 0.0;

    imgBytesPerPixel = 1;
  } else if (pixel_format == "BayerRG12p" ||
             pixel_format == "BayerRG12Packed" ||
             pixel_format == "BayerGR12p" ||
             pixel_format == "BayerGR12Packed" ||
             pixel_format == "BayerGB12p" ||
             pixel_format == "BayerGB12Packed" ||
             pixel_format == "BayerBG12p" ||
             pixel_format == "BayerBG12Packed" || pixel_format == "Mono12p" ||
             pixel_format == "Mono12Packed") {
    bitRangeMax = 11.0;
    bpp = 11.0;
    offset = 11.0;
    bitRangeHigh = 11.0;
    bitRangeLow = 4.0;
  } else if (pixel_format == "BayerRG12" || pixel_format == "BayerRG16" ||
             pixel_format == "BayerGR12" || pixel_format == "BayerGR16" ||
             pixel_format == "BayerGB12" || pixel_format == "BayerGB16" ||
             pixel_format == "BayerBG12" || pixel_format == "BayerBG16" ||
             pixel_format == "Mono12" || pixel_format == "Mono16") {
    bitRangeMax = 15.0;
    bpp = 15.0;
    offset = 15.0;
    bitRangeHigh = 15.0;
    bitRangeLow = 8.0;

    imgBytesPerPixel = 2;
  } else if (pixel_format == "BayerRG24" || pixel_format == "BayerGR24" ||
             pixel_format == "BayerGB24" || pixel_format == "BayerGB24" ||
             pixel_format == "Mono24") {
    bitRangeMax = 23.0;
    bpp = 23.0;
    offset = 23.0;
    bitRangeHigh = 23.0;
    bitRangeLow = 16.0;

    imgBytesPerPixel = 3;
  }

  // Bit range for scaled image
  for (int i = 0; i < m_param_scaled_bit_offset; i++) {
    offset = offset - 1.0;
    bitRangeHigh = bitRangeHigh - 1;
    bitRangeLow = bitRangeLow - 1;

    if (offset < 7.0) {
      bitRangeHigh = bitRangeMax;
      bitRangeLow = bitRangeMax - 8.0 + 1.0;
      offset = bitRangeMax;
    }
  }

  // This identifies the color processing mode depending on the pixel format
  std::string bayerPatternString = std::string(pixel_format).substr(0, 7);
  cvBayerPattern = cv::COLOR_BayerBG2BGR;
  cvTile = "BG";

  if (bayerPatternString == "BayerRG") {
    cvBayerPattern = cv::COLOR_BayerBG2BGR;
    cvTile = "BG";
  } else if (bayerPatternString == "BayerGR") {
    cvBayerPattern = cv::COLOR_BayerGB2BGR;
    cvTile = "GB";
  } else if (bayerPatternString == "BayerGB") {
    cvBayerPattern = cv::COLOR_BayerGR2BGR;
    cvTile = "GR";
  } else if (bayerPatternString == "BayerBG") {
    cvBayerPattern = cv::COLOR_BayerRG2BGR;
    cvTile = "RG";
  } else {
    // Assume Mono pixel format
    cvBayerPattern = cv::COLOR_BayerBG2GRAY;
  }

  return 0;
}

// max tested m_param_frame_rate for TRI054S-CC is 13 for PTP triggering (mode
// 2) with binning == 2 max tested m_param_frame_rate for TDR054S-CC is 9 for
// PTP Sync (mode 3) and PTP triggering (mode 2) with binning == 2
void CamStreamer::TriggerImages(void) {
  RCLCPP_INFO_STREAM(this->get_logger(), "Trigger: Start thread");

  while (m_param_trigger_mode == 2 && m_pSystem && rclcpp::ok()) {
    if (!m_pDevice or !m_isStreaming) {
      RCLCPP_WARN_STREAM_ONCE(this->get_logger(),
                              "Trigger: Wait for the camera to be streaming");
      rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(1e3)));
      continue;
    }

    RCLCPP_DEBUG_STREAM(this->get_logger(),
                        "Trigger: Start sending PTP triggering");

    try {
      const int frame_interval_ns = 1e9 / m_param_frame_rate;
      Arena::ExecuteNode(m_pDevice->GetNodeMap(), "PtpDataSetLatch");
      int64_t cur_ptp = Arena::GetNodeValue<int64_t>(m_pDevice->GetNodeMap(),
                                                     "PtpDataSetLatchValue");
      int64_t tar_ptp = (static_cast<int64_t>(cur_ptp / 1e9) + 1) * 1e9;
      if ((tar_ptp - cur_ptp) < 0 or
          (tar_ptp - cur_ptp) > static_cast<int64_t>(1e10)) {
        RCLCPP_WARN_STREAM_ONCE(this->get_logger(),
                                "Trigger: Something wrong, start over");
        continue;
      }
      rclcpp::sleep_for(std::chrono::nanoseconds(tar_ptp - cur_ptp));

      for (long iSec = 0; rclcpp::ok(); iSec++) {
        tar_ptp = static_cast<int64_t>((tar_ptp + 5e8) / 1e9) *
                  1e9; // realign every second
        for (int iFrame = 0; iFrame < m_param_frame_rate && m_pDevice &&
                             m_isStreaming && rclcpp::ok();
             iFrame++) {
          // trigger next frame (e.g., 100 ms later if m_param_frame_rate = 10)
          tar_ptp += frame_interval_ns;
          // Fire an Action Command 100 ms from now
          RCLCPP_DEBUG_STREAM(this->get_logger(),
                              "Trigger: Scheduled Action Command set for time: "
                                  << tar_ptp << " ns");

          Arena::SetNodeValue<int64_t>(
              m_pSystem->GetTLSystemNodeMap(), "ActionCommandDeviceKey",
              m_param_camera_serial); // TODO is it necessary to specify a
                                      // different device key if the node is
                                      // running as an independent process?
          Arena::SetNodeValue<int64_t>(m_pSystem->GetTLSystemNodeMap(),
                                       "ActionCommandExecuteTime", tar_ptp);
          // RCLCPP_DEBUG_STREAM(this->get_logger(), "Fire command");
          Arena::ExecuteNode(m_pSystem->GetTLSystemNodeMap(),
                             "ActionCommandFireCommand");

          Arena::ExecuteNode(m_pDevice->GetNodeMap(), "PtpDataSetLatch");
          cur_ptp = Arena::GetNodeValue<int64_t>(m_pDevice->GetNodeMap(),
                                                 "PtpDataSetLatchValue");
          if ((tar_ptp - cur_ptp) < 0 or
              (tar_ptp - cur_ptp) > static_cast<int>(1e10)) {
            RCLCPP_WARN_STREAM_ONCE(this->get_logger(),
                                    "Trigger: Something wrong, start over");
            continue;
          }
          rclcpp::sleep_for(std::chrono::nanoseconds(tar_ptp - cur_ptp));
        }
      }
    } catch (GenICam::TimeoutException &ge) {
      // Catch disconnection
      //    Disconnections will most likely show themselves as read/write
      //    timeouts. This is caused as the host attempts to signal the
      //    device, but the device doesn't respond, timing out.
      RCLCPP_WARN_STREAM(this->get_logger(),
                         "Trigger: Camera timeout, retry later");
      rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(1e3)));
    } catch (GenICam::GenericException &ge) {
      throw std::runtime_error("Trigger: GenICam exception thrown: " +
                               std::string(ge.what()));
    } catch (std::exception &ex) {
      throw std::runtime_error("Trigger: Standard exception thrown: " +
                               std::string(ex.what()));
    } catch (...) {
      throw std::runtime_error("Trigger: Unexpected exception thrown");
    }
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "Trigger: Thread exits");
}

// demonstrates acquisition
// (1) sets acquisition mode
// (2) sets buffer handling mode
// (3) enables auto negotiate packet size
// (4) enables packet resend
// (5) starts the stream
// (6) gets a number of images
// (7) prints information from images
// (8) requeues buffers
// (9) stops the stream
void CamStreamer::AcquireImages(void) {
  RCLCPP_INFO_STREAM(this->get_logger(), "Acquire: Start thread");

  while (m_pSystem && rclcpp::ok()) {
    try {
      // handle camera connection and reconnection
      if (connect_camera()) {
        rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(1e4)));
        continue;
      }

      configure_camera();

      cv::Mat mScaled;
      cv::Mat mScaledProcessed;

      double bpp;
      double offset;
      size_t imgBytesPerPixel;
      cv::ColorConversionCodes cvBayerPattern;
      std::string cvTile;

      // Start stream
      //    Start the stream before grabbing any images. Starting the stream
      //    allocates buffers, which can be passed in as an argument (default:
      //    10), and begins filling them with data. Starting the stream blocks
      //    write access to many features such as width, height, and pixel
      //    format, as well as acquisition and buffer handling modes, among
      //    others. The stream needs to be stopped later.
      m_pDevice->StartStream(100);
      RCLCPP_INFO_STREAM(this->get_logger(), "Acquire: Start stream");
      m_isStreaming = true;

      for (int i = 0; m_pDevice && rclcpp::ok(); i++) {
        Arena::IImage *pImage = m_pDevice->GetImage(m_param_timeout_ms);

        if (!pImage->IsIncomplete()) {
          uint64_t frameId = pImage->GetFrameId();
          uint64_t imgWidth = pImage->GetWidth();
          uint64_t imgHeight = pImage->GetHeight();
          size_t imgBpp = pImage->GetBitsPerPixel();
          GenICam::gcstring imgPixelFormat = GetPixelFormatName(
              static_cast<PfncFormat>(pImage->GetPixelFormat()));
          rclcpp::Time stamp;
          if (m_param_use_ptp_time) {
            stamp = rclcpp::Time(pImage->GetTimestampNs());
            RCLCPP_DEBUG_STREAM(this->get_logger(),
                                "Received new image "
                                    << frameId << " with PTP timestamp: "
                                    << pImage->GetTimestampNs());
          } else {
            stamp = rclcpp::Node::now();
            RCLCPP_DEBUG_STREAM(this->get_logger(),
                                "Received new image " << frameId);
          }

          if (std::string(imgPixelFormat).find("Bayer") != std::string::npos) {
            obtain_meta(imgPixelFormat, bpp, offset, imgBytesPerPixel,
                        cvBayerPattern, cvTile);

            if (imgBpp > 8) { // bayer hdr for Lucid Vision TRI054
              cv::Mat img = LoadBayerHdrImageIntoOpenCV(
                  pImage, imgWidth, imgHeight, imgBytesPerPixel, cvTile,
                  m_param_debayer_method); // debayer

              HdrImage hdr_image;
              hdr_image.stamp = stamp;
              hdr_image.seq = frameId;
              hdr_image.img = img;
              m_p_hdr_image_queue->push(hdr_image);

              // Scaled 8-bits
              Arena::IImage *pScaled = Arena::ImageFactory::SelectBitsAndScale(
                  pImage, 8, bpp - offset);
              mScaled = cv::Mat((int)imgHeight, (int)imgWidth, CV_8UC1,
                                (void *)pScaled->GetData());
              cv::cvtColor(mScaled, mScaledProcessed,
                           cvBayerPattern); // debayer
              Arena::ImageFactory::Destroy(pScaled);
              // TODO: publish mScaledProcessed too?
              RCLCPP_DEBUG_STREAM(this->get_logger(),
                                  "Finished processing new image "
                                      << frameId << " ("
                                      << mScaledProcessed.cols << "x"
                                      << mScaledProcessed.rows << ")");
            } else { // bayer non-hdr
              mScaled =
                  cv::Mat((int)pImage->GetHeight(), (int)pImage->GetWidth(),
                          CV_8UC1, (void *)pImage->GetData());
              cv::cvtColor(mScaled, mScaledProcessed,
                           cvBayerPattern); // debayer
              HdrImage processed_image;
              processed_image.stamp = stamp;
              processed_image.seq = frameId;
              processed_image.img = mScaledProcessed;
              m_p_processed_image_queue->push(processed_image);
            }
          } else if (std::string(imgPixelFormat).find("RGB8") !=
                     std::string::npos) { // rgb for Lucid Vision TDR054
            cv::Mat img_rbg =
                LoadRgbImageIntoOpenCV(pImage, imgWidth, imgHeight);
            cv::Mat img_bgr;
            cv::cvtColor(img_rbg, img_bgr, cv::COLOR_RGB2BGR);
            HdrImage processed_image;
            processed_image.stamp = stamp;
            processed_image.seq = frameId;
            processed_image.img = img_bgr;
            m_p_processed_image_queue->push(processed_image);
          }

          m_pDevice->RequeueBuffer(pImage);
        }
      }
    } catch (GenICam::TimeoutException &ge) {
      // Catch disconnection
      //    Disconnections will most likely show themselves as read/write
      //    timeouts. This is caused as the host attempts to signal the
      //    device, but the device doesn't respond, timing out.
      m_isStreaming = false;
      RCLCPP_WARN_STREAM(this->get_logger(),
                         "Acquire: Camera timeout, try to reconnect later");
      RCLCPP_INFO_STREAM(this->get_logger(), "Acquire: Stop stream");
      if (m_pDevice != nullptr)
        m_pDevice->StopStream();
      RCLCPP_INFO_STREAM(this->get_logger(), "Acquire: Close device");
      disconnect_camera();
      rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(1e3)));
    } catch (GenICam::GenericException &ge) {
      throw std::runtime_error("Acquire: GenICam exception thrown: " +
                               std::string(ge.what()));
    } catch (std::exception &ex) {
      throw std::runtime_error("Acquire: Standard exception thrown: " +
                               std::string(ex.what()));
    } catch (...) {
      throw std::runtime_error("Acquire: Unexpected exception thrown");
    }
  }

  m_isStreaming = false;
  RCLCPP_INFO_STREAM(this->get_logger(), "Acquire: Stop stream");
  if (m_pDevice != nullptr)
    m_pDevice->StopStream();
  RCLCPP_INFO_STREAM(this->get_logger(), "Acquire: Close device");
  disconnect_camera();
  RCLCPP_INFO_STREAM(this->get_logger(), "Acquire: Thread exits");
}

void CamStreamer::ProcessImages(void) {
  cv::Mat processed_hdr_img;
  cv::Mat processed_img;

  RCLCPP_INFO_STREAM(this->get_logger(), "Tonemap: Start thread");

  for (int i = 0; rclcpp::ok(); i++) {
    if (m_p_hdr_image_queue->full()) {
      RCLCPP_WARN_STREAM_ONCE(
          this->get_logger(),
          "Tonemap: Image queue full, consider reducing FPS");
    }
    HdrImage hdr_image;
    m_p_hdr_image_queue->pop(hdr_image);
    RCLCPP_DEBUG_STREAM(this->get_logger(),
                        "Start tonemapping " << hdr_image.seq);
    processed_hdr_img = ProcessImageInOpenCV(
        hdr_image.img, m_param_tonemap_method, m_param_tonemap_gamma);
    processed_hdr_img.convertTo(processed_img, CV_8UC3, 255);
    RCLCPP_DEBUG_STREAM(this->get_logger(), "End tonemapping "
                                                << hdr_image.seq << " ("
                                                << processed_img.cols << "x"
                                                << processed_img.rows << ")");
    HdrImage processed_image;
    processed_image.stamp = hdr_image.stamp;
    processed_image.seq = hdr_image.seq;
    processed_image.img = processed_img;
    m_p_processed_image_queue->push(processed_image);
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "Tonemap: Thread exits");
}

void CamStreamer::PublishImages(void) {
  RCLCPP_INFO_STREAM(this->get_logger(), "Publish: Start thread");

  cv::Size cv_windowSize = cv::Size(1280, 720);
  std::string window_title = "ROS Image";

  for (int i = 0; rclcpp::ok(); i++) {
    if (m_p_processed_image_queue->full()) {
      RCLCPP_WARN_STREAM_ONCE(
          this->get_logger(),
          "Publish: Image queue full, consider reducing FPS");
    }
    HdrImage processed_image;
    m_p_processed_image_queue->pop(processed_image);
    publish_image(processed_image.stamp, processed_image.img);

    if (m_param_viz) {
      cv::namedWindow(window_title, cv::WINDOW_NORMAL);
      cv::resizeWindow(window_title, cv_windowSize);
      cv::imshow(window_title, processed_image.img);
      cv::waitKey(1);
    }
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "Publish: Thread exits");
}

void CamStreamer::print_diagnosis(void) {
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "TriggerArmed=" << Arena::GetNodeValue<bool>(
                         m_pDevice->GetNodeMap(), "TriggerArmed"));
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "ActionQueueSize=" << Arena::GetNodeValue<int64_t>(
                         m_pDevice->GetNodeMap(), "ActionQueueSize"));
}

void CamStreamer::publish_image(rclcpp::Time stamp, const cv::Mat &image,
                                std::string encoding) {
  sensor_msgs::msg::Image img_msg;
  std_msgs::msg::Header header;
  header.stamp = stamp;
  header.frame_id = m_param_frame_id;

  try {
    cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, encoding, image);
    img_bridge.toImageMsg(img_msg);

  } catch (...) {
    throw std::runtime_error("Runtime error in publish_image()");
  }

  auto ci = std::make_unique<sensor_msgs::msg::CameraInfo>(
      m_p_camera_info->getCameraInfo());
  ci->header = img_msg.header;

  m_p_camera_publisher->publish(img_msg, *ci);
}

void CamStreamer::init_camera_info(std::string camera_name,
                                   std::string camera_info_url) {
  m_p_camera_info = std::make_shared<camera_info_manager::CameraInfoManager>(
      this, camera_name);
  if (m_p_camera_info->validateURL(camera_info_url)) {
    m_p_camera_info->loadCameraInfo(camera_info_url);
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(),
                       "Invalid camera info URL: " << camera_info_url.c_str());
  }
}

CamStreamer::CamStreamer(rclcpp::NodeOptions node_options)
    : rclcpp::Node("hdr_camera_streamer_node",
                   node_options.allow_undeclared_parameters(true)),
      m_pSystem(nullptr), m_pDevice(nullptr), m_isStreaming(false) {
  declare_parameter("pixel_format", "BayerRG24");
  declare_parameter("timeout_ms", 2000);
  declare_parameter("tonemap_method", 1);
  declare_parameter("tonemap_gamma", 2.0f);
  declare_parameter("scaled_bit_offset", 8);
  declare_parameter("trigger_mode", 2);
  declare_parameter("trigger_fps", 10);
  declare_parameter("exposure_time_us", 10000);
  declare_parameter("debayer_method", 0);
  declare_parameter("use_image_crop", false);
  declare_parameter("image_width", 1920);
  declare_parameter("image_height", 1080);
  declare_parameter("image_offset_x", 480);
  declare_parameter("image_offset_y", 390);
  declare_parameter("use_max_packet_size", true);
  declare_parameter("use_binning", true);
  declare_parameter("flip_horizontal", false);
  declare_parameter("flip_vertical", false);
  declare_parameter("use_system_default_qos", true);
  declare_parameter("use_ptp_time", true);
  declare_parameter("viz", false);
  m_param_pixel_format = get_parameter("pixel_format").as_string();
  m_param_timeout_ms = get_parameter("timeout_ms").as_int();
  m_param_tonemap_method = get_parameter("tonemap_method").as_int(); // tonemap
  m_param_tonemap_gamma =
      get_parameter("tonemap_gamma")
          .as_double(); // // tonemap parameters for the OpenCV controls
  m_param_scaled_bit_offset =
      get_parameter("scaled_bit_offset")
          .as_int(); // scaled image bit offset from 24 bit
  m_param_trigger_mode =
      get_parameter("trigger_mode")
          .as_int(); // 0: free running, 1: hardware triggering, 2: PTP
                     // triggering, 3: PTP Sync
  m_param_frame_rate =
      get_parameter("trigger_fps")
          .as_int(); // only for free running and PTP triggering mode
  m_param_exposure_time = get_parameter("exposure_time_us").as_int();
  m_param_debayer_method = get_parameter("debayer_method").as_int(); // debayer
  m_param_use_image_crop = get_parameter("use_image_crop").as_bool();
  m_param_image_width = get_parameter("image_width").as_int();
  m_param_image_height = get_parameter("image_height").as_int();
  m_param_image_offset_x = get_parameter("image_offset_x").as_int();
  m_param_image_offset_y = get_parameter("image_offset_y").as_int();
  m_param_use_max_packet_size = get_parameter("use_max_packet_size").as_bool();
  m_param_use_binning = get_parameter("use_binning").as_bool();
  m_param_flip_horizontal = get_parameter("flip_horizontal").as_bool();
  m_param_flip_vertical = get_parameter("flip_vertical").as_bool();
  m_param_use_sys_default_qos =
      get_parameter("use_system_default_qos").as_bool();
  m_param_use_ptp_time = get_parameter("use_ptp_time").as_bool();
  m_param_viz = get_parameter("viz").as_bool();

  declare_parameter("camera_ns", "cam");
  declare_parameter("camera_serial", -1);
  declare_parameter("camera_info_url", "");
  declare_parameter("frame_id", "");
  m_param_camera_ns = get_parameter("camera_ns").as_string();
  m_param_camera_serial = get_parameter("camera_serial").as_int();
  m_param_camera_info_url = get_parameter("camera_info_url").as_string();
  m_param_frame_id = get_parameter("frame_id").as_string();

  if (m_param_camera_serial < 0) {
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "Camera serial invalid: " << m_param_camera_serial);
    throw std::runtime_error("Camera serial invalid: " +
                             std::to_string(m_param_camera_serial));
  }

  init_camera_info(m_param_camera_ns, m_param_camera_info_url);

  m_p_hdr_image_queue = std::make_unique<BlockingQueue<HdrImage>>(5);
  m_p_processed_image_queue = std::make_unique<BlockingQueue<HdrImage>>(5);

  rclcpp::QoS system_default_qos = rclcpp::SystemDefaultsQoS();
  system_default_qos.keep_last(
      10); // to allow intra-process comm in a composition container
  system_default_qos.durability_volatile(); // to allow intra-process comm in a
                                            // composition container
  rclcpp::QoS sensor_data_qos = rclcpp::SensorDataQoS();
  auto selected_qos =
      m_param_use_sys_default_qos ? system_default_qos : sensor_data_qos;
  m_p_camera_publisher = std::make_shared<image_transport::CameraPublisher>(
      image_transport::create_camera_publisher(
          this, "/" + m_param_camera_ns + "/image",
          selected_qos.get_rmw_qos_profile()));

  try {
    RCLCPP_INFO_STREAM(this->get_logger(), "Opening Arena system");
    m_pSystem = Arena::OpenSystem();
    RCLCPP_INFO_STREAM(this->get_logger(), "Spinning up processing threads");
    m_p_image_trigger = std::unique_ptr<std::thread>(
        new std::thread(&CamStreamer::TriggerImages, this));
    m_p_image_producer = std::unique_ptr<std::thread>(
        new std::thread(&CamStreamer::AcquireImages, this));
    m_p_image_consumer = std::unique_ptr<std::thread>(
        new std::thread(&CamStreamer::ProcessImages, this));
    m_p_image_publisher = std::unique_ptr<std::thread>(
        new std::thread(&CamStreamer::PublishImages, this));
  } catch (GenICam::GenericException &ge) {
    throw std::runtime_error("GenICam exception thrown: " +
                             std::string(ge.what()));
  } catch (std::exception &ex) {
    throw std::runtime_error("Standard exception thrown: " +
                             std::string(ex.what()));
  } catch (...) {
    throw std::runtime_error("Unexpected exception thrown");
  }
}

CamStreamer::~CamStreamer() {
  RCLCPP_INFO_STREAM(this->get_logger(), "Terminating all working threads");
  if (m_p_image_trigger != nullptr)
    m_p_image_trigger->join();
  if (m_p_image_producer != nullptr)
    m_p_image_producer->join();
  if (m_p_image_publisher != nullptr)
    m_p_image_publisher->join();
  RCLCPP_INFO_STREAM(this->get_logger(), "All threads exit");

  // clean up
  if (m_param_viz)
    cv::destroyAllWindows();
  disconnect_camera();
  RCLCPP_INFO_STREAM(this->get_logger(), "Arena system shutdown");
  if (m_pSystem != nullptr)
    Arena::CloseSystem(m_pSystem);
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(CamStreamer)
