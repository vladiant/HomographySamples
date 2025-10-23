/*
 * File:   cTJpeg.cpp
 *
 * Created on May 9, 2012, 10:17 AM
 */

#include "cTJpeg.h"

#include <setjmp.h>
#include <stdio.h>
#include <string.h>

#include <iostream>
using namespace std;

void cTJpeg::initialize() {
  // cout<<"cTJpeg::initialize()\n";
  buffer = 0;
  width = 0;
  height = 0;
  channels = 0;
  colorspace = JCS_UNKNOWN;
}

cTJpeg::cTJpeg() {
  // cout<<"cTJpeg::cTJpeg()\n";
  initialize();
}

cTJpeg::cTJpeg(const cTJpeg& orig) {
  // cout<<"cTJpeg::cTJpeg(const cTJpeg& orig)\n";
  initialize();
  operator=(orig);
}

cTJpeg::cTJpeg(const cTImg& orig) {
  initialize();
  operator=(orig);
}

cTJpeg& cTJpeg::operator=(const cTImg& orig) {
  create(orig, true);
  return *this;
}

cTJpeg& cTJpeg::operator=(const cTJpeg& orig) {
  // cout<<"cTJpeg::operator= (const cTJpeg& orig)\n";
  if (&orig == this) return *this;

  colorspace = orig.colorspace;
  create(orig, true);

  return *this;
}

cTJpeg::~cTJpeg() {
  // cout<<"cTJpeg::~cTJpeg()\n";
  deallocate();
}

bool cTJpeg::allocate(unsigned int size) {
  try {
    deallocate();
    if (size > 0) {
      buffer = new unsigned char[size];
      return true;
    }
    return false;
  } catch (std::bad_alloc) {
    buffer = 0;
    return false;
  }
}

void cTJpeg::deallocate() {
  if (buffer != 0) {
    delete[] buffer;
    buffer = 0;
  }
}

cTImg cTJpeg::getImg() const {
  cTImg img;
  img.width = width;
  img.height = height;
  img.channels = channels;
  img.buffer = buffer;
  return img;
}

bool cTJpeg::create(const cTJpeg& img, bool copy) {
  if (copy)
    return create(img.getWidth(), img.getHeight(), img.getChannels());
  else
    return create(img.getWidth(), img.getHeight(), img.getChannels(),
                  img.getData());
}

bool cTJpeg::create(const cTImg& img, bool copy) {
  if (copy)
    return create(img.width, img.height, img.channels);
  else
    return create(img.width, img.height, img.channels, img.buffer);
}

bool cTJpeg::create(unsigned short iwidth, unsigned short iheight,
                    unsigned short ichannels) {
  width = iwidth;
  height = iheight;
  channels = ichannels;
  colorspace = JCS_UNKNOWN;

  return allocate(width * height * channels);
}

bool cTJpeg::create(unsigned short iwidth, unsigned short iheight,
                    unsigned short ichannels, const unsigned char* data) {
  if (!create(iwidth, iheight, ichannels)) return false;

  if (width * height * channels > 0) {
    memcpy(buffer, data, width * height * channels);
  }

  return true;
}

bool cTJpeg::fileLoad(const char* filename, J_COLOR_SPACE icolorspace) {
  struct jpeg_error_mgr jerr;
  struct jpeg_decompress_struct cinfo;
  FILE* infile;
  JSAMPARRAY buf;
  unsigned char* out;
  int row_stride;

  if ((infile = fopen(filename, "rb")) == NULL) {
    fprintf(stderr, "can't open %s\n", filename);
    return false;
  }

  cinfo.err = jpeg_std_error(&jerr);

  colorspace = icolorspace;
  cinfo.out_color_space = colorspace;
  // cinfo.out_color_components = 1;
  // cinfo.output_components = 1;

  jpeg_create_decompress(&cinfo);
  jpeg_stdio_src(&cinfo, infile);
  jpeg_read_header(&cinfo, TRUE);
  if (!jpeg_start_decompress(&cinfo)) {
    fprintf(stderr, "[Error] Can't initiate conversion to selected format\n");
    jpeg_destroy_decompress(&cinfo);
    fclose(infile);
    return false;
  }

  row_stride = cinfo.output_width * cinfo.output_components;
  buf = (*cinfo.mem->alloc_sarray)((j_common_ptr)&cinfo, JPOOL_IMAGE,
                                   row_stride, 1);

  if (!create(cinfo.output_width, cinfo.output_height,
              cinfo.output_components)) {
    fprintf(stderr, "[Error] Can't allocate memory for jpeg decompression\n");
    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);
    fclose(infile);
    return false;
  }

  width = cinfo.output_width;
  height = cinfo.output_height;
  channels = cinfo.output_components;
  colorspace = icolorspace;

  out = buffer;
  while (cinfo.output_scanline < cinfo.output_height) {
    jpeg_read_scanlines(&cinfo, buf, 1);
    memcpy(out, buf[0], row_stride);
    out += row_stride;
  }

  // printf("%dx%d\n", cinfo.image_width, cinfo.image_height);

  jpeg_finish_decompress(&cinfo);
  jpeg_destroy_decompress(&cinfo);
  fclose(infile);

  return true;
}

bool cTJpeg::fileSave(const char* filename, int quality,
                      J_COLOR_SPACE icolorspace) {
  struct jpeg_compress_struct cinfo;
  struct jpeg_error_mgr jerr;
  FILE* outfile;
  JSAMPROW row_pointer[1];
  int row_stride;

  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_compress(&cinfo);

  if ((outfile = fopen(filename, "wb")) == NULL) {
    fprintf(stderr, "[Error] Can't open %s\n", filename);
    return false;
  }
  jpeg_stdio_dest(&cinfo, outfile);

  if (icolorspace != JCS_UNKNOWN)
    cinfo.in_color_space = icolorspace;
  else
    cinfo.in_color_space = colorspace;

  cinfo.image_width = width;
  cinfo.image_height = height;
  cinfo.input_components = channels;

  jpeg_set_defaults(&cinfo);
  jpeg_set_quality(&cinfo, quality, TRUE);
  jpeg_start_compress(&cinfo, TRUE);

  row_stride = width * channels;

  unsigned char* in = buffer;
  while (cinfo.next_scanline < cinfo.image_height) {
    row_pointer[0] = in;
    jpeg_write_scanlines(&cinfo, row_pointer, 1);
    in += row_stride;
  }

  jpeg_finish_compress(&cinfo);
  fclose(outfile);
  jpeg_destroy_compress(&cinfo);

  return true;
}
