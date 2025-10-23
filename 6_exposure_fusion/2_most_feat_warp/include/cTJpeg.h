/*
 * File:   cTJpeg.h
 *
 * Created on May 9, 2012, 10:17 AM
 */

#pragma once

#include <iostream>
extern "C" {
#include <jpeglib.h>
}

#include "cTImg.h"

class cTJpeg : protected cTImg {
 public:
  cTJpeg();
  cTJpeg(const cTImg& orig);
  cTJpeg(const cTJpeg& orig);
  cTJpeg& operator=(const cTImg& orig);
  cTJpeg& operator=(const cTJpeg& orig);
  ~cTJpeg();

  inline bool create(const cTImg& img, bool copy = false);
  inline bool create(const cTJpeg& img, bool copy = false);
  bool create(unsigned short width, unsigned short height,
              unsigned short channels);
  bool create(unsigned short width, unsigned short height,
              unsigned short channels, const unsigned char* data);
  void destroy() { deallocate(); }

  bool fileLoad(const char* filename, J_COLOR_SPACE icolorspace = JCS_RGB);
  bool fileSave(const char* filename, int quality = 100,
                J_COLOR_SPACE icolorspace = JCS_UNKNOWN);

  bool isEmpty() { return (buffer == 0 ? true : false); }

  cTImg getImg() const;
  inline unsigned char* getData() const { return buffer; }
  inline unsigned short getWidth() const { return width; }
  inline unsigned short getHeight() const { return height; }
  inline unsigned short getChannels() const { return channels; }

 private:
  void initialize();
  bool allocate(unsigned int size);
  void deallocate();

  J_COLOR_SPACE colorspace;
};
