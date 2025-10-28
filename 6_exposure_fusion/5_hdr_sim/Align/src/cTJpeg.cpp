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
