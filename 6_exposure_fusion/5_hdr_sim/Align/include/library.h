/*
 * File:   library.h
 *
 * Created on February 24, 2012, 12:06 PM
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

class cTImg;

int hdr_proc(cTImg* img, int count, int* reference, int prog);
int hdr_proc_files(const char** input_files, const char** output_files,
                   int count, int* reference, int prog);

#ifdef __cplusplus
}
#endif
