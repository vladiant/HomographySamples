#include "common.h"

long hdr_getuTime(void) {
  struct timeval now;

  gettimeofday(&now, NULL);
  return (long)(now.tv_sec * 1000000 + now.tv_usec);
}

long hdr_getTime(void) {
  struct timeval now;

  gettimeofday(&now, NULL);
  return (long)(now.tv_sec * 1000 + now.tv_usec / 1000);
}
