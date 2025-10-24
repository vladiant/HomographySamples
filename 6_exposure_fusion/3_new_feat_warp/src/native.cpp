#include <library.h>

#include "../private_include/common.h"
#include "ivy_mobile_module_Native.h"

#ifdef __cplusplus
extern "C" {
#endif

void Java_ivy_mobile_module_Native_createHDRwithJNI(JNIEnv* env, jobject,
                                                    jobjectArray in,
                                                    jstring out) {
  jstring x[16];
  jboolean iscopy;
  const char* _outName;
  const char* _fileNames[16];
  jsize length = (*env).GetArrayLength(in);
  if (length > 16) length = 16;

  for (int i = 0; i < length; ++i) {
    x[i] = (jstring)(*env).GetObjectArrayElement(in, i);
    _fileNames[i] = (*env).GetStringUTFChars(x[i], &iscopy);
  }

  _outName = (*env).GetStringUTFChars(out, &iscopy);

  hdr_proc_files(_fileNames, &_outName, length, -1, 2);

  for (int i = 0; i < length; i++) {
    (*env).ReleaseStringUTFChars(x[i], _fileNames[i]);
  }
  (*env).ReleaseStringUTFChars(out, _outName);
}

#ifdef __cplusplus
}
#endif
