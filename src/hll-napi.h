#ifndef HLL_NAPI_H
#define HLL_NAPI_H
#include <napi.h>
#include "HllObject.h"


class HllNative : public Napi::ObjectWrap<HllNative> {
 public:
  static Napi::Object Init(Napi::Env env, Napi::Object exports);
  HllNative(const Napi::CallbackInfo& info);
  ~HllNative();
  Napi::Value ToBuffer(const Napi::CallbackInfo& info);
  

 private:
  void add(const Napi::CallbackInfo& info);
  void merge(const Napi::CallbackInfo& info);
  void deserialize(const Napi::CallbackInfo& info);
  Napi::Value cardinality(const Napi::CallbackInfo& info);

  static Napi::FunctionReference constructor;
  
  HllObject *hll;
  
};

#endif
