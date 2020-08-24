


#include "hll-napi.h"
#include <napi.h>

Napi::FunctionReference HllNative::constructor;


Napi::Object HllNative::Init(Napi::Env env, Napi::Object exports) {
  Napi::HandleScope scope(env);

  Napi::Function func = DefineClass(env, "Hll", {
    InstanceMethod("add", &HllNative::add),
    InstanceMethod("merge", &HllNative::merge),
    InstanceMethod("cardinality", &HllNative::cardinality),
    InstanceMethod("deserialize", &HllNative::deserialize)
  });

  constructor = Napi::Persistent(func);
  constructor.SuppressDestruct();

  exports.Set("Bff", func);
  return exports;
}

HllNative::HllNative(const Napi::CallbackInfo& info) : Napi::ObjectWrap<HllNative>(info)  {
  const Napi::Env env = info.Env();

  if (info.Length() != 0) {
    Napi::TypeError::New(env, "Wrong number of arguments").ThrowAsJavaScriptException();
    return;
  }


  this->hll = new HllObject();
}

HllNative::~HllNative() {
  delete this->hll;
}

void HllNative::add(const Napi::CallbackInfo& info) {
  const Napi::Env env = info.Env();

  if (info.Length() != 1) {
    Napi::TypeError::New(env, "Wrong number of arguments").ThrowAsJavaScriptException();
  }

  if (!info[0].IsString()) {
    Napi::TypeError::New(env, "Only strings are supported").ThrowAsJavaScriptException();
  }

  Napi::String str = info[0].As<Napi::String>();
  std::string val = str.Utf8Value();

  this->hll->add(val.c_str(), val.length());

}

void HllNative::merge(const Napi::CallbackInfo& info) {
  const Napi::Env env = info.Env();

  Hll* hllObj = Napi::ObjectWrap<Hll>::Unwrap(info[0].As<Napi::Object>());
  
  this->hll->merge(*hllObj->hll);
}

void HllNative::deserialize(const Napi::CallbackInfo& info) {
  const Napi::Env env = info.Env();

  if (info.Length() != 1) {
    Napi::TypeError::New(env, "Wrong number of arguments").ThrowAsJavaScriptException();
  }

  if (!info[0].IsString()) {
    Napi::TypeError::New(env, "Only strings are supported").ThrowAsJavaScriptException();
  }

  Napi::String str = info[0].As<Napi::String>();
  std::string val = str.Utf8Value();

  this->hll->deserialize(val.c_str());
}

Napi::Value HllNative::cardinality(const Napi::CallbackInfo& info) {
  const Napi::Env env = info.Env();
  uint64_t cardinality = this->hll->getCardinality();
  return Napi::Number::New(env,cardinality);
}

Napi::Object Init(Napi::Env env, Napi::Object exports) {
  return HllNative::Init(env, exports);
}

NODE_API_MODULE(NODE_GYP_MODULE_NAME, Init)

