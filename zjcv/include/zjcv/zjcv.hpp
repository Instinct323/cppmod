#ifndef ZJCV__ZJCV_HPP
#define ZJCV__ZJCV_HPP

// 对于一些无法通过继承实现的类
// 通过宏定义标注 "内置方法" 和 "自定义方法"

#ifdef ZJCV_BUILTIN
#error "ZJCV_BUILTIN already defined"
#endif
#define ZJCV_BUILTIN

#ifdef ZJCV_CUSTOM
#error "ZJCV_CUSTOM already defined"
#endif
#define ZJCV_CUSTOM

#endif
