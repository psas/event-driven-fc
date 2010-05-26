#ifndef COMPILER_H
#define COMPILER_H

#if __GNUC__ >= 4 || (__GNUC__ == 3 && __GNUC_MINOR__ >= 3)
#define ATTR_WARN_UNUSED_RESULT __attribute__((warn_unused_result))
#else
#define ATTR_WARN_UNUSED_RESULT
#endif

#if __GNUC__ >= 3 || (__GNUC__ == 2 && __GNUC_MINOR__ >= 3)
#define ATTR_FORMAT(func,fmt,args) __attribute__((format(func,fmt,args)))
#else
#define ATTR_FORMAT(func,fmt,args)
#endif

#endif /* COMPILER_H */
