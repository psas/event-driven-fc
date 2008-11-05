#ifndef COMPILER_H
#define COMPILER_H

#if __GNUC__ >= 4 || (__GNUC__ == 3 && __GNUC_MINOR__ >= 3)
#define ATTR_WARN_UNUSED_RESULT __attribute__((warn_unused_result))
#else
#define ATTR_WARN_UNUSED_RESULT
#endif

#endif /* COMPILER_H */
