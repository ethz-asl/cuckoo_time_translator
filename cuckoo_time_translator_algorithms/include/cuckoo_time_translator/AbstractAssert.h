#ifndef H7E81A0F8_04E4_435D_8F39_103889A163E8
#define H7E81A0F8_04E4_435D_8F39_103889A163E8

#include <console_bridge/console.h>

#ifdef DEFINE_LOGGING_MACROS

#define logError(fmt, ...)  CONSOLE_BRIDGE_logError(fmt, ##__VA_ARGS__) 
#define logWarn(fmt, ...)   CONSOLE_BRIDGE_logWarn(fmt, ##__VA_ARGS__)
#define logInform(fmt, ...) CONSOLE_BRIDGE_logInform(fmt, ##__VA_ARGS__)
#define logDebug(fmt, ...)  CONSOLE_BRIDGE_logDebug(fmt, ##__VA_ARGS__)

#endif

#define AASSERT(x, message) \
  do { \
    if (!(x)) { \
      logError("ASSERTION %s FAILED: %s (%s:%d)", #x, message, __FILE__, __LINE__); \
      throw std::runtime_error(message); \
    } \
  } while (0)

#define AASSERT_GE(a, b, message) AASSERT(a >= b, message)
#define AASSERT_GT(a, b, message) AASSERT(a > b, message)
#define AASSERT_LE(a, b, message) AASSERT(a <= b, message)
#define AASSERT_LT(a, b, message) AASSERT(a < b, message)

#ifdef NDEBUG
#define AASSERT_DBG(a, message)
#define AASSERT_GE_DBG(a, b, message)
#define AASSERT_GT_DBG(a, b, message)
#define AASSERT_LE_DBG(a, b, message)
#define AASSERT_LT_DBG(a, b, message)
#else
#define AASSERT_DBG(x, message) ASSERT(x, message)
#define AASSERT_GE_DBG(a, b, message) AASSERT_GE(a, b, message)
#define AASSERT_GT_DBG(a, b, message) AASSERT_GT(a, b, message)
#define AASSERT_LE_DBG(a, b, message) AASSERT_LE(a, b, message)
#define AASSERT_LT_DBG(a, b, message) AASSERT_LT(a, b, message)
#endif

#endif /* H7E81A0F8_04E4_435D_8F39_103889A163E8 */
