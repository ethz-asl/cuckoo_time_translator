#ifndef H7E81A0F8_04E4_435D_8F39_103889A163E8
#define H7E81A0F8_04E4_435D_8F39_103889A163E8

#ifdef USE_ROS_ASSERTIONS
#include <ros/assert.h>
#define AASSERT(X) ROS_ASSERT(X)
#else
#include <cassert>
#define AASSERT(x) assert(x)
#endif

#define AASSERT_GE(a, b, message) AASSERT(a >= b && message)
#define AASSERT_GT(a, b, message) AASSERT(a > b && message)
#define AASSERT_LE(a, b, message) AASSERT(a <= b && message)
#define AASSERT_LT(a, b, message) AASSERT(a < b && message)

#ifdef NDEBUG
#define AASSERT_GE_DBG(a, b, message)
#define AASSERT_LE_DBG(a, b, message)
#define AASSERT_LT_DBG(a, b, message)
#else
#define AASSERT_GE_DBG(a, b, message) AASSERT_GE(a, b, message)
#define AASSERT_LE_DBG(a, b, message) AASSERT_LE(a, b, message)
#define AASSERT_LT_DBG(a, b, message) AASSERT_LT(a, b, message)
#endif

#endif /* H7E81A0F8_04E4_435D_8F39_103889A163E8 */
