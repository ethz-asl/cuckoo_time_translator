#ifndef HCEF9D5FC_9095_462B_B900_048DD967629F
#define HCEF9D5FC_9095_462B_B900_048DD967629F

#include <console_bridge/console.h>

#ifdef logError

#define CUCKOO_TIME_TRANSLATOR_logError(fmt, ...)  logError(fmt, ##__VA_ARGS__) 
#define CUCKOO_TIME_TRANSLATOR_logWarn(fmt, ...)   logWarn(fmt, ##__VA_ARGS__)
#define CUCKOO_TIME_TRANSLATOR_logInform(fmt, ...) logInform(fmt, ##__VA_ARGS__)
#define CUCKOO_TIME_TRANSLATOR_logDebug(fmt, ...)  logDebug(fmt, ##__VA_ARGS__)

#else

#define CUCKOO_TIME_TRANSLATOR_logError(fmt, ...)  CONSOLE_BRIDGE_logError(fmt, ##__VA_ARGS__) 
#define CUCKOO_TIME_TRANSLATOR_logWarn(fmt, ...)   CONSOLE_BRIDGE_logWarn(fmt, ##__VA_ARGS__)
#define CUCKOO_TIME_TRANSLATOR_logInform(fmt, ...) CONSOLE_BRIDGE_logInform(fmt, ##__VA_ARGS__)
#define CUCKOO_TIME_TRANSLATOR_logDebug(fmt, ...)  CONSOLE_BRIDGE_logDebug(fmt, ##__VA_ARGS__)

#endif

#endif /* HCEF9D5FC_9095_462B_B900_048DD967629F */
