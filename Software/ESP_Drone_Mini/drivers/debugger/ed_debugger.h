#ifndef __ED_DEBUGGER_H__
#define __ED_DEBUGGER_H__

#include <stdint.h>

#ifndef COUNT_ARGS
#define COUNT_ARGS(X...) __COUNT_ARGS(, ##X, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0)
#define __COUNT_ARGS(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _n, X...) _n
#endif

extern const float *__vofa_package_tail__;


/**
 * @brief: Create a debugger, this debugger will be implemented based on TCP
 * @param: The port of the tcp server.
 */
int ed_debugger_create(int port);

/**
 * @brief: Send several floating-point numbers to the "VOFA+" software.
 * @note: The data engine should be selected as JustFloat, 
            which package structure ends with { 0x00, 0x00, 0x80, 0x78 }.
            This package tail is a type of NAN specified in IEEE 754.
 */
void ed_debugger_send_vofa(int nums, ...);
#define ed_debugger_send_float(...) ed_debugger_send_vofa(COUNT_ARGS(X ##__VA_ARGS__) + 1, ##__VA_ARGS__, *__vofa_package_tail__)


/**
 * @brief: bind float type to id.
 * @note: the id should be as small as possible.
 */
void ed_debugger_bind_float(int id, float* f);




#endif
