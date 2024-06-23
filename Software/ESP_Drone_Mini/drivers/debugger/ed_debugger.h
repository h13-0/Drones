#ifndef __ED_DEBUGGER_H__
#define __ED_DEBUGGER_H__

#include <stdint.h>

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
#define ed_debugger_send_float(...) ed_debugger_send_vofa(COUNT_ARGS(X ##__VA_ARGS__), ##__VA_ARGS__, *__vofa_package_tail__)


/**
 * @brief: bind float type to id.
 * @note: the id should be as small as possible.
 */
void ed_debugger_bind_float(int id, float* f);




#endif
