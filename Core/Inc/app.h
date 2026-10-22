#ifndef __APP_H
#define __APP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void App_Init(void);
void App_OnLinkReset(void);
void App_OnTcpData(const uint8_t *data, uint16_t len);
void App_Process(void);

#ifdef __cplusplus
}
#endif

#endif
