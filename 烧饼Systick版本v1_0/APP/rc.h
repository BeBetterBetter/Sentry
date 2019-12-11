#ifndef __RC_H
#define __RC_H

#include "system.h"
#include "usart2.h"
#include "myMath.h"
#include "myInclude.h"
#include "chassis.h"
//

void RemotrCtrlInit(void);
void RemoteCtrlRecieve(void);
void RemoteCtrlProtocol(void);

void GimbalRemoteCtrlProtocol(void);


#endif

