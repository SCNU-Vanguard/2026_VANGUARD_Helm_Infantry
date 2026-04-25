#ifndef __VPC_H__
#define __VPC_H__

#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "main.h"
#include "semphr.h"

#include "INS.h"


void VPC_Init(void);
void VPC_UpdatePackets(void);
void Choose_VPC_Type(void);

#endif /* __VPC_H__ */
