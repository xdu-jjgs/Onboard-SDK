/**
 ********************************************************************
 * @file    osdk_timeSync.h
 * @version V2.0.0
 * @date    2019/07/01
 * @brief   This is the header file for "osdk_timeSync.c", defining the
 *structure and
 * (exported) function prototypes.
 *
 * @copyright (c) 2018-2019 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef OSDK_TIMESYNC_H
#define OSDK_TIMESYNC_H

/* Includes ------------------------------------------------------------------*/
#include "osdk_command_instance.h"
#include "osdk_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
E_OsdkStat OsdkTimeSync_Init(void);
E_OsdkStat OsdkTimeSync_GetSyncTimeMs(uint32_t *syncTimeMs);

#ifdef __cplusplus
}
#endif

#endif  // OSDK_TIMESYNC_H
/************************ (C) COPYRIGHT DJI Innovations *******END OF FILE******/
