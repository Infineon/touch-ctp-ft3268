/*******************************************************************************
* \file mtb_ctp_ft3268.h
*
* \brief
* Provides constants, parameter values, and API prototypes for the FT3268
* touch panel driver library.
*
********************************************************************************
* \copyright
* Copyright 2025 Cypress Semiconductor Corporation (an Infineon company)
*
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#ifndef MTB_CTP_FT3268_H
#define MTB_CTP_FT3268_H


#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cy_pdl.h"


/*******************************************************************************
* Macros
*******************************************************************************/

/*! @brief Error code definition. */
#define CY_RSLT_FT3268_ERR_BASE       (0x9400U)

/* FT3268 device error */
#define CY_RSLT_FT3268_DEV_ERR \
               CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR, CY_RSLT_FT3268_ERR_BASE, 2)

/*! @brief FT3268 maximum number of simultaneously detected touches. */
#define MTB_CTP_FT3268_MAX_TOUCHES    (5U)

/* Touch controller registers */
#define MTB_CTP_REG_CHIP_ID           (0xA3)

#define MTB_CTP_REG_CHIP_ID2          (0x9F)

#define MTB_CTP_REG_FW_VER            (0xA6)

#define MTB_CTP_REG_VENDOR_ID         (0xA8)

#define MTB_CTP_REG_READ_TOUCH_DATA   (0x01)

/* FT3268 chip parameters */
#define MTB_CTP_VAL_CHIP_ID           (0x64)

#define MTB_CTP_VAL_CHIP_ID2          (0x56)

#define MTB_CTP_VAL_FW_VER            (0x12)

#define MTB_CTP_VAL_VENDOR_ID         (0x11)

/*******************************************************************************
* Data Structures
*******************************************************************************/
/* Touch events */
typedef enum
{
    MTB_CTP_TOUCH_DOWN,    /* The state changed to touched. */
    MTB_CTP_TOUCH_UP,      /* The state changed to not touched. */
    MTB_CTP_TOUCH_CONTACT, /* There is a continuous touch being detected. */
    MTB_CTP_TOUCH_RESERVED /* No touch information available. */
} mtb_ctp_touch_event_t;

/* Touch point definition */
typedef struct
{
    uint8_t XH;
    uint8_t XL;
    uint8_t YH;
    uint8_t YL;
    uint8_t reserved[2];
} mtb_ctp_ft3268_touch_point_t;

typedef struct
{
    uint8_t gesture_ID;                                                    /* Gesture ID */
    uint8_t touch_detection_count;                                         /* Touch detection count
                                                                            */
    mtb_ctp_ft3268_touch_point_t touch_points[MTB_CTP_FT3268_MAX_TOUCHES]; /* Touch point values */
} mtb_ctp_ft3268_touch_data_t;

/* FT3268 touch controller configuration structure */
typedef struct
{
    CySCB_Type* scb_inst;
    cy_stc_scb_i2c_context_t* i2c_context;
    GPIO_PRT_Type* rst_port;
    uint32_t rst_pin;
    GPIO_PRT_Type* int_port;
    uint32_t int_pin;
    IRQn_Type int_num;
    volatile bool touch_event;
} mtb_ctp_ft3268_config_t;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
cy_rslt_t mtb_ctp_ft3268_init(mtb_ctp_ft3268_config_t* mtb_ctp_ft3268_config);
cy_en_scb_i2c_status_t mtb_ctp_ft3268_get_single_touch(mtb_ctp_touch_event_t* touch_event,
                                                       int* touch_x, int* touch_y);
void mtb_ctp_ft3268_interrupt_handler(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* MTB_CTP_FT3268_H */

/* [] END OF FILE */
