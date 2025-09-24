/*******************************************************************************
* \file mtb_ctp_ft3268.c
*
* \brief
* Provides implementation of the FT3268 touch panel driver library.
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


/*******************************************************************************
* Header Files
*******************************************************************************/
#include "mtb_ctp_ft3268.h"

/*******************************************************************************
* Macros
*******************************************************************************/

/* I2C slave address */
#define MTB_CTP_FT3268_I2C_ADDRESS         (0x38)

#define RST_PIN_LOW_HOLD_TIME_MS           (5U)

#define POWER_ON_DELAY_MS                  (70U)

#define RETRY_COUNT                        (5U)

#define I2C_TIMEOUT                        (5U)

#define I2C_DELAY_MS                       (1U)

#define TP_INTERRUPT_PRIORITY              (3U)

#define NUM_CMD                            (4U)

#define TOUCH_BUFFER_SIZE                  (32U)

#define RESET_VAL                          (0U)

#define SET_VAL                            (1U)

#define DISP_VERTICAL_MAX                  (465U)

#define DISP_HORIZONTAL_MAX                (465U)

#define TP_MAX_VAL_X                       (360U)

#define TP_MAX_VAL_Y                       (360U)

#define GPIO_LOW                           (0U)

#define GPIO_HIGH                          (1U)

#define ERROR_STATUS                       (0U)


#define TOUCH_POINT_GET_EVENT(T) ((mtb_ctp_touch_event_t)(uint8_t)((T).XH >> 6U))
#define TOUCH_POINT_GET_ID(T)    ((T).YH >> 4)
#define TOUCH_POINT_GET_X(T)     (int)((((uint16_t)(T).XH & 0x0fU) << 8) | \
                                      (uint16_t)(T).XL)
#define TOUCH_POINT_GET_Y(T)     (int)((((uint16_t)(T).YH & 0x0fU) << 8) | \
                                      (uint16_t)(T).YL)

/*******************************************************************************
* Global Variables
*******************************************************************************/
static uint8_t ft3268_param_map[NUM_CMD] = { MTB_CTP_VAL_CHIP_ID,
                                             MTB_CTP_VAL_CHIP_ID2,
                                             MTB_CTP_VAL_FW_VER,
                                             MTB_CTP_VAL_VENDOR_ID };

static mtb_ctp_ft3268_config_t* ft3268_config = NULL;


/*******************************************************************************
* Function Name: mtb_ctp_i2c_write_packet
********************************************************************************
* This function performs I2C master write operation
*
* \param *write_buffer
* Pointer to I2C write buffer
*
* \param buff_size
* I2C write buffer size
*
* \return cy_en_scb_i2c_status_t
* I2C error status
*
*******************************************************************************/
static cy_en_scb_i2c_status_t mtb_ctp_i2c_write_packet(uint8_t* write_buffer,
                                                       int buff_size)
{
    cy_en_scb_i2c_status_t i2c_status = CY_SCB_I2C_SUCCESS;
    cy_stc_scb_i2c_master_xfer_config_t transfer;
    uint8_t retry_count = RETRY_COUNT;
    uint8_t timeout_count  = I2C_TIMEOUT;

    /* Configure write transaction */
    transfer.slaveAddress = MTB_CTP_FT3268_I2C_ADDRESS;
    transfer.buffer       = write_buffer;
    transfer.bufferSize   = buff_size;

    /* Generate Stop condition the end of transaction */
    transfer.xferPending  = false;

    do
    {
        /* Initiate write transaction. */
        /* The Start condition is generated to begin this transaction */
        i2c_status = Cy_SCB_I2C_MasterWrite(ft3268_config->scb_inst, &transfer,
                                            ft3268_config->i2c_context);

        i2c_status =  (cy_en_scb_i2c_status_t)Cy_SCB_I2C_MasterGetStatus(ft3268_config->scb_inst,
                                                                         ft3268_config->i2c_context);

        /* Wait for transaction completion */
        while ((CY_SCB_I2C_MASTER_BUSY & i2c_status) &&
               (RESET_VAL != timeout_count))
        {
            Cy_SysLib_Delay(I2C_DELAY_MS);
            timeout_count--;
            i2c_status = (cy_en_scb_i2c_status_t)Cy_SCB_I2C_MasterGetStatus(ft3268_config->scb_inst,
                                                                            ft3268_config->i2c_context);
        }

        /* Check of the I2C master write is a success */
        if (CY_SCB_I2C_MASTER_WR_CMPLT_EVENT != i2c_status)
        {
            retry_count--;
            timeout_count = I2C_TIMEOUT;
        }
        else
        {
            retry_count = RESET_VAL;
        }
    } while (retry_count);

    return i2c_status;
}


/*******************************************************************************
* Function Name: mtb_ctp_i2c_read_packet
********************************************************************************
* This function performs I2C master read operation
*
* \param *read_buffer
* Pointer to I2C read buffer
*
* \param buff_size
* I2C read buffer size
*
* \return cy_en_scb_i2c_status_t
* I2C error status
*
*******************************************************************************/
static cy_en_scb_i2c_status_t mtb_ctp_i2c_read_packet(uint8_t* read_buffer,
                                                      int buff_size)
{
    cy_en_scb_i2c_status_t i2c_status = CY_SCB_I2C_SUCCESS;
    cy_stc_scb_i2c_master_xfer_config_t transfer;
    uint8_t retry_count = RETRY_COUNT;
    uint8_t timeout_count = I2C_TIMEOUT;

    /* Configure read transaction */
    transfer.slaveAddress = MTB_CTP_FT3268_I2C_ADDRESS;
    transfer.buffer       = read_buffer;
    transfer.bufferSize   = buff_size;

    /* Generate Stop condition the end of transaction */
    transfer.xferPending  = false;

    do
    {
        /* Initiate read transaction. */
        i2c_status = Cy_SCB_I2C_MasterRead(ft3268_config->scb_inst, &transfer,
                                           ft3268_config->i2c_context);

        i2c_status =  (cy_en_scb_i2c_status_t)Cy_SCB_I2C_MasterGetStatus(ft3268_config->scb_inst,
                                                                         ft3268_config->i2c_context);

        /* Wait for transaction completion */
        while ((CY_SCB_I2C_MASTER_BUSY & i2c_status) && (RESET_VAL != timeout_count))
        {
            Cy_SysLib_Delay(I2C_DELAY_MS);
            timeout_count--;
            i2c_status =  (cy_en_scb_i2c_status_t)Cy_SCB_I2C_MasterGetStatus(
                ft3268_config->scb_inst,
                ft3268_config->i2c_context);
        }

        /* Check of the I2C master read is a success */
        if (CY_SCB_I2C_SUCCESS != i2c_status)
        {
            retry_count--;
            timeout_count = I2C_TIMEOUT;
        }
        else
        {
            retry_count = RESET_VAL;
        }
    } while (retry_count);

    return i2c_status;
}


/*******************************************************************************
* Function Name: mtb_ctp_ft3268_is_alive
********************************************************************************
* This function reads the chip and firmware details of ft3268
*
* \param void
*
* \return cy_rslt_t
* Error status
*
*******************************************************************************/
static cy_rslt_t mtb_ctp_ft3268_is_alive(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    cy_en_scb_i2c_status_t i2c_status = CY_SCB_I2C_SUCCESS;
    uint8_t cmd[NUM_CMD] = { MTB_CTP_REG_CHIP_ID, MTB_CTP_REG_CHIP_ID2, MTB_CTP_REG_FW_VER,
                             MTB_CTP_REG_VENDOR_ID };
    uint8_t idx;
    uint8_t init_data[NUM_CMD] = { 0 };

    /* Verify chip information */
    for (idx = 0; idx < NUM_CMD; idx++)
    {
        i2c_status = mtb_ctp_i2c_write_packet(&cmd[idx], sizeof(cmd[0]));
        if (CY_SCB_I2C_MASTER_WR_CMPLT_EVENT == i2c_status)
        {
            i2c_status = mtb_ctp_i2c_read_packet(&init_data[idx], sizeof(cmd[0]));
            if ((CY_SCB_I2C_SUCCESS == i2c_status))
            {
                if (ft3268_param_map[idx] != init_data[idx])
                {
                    result =  CY_RSLT_FT3268_DEV_ERR;
                    break;
                }
            }
            else
            {
                result = (cy_rslt_t)i2c_status;
                break;
            }
        }
        else
        {
            result = (cy_rslt_t)i2c_status;
            break;
        }
    }

    return result;
}


/*******************************************************************************
* Function Name: mtb_ctp_ft3268_interrupt_handler
********************************************************************************
* Touch panel GPIO interrupt handler to detect touch events on the sensor.
* When a finger touches on the sensor surface, the INT pin will be pulled high
* by FT3268 and we set the touch_event flag true.
*
* \param void
*
* \return void
*
*******************************************************************************/
__WEAK void mtb_ctp_ft3268_interrupt_handler(void)
{
    if ((NULL != ft3268_config) &&
        Cy_GPIO_GetInterruptStatus(ft3268_config->int_port, ft3268_config->int_pin))
    {
        ft3268_config->touch_event = true;
        Cy_GPIO_ClearInterrupt(ft3268_config->int_port, ft3268_config->int_pin);
    }
}


/*******************************************************************************
* Function Name: mtb_ctp_ft3268_init
********************************************************************************
* Performs FT3268 Touch panel driver initialization using I2C interface.
*
* \param *mtb_ft3268_config
* Pointer to FT3268 configuration structure
*
* \return cy_rslt_t
* Initialization status.
*
* \func usage
* \snippet snippet/main.c mtb_ctp_ft3268_init
*
*******************************************************************************/
cy_rslt_t mtb_ctp_ft3268_init(mtb_ctp_ft3268_config_t* mtb_ft3268_config)
{
    cy_rslt_t status    = CY_RSLT_SUCCESS;

    CY_ASSERT(NULL != mtb_ft3268_config);

    ft3268_config = mtb_ft3268_config;

    cy_stc_sysint_t ctp_irq_cfg =
    {
        .intrSrc        = ft3268_config->int_num,
        .intrPriority   = TP_INTERRUPT_PRIORITY
    };

    /* Initialize CTP RESET GPIO pin with initial value as HIGH */
    Cy_GPIO_Pin_FastInit(ft3268_config->rst_port, ft3268_config->rst_pin,
                         CY_GPIO_DM_STRONG, GPIO_HIGH, HSIOM_SEL_GPIO);

    /* Perform reset sequence to enable FT3268 touch controller */
    Cy_GPIO_Write(ft3268_config->rst_port, ft3268_config->rst_pin, GPIO_LOW);
    Cy_SysLib_Delay(RST_PIN_LOW_HOLD_TIME_MS);
    Cy_GPIO_Write(ft3268_config->rst_port, ft3268_config->rst_pin, GPIO_HIGH);
    Cy_SysLib_Delay(POWER_ON_DELAY_MS);

    /* Check whether FT3268 touch controller is alive or not */
    status = mtb_ctp_ft3268_is_alive();

    if (CY_RSLT_SUCCESS == status)
    {
        /* Initialize CTP IRQ pin and config touch interrupt */
        Cy_GPIO_Pin_FastInit(ft3268_config->int_port, ft3268_config->int_pin, CY_GPIO_DM_PULLUP,
                             GPIO_HIGH, HSIOM_SEL_GPIO);
        Cy_GPIO_SetInterruptEdge(ft3268_config->int_port, ft3268_config->int_pin,
                                 CY_GPIO_INTR_FALLING);
        Cy_GPIO_SetInterruptMask(ft3268_config->int_port, ft3268_config->int_pin, SET_VAL);
        Cy_SysInt_Init(&ctp_irq_cfg, &mtb_ctp_ft3268_interrupt_handler);
        NVIC_EnableIRQ(ctp_irq_cfg.intrSrc);
    }
    else
    {
        status = CY_RSLT_FT3268_DEV_ERR;
    }

    return status;
}


/*******************************************************************************
* Function Name: mtb_ctp_ft3268_read_raw_touch_data
********************************************************************************
* This function reads the raw x, y coordinate data from the touch sensor.
*
* \param *touch_data
* Pointer to the buffer to store touch data.
*
* \param size
* Size of the buffer to store touch data.
*
* \return cy_en_scb_i2c_status_t
* I2C error status
*
*******************************************************************************/
static cy_en_scb_i2c_status_t mtb_ctp_ft3268_read_raw_touch_data(uint8_t* touch_data,
                                                                 int size)
{
    cy_en_scb_i2c_status_t i2c_status = CY_SCB_I2C_SUCCESS;
    uint8_t get_touch_data = MTB_CTP_REG_READ_TOUCH_DATA;

    if (NULL != touch_data)
    {
        i2c_status = mtb_ctp_i2c_write_packet(&get_touch_data, sizeof(get_touch_data));
        if (CY_SCB_I2C_MASTER_WR_CMPLT_EVENT == i2c_status)
        {
            i2c_status = mtb_ctp_i2c_read_packet(touch_data, size);
        }
    }

    return i2c_status;
}


/*******************************************************************************
* Function Name: mtb_ctp_ft3268_get_single_touch
********************************************************************************
* Reads touch data from the FT3268 touch panel driver using I2C interface.
*
* \param *touch_event
* Pointer to the variable to hold detected touch event.
*
* \param *touch_x
* Pointer to the variable of X touch co-ordinate.
*
* \param *touch_y
* Pointer to the variable of Y touch co-ordinate
*
* \return cy_en_scb_i2c_status_t
* I2C error status
*
*******************************************************************************/
cy_en_scb_i2c_status_t mtb_ctp_ft3268_get_single_touch(mtb_ctp_touch_event_t* touch_event,
                                                       int* touch_x, int* touch_y)
{
    cy_en_scb_i2c_status_t i2c_status = CY_SCB_I2C_SUCCESS;
    static uint8_t touch_buff[TOUCH_BUFFER_SIZE];
    mtb_ctp_touch_event_t touch_event_local;

    i2c_status = mtb_ctp_ft3268_read_raw_touch_data(touch_buff, sizeof(touch_buff));

    if (CY_SCB_I2C_SUCCESS == i2c_status)
    {
        mtb_ctp_ft3268_touch_data_t* touch_data = (mtb_ctp_ft3268_touch_data_t*)(void*)(touch_buff);

        touch_event_local = TOUCH_POINT_GET_EVENT(touch_data->touch_points[0]);

        /* Update coordinates only if there is touch detected */
        if ((MTB_CTP_TOUCH_DOWN == touch_event_local) \
            || (MTB_CTP_TOUCH_CONTACT == touch_event_local))
        {
            if (NULL != touch_x)
            {
                *touch_x = TOUCH_POINT_GET_X(touch_data->touch_points[0]);
            }
            if (NULL != touch_y)
            {
                *touch_y = TOUCH_POINT_GET_Y(touch_data->touch_points[0]);
            }
        }

        if (NULL != touch_event)
        {
            *touch_event = touch_event_local;
        }
    }

    return i2c_status;
}


/* [] END OF FILE */
