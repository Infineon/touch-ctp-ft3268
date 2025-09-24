# FT3268 capacitive touch panel driver library API reference guide

## General description

Basic set of API for interacting with the touch panel. This library provides functions for supporting a touch panel of 1.43 inch circular [CO5300](https://admin.osptek.com/uploads/CO_5300_Datasheet_V0_00_20230328_07edb82936.pdf) wearable display driven by an FT3268 touch panel driver.

## Code snippets

### Snippet 1: Touch driver initialization and reading touch data

The following snippet initializes the touch panel driver and reads touch data from it.

  ```
  #include "mtb_ctp_ft3268.h"
  #include "cybsp.h"

  /*****************************************************************************
  * Macros
  *****************************************************************************/
  #define INT_PRIORITY (2UL)
  /* Following GPIO pins should be configured per board schematics in the
   * Device Configurator. 
   * The values provided here serve as an example from driver API usage
   * perspective.
   */
  #define INT_PIN_NUM (2U)
  #define RST_PIN_NUM (3U)


  /*****************************************************************************
  * Global Variables
  *****************************************************************************/
  cy_stc_scb_i2c_context_t i2c_controller_context;

  cy_stc_sysint_t mI2C_SCB_IRQ_cfg =
  {
      .intrSrc      = CYBSP_I2C_CONTROLLER_IRQ,
      .intrPriority = INT_PRIORITY
  };


  /*****************************************************************************
  * Function name: i2c_interrupt_callback
  *****************************************************************************/
  void i2c_interrupt_callback(void)
  {
      Cy_SCB_I2C_Interrupt(CYBSP_I2C_CONTROLLER_HW, &i2c_controller_context);
  }


  /*****************************************************************************
  * Code
  *****************************************************************************/
  int main(void)
  {
      mtb_ctp_touch_event_t touch_event;

      cy_en_scb_i2c_status_t i2c_status;
      cy_rslt_t result;
      int x = 0;
      int y = 0;

      mtb_ctp_ft3268_config_t ft3268_params =
      {
          .scb_inst    = CYBSP_I2C_CONTROLLER_HW,
          .i2c_context = &i2c_controller_context,
          .rst_port    = GPIO_PRT17,
          .int_port    = GPIO_PRT17,
          .rst_pin     = RST_PIN_NUM,
          .int_pin     = INT_PIN_NUM,
          .int_num     = ioss_interrupts_gpio_17_IRQn,
      };

      /* Configures SCB0 as I2C controller. */
      i2c_status = Cy_SCB_I2C_Init(CYBSP_I2C_CONTROLLER_HW, &CYBSP_I2C_CONTROLLER_config, &i2c_controller_context);
      if (CY_SCB_I2C_SUCCESS != i2c_status )
      {
          /* Handles possible errors. */
          CY_ASSERT(0);
      }

      /* Interrupt initialization for SCB block */
      Cy_SysInt_Init(&mI2C_SCB_IRQ_cfg, &i2c_interrupt_callback);
      NVIC_EnableIRQ((IRQn_Type)mI2C_SCB_IRQ_cfg.intrSrc);

      /* Enable I2C */
      Cy_SCB_I2C_Enable(CYBSP_I2C_CONTROLLER_HW);

      /* Initialize FT3268 touch driver */
      result = mtb_ctp_ft3268_init(&ft3268_params);
      if (CY_RSLT_SUCCESS == result)
      {
          /* Check if touch event is detected */
          if(ft3268_params.touch_event)
          {
              /* Read touch coordinates */
              i2c_status = mtb_ctp_ft3268_get_single_touch(&touch_event, &x, &y);
              if (CY_SCB_I2C_SUCCESS != i2c_status)
              {
                  CY_ASSERT(0);
              }
              ft3268_params.touch_event = false;
          }
      }

      for (;;)
      {
      }
  }
  ```

## Global variables
```
static uint32_t prev_touch_x                    Last X touch coordinate
static uint32_t prev_touch_y                    Last Y touch coordinate
static uint8_t touch_buff[TOUCH_BUFFER_SIZE]    Buffer for storing touch data
static uint8_t ft3268_param_map[NUM_CMD]        Chip data map
```


## Functions
cy_rslt_t `mtb_ctp_ft3268_init(mtb_ctp_ft3268_config_t* mtb_ctp_ft3268_config)`
>Performs initialization of the FT3268 touch panel driver using I2C interface.

cy_en_scb_i2c_status_t `mtb_ctp_ft3268_get_single_touch(mtb_ctp_touch_event_t* touch_event,
                                                        int* touch_x, int* touch_y)`
>Reads touch data from the FT3268 touch panel driver using I2C interface.


## Function documentation
#### mtb_ctp_ft3268_init
- cy_rslt_t `mtb_ctp_ft3268_init(mtb_ctp_ft3268_config_t* mtb_ctp_ft3268_config)`

> **Summary:** Performs FT3268 Touch panel driver initialization using I2C interface.
>
> **Parameter:**
>  Parameters                    |  Description       
>  :-------                      |  :------------
>  [in] mtb_ctp_ft3268_config    |  Pointer to the structure containing configurations for FT3268
>
> Return:
>  - cy_rslt_t       :              Error status

#### mtb_ctp_ft3268_get_single_touch
- cy_en_scb_i2c_status_t `mtb_ctp_ft3268_get_single_touch(mtb_ctp_touch_event_t* touch_event,
                                                          int* touch_x, int* touch_y)`
> **Summary:** Reads touch data from the FT3268 touch panel driver using I2C interface.
> 
> **Parameter:**
>  Parameters            |  Description       
>  :-------              |  :------------
>  [in] touch_event      |  Pointer to the variable to hold detected touch event
>  [in] touch_x          |  Pointer to the variable of X touch co-ordinate
>  [in] touch_y          |  Pointer to the variable of Y touch co-ordinate
>
> Return:
>  - cy_en_scb_i2c_status_t       :         I2C error status

#### mtb_ctp_ft3268_interrupt_handler
- __WEAK void `mtb_ctp_ft3268_interrupt_handler(void)`
> **Summary:** Interrupt handler for touch panel interrupt. This function has a weak linkage to enable user defined implementations.
>

---
© 2025, Cypress Semiconductor Corporation (an Infineon company)
