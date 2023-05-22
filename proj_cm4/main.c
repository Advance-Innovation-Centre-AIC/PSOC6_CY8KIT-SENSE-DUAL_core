/*******************************************************************************
 * File Name:   main.c
 *
 * Description: This is the source code for CM4 in the the Dual CPU IPC Semaphore
 *              Application for ModusToolbox.
 *
 * Related Document: See README.md
 *
 *
 ********************************************************************************
 * Copyright 2020-2022, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 ********************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "ipc_def.h"
#include "cy_retarget_io.h"
#include "xensiv_dps3xx_mtb.h"

// Set LED status semaphore
#if ENABLE_SEMA
#define LED_STATE   CYBSP_LED_STATE_OFF
#else
#define LED_STATE   CYBSP_LED_STATE_ON
#endif
// Set sensor interface i2c and parameter
#define ADC_MAX_VALUE 4095
#define PRESSURE_SENSOR_VOLTAGE 3.3
#define PRESSURE_SENSOR_SLOPE 0.04
#define PRESSURE_SENSOR_OFFSET 0.1

xensiv_dps3xx_t pressure_sensor;
cyhal_i2c_t i2c;
cyhal_i2c_cfg_t i2c_cfg = {
    .is_slave = false,
    .address = 0,
    .frequencyhal_hz = 400000
};

#define DPS_I2C_SDA (P6_1)
#define DPS_I2C_SCL (P6_0)

int main(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    float voltage, pressure_hPa, pressure_mbar;
    /* Initialize the device and board peripherals */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Free the hardware instance object iff initialized by other core
     * before initializing the same hardware instance object in this core. */
    cyhal_hwmgr_free(&CYBSP_UART_obj);
    cyhal_hwmgr_free(&CYBSP_DEBUG_UART_RX_obj);
    cyhal_hwmgr_free(&CYBSP_DEBUG_UART_TX_obj);
    cyhal_hwmgr_free(&CYBSP_SW2_obj);

    /* Initialize retarget-io to use the debug UART port. */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
            CY_RETARGET_IO_BAUDRATE);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    /* Initialize the User Button */
    result = cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    /* Initialize the User LED */
    result = cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, LED_STATE);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    /* Initialize i2c for pressure sensor */
    result = cyhal_i2c_init(&i2c, DPS_I2C_SDA, DPS_I2C_SCL, NULL);
    CY_ASSERT(result == CY_RSLT_SUCCESS);
    result = cyhal_i2c_configure(&i2c, &i2c_cfg);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    /* Initialize pressure sensor */
    result = xensiv_dps3xx_mtb_init_i2c(&pressure_sensor, &i2c, XENSIV_DPS3XX_I2C_ADDR_DEFAULT);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    /* Unlock the semaphore and wake-up the CM0+ */
    Cy_IPC_Sema_Clear(SEMA_NUM, false);
    __SEV();

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("******************  IPC Semaphore Example  ****************** \r\n\n");

    printf("<Press the kit's user button to print messages>\r\n\n");

    for (;;)
    {
        if (cyhal_gpio_read(CYBSP_USER_BTN) == CYBSP_BTN_PRESSED)
        {
        #if ENABLE_SEMA
            /* Attempt to lock the semaphore */
            if (Cy_IPC_Sema_Set(SEMA_NUM, false) == CY_IPC_SEMA_SUCCESS)
        #endif
            {
                /* Pressure sensor task */
                printf("Pressure sent from CM4\r\n");
                /* Get the pressure and temperature data and print the results to the UART */
                       float pressure, temperature;
                       xensiv_dps3xx_read(&pressure_sensor, &pressure, &temperature);

                       // Convert the ADC reading to voltage
                           voltage = (float) pressure * PRESSURE_SENSOR_VOLTAGE / ADC_MAX_VALUE;

                           // Convert the voltage to pressure in hPa
                           pressure_hPa = (voltage - PRESSURE_SENSOR_OFFSET) / PRESSURE_SENSOR_SLOPE;

                           // Convert the pressure to mbar
                           pressure_mbar = pressure_hPa * 10;

                       printf("Pressure   : %8f\r\n", pressure);
                       printf("Pressure (hectopascal)   : %8f hPa\r\n", pressure_hPa);
                       printf("Pressure (millibar)  : %8f mbar\r\n", pressure_mbar);
                       printf("Temperature : %8f Celcius\r\n", temperature);
                       // For Serial Studio
                       //printf("/*%.2f,%.2f,%.2f*/\r\n",pressure_hPa, pressure_mbar);

                       cyhal_system_delay_ms(100);

            #if ENABLE_SEMA
                while (CY_IPC_SEMA_SUCCESS != Cy_IPC_Sema_Clear(SEMA_NUM, false));
            #endif
            }
        }
    }

}

/* [] END OF FILE */
