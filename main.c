/******************************************************************************
* File Name:   main.c
*
*This example demonstrates the fault handling functionality of
*              T2G MCU and how to add a custom fault handling function to
*              find the fault location. The UART interface will be used for
*              showing ARM register information
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2022, Cypress Semiconductor Corporation (an Infineon company) or
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
*******************************************************************************/

/******************************************************************************
* Include header files
******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"


/*******************************************************************************
* Macros
********************************************************************************/
#define CMD_USAGE_FAULT        ('u')
#define CMD_BUS_FAULT          ('b')
#define UART_TIMEOUT_MS        (10)    /* in milliseconds */

/*******************************************************************************
* Function Prototypes
********************************************************************************/
static void configure_fault_register(void);
static void force_bus_fault(void);
static uint32_t force_usage_fault(volatile uint32_t* intVal);

/*******************************************************************************
* Global Variables
********************************************************************************/
/* Variable used for generating Bus Fault by modifying the constant variable */
const uint32_t write_to_cause_fault_cm7 = 0u;

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  This is the main function for CM7 CPU. This function
*    1. initializes BSP
*    2. retarget-io for debug message printing on terminal application
*    3. configure fault registers
*    4. check for the user input and process the user command
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    uint8_t cmd;
    volatile uint32_t var = 0;
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                        CY_RETARGET_IO_BAUDRATE);

    configure_fault_register();

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("****************** "
           " MCU: Fault Handling Basics "
           "******************\r\n\n");

    printf("Press 'u' key to create Usage Fault\r\n");
    printf("Press 'b' key to create Bus Fault\r\n");
    printf("Press the Reset button to start over after triggering a fault\r\n");

    for (;;)
    {
        result = cyhal_uart_getc(&cy_retarget_io_uart_obj, &cmd, \
                                 UART_TIMEOUT_MS);
        if (result != CY_RSLT_ERR_CSP_UART_GETC_TIMEOUT)
        {
            if (CMD_BUS_FAULT == cmd)
            {
                printf("\r\nForce Bus Fault!\r\n");
                force_bus_fault();
            }
            if (CMD_USAGE_FAULT == cmd)
            {

                printf("\r\nForce Usage Fault!\r\n");

                if (0 != force_usage_fault(&var))
                {
                    printf("\rFailed to create CM4 Usage"\
                           " Fault!\r\n");
                }

            }

        }
    }
}


/*******************************************************************************
* Function Name: configure_fault_register
********************************************************************************
* Summary:
*  This function configures the fault registers(bus fault and usage fault). See
*  the Arm documentation for more details on the registers.
*
*******************************************************************************/
static void configure_fault_register(void)
{
    /* Set SCB->SHCSR.BUSFAULTENA so that BusFault handler instead of the
     * HardFault handler handles the BusFault.
     */
    SCB->SHCSR |= SCB_SHCSR_BUSFAULTENA_Msk;

    /* Enable UsageFault when processor executes an divide by 0 */
    SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk;

    /* Set SCB->SHCSR.USGFAULTENA so that faults such as DIVBYZERO, UNALIGNED,
     * UNDEFINSTR etc are handled by UsageFault handler instead of the HardFault
     * handler.
     */
    SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk;

}

/*******************************************************************************
* Function Name: force_bus_fault
********************************************************************************
* Summary:
*  The BusFault exception will be generated intentionally by writing a value
*  into read-only area.
*
*******************************************************************************/
static void force_bus_fault(void)
{
    *(volatile uint32_t *)&write_to_cause_fault_cm7 = 10u;
}


/*******************************************************************************
* Function Name: force_usage_fault
********************************************************************************
* Summary:
*  This function divides 100 by intVal. It will generate the usage fault, if the
*  intVal is zero.
*  This simple logic is intentionally used to avoid having the compiler
*  optimize out the code.
*
* Parameters:
*  uint32_t* intVal : variable used to create usage fault
*
* Return:
*  uint32_t
*
*******************************************************************************/
static uint32_t force_usage_fault(volatile uint32_t* intVal)
{
    volatile uint32_t fault_num = 100u;

    /* If *intVal = 0u then it triggers a fault because of DIVBYZERO (Divide by
     * zero). The SCB->UFSR bit 9(=CFSR bit 25) will be set to 1, once the fault
     * has occurred.
     */
    fault_num /= *intVal;

    return fault_num;
}

/*******************************************************************************
* Function Name: void Cy_SysLib_ProcessingFault(void)
********************************************************************************
* Summary:
*  This function prints out the stack register at the moment the hard fault
*  occurred. cy_syslib.c defines this as a  __WEAK function, so this function
*  replaces the weak function. Cy_SysLib_ProcessingFault() is called at the
*  end of Cy_SysLib_FaultHandler() function, which is the default exception
*  handler set for hard faults.
*
*******************************************************************************/
void Cy_SysLib_ProcessingFault(void)
{
    /* If Bus Fault valid or MemManage fault bit is set to 1, print Bus Ffault or Usage Fault Address */
    if (cy_faultFrame.cfsr.cfsrBits.imprecisErr || cy_faultFrame.cfsr.cfsrBits.divByZero)
    {
        printf("Bus Fault! \r\nFault address = 0x%08lx\r\n", (unsigned long)cy_faultFrame.cfsr.cfsrReg);
    }
    printf("r0 = 0x%08lx\r\n", (unsigned long)cy_faultFrame.r0);
    printf("r1 = 0x%08lx\r\n", (unsigned long)cy_faultFrame.r1);
    printf("r2 = 0x%08lx\r\n", (unsigned long)cy_faultFrame.r2);
    printf("r3 = 0x%08lx\r\n", (unsigned long)cy_faultFrame.r3);
    printf("r12 = 0x%08lx\r\n", (unsigned long)cy_faultFrame.r12);
    printf("lr = 0x%08lx\r\n", (unsigned long)cy_faultFrame.lr);
    printf("pc = 0x%08lx\r\n", (unsigned long)cy_faultFrame.pc);
    printf("psr = 0x%08lx\r\n", (unsigned long)cy_faultFrame.psr);

    while (1);
}

/* [] END OF FILE */
