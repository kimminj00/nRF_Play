/**
 * Copyright (c) 2016 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "app_error.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "app_util_platform.h"
#include "nrf_strerror.h"

#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
#include "nrf_sdm.h"
#endif

#if defined(SEONGJI_DEF_APPLICATION)
#include "hardfault.h"
#include "cfg_board_def.h"
#include "SEGGER_RTT.h"
#include "nrf_delay.h"
#include "cfg_stored_log.h"
#include "cfg_board.h"
#include "cfg_shared_ram_bl_app.h"

#ifndef CFSR_MMARVALID
  #define CFSR_MMARVALID (1 << (0 + 7))
#endif

#ifndef CFSR_BFARVALID
  #define CFSR_BFARVALID (1 << (8 + 7))
#endif
uint32_t m_app_error_fault_handler_lrReg;
uint32_t m_app_error_fault_cfsr_reg;
uint32_t m_app_error_fault_mmfar_reg;
uint32_t m_app_error_fault_bfar_reg;

#if !NRF_MODULE_ENABLED(HARDFAULT_HANDLER)
void HardFault_Handler(void) __attribute__(( naked ));

void HardFault_c_handler(uint32_t * p_stack_address)
{
    volatile int i;
    HardFault_stack_t * p_stack;

    p_stack = (HardFault_stack_t *)p_stack_address;
    m_app_error_fault_handler_lrReg = p_stack->lr;
    m_app_error_fault_cfsr_reg = SCB->CFSR;
    if (m_app_error_fault_cfsr_reg & CFSR_MMARVALID)
    {
        m_app_error_fault_mmfar_reg = SCB->MMFAR;
    }
    if (m_app_error_fault_cfsr_reg & CFSR_BFARVALID)
    {
        m_app_error_fault_bfar_reg = SCB->BFAR;
    }
#ifndef FUNCTION_TEST_BOARD
    RTC2_date_time_check_N_backup_to_shared_mem(2);  //2 is reboot delay
    m_cfg_shared_ram_bl_app[CFG_SHARED_RAM_MEM_INDEX_ABNORMAL_RESET] = m_app_error_fault_handler_lrReg;
    imSTL_printf_nolock("%s pc:%08X,lr:%08X,CFSR:%08X,MMFAR:%08X,BFAR:%08X\n", imSTL_MSG_HEADER_HARDFAULT, p_stack->pc, p_stack->lr, m_app_error_fault_cfsr_reg, m_app_error_fault_mmfar_reg, m_app_error_fault_bfar_reg);   
#else
    char log_buf[128];
    unsigned log_size;
    log_size = sprintf(log_buf, "HardFault lr:0x%p\n", (void *)m_app_error_fault_handler_lrReg);
    SEGGER_RTT_Write(0, log_buf, log_size);
    i = 0x8fffff;
    while(--i);
#endif
    NVIC_SystemReset();

}

void HardFault_Handler(void)
{
    __ASM volatile(
    "   tst lr, #4                              \n"

    /* PSP is quite simple and does not require additional handler */
    "   itt ne                                  \n"
    "   mrsne r0, psp                           \n"
    /* Jump to the handler, do not store LR - returning from handler just exits exception */
    "   bne  HardFault_Handler_Continue         \n"

    /* Processing MSP requires stack checking */
    "   mrs r0, msp                             \n"

    "   ldr   r1, =__StackTop                   \n"
    "   ldr   r2, =__StackLimit                 \n"

    /* MSP is in the range of the stack area */
    "   cmp   r0, r1                            \n"
    "   bhi   HardFault_MoveSP                  \n"
    "   cmp   r0, r2                            \n"
    "   bhi   HardFault_Handler_Continue        \n"

    "HardFault_MoveSP:                          \n"
    "   mov   sp, r1                            \n"
    "   mov   r0, #0                            \n"

    "HardFault_Handler_Continue:                \n"
    "   ldr r3, =%0                             \n"
    "   bx r3                                   \n"
    "   .ltorg                                  \n"
    : : "X"(HardFault_c_handler)
    );
}
#endif

void MemoryManagement_Handler(void)
{
#ifndef FUNCTION_TEST_BOARD
    RTC2_date_time_check_N_backup_to_shared_mem(2);  //2 is reboot delay
    m_cfg_shared_ram_bl_app[CFG_SHARED_RAM_MEM_INDEX_ABNORMAL_RESET] = 1;
    imSTL_printf_nolock("%s MM\n", imSTL_MSG_HEADER_OTHERFAULT);
#endif
    NVIC_SystemReset();
}

void BusFault_Handler(void)
{
#ifndef FUNCTION_TEST_BOARD
    RTC2_date_time_check_N_backup_to_shared_mem(2);  //2 is reboot delay
    m_cfg_shared_ram_bl_app[CFG_SHARED_RAM_MEM_INDEX_ABNORMAL_RESET] = 2;
    imSTL_printf_nolock("%s BF\n", imSTL_MSG_HEADER_OTHERFAULT);
#endif
    NVIC_SystemReset();
}

void UsageFault_Handler(void)
{
#ifndef FUNCTION_TEST_BOARD
    RTC2_date_time_check_N_backup_to_shared_mem(2);  //2 is reboot delay
    m_cfg_shared_ram_bl_app[CFG_SHARED_RAM_MEM_INDEX_ABNORMAL_RESET] = 3;
    imSTL_printf_nolock("%s UF\n", imSTL_MSG_HEADER_OTHERFAULT);
#endif
    NVIC_SystemReset();
}

void SVC_Handler(void)
{
#ifndef FUNCTION_TEST_BOARD
    RTC2_date_time_check_N_backup_to_shared_mem(2);  //2 is reboot delay
    m_cfg_shared_ram_bl_app[CFG_SHARED_RAM_MEM_INDEX_ABNORMAL_RESET] = 4;
    imSTL_printf_nolock("%s SVC\n", imSTL_MSG_HEADER_OTHERFAULT);
#endif
    NVIC_SystemReset();
}

void PendSV_Handler(void)
{
#ifndef FUNCTION_TEST_BOARD
    RTC2_date_time_check_N_backup_to_shared_mem(2);  //2 is reboot delay
    m_cfg_shared_ram_bl_app[CFG_SHARED_RAM_MEM_INDEX_ABNORMAL_RESET] = 5;
    imSTL_printf_nolock("%s PSV\n", imSTL_MSG_HEADER_OTHERFAULT);
#endif
    NVIC_SystemReset();
}

void SysTick_Handler(void)
{
#ifndef FUNCTION_TEST_BOARD
    RTC2_date_time_check_N_backup_to_shared_mem(2);  //2 is reboot delay
    m_cfg_shared_ram_bl_app[CFG_SHARED_RAM_MEM_INDEX_ABNORMAL_RESET] = 6;
    imSTL_printf_nolock("%s STICK\n", imSTL_MSG_HEADER_OTHERFAULT);
#endif
    NVIC_SystemReset();
}

void Dummy_Handler(void)
{
#ifndef FUNCTION_TEST_BOARD
    RTC2_date_time_check_N_backup_to_shared_mem(2);  //2 is reboot delay
    m_cfg_shared_ram_bl_app[CFG_SHARED_RAM_MEM_INDEX_ABNORMAL_RESET] = 7;
    imSTL_printf_nolock("%s DUMMY\n", imSTL_MSG_HEADER_OTHERFAULT);
#endif
    NVIC_SystemReset();
}

#endif

/*lint -save -e14 */
/**
 * Function is implemented as weak so that it can be overwritten by custom application error handler
 * when needed.
 */
__WEAK void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    __disable_irq();
    NRF_LOG_FINAL_FLUSH();

#if defined(SEONGJI_DEF_APPLICATION)
    char log_buf[128];
    unsigned log_size;

    nrf_delay_ms(200);
    log_size = sprintf(log_buf, "app_error_fault! id: 0x%08x, pc: 0x%08x, info: 0x%08x, lr:0x%p\n", id, pc, info, m_app_error_fault_handler_lrReg);
    SEGGER_RTT_Write(0, log_buf, log_size);
#ifndef FUNCTION_TEST_BOARD
    RTC2_date_time_check_N_backup_to_shared_mem(2); //2 is reboot delay
    m_cfg_shared_ram_bl_app[CFG_SHARED_RAM_MEM_INDEX_ABNORMAL_RESET] = m_app_error_fault_handler_lrReg;
    imSTL_printf("%s pc:0x%08X, lr:0x%08X\n", imSTL_MSG_HEADER_APP_ERROR_FAULT, pc, m_app_error_fault_handler_lrReg);
    switch (id)
    {
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
        case NRF_FAULT_ID_SD_ASSERT:
            imSTL_printf("SD_ASSERT\n");
            break;
        case NRF_FAULT_ID_APP_MEMACC:
            imSTL_printf("APP_MEMACC\n");
            break;
#endif
        case NRF_FAULT_ID_SDK_ASSERT:
        {
            assert_info_t * p_info = (assert_info_t *)info;
            imSTL_printf("SDK_ASSERT at %s:%u\n", p_info->p_file_name, p_info->line_num);
            break;
        }
        case NRF_FAULT_ID_SDK_ERROR:
        {
            error_info_t * p_info = (error_info_t *)info;
            imSTL_printf("SDK_ERROR %u [%s] at %s:%u PC at: 0x%08x\n",
                        p_info->err_code,
                        nrf_strerror_get(p_info->err_code),
                        p_info->p_file_name,
                        p_info->line_num,
                        pc);
            break;
        }
        default:
            imSTL_printf("UNKNOWN FAULT at 0x%08X\n", pc);
            break;
    }
#endif
    nrf_delay_ms(200);
//    NRF_BREAKPOINT_COND;  // On assert, the system can only recover with a reset.
    
    NVIC_SystemReset();
#else
#ifndef DEBUG
    NRF_LOG_ERROR("Fatal error");
#else
    switch (id)
    {
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
        case NRF_FAULT_ID_SD_ASSERT:
            NRF_LOG_ERROR("SOFTDEVICE: ASSERTION FAILED");
            break;
        case NRF_FAULT_ID_APP_MEMACC:
            NRF_LOG_ERROR("SOFTDEVICE: INVALID MEMORY ACCESS");
            break;
#endif
        case NRF_FAULT_ID_SDK_ASSERT:
        {
            assert_info_t * p_info = (assert_info_t *)info;
            NRF_LOG_ERROR("ASSERTION FAILED at %s:%u",
                          p_info->p_file_name,
                          p_info->line_num);
            break;
        }
        case NRF_FAULT_ID_SDK_ERROR:
        {
            error_info_t * p_info = (error_info_t *)info;
            NRF_LOG_ERROR("ERROR %u [%s] at %s:%u\r\nPC at: 0x%08x",
                          p_info->err_code,
                          nrf_strerror_get(p_info->err_code),
                          p_info->p_file_name,
                          p_info->line_num,
                          pc);
             NRF_LOG_ERROR("End of error report");
            break;
        }
        default:
            NRF_LOG_ERROR("UNKNOWN FAULT at 0x%08X", pc);
            break;
    }
#endif

    NRF_BREAKPOINT_COND;
    // On assert, the system can only recover with a reset.

#ifndef DEBUG
    NRF_LOG_WARNING("System reset");
    NVIC_SystemReset();
#else
    app_error_save_and_stop(id, pc, info);
#endif // DEBUG
#endif
}
/*lint -restore */
