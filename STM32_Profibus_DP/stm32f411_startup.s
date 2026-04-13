/*****************************************************************************
 * STM32F411CEU6 Startup Code
 * Minimal startup for bare-metal Profibus DP slave
 *
 * - Defines vector table with all Cortex-M4 + STM32F411 interrupt handlers
 * - Copies .data from Flash to SRAM
 * - Zeroes .bss
 * - Calls main()
 *****************************************************************************/

    .syntax unified
    .cpu cortex-m4
    .fpu softvfp
    .thumb

/* ========================================================================= */
/*  Linker symbols                                                           */
/* ========================================================================= */
    .word _sidata       /* Start of .data initializers in Flash */
    .word _sdata        /* Start of .data in SRAM */
    .word _edata        /* End of .data in SRAM */
    .word _sbss         /* Start of .bss in SRAM */
    .word _ebss         /* End of .bss in SRAM */
    .word _estack       /* Top of stack (end of SRAM) */

/* ========================================================================= */
/*  Reset Handler                                                            */
/* ========================================================================= */
    .section .text.Reset_Handler
    .weak Reset_Handler
    .type Reset_Handler, %function
Reset_Handler:
    /* Set stack pointer */
    ldr r0, =_estack
    mov sp, r0

    /* Copy .data section from Flash to SRAM */
    ldr r0, =_sdata
    ldr r1, =_edata
    ldr r2, =_sidata
    movs r3, #0
    b .Ldata_loop_check
.Ldata_copy:
    ldr r4, [r2, r3]
    str r4, [r0, r3]
    adds r3, r3, #4
.Ldata_loop_check:
    adds r4, r0, r3
    cmp r4, r1
    bcc .Ldata_copy

    /* Zero .bss section */
    ldr r0, =_sbss
    ldr r1, =_ebss
    movs r2, #0
    b .Lbss_loop_check
.Lbss_zero:
    str r2, [r0]
    adds r0, r0, #4
.Lbss_loop_check:
    cmp r0, r1
    bcc .Lbss_zero

    /* Call main() */
    bl main

    /* If main returns, loop forever */
.Lhang:
    b .Lhang

    .size Reset_Handler, .-Reset_Handler

/* ========================================================================= */
/*  Default Handler for unused interrupts                                    */
/* ========================================================================= */
    .section .text.Default_Handler, "ax", %progbits
    .weak Default_Handler
    .type Default_Handler, %function
Default_Handler:
    b Default_Handler
    .size Default_Handler, .-Default_Handler

/* ========================================================================= */
/*  Vector Table                                                             */
/*  Placed at 0x00000000 (mapped to Flash at 0x08000000)                     */
/* ========================================================================= */
    .section .isr_vector, "a", %progbits
    .type g_pfnVectors, %object
    .size g_pfnVectors, .-g_pfnVectors

g_pfnVectors:
    /* Cortex-M4 System Exceptions */
    .word _estack                   /* 0x0000: Initial Stack Pointer */
    .word Reset_Handler             /* 0x0004: Reset */
    .word NMI_Handler               /* 0x0008: NMI */
    .word HardFault_Handler         /* 0x000C: Hard Fault */
    .word MemManage_Handler         /* 0x0010: Memory Management Fault */
    .word BusFault_Handler          /* 0x0014: Bus Fault */
    .word UsageFault_Handler        /* 0x0018: Usage Fault */
    .word 0                         /* 0x001C: Reserved */
    .word 0                         /* 0x0020: Reserved */
    .word 0                         /* 0x0024: Reserved */
    .word 0                         /* 0x0028: Reserved */
    .word SVC_Handler               /* 0x002C: SVCall */
    .word DebugMon_Handler          /* 0x0030: Debug Monitor */
    .word 0                         /* 0x0034: Reserved */
    .word PendSV_Handler            /* 0x0038: PendSV */
    .word SysTick_Handler           /* 0x003C: SysTick */

    /* STM32F411 Peripheral Interrupts */
    .word WWDG_IRQHandler               /* 0:  Window Watchdog */
    .word PVD_IRQHandler                /* 1:  PVD through EXTI */
    .word TAMP_STAMP_IRQHandler         /* 2:  Tamper and TimeStamp */
    .word RTC_WKUP_IRQHandler           /* 3:  RTC Wakeup */
    .word FLASH_IRQHandler              /* 4:  Flash */
    .word RCC_IRQHandler                /* 5:  RCC */
    .word EXTI0_IRQHandler              /* 6:  EXTI Line 0 */
    .word EXTI1_IRQHandler              /* 7:  EXTI Line 1 */
    .word EXTI2_IRQHandler              /* 8:  EXTI Line 2 */
    .word EXTI3_IRQHandler              /* 9:  EXTI Line 3 */
    .word EXTI4_IRQHandler              /* 10: EXTI Line 4 */
    .word DMA1_Stream0_IRQHandler       /* 11: DMA1 Stream 0 */
    .word DMA1_Stream1_IRQHandler       /* 12: DMA1 Stream 1 */
    .word DMA1_Stream2_IRQHandler       /* 13: DMA1 Stream 2 */
    .word DMA1_Stream3_IRQHandler       /* 14: DMA1 Stream 3 */
    .word DMA1_Stream4_IRQHandler       /* 15: DMA1 Stream 4 */
    .word DMA1_Stream5_IRQHandler       /* 16: DMA1 Stream 5 */
    .word DMA1_Stream6_IRQHandler       /* 17: DMA1 Stream 6 */
    .word ADC_IRQHandler                /* 18: ADC1 */
    .word 0                             /* 19: Reserved */
    .word 0                             /* 20: Reserved */
    .word 0                             /* 21: Reserved */
    .word 0                             /* 22: Reserved */
    .word EXTI9_5_IRQHandler            /* 23: EXTI Lines 5-9 */
    .word TIM1_BRK_TIM9_IRQHandler      /* 24: TIM1 Break / TIM9 */
    .word TIM1_UP_TIM10_IRQHandler      /* 25: TIM1 Update / TIM10 */
    .word TIM1_TRG_COM_TIM11_IRQHandler /* 26: TIM1 Trigger / TIM11 */
    .word TIM1_CC_IRQHandler            /* 27: TIM1 Capture Compare */
    .word TIM2_IRQHandler               /* 28: TIM2 *** PROFIBUS TIMER *** */
    .word TIM3_IRQHandler               /* 29: TIM3 */
    .word TIM4_IRQHandler               /* 30: TIM4 */
    .word I2C1_EV_IRQHandler            /* 31: I2C1 Event */
    .word I2C1_ER_IRQHandler            /* 32: I2C1 Error */
    .word I2C2_EV_IRQHandler            /* 33: I2C2 Event */
    .word I2C2_ER_IRQHandler            /* 34: I2C2 Error */
    .word SPI1_IRQHandler               /* 35: SPI1 */
    .word SPI2_IRQHandler               /* 36: SPI2 */
    .word USART1_IRQHandler             /* 37: USART1 *** PROFIBUS UART *** */
    .word USART2_IRQHandler             /* 38: USART2 */
    .word 0                             /* 39: Reserved */
    .word EXTI15_10_IRQHandler          /* 40: EXTI Lines 10-15 */
    .word RTC_Alarm_IRQHandler          /* 41: RTC Alarm */
    .word OTG_FS_WKUP_IRQHandler        /* 42: USB OTG FS Wakeup */
    .word 0                             /* 43: Reserved */
    .word 0                             /* 44: Reserved */
    .word 0                             /* 45: Reserved */
    .word 0                             /* 46: Reserved */
    .word DMA1_Stream7_IRQHandler       /* 47: DMA1 Stream 7 */
    .word 0                             /* 48: Reserved */
    .word SDIO_IRQHandler               /* 49: SDIO */
    .word TIM5_IRQHandler               /* 50: TIM5 */
    .word SPI3_IRQHandler               /* 51: SPI3 */
    .word 0                             /* 52: Reserved */
    .word 0                             /* 53: Reserved */
    .word 0                             /* 54: Reserved */
    .word 0                             /* 55: Reserved */
    .word DMA2_Stream0_IRQHandler       /* 56: DMA2 Stream 0 */
    .word DMA2_Stream1_IRQHandler       /* 57: DMA2 Stream 1 */
    .word DMA2_Stream2_IRQHandler       /* 58: DMA2 Stream 2 *** PROFIBUS DMA RX *** */
    .word DMA2_Stream3_IRQHandler       /* 59: DMA2 Stream 3 */
    .word DMA2_Stream4_IRQHandler       /* 60: DMA2 Stream 4 */
    .word 0                             /* 61: Reserved */
    .word 0                             /* 62: Reserved */
    .word 0                             /* 63: Reserved */
    .word 0                             /* 64: Reserved */
    .word 0                             /* 65: Reserved */
    .word 0                             /* 66: Reserved */
    .word OTG_FS_IRQHandler             /* 67: USB OTG FS */
    .word DMA2_Stream5_IRQHandler       /* 68: DMA2 Stream 5 */
    .word DMA2_Stream6_IRQHandler       /* 69: DMA2 Stream 6 */
    .word DMA2_Stream7_IRQHandler       /* 70: DMA2 Stream 7 */
    .word USART6_IRQHandler             /* 71: USART6 */
    .word I2C3_EV_IRQHandler            /* 72: I2C3 Event */
    .word I2C3_ER_IRQHandler            /* 73: I2C3 Error */
    .word 0                             /* 74: Reserved */
    .word 0                             /* 75: Reserved */
    .word 0                             /* 76: Reserved */
    .word 0                             /* 77: Reserved */
    .word 0                             /* 78: Reserved */
    .word 0                             /* 79: Reserved */
    .word 0                             /* 80: Reserved */
    .word FPU_IRQHandler                /* 81: FPU */
    .word 0                             /* 82: Reserved */
    .word 0                             /* 83: Reserved */
    .word SPI4_IRQHandler               /* 84: SPI4 */
    .word SPI5_IRQHandler               /* 85: SPI5 */

/* ========================================================================= */
/*  Weak aliases to Default_Handler for all handlers                         */
/*  Override in C by defining the function with the same name.               */
/* ========================================================================= */

    .weak NMI_Handler
    .thumb_set NMI_Handler, Default_Handler

    .weak HardFault_Handler
    .thumb_set HardFault_Handler, Default_Handler

    .weak MemManage_Handler
    .thumb_set MemManage_Handler, Default_Handler

    .weak BusFault_Handler
    .thumb_set BusFault_Handler, Default_Handler

    .weak UsageFault_Handler
    .thumb_set UsageFault_Handler, Default_Handler

    .weak SVC_Handler
    .thumb_set SVC_Handler, Default_Handler

    .weak DebugMon_Handler
    .thumb_set DebugMon_Handler, Default_Handler

    .weak PendSV_Handler
    .thumb_set PendSV_Handler, Default_Handler

    /* SysTick_Handler is defined in main.c, NOT weak-aliased here */

    .weak WWDG_IRQHandler
    .thumb_set WWDG_IRQHandler, Default_Handler

    .weak PVD_IRQHandler
    .thumb_set PVD_IRQHandler, Default_Handler

    .weak TAMP_STAMP_IRQHandler
    .thumb_set TAMP_STAMP_IRQHandler, Default_Handler

    .weak RTC_WKUP_IRQHandler
    .thumb_set RTC_WKUP_IRQHandler, Default_Handler

    .weak FLASH_IRQHandler
    .thumb_set FLASH_IRQHandler, Default_Handler

    .weak RCC_IRQHandler
    .thumb_set RCC_IRQHandler, Default_Handler

    .weak EXTI0_IRQHandler
    .thumb_set EXTI0_IRQHandler, Default_Handler

    .weak EXTI1_IRQHandler
    .thumb_set EXTI1_IRQHandler, Default_Handler

    .weak EXTI2_IRQHandler
    .thumb_set EXTI2_IRQHandler, Default_Handler

    .weak EXTI3_IRQHandler
    .thumb_set EXTI3_IRQHandler, Default_Handler

    .weak EXTI4_IRQHandler
    .thumb_set EXTI4_IRQHandler, Default_Handler

    .weak DMA1_Stream0_IRQHandler
    .thumb_set DMA1_Stream0_IRQHandler, Default_Handler

    .weak DMA1_Stream1_IRQHandler
    .thumb_set DMA1_Stream1_IRQHandler, Default_Handler

    .weak DMA1_Stream2_IRQHandler
    .thumb_set DMA1_Stream2_IRQHandler, Default_Handler

    .weak DMA1_Stream3_IRQHandler
    .thumb_set DMA1_Stream3_IRQHandler, Default_Handler

    .weak DMA1_Stream4_IRQHandler
    .thumb_set DMA1_Stream4_IRQHandler, Default_Handler

    .weak DMA1_Stream5_IRQHandler
    .thumb_set DMA1_Stream5_IRQHandler, Default_Handler

    .weak DMA1_Stream6_IRQHandler
    .thumb_set DMA1_Stream6_IRQHandler, Default_Handler

    .weak ADC_IRQHandler
    .thumb_set ADC_IRQHandler, Default_Handler

    .weak EXTI9_5_IRQHandler
    .thumb_set EXTI9_5_IRQHandler, Default_Handler

    .weak TIM1_BRK_TIM9_IRQHandler
    .thumb_set TIM1_BRK_TIM9_IRQHandler, Default_Handler

    .weak TIM1_UP_TIM10_IRQHandler
    .thumb_set TIM1_UP_TIM10_IRQHandler, Default_Handler

    .weak TIM1_TRG_COM_TIM11_IRQHandler
    .thumb_set TIM1_TRG_COM_TIM11_IRQHandler, Default_Handler

    .weak TIM1_CC_IRQHandler
    .thumb_set TIM1_CC_IRQHandler, Default_Handler

    /* TIM2_IRQHandler is defined in main.c, NOT weak-aliased here */

    .weak TIM3_IRQHandler
    .thumb_set TIM3_IRQHandler, Default_Handler

    .weak TIM4_IRQHandler
    .thumb_set TIM4_IRQHandler, Default_Handler

    .weak I2C1_EV_IRQHandler
    .thumb_set I2C1_EV_IRQHandler, Default_Handler

    .weak I2C1_ER_IRQHandler
    .thumb_set I2C1_ER_IRQHandler, Default_Handler

    .weak I2C2_EV_IRQHandler
    .thumb_set I2C2_EV_IRQHandler, Default_Handler

    .weak I2C2_ER_IRQHandler
    .thumb_set I2C2_ER_IRQHandler, Default_Handler

    .weak SPI1_IRQHandler
    .thumb_set SPI1_IRQHandler, Default_Handler

    .weak SPI2_IRQHandler
    .thumb_set SPI2_IRQHandler, Default_Handler

    /* USART1_IRQHandler is defined in main.c, NOT weak-aliased here */

    .weak USART2_IRQHandler
    .thumb_set USART2_IRQHandler, Default_Handler

    .weak EXTI15_10_IRQHandler
    .thumb_set EXTI15_10_IRQHandler, Default_Handler

    .weak RTC_Alarm_IRQHandler
    .thumb_set RTC_Alarm_IRQHandler, Default_Handler

    .weak OTG_FS_WKUP_IRQHandler
    .thumb_set OTG_FS_WKUP_IRQHandler, Default_Handler

    .weak DMA1_Stream7_IRQHandler
    .thumb_set DMA1_Stream7_IRQHandler, Default_Handler

    .weak SDIO_IRQHandler
    .thumb_set SDIO_IRQHandler, Default_Handler

    .weak TIM5_IRQHandler
    .thumb_set TIM5_IRQHandler, Default_Handler

    .weak SPI3_IRQHandler
    .thumb_set SPI3_IRQHandler, Default_Handler

    .weak DMA2_Stream0_IRQHandler
    .thumb_set DMA2_Stream0_IRQHandler, Default_Handler

    .weak DMA2_Stream1_IRQHandler
    .thumb_set DMA2_Stream1_IRQHandler, Default_Handler

    .weak DMA2_Stream2_IRQHandler
    .thumb_set DMA2_Stream2_IRQHandler, Default_Handler

    .weak DMA2_Stream3_IRQHandler
    .thumb_set DMA2_Stream3_IRQHandler, Default_Handler

    .weak DMA2_Stream4_IRQHandler
    .thumb_set DMA2_Stream4_IRQHandler, Default_Handler

    .weak OTG_FS_IRQHandler
    .thumb_set OTG_FS_IRQHandler, Default_Handler

    .weak DMA2_Stream5_IRQHandler
    .thumb_set DMA2_Stream5_IRQHandler, Default_Handler

    .weak DMA2_Stream6_IRQHandler
    .thumb_set DMA2_Stream6_IRQHandler, Default_Handler

    .weak DMA2_Stream7_IRQHandler
    .thumb_set DMA2_Stream7_IRQHandler, Default_Handler

    .weak USART6_IRQHandler
    .thumb_set USART6_IRQHandler, Default_Handler

    .weak I2C3_EV_IRQHandler
    .thumb_set I2C3_EV_IRQHandler, Default_Handler

    .weak I2C3_ER_IRQHandler
    .thumb_set I2C3_ER_IRQHandler, Default_Handler

    .weak FPU_IRQHandler
    .thumb_set FPU_IRQHandler, Default_Handler

    .weak SPI4_IRQHandler
    .thumb_set SPI4_IRQHandler, Default_Handler

    .weak SPI5_IRQHandler
    .thumb_set SPI5_IRQHandler, Default_Handler

    .end
