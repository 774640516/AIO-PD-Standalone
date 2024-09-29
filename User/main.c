/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/04/06
 * Description        : Main program body.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/*
 *@Note
 *
 * PD SNK Sample code
 *
 * This sample code may have compatibility issues and is for learning purposes only.
 * If you want to develop a PD project, please contact FAE.

 * Make sure that the board is not powered on before use.
 * Be sure to pay attention to the voltage when changing the request
 * to prevent burning the board.
 *
 * There is no integrated 5.1K pull-down inside the chip,
 * CC_PD is only for st atus differentiation,
 * bit write 1 means SNK mode, write 0 means SCR mode
 *
 * Modify "PDO_Request( PDO_INDEX_1 )" in pd process.c, line 753, to modify the request voltage.
 *
 * According to the usage scenario of PD SNK, whether
 * it is removed or not should be determined by detecting
 * the Vbus voltage, this code only shows the detection
 * and the subsequent communication flow.
 */

#include "debug.h"
#include "PD_Process.h"
// #include <ch32x035_usbfs_device.h>
// #include "usbd_compatibility_hid.h"

#include "string.h"

#define VALVE_TIME_OUT 20000
#define VALVE_CHECK_TIME1 5000
#define VALVE_CHECK_TIME2 8000

#define OPEN_HALL_PIN GPIO_Pin_1
#define CLOSE_HALL_PIN GPIO_Pin_9

void TIM1_UP_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

UINT8 Tim_Ms_Cnt = 0x00;
UINT16 Test_Time = 0;
UINT8 APX823_WDI_Status = 0;
uint16_t APX823_WDI_TIME = 250;
UINT16 Time_data;

uint8_t valve_status = 0; // 阀门控制参数
uint8_t valve_set_status = 0;
uint8_t valve_in_place_status = 0;
uint16_t valve_time = 0;

UINT8 ADC_TEST = 0; // adc获取电压参数
UINT8 adc_status;
UINT16 adc_time;
uint16_t adc_jval1;
uint16_t adc_vbus;
uint16_t adc_cap_I;
uint16_t adc_cap_U;

uint8_t cap_start = 0;
uint8_t cap_status = 0;
uint16_t cap_buffer[5];
uint8_t cap_buffer_addr = 0;
uint16_t cap_V;
uint16_t cap_last_V = 0;
uint16_t cap_deviation;
uint8_t cap_count = 0;

uint8_t hall_start = 0;
uint16_t hall_time = 0;
uint8_t open_hall = 1;
uint8_t close_hall = 1;
uint8_t open_hall_time = 0;
uint8_t close_hall_time = 0;

uint16_t check_time = 0;

uint8_t PD_AIO_Connect = 0;

uint8_t led_status = 0;
uint16_t led_time = 0;

uint8_t open_error_flag = 0;
uint8_t close_error_flag = 0;
uint8_t check_error_flag = 0;

/*********************************************************************
 * @fn      TIM1_Init
 *
 * @brief   Initialize TIM1
 *
 * @return  none
 */
void TIM1_Init(u16 arr, u16 psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    TIM_TimeBaseInitStructure.TIM_Period = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0x00;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
}
void GPIO_Valve_Input_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}
void GPIO_APX823_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    // GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    // GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    // GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // GPIO_WriteBit(GPIOA, GPIO_Pin_3, Bit_SET);
    GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET);
}
void ADC_VBUS_CAP_Init(void)
{
    ADC_InitTypeDef ADC_InitStructure = {0};
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    ADC_DeInit(ADC1);

    ADC_CLKConfig(ADC1, ADC_CLK_Div6);

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_Cmd(ADC1, ENABLE);

    ADC_TEST = 0;
}

void GPIO_LED_BUZZ_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_WriteBit(GPIOB, GPIO_Pin_6, Bit_RESET);
    GPIO_WriteBit(GPIOB, GPIO_Pin_7, Bit_SET);
    GPIO_WriteBit(GPIOB, GPIO_Pin_8, Bit_SET);

    led_status = 0;
}

void my_adc_Handle()
{
    Test_Time += Tmr_Ms_Dlt;
    if (Test_Time >= 500)
    {
        Test_Time = 0;
    }
    if (Tmr_Ms_Dlt)
    {
        switch (adc_status)
        {
        case 0:
            if (ADC_TEST == 0)
            {
                ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_11Cycles);
            }
            else if (ADC_TEST == 1)
            {
                ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_11Cycles);
            }
            else if (ADC_TEST == 2)
            {
                ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_11Cycles);
            }
            ADC_SoftwareStartConvCmd(ADC1, ENABLE);
            adc_status = 1;
            break;
        case 1:
            if (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC))
            {
                adc_status = 2;
                adc_time = 500;
            }
            break;
        case 2:
            if (adc_time > Tmr_Ms_Dlt)
                adc_time -= Tmr_Ms_Dlt;
            else
                adc_time = 0;
            if (adc_time == 0)
                adc_status = 3;
            break;
            break;
        case 3:

            ADC_SoftwareStartConvCmd(ADC1, DISABLE);
            adc_jval1 = ADC_GetConversionValue(ADC1);
            // if (ADC_TEST == 0)
            //     printf("\r\n");
            // printf("ADC %d:%04d\r\n", ADC_TEST, adc_jval1);
            if (ADC_TEST == 0)
            {
                adc_vbus = adc_jval1;
            }
            else if (ADC_TEST == 1)
            {
                adc_cap_I = adc_jval1;
            }
            else
            {
                adc_cap_U = adc_jval1;
            }

            adc_status = 0;
            adc_time = 500;
            ADC_TEST++;
            ADC_TEST %= 3;
            if (ADC_TEST == 0)
            {
                cap_start = 1;
            }

            break;
        case 4:

            if (adc_time > Tmr_Ms_Dlt)
                adc_time -= Tmr_Ms_Dlt;
            else
                adc_time = 0;
            if (adc_time == 0)
                adc_status = 0;
            break;
        }
    }
}
void my_cap_Handle()
{
    if (cap_start)
    {
        cap_start = 0;
        //        printf("cap 123123123\r\n");
        switch (cap_status)
        {
        case 0:
            cap_status = 1;
            cap_buffer_addr = 0;
            break;
        case 1:
            cap_buffer_addr++;
            if (cap_buffer_addr >= 5)
            {
                cap_status = 2;
                cap_count = 0;
                cap_buffer_addr = 0;
                printf("cap start\r\n");
            }
            break;
        case 2:
            cap_buffer[cap_buffer_addr++] = adc_cap_U;
            cap_count++;
            if (cap_buffer_addr >= 5)
            {
                cap_V = 0;
                for (uint8_t i = 0; i < 5; i++)
                {
                    cap_V += cap_buffer[i];
                }
                cap_V /= 5;
                if (cap_count >= 10)
                {
                    if (cap_V < cap_last_V)
                    {
                        cap_status = 10;
                        // cap error
                        // my_usb_send(5);
                        printf("cap_V < cap_last_V\r\n");
                    }
                    else
                    {
                        cap_status = 3;
                        printf("cap_V > cap_last_V ok\r\n");
                    }
                }
                cap_last_V = cap_V;
            }
            if (cap_count >= 100)
            {
                cap_status = 10;
                // my_usb_send(5);
                // cap timeout error
                printf("cap time out\r\n");
            }
            if (adc_cap_U < adc_cap_I)
            {
                cap_deviation = adc_cap_I - adc_cap_U;
                if (cap_deviation < 200)
                {
                    cap_status = 3;
                    printf("(adc_cap_U - adc_cap_I) < 200(200 ~= 0.32V)\r\n");
                }
            }
            else
            {
                cap_status = 3;
                printf("adc_cap_U >= adc_cap_I\r\n");
            }
            if (adc_cap_I < 2800)
            {
                cap_status = 10;
                // my_usb_send(5);
                // cap I low error
                printf("adc_cap_I < 2800(2800 ~= 4.51V)");
            }
            break;
        case 3:
            break;
        case 10:
            break;
        }
    }
}

uint8_t my_usb_HID_Receive_Check(uint8_t *buffer, uint8_t buffer_size)
{
    uint8_t check = 0;
    if (buffer_size != (buffer[5] + 7))
    {
        printf("buffer_size = %d  buffer[5] = %d  size error\r\n", buffer_size, buffer[5]);
        return 0;
    }
    if (buffer[1] != 0x55 || buffer[2] != 0xaa || buffer[3] != 0x01)
    {
        printf("%d  %d  %d Default data error\r\n", buffer[1], buffer[2], buffer[3]);
        return 0;
    }
    if (buffer[4] > 10)
    {
        printf("%d  command error\r\n", buffer[4]);
        return 0;
    }
    for (uint8_t i = 1; i < (buffer_size - 1); i++)
        check += buffer[i];
    if (buffer[buffer_size - 1] != check)
    {
        printf("%d  %d  check error\r\n", check, buffer[buffer_size - 1]);
        return 0;
    }
    return 1;
}

void my_PD_AIO_Receive(uint8_t *receive_buff)
{
    if (receive_buff[1] == 0x00 && receive_buff[2] == 0x00 && receive_buff[3] == 0x00)
    {
        switch (receive_buff[0])
        {
        case 1: // 连接
            printf("PD AIO Connect\r\n");
            PD_AIO_Connect = 1;
            hall_start = 1;
            hall_time = 500;
            led_status = 1;
            break;
        case 2: // 开阀
            printf("PD AIO Open Valve\r\n");
            valve_in_place_status = 1;
            led_status = 1;
            open_error_flag = 0;
            break;
        case 3: // 关阀
            printf("PD AIO Close Valve\r\n");
            valve_in_place_status = 0;
            led_status = 1;
            close_error_flag = 0;
            break;
        case 4: // 自检
            printf("PD AIO Check\r\n");
            valve_status = 2;
            GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET); // HIGH 开阀，LOW 关阀
            valve_time = VALVE_TIME_OUT;
            valve_in_place_status = 2;
            check_time = 15000;
            led_status = 1;
            check_error_flag = 0;
            break;
        }
    }
}
uint8_t pd_valve_send_buff[16];

void my_PD_AIO_send(uint8_t status, uint8_t data)
{
    pd_valve_send_buff[0] = status;
    pd_valve_send_buff[1] = data;
    pd_valve_send_buff[2] = 0xAA;
    pd_valve_send_buff[3] = 0x55;

    PD_Load_Header(0x00, DEF_TYPE_TEST);
    PD_Send_Handle(pd_valve_send_buff, 4);
}

void my_TIM_Handle()
{
    TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);
    Tmr_Ms_Dlt = Tim_Ms_Cnt - Tmr_Ms_Cnt_Last;
    Tmr_Ms_Cnt_Last = Tim_Ms_Cnt;
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
}
void my_PD_Handle()
{
    PD_Ctl.Det_Timer += Tmr_Ms_Dlt;
    if (PD_Ctl.Det_Timer > 4)
    {
        PD_Ctl.Det_Timer = 0;
        PD_Det_Proc();
    }
    PD_Main_Proc();
}

void my_valve_Handle()
{
    switch (valve_status)
    {
    case 0:
        valve_set_status = 0;
        valve_in_place_status = 0;
        open_error_flag = 0;
        close_error_flag = 0;
        check_error_flag = 0;
        if (valve_in_place_status == 0)
        {
            valve_status = 1;
            GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET); // HIGH 开阀，LOW 关阀
            valve_time = VALVE_TIME_OUT;
        }
        if (valve_in_place_status == 1)
        {
            valve_status = 2;
            GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET); // HIGH 开阀，LOW 关阀
            valve_time = VALVE_TIME_OUT;
        }
        break;
    case 1:
        if (GPIO_ReadInputDataBit(GPIOB, CLOSE_HALL_PIN))
        {
            if (valve_time > Tmr_Ms_Dlt)
                valve_time -= Tmr_Ms_Dlt;
            else if(close_error_flag == 0)
            {
                close_error_flag = 1;
                my_PD_AIO_send(1, 0);
                printf("close vlave timeout error\r\n");
                if (led_status == 1)
                    led_status = 2;
                // close vlave timeout error
            }
        }
        else
        {
            valve_time = VALVE_TIME_OUT;
            // if (GPIO_ReadInputDataBit(GPIOB, OPEN_HALL_PIN) == 0)
            // {
            //     valve_status = 10;
            //     my_PD_AIO_send(1, 1);
            //     printf("close hall low and open hall low error\r\n");
            //     // close hall low and open hall low error
            // }
        }
        if (valve_in_place_status == 1)
        {
            valve_status = 2;
            GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET); // HIGH 开阀，LOW 关阀
            valve_time = VALVE_TIME_OUT;
        }
        break;
    case 2:
        if (GPIO_ReadInputDataBit(GPIOB, OPEN_HALL_PIN))
        {
            if (valve_time > Tmr_Ms_Dlt)
                valve_time -= Tmr_Ms_Dlt;
            else if(open_error_flag == 0)
            {
                open_error_flag = 1;
                my_PD_AIO_send(1, 2);
                valve_time = VALVE_TIME_OUT;
                printf("open valve timeout error\r\n");
                if (led_status == 1)
                    led_status = 2;
                // open valve timeout error
            }
        }
        else
        {
            valve_time = VALVE_TIME_OUT;
            // if (GPIO_ReadInputDataBit(GPIOB, CLOSE_HALL_PIN) == 0)
            // {
            //     my_PD_AIO_send(1, 3);
            //     printf("close hall low and open hall low error\r\n");
            //     // close hall low and open hall low error
            //     valve_time = VALVE_TIME_OUT;
            // }
        }
        if (valve_in_place_status == 2)
        {
            if (check_time > Tmr_Ms_Dlt)
                check_time -= Tmr_Ms_Dlt;
            else
            {
                if (GPIO_ReadInputDataBit(GPIOB, OPEN_HALL_PIN) == Bit_RESET)
                {
                    valve_status = 3;
                    GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET); // HIGH 开阀，LOW 关阀
                    valve_time = VALVE_CHECK_TIME1;
                    
                }
                else
                {
                    valve_in_place_status = 1;
                    my_PD_AIO_send(1, 4);
                    printf("open valve timeout, check error\r\n");
                    if (led_status == 1)
                        led_status = 2;
                }
            }
        }
        if (valve_in_place_status == 0)
        {
            valve_status = 1;
            GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET); // HIGH 开阀，LOW 关阀
            valve_time = VALVE_TIME_OUT;
        }
        break;
    case 3:
        if (valve_time > Tmr_Ms_Dlt)
            valve_time -= Tmr_Ms_Dlt;
        else
        {
            if (GPIO_ReadInputDataBit(GPIOB, OPEN_HALL_PIN) == 1)
            {
                valve_status = 4;
                GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET); // HIGH 开阀，LOW 关阀
                valve_time = VALVE_CHECK_TIME2;
            }
            else
            {
                my_PD_AIO_send(1, 5);
                valve_in_place_status = 1;
                valve_status = 2;
                GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET); // HIGH 开阀，LOW 关阀
                printf("close valve 3s open hall low check error\r\n");
                if (led_status == 1)
                    led_status = 2;
                // close valve 1.5s  close hall low or open hall low check error
            }
        }
        if (valve_in_place_status == 0)
        {
            valve_status = 1;
            GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET); // HIGH 开阀，LOW 关阀
            valve_time = VALVE_TIME_OUT;
        }
        if (valve_in_place_status == 1)
        {
            valve_status = 2;
            GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET); // HIGH 开阀，LOW 关阀
            valve_time = VALVE_TIME_OUT;
        }
        break;
    case 4:
        if (valve_time > Tmr_Ms_Dlt)
            valve_time -= Tmr_Ms_Dlt;
        else
        {
            my_PD_AIO_send(1, 6);
            valve_in_place_status = 1;
            valve_status = 2;
            GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET); // HIGH 开阀，LOW 关阀
            printf("open valve 6s open hall HIGH check error\r\n");
            if (led_status == 1)
                led_status = 2;
        }
        if (GPIO_ReadInputDataBit(GPIOB, OPEN_HALL_PIN) == Bit_RESET)
        {
            my_PD_AIO_send(3, 0);
            valve_in_place_status = 1;
            valve_status = 2;
            GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET); // HIGH 开阀，LOW 关阀
            valve_time = VALVE_TIME_OUT;
        }

        if (valve_in_place_status == 0)
        {
            valve_status = 1;
            GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET); // HIGH 开阀，LOW 关阀
            valve_time = VALVE_TIME_OUT;
        }
        if (valve_in_place_status == 1)
        {
            valve_status = 2;
            GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET); // HIGH 开阀，LOW 关阀
            valve_time = VALVE_TIME_OUT;
        }
        break;
    // case 10:
    //     if (valve_in_place_status == 0)
    //     {
    //         valve_status = 1;
    //         GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET); // HIGH 开阀，LOW 关阀
    //         valve_time = VALVE_TIME_OUT;
    //     }
    //     if (valve_in_place_status == 1)
    //     {
    //         valve_status = 2;
    //         GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET); // HIGH 开阀，LOW 关阀
    //         valve_time = VALVE_TIME_OUT;
    //     }
    //     break;
    // case 11:
    //     if (valve_in_place_status == 2)
    //     {
    //         if (check_time > Tmr_Ms_Dlt)
    //             check_time -= Tmr_Ms_Dlt;
    //         else
    //         {
    //             if (GPIO_ReadInputDataBit(GPIOB, OPEN_HALL_PIN) == Bit_RESET)
    //             {
    //                 valve_status = 3;
    //                 GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET); // HIGH 开阀，LOW 关阀
    //                 valve_time = VALVE_CHECK_TIME1;
    //             }
    //             else
    //             {
    //                 valve_in_place_status = 1;
    //                 // my_usb_send(2);
    //                 my_PD_AIO_send(1, 4);
    //                 printf("open valve timeout, check error\r\n");
    //                 if (led_status == 1)
    //                     led_status = 2;
    //             }
    //         }
    //     }
    //     if (valve_in_place_status == 0)
    //     {
    //         valve_status = 1;
    //         GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET); // HIGH 开阀，LOW 关阀
    //         valve_time = VALVE_TIME_OUT;
    //     }
    //     break;
    }
}

void my_led_buzz_Handle()
{
    static uint8_t led_count = 0;
    switch (led_status)
    {
    case 0:
        break;
    case 1:
        GPIO_WriteBit(GPIOB, GPIO_Pin_8, Bit_SET);   // 关红灯
        GPIO_WriteBit(GPIOB, GPIO_Pin_6, Bit_RESET); // 关蜂鸣器
        break;
    case 2:
        led_status = 3;
        led_count = 0;
        led_time = 200;
        GPIO_WriteBit(GPIOB, GPIO_Pin_8, Bit_RESET); // 亮红灯
        GPIO_WriteBit(GPIOB, GPIO_Pin_6, Bit_SET);   // 响蜂鸣器
        break;
    case 3:
        if (led_time > Tmr_Ms_Dlt)
            led_time -= Tmr_Ms_Dlt;
        else
        {
            led_count++;
            GPIO_WriteBit(GPIOB, GPIO_Pin_8, Bit_SET);   // 关红灯
            GPIO_WriteBit(GPIOB, GPIO_Pin_6, Bit_RESET); // 关蜂鸣器
            if (led_count < 6)
            {
                led_status = 4;
                led_time = 200;
            }
            else
            {
                led_status = 5;
                led_time = 10000;
            }
        }
        break;
    case 4:
        if (led_time > Tmr_Ms_Dlt)
            led_time -= Tmr_Ms_Dlt;
        else
        {
            led_status = 3;
            led_time = 200;
            GPIO_WriteBit(GPIOB, GPIO_Pin_8, Bit_RESET); // 亮红灯
            GPIO_WriteBit(GPIOB, GPIO_Pin_6, Bit_SET);   // 响蜂鸣器
        }
        break;
    case 5:
        if (led_time > Tmr_Ms_Dlt)
            led_time -= Tmr_Ms_Dlt;
        else
        {
            led_status = 2;
        }
        break;
    }
}

void IWDG_Feed_Init(u16 prer, u16 rlr)
{
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(prer);
    IWDG_SetReload(rlr);
    IWDG_ReloadCounter();
    IWDG_Enable();
}
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(921600);
    printf("SystemClk:%d\r\n", SystemCoreClock);
    printf("ChipID:%08x\r\n", DBGMCU_GetCHIPID());
    printf("V3.0.0  12V\r\n");
    PD_Init();
    GPIO_LED_BUZZ_Init();
    GPIO_Valve_Input_Init(); // PB8 关阀到位LOW              PB9开阀到位LOW
    GPIO_APX823_Init();      // HIGH 开阀，LOW 关阀
    ADC_VBUS_CAP_Init();
    TIM1_Init(999, 48 - 1);
    adc_status = 3;

    GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET); // HIGH 开阀，LOW 关阀

    IWDG_Feed_Init(IWDG_Prescaler_32, 4000);
    while (1)
    {
        IWDG_ReloadCounter();
        my_TIM_Handle();
        my_PD_Handle();
        my_led_buzz_Handle();
        my_valve_Handle();
        if (hall_start == 1 && hall_time == 0)
        {
            hall_time = 5;
            if (open_hall != GPIO_ReadInputDataBit(GPIOB, OPEN_HALL_PIN))
            {
                open_hall_time++;
                if (open_hall_time >= 4)
                {
                    open_hall = GPIO_ReadInputDataBit(GPIOB, OPEN_HALL_PIN);
                    // my_usb_send_Hall(!open_hall, !close_hall);
                    my_PD_AIO_send(2, ((!open_hall) << 1) + (!close_hall));
                }
            }
            else
            {
                open_hall_time = 0;
            }
            if (close_hall != GPIO_ReadInputDataBit(GPIOB, CLOSE_HALL_PIN))
            {
                close_hall_time++;
                if (close_hall_time >= 4)
                {
                    close_hall = GPIO_ReadInputDataBit(GPIOB, CLOSE_HALL_PIN);
                    // my_usb_send_Hall(!open_hall, !close_hall);
                    my_PD_AIO_send(2, ((!open_hall) << 1) + (!close_hall));
                }
            }
            else
            {
                close_hall_time = 0;
            }
        }
        my_adc_Handle();
        my_cap_Handle();
    }
}

/**************z*******************************************************
 * @fn      TIM1_UP_IRQHandler
 *
 * @brief   This function handles TIM1 interrupt.
 *
 * @return  nonex
 */
void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
    {
        if (hall_time)
            hall_time--;
        Tim_Ms_Cnt++;
        Time_data++;
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}
