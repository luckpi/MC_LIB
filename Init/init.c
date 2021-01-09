#include "opa.h"
#include "pwm.h"
#include "init.h"
#include "dmac.h"
#include "common.h"
#include "sensorless.h"
#include "MotorConfig.h"

/**************************************************************************************************
 函 数 名  : OPA_Init
 功能描述  : 通用运放初始化
 输入参数  : 无
 输出参数  : void
**************************************************************************************************/
void OPA_init(void)
{
    stc_opa_gain_config_t strGain;
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    Sysctrl_SetPeripheralGate(SysctrlPeripheralOpa, TRUE);
    DDL_ZERO_STRUCT(strGain);
    strGain.enNoInGain = Gain16; // 正向增益 PGA = 16
    OP1_INP();
    OP2_INP();
    OPA_Init();
    OPA_Operate(OPA1, OpaForWardMode, &strGain); // 正向增益
    OPA_Operate(OPA2, OpaForWardMode, &strGain); // 正向增益
}
/**************************************************************************************************
 函 数 名  : fputc
 功能描述  : printf串口转发
 输入参数  : 无
 输出参数  : void
**************************************************************************************************/
int fputc(int ch, FILE *f)
{
    if (((uint8_t)ch) == '\n')
    {
        Uart_SendData(UARTCH0, '\r');
    }
    Uart_SendData(UARTCH0, ch);
    return ch;
}
/**************************************************************************************************
 函 数 名  : Clk_Init
 功能描述  : 系统时钟初始化
 输入参数  : 无
 输出参数  : void
**************************************************************************************************/
void Clk_init(void)
{
    stc_sysctrl_clk_config_t stcClkConfig; // 配置时钟
    stc_sysctrl_pll_config_t stcPLLCfg;    // 配置PLL时钟
    en_flash_waitcycle_t enFlashWait;
    DDL_ZERO_STRUCT(stcClkConfig); //结构体初始化清零
    DDL_ZERO_STRUCT(stcPLLCfg);
    ///< 开启FLASH外设时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralFlash, TRUE);

    ///< 开启硬件除法外设时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralDiv, TRUE);
    enFlashWait = FlashWaitCycle1; //读等待周期设置为1（当HCLK大于24MHz时必须至少为1）
    Flash_WaitCycle(enFlashWait);  // Flash 等待1个周期

    ///< 时钟初始化前，优先设置要使用的时钟源：此处配置PLL
    Sysctrl_SetRCHTrim(SysctrlRchFreq4MHz); //PLL使用RCH作为时钟源，因此需要先设置RCH

    stcPLLCfg.enInFreq = SysctrlPllInFreq4_6MHz;     //RCH 4MHz
    stcPLLCfg.enOutFreq = SysctrlPllOutFreq36_48MHz; //PLL 输出48MHz
    stcPLLCfg.enPllClkSrc = SysctrlPllRch;           //输入时钟源选择RCH
    stcPLLCfg.enPllMul = SysctrlPllMul12;            //4MHz x 12 = 48MHz
    Sysctrl_SetPLLFreq(&stcPLLCfg);

    ///< 选择PLL作为HCLK时钟源;
    stcClkConfig.enClkSrc = SysctrlClkPLL;
    ///< HCLK SYSCLK
    stcClkConfig.enHClkDiv = SysctrlHclkDiv1;
    ///< PCLK 为HCLK
    stcClkConfig.enPClkDiv = SysctrlPclkDiv1;
    ///< 系统时钟初始化
    Sysctrl_ClkInit(&stcClkConfig);
}
/**************************************************************************************************
 函 数 名  : LED_Init
 功能描述  : LED初始化
 输入参数  : 无
 输出参数  : void
**************************************************************************************************/
void GPIO_init(void)
{
    stc_gpio_config_t GpioCfg;
    DDL_ZERO_STRUCT(GpioCfg);
    ///< 打开GPIO外设时钟门控
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    ///< 端口方向配置 -> 输出
    GpioCfg.enDir = GpioDirOut;
    ///< 端口驱动能力配置 -> 高驱动能力
    GpioCfg.enDrv = GpioDrvH;
    ///< 端口上下拉配置 -> 无上下拉;
    GpioCfg.enPuPd = GpioNoPuPd;
    ///< 端口开漏输出配置->开漏输出关闭
    GpioCfg.enOD = GpioOdDisable;
    ///< 端口输入/输出值寄存器总线控制模式配置->AHB
    GpioCfg.enCtrlMode = GpioAHB;
    ///< GPIO IO PC13初始化（LED）
    Gpio_Init(GpioPortB, GpioPin5, &GpioCfg); // LED
    Gpio_Init(GpioPortB, GpioPin7, &GpioCfg); // LED
    Gpio_WriteOutputIO(GpioPortB, GpioPin5, FALSE);
    Gpio_WriteOutputIO(GpioPortB, GpioPin7, FALSE);
    GpioCfg.enDir = GpioDirIn;
    Gpio_Init(GpioPortC, GpioPin13, &GpioCfg); // KEY0
    Gpio_Init(GpioPortC, GpioPin14, &GpioCfg); // KEY1
}
/**************************************************************************************************
 函 数 名  : Uart_Init
 功能描述  : 串口初始化
 输入参数  : 无
 输出参数  : void
**************************************************************************************************/
void UART_init(void)
{
    uint16_t u16Scnt = 0;
    stc_gpio_config_t stcGpioCfg;
    stc_uart_config_t stcConfig;
    stc_uart_irq_cb_t stcUartIrqCb;
    stc_uart_multimode_t stcMulti;
    stc_uart_baud_t stcBaud;

    en_uart_mmdorck_t enTb8;

    DDL_ZERO_STRUCT(stcGpioCfg);
    DDL_ZERO_STRUCT(stcConfig);
    DDL_ZERO_STRUCT(stcUartIrqCb);
    DDL_ZERO_STRUCT(stcMulti);
    DDL_ZERO_STRUCT(stcBaud);
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    Sysctrl_SetPeripheralGate(SysctrlPeripheralUart0, TRUE);
    stcGpioCfg.enDir = GpioDirOut;
    Gpio_Init(GpioPortB, GpioPin8, &stcGpioCfg);
    Gpio_SetAfMode(GpioPortB, GpioPin8, GpioAf7); //TX
    stcGpioCfg.enDir = GpioDirIn;
    Gpio_Init(GpioPortB, GpioPin9, &stcGpioCfg);
    Gpio_SetAfMode(GpioPortB, GpioPin9, GpioAf7); //RX

    stcUartIrqCb.pfnRxIrqCb = NULL;
    stcUartIrqCb.pfnTxIrqCb = NULL;
    stcUartIrqCb.pfnRxFEIrqCb = NULL;
    stcUartIrqCb.pfnPEIrqCb = NULL;
    stcUartIrqCb.pfnCtsIrqCb = NULL;
    stcConfig.pstcIrqCb = &stcUartIrqCb;
    stcConfig.bTouchNvic = FALSE;
    if (TRUE == stcConfig.bTouchNvic)
    {
        EnableNvic(UART0_IRQn, IrqLevel3, TRUE);
    }
    stcConfig.enRunMode = UartMode3; //模式3
    stcConfig.enStopBit = Uart1bit;  //1位停止位

    stcMulti.enMulti_mode = UartNormal;    //正常工作模式
    Uart_SetMultiMode(UARTCH0, &stcMulti); //多主机单独配置
    enTb8 = UartEven;                      //偶校验
    Uart_SetMMDOrCk(UARTCH0, enTb8);
    Uart_Init(UARTCH0, &stcConfig); //串口初始化函数

    Uart_SetClkDiv(UARTCH0, Uart8Or16Div); //采样分频
    stcBaud.u32Pclk = Sysctrl_GetPClkFreq();
    stcBaud.enRunMode = UartMode3;
    stcBaud.u32Baud = 115200;
    u16Scnt = Uart_CalScnt(UARTCH0, &stcBaud); //波特率值计算
    Uart_SetBaud(UARTCH0, u16Scnt);            //波特率设置

    Uart_ClrStatus(UARTCH0, UartRC);  //清接收请求
    Uart_EnableFunc(UARTCH0, UartRx); //使能收发
}
/**************************************************************************************************
 函 数 名  : DMA_Init
 功能描述  : ADC_DMA初始化
 输入参数  : 无
 输出参数  : void
**************************************************************************************************/
void DMA_init(void)
{
    stc_dma_config_t stcDmaCfg;
    DDL_ZERO_STRUCT(stcDmaCfg);
    Sysctrl_SetPeripheralGate(SysctrlPeripheralDma, TRUE);
    stcDmaCfg.enMode = DmaBlock;
    stcDmaCfg.u16BlockSize = 2;
    stcDmaCfg.u16TransferCnt = 1; //Block模式，一次传输数据大小为 1,传输2次
    stcDmaCfg.enTransferWidth = Dma32Bit;
    stcDmaCfg.enSrcAddrMode = AddressIncrease;
    stcDmaCfg.enDstAddrMode = AddressIncrease;
    stcDmaCfg.bDestAddrReloadCtl = TRUE;
    stcDmaCfg.bSrcAddrReloadCtl = TRUE;
    stcDmaCfg.bSrcBcTcReloadCtl = TRUE;
    stcDmaCfg.u32SrcAddress = (uint32_t) & (M0P_ADC->SQRRESULT0);
    stcDmaCfg.u32DstAddress = (uint32_t) & (ADCSample.POT);
    stcDmaCfg.bMsk = TRUE;               //DMAC 在传输完成时不清除 CONFA:ENS 位。这个功能允许连续传输而不需要 CPU 干预。
    stcDmaCfg.enRequestNum = ADCSQRTrig; //设置为ADC SQR触发

    Dma_InitChannel(DmaCh0, &stcDmaCfg);
    //使能DMA，使能DMA0
    Dma_Enable();
    Dma_EnableChannel(DmaCh0);
}
/**************************************************************************************************
 函 数 名  : ADC_Init
 功能描述  : ADC初始化
 输入参数  : 无
 输出参数  : void
**************************************************************************************************/
void ADC_init(void)
{
    uint8_t u8AdcSqrScanCnt;
    uint8_t u8AdcJqrScanCnt;
    stc_adc_cfg_t stcAdcCfg;
    stc_adc_irq_t stcAdcIrq;
    stc_adc_irq_calbakfn_pt_t stcAdcIrqCalbaks;
    stc_adc_ext_trig_cfg_t stcAdcExtTrigCfg;

    DDL_ZERO_STRUCT(stcAdcCfg);
    DDL_ZERO_STRUCT(stcAdcIrq);
    DDL_ZERO_STRUCT(stcAdcIrqCalbaks);
    DDL_ZERO_STRUCT(stcAdcExtTrigCfg);

    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);

    Gpio_SetAnalogMode(GpioPortA, GpioPin2);  //PA02 POT
    Gpio_SetAnalogMode(GpioPortB, GpioPin15); //PB15 VOLTAGE

    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdcBgr, TRUE);

    //ADC配置
    Adc_Enable();
    M0P_BGR->CR_f.BGR_EN = 0x1u; //BGR必须使能
    M0P_BGR->CR_f.TS_EN = 0x0u;
    delay100us(10);

    stcAdcCfg.enAdcOpMode = AdcSCanMode;          // 连续采样模式
    stcAdcCfg.enAdcClkDiv = AdcClkSysTDiv2;       // Adc工作时钟 PCLK/2
    stcAdcCfg.enAdcSampTimeSel = AdcSampTime8Clk; // 采样时钟 8个周期
    stcAdcCfg.enAdcRefVolSel = RefVolSelAVDD;     // 为了速度使用VDD
    stcAdcCfg.bAdcInBufEn = FALSE;

    Adc_Init(&stcAdcCfg); //Adc初始化

    // 配置顺序扫描转换通道,采样顺序CH0 --> CH1
    Adc_ConfigSqrChannel(CH0MUX, AdcExInputCH2);  // 电位器
    Adc_ConfigSqrChannel(CH1MUX, AdcExInputCH22); // 电压

    u8AdcSqrScanCnt = 2; //转换次数2次(3-1已在库函数内计算)

    Adc_ConfigDmaTrig(DmaSqr);                             // DMA传输
    Adc_ConfigSqrMode(&stcAdcCfg, u8AdcSqrScanCnt, FALSE); // 配置扫描扫描转换模式
    Adc_SQR_Start();                                       // 顺序扫描开始

    // 配置插队扫描转换通道,采样顺序CH0 --> CH1
    Adc_ConfigJqrChannel(JQRCH0MUX, AdcOPA1Input); // IU
    Adc_ConfigJqrChannel(JQRCH1MUX, AdcOPA2Input); // IV
    EnableNvic(ADC_IRQn, IrqLevel1, TRUE);         //Adc开中断

    // Adc_EnableIrq(); // 使能Adc中断 放在强拖之前开

    stcAdcIrq.bAdcJQRIrq = TRUE;
    stcAdcIrqCalbaks.pfnAdcJQRIrq = ADC_ISR;
    Adc_ConfigIrq(&stcAdcIrq, &stcAdcIrqCalbaks); //中断函数入口配置

    u8AdcJqrScanCnt = 2; //转换次数2次

    Adc_ConfigJqrMode(&stcAdcCfg, u8AdcJqrScanCnt, FALSE); //配置插队扫描转换模式

    stcAdcExtTrigCfg.enAdcExtTrigRegSel = AdcExtTrig1;
    stcAdcExtTrigCfg.enAdcTrig1Sel = AdcTrigTimer3;
    Adc_ExtTrigCfg(&stcAdcExtTrigCfg); //Timer0触发插队扫描转换
}
/**************************************************************************************************
 函 数 名  : PWM_Init
 功能描述  : PWM初始化
 输入参数  : 无
 输出参数  : void
**************************************************************************************************/
void PWM_init(void)
{
    uint16_t u16ArrValue;
    uint16_t u16CntValue;
    uint8_t u8ValidPeriod;

    stc_tim3_mode23_config_t stcTim3BaseCfg;
    stc_tim3_m23_compare_config_t stcTim3PortCmpCfg;
    stc_tim3_m23_adc_trig_config_t stcTim3TrigAdc;
    stc_tim3_m23_dt_config_t stcTim3DeadTimeCfg;
    stc_gpio_config_t stcTIM3Port;
    DDL_ZERO_STRUCT(stcTim3BaseCfg); //结构体初始化清零
    DDL_ZERO_STRUCT(stcTIM3Port);
    DDL_ZERO_STRUCT(stcTim3PortCmpCfg);
    DDL_ZERO_STRUCT(stcTim3TrigAdc);
    DDL_ZERO_STRUCT(stcTim3DeadTimeCfg);

    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); //GPIO 外设时钟使能
    Sysctrl_SetPeripheralGate(SysctrlPeripheralTim3, TRUE); //Timer3外设时钟使能

    stcTIM3Port.enDir = GpioDirOut;

    Gpio_Init(GpioPortA, GpioPin8, &stcTIM3Port);
    Gpio_SetAfMode(GpioPortA, GpioPin8, GpioAf2); //PA08设置为TIM3_CH0A

    Gpio_Init(GpioPortA, GpioPin7, &stcTIM3Port);
    Gpio_SetAfMode(GpioPortA, GpioPin7, GpioAf4); //PA07设置为TIM3_CH0B

    Gpio_Init(GpioPortA, GpioPin9, &stcTIM3Port);
    Gpio_SetAfMode(GpioPortA, GpioPin9, GpioAf2); //PA09设置为TIM3_CH1A

    Gpio_Init(GpioPortB, GpioPin0, &stcTIM3Port);
    Gpio_SetAfMode(GpioPortB, GpioPin0, GpioAf2); //PB00设置为TIM3_CH1B

    Gpio_Init(GpioPortA, GpioPin10, &stcTIM3Port);
    Gpio_SetAfMode(GpioPortA, GpioPin10, GpioAf2); //PA10设置为TIM3_CH2A

    Gpio_Init(GpioPortB, GpioPin1, &stcTIM3Port);
    Gpio_SetAfMode(GpioPortB, GpioPin1, GpioAf3); //PB01设置为TIM3_CH2B

    stcTim3BaseCfg.enWorkMode = Tim3WorkMode3;          //三角波模式 中央对齐模式
    stcTim3BaseCfg.enCT = Tim3Timer;                    //定时器功能，计数时钟为内部PCLK
    stcTim3BaseCfg.enPRS = Tim3PCLKDiv1;                //PCLK
    stcTim3BaseCfg.enPWMTypeSel = Tim3ComplementaryPWM; //互补输出PWM
    stcTim3BaseCfg.enPWM2sSel = Tim3SinglePointCmp;     //单点比较功能
    stcTim3BaseCfg.bOneShot = FALSE;                    //循环计数
    stcTim3BaseCfg.bURSSel = TRUE;                      //上下溢更新

    // stcTim3BaseCfg.pfnTim3Cb = Tim3Int; //中断函数入口

    Tim3_Mode23_Init(&stcTim3BaseCfg); //TIM3 的模式0功能初始化

    u16ArrValue = PWM_FRE_SETATA;
    Tim3_M23_ARRSet(u16ArrValue, TRUE); //设置重载值,并使能缓存

    PWMChangeDuty(&svm); // 下管全开

    stcTim3PortCmpCfg.enCHxACmpCtrl = Tim3PWMMode2;      //OCREFA输出控制OCMA:PWM模式2
    stcTim3PortCmpCfg.enCHxAPolarity = Tim3PortPositive; //正常输出
    stcTim3PortCmpCfg.bCHxACmpBufEn = TRUE;              //A通道缓存控制
    stcTim3PortCmpCfg.enCHxACmpIntSel = Tim3CmpIntNone;  //A通道比较控制:无

    stcTim3PortCmpCfg.enCHxBCmpCtrl = Tim3PWMMode2;      //OCREFB输出控制OCMB:PWM模式2(PWM互补模式下也要设置，避免强制输出)
    stcTim3PortCmpCfg.enCHxBPolarity = Tim3PortPositive; //正常输出
    // stcTim3PortCmpCfg.bCHxBCmpBufEn = TRUE;              //B通道缓存控制
    stcTim3PortCmpCfg.enCHxBCmpIntSel = Tim3CmpIntNone; //B通道比较控制:无

    Tim3_M23_PortOutput_Config(Tim3CH0, &stcTim3PortCmpCfg); //比较输出端口配置
    Tim3_M23_PortOutput_Config(Tim3CH1, &stcTim3PortCmpCfg); //比较输出端口配置
    Tim3_M23_PortOutput_Config(Tim3CH2, &stcTim3PortCmpCfg); //比较输出端口配置

    PortOutput_Config(0, 0, 0, 0, 0, 0); // 默认关掉PWM

    stcTim3TrigAdc.bEnTrigADC = TRUE;         //使能ADC触发全局控制
    stcTim3TrigAdc.bEnUevTrigADC = TRUE;      //Uev更新触发ADC
    Tim3_M23_TrigADC_Config(&stcTim3TrigAdc); //触发ADC配置

    stcTim3DeadTimeCfg.bEnDeadTime = TRUE;
    stcTim3DeadTimeCfg.u8DeadTimeValue = 0x73;
    Tim3_M23_DT_Config(&stcTim3DeadTimeCfg); //死区设置

    u8ValidPeriod = 1;                      //事件更新周期设置，0表示三角波每半个周期更新一次，每+1代表延迟半个周期
    Tim3_M23_SetValidPeriod(u8ValidPeriod); //间隔周期设置

    u16CntValue = 0;

    Tim3_M23_Cnt16Set(u16CntValue); //设置计数初值

    // Tim3_ClearAllIntFlag();                 //清中断标志
    // Tim3_Mode23_EnableIrq(Tim3UevIrq);      //使能TIM3 UEV更新中断
    // EnableNvic(TIM3_IRQn, IrqLevel0, TRUE); //TIM3中断使能

    Tim3_M23_EnPWM_Output(TRUE, FALSE); //端口输出使能

    Tim3_M23_Run(); //运行
}
