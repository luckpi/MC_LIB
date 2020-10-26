#ifndef _BEMF_H
#define _BEMF_H
extern void CheckZeroCrossing(void);
extern void ADC_ISR(void);
#define Q_Calc(a, b) ((a + a + a) > (b) ? (1) : (0))
#define POLE_PAIRS (4) // 极对数
#endif
