/*
 * prb_pwmss.c
 *
 *  Created on: 10 mar 2019
 *      Author: andrea
 */
#include <prb_pwmss.h>

volatile sysPwmss* PRU_PWMSS[3] = { &PWMSS0, &PWMSS1, &PWMSS2};
uint32_t* CM_PER_PWMCSS_CLKCTRL[3] = {(uint32_t*) 0x44E000D4, (uint32_t*) 0x44E000CC, (uint32_t*) 0x44E000D8};
uint32_t* PWMSS_CTRL_REG = (uint32_t*)0x44E10664;
uint8_t pru_pwmss_lib_initialized[3] = {0,0,0};

uint8_t pru_pwmss_lib_Init(uint8_t pwmssDevice) {
    /*
     * A T T E N Z I O N E!
     * https://beagleboard.org/discuss?place=msg%2Fbeagleboard%2FLZhL4S9taic%2FK4HCC6d_AgAJ
     * Per utilizzare pwmss da PRU con kernel 4 linux è necessario modificare
     * la configurazione dei ehrpwmX_tbclk aggiungendo ti,set-bit-to-disable
     *
     *     ehrpwm0_tbclk: ehrpwm0_tbclk@44e10664 {
        #clock-cells = <0>;
        compatible = "ti,gate-clock";
        clocks = <&l4ls_gclk>;
        ti,bit-shift = <0>;
        ti,set-bit-to-disable;
        reg = <0x0664>;
    };

    ehrpwm1_tbclk: ehrpwm1_tbclk@44e10664 {
        #clock-cells = <0>;
        compatible = "ti,gate-clock";
        clocks = <&l4ls_gclk>;
        ti,bit-shift = <1>;
        ti,set-bit-to-disable;
        reg = <0x0664>;
    };

    ehrpwm2_tbclk: ehrpwm2_tbclk@44e10664 {
        #clock-cells = <0>;
        compatible = "ti,gate-clock";
        clocks = <&l4ls_gclk>;
        ti,bit-shift = <2>;
        ti,set-bit-to-disable;
        reg = <0x0664>;
    };

    La soluzione è riferita da questa discussione:
    https://groups.google.com/forum/#!msg/beagleboard/eVgyVduT288/XgsiUiNiBwAJ

    La modifica l'ho stata fatta su am335x-boneblack-uboot.dts e funziona correttamente.

     */
    /* PinMux configured with device tree */
    (*(CM_PER_PWMCSS_CLKCTRL[pwmssDevice])) = 2; /* enable module */

    PRU_PWMSS[pwmssDevice]->CLKCONFIG_bit.EPWMCLK_EN = 0b1; /* enable clock */
    PRU_PWMSS[pwmssDevice]->EPWM_TBCTL = 0xFFFF & (0x8000 | 0x0030 | 0x1400); /* FREE RUN on, SUNC0SEL off, CLKDIV = 5 (3.125MHz) */

    /* Frequenza PWM */
    /* SYSCLKOUT=100MHz, HSPCLKDIV=0
     * CLKDIV = 5 => TBCLK = 100MHz/32 = 3.125MHz
     * PWMPERIOD = (TBPRD + 1)/TBCLK; a 50Hz deve valere 20000us
     * per avere un PWMPERIOD di 20ms serve TBPRD = 20000us * TBCLK - 1 = 20000us * 3.125MHz - 1 = 62499;
     * */
    PRU_PWMSS[pwmssDevice]->EPWM_TBPRD = 0xF423; // 62499
    PRU_PWMSS[pwmssDevice]->EPWM_AQCTLA = 0xFFFF & (0x0002 | 0x0030 ); // ZRO = 2, CAU = 3, il resto = 0
    PRU_PWMSS[pwmssDevice]->EPWM_AQCTLB = 0xFFFF & (0x0002 | 0x0300 ); // ZRO = 2, CBU = 3, il resto = 0
    PRU_PWMSS[pwmssDevice]->EPWM_CMPA = 0; //
    PRU_PWMSS[pwmssDevice]->EPWM_CMPB = 0; //
    *PWMSS_CTRL_REG = 7; // enable  pwmss_ctrl Register: bits pwmss2_tbclk, pwmss1_tbclke, pwmss0_tbclke
    pru_pwmss_lib_initialized[pwmssDevice] = 1;
    return 0;
}
uint8_t pru_pwmss_lib_IsRunning(uint8_t pwmssDevice)
{
    return (PRU_PWMSS[pwmssDevice]->CLKCONFIG_bit.EPWMCLK_EN);
}

void pru_pwmss_lib_Start(uint8_t pwmssDevice)
{
    if (!pru_pwmss_lib_IsRunning(pwmssDevice))
    {
        (*CM_PER_PWMCSS_CLKCTRL[pwmssDevice]) = 2; // enable module
    }
}
void pru_pwmss_lib_Stop(uint8_t pwmssDevice)
{
    if (pru_pwmss_lib_IsRunning(pwmssDevice))
    {
        (*CM_PER_PWMCSS_CLKCTRL[pwmssDevice]) = 0; // disable module
    }
}
void pru_pwmss_lib_SetData(uint8_t pwmssDevice, uint16_t period, uint16_t duA, uint16_t duB) {
    PRU_PWMSS[pwmssDevice]->EPWM_TBPRD = period;
    PRU_PWMSS[pwmssDevice]->EPWM_CMPA = duA;
    PRU_PWMSS[pwmssDevice]->EPWM_CMPB = duB;
}
void pru_pwmss_lib_SetDuty(uint8_t pwmssDevice, uint16_t duA, uint16_t duB) {
    PRU_PWMSS[pwmssDevice]->EPWM_CMPA = duA;
    PRU_PWMSS[pwmssDevice]->EPWM_CMPB = duB;
}
void pru_pwmss_lib_SetPeriod(uint8_t pwmssDevice, uint16_t period) {
    PRU_PWMSS[pwmssDevice]->EPWM_TBPRD = period;
}

