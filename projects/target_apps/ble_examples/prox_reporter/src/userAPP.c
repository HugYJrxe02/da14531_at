
#include "userAPP.h"
#include "timer0_2.h"
#include "timer0.h"
#include "user_periph_setup.h"      
#include "arch_console.h"
#include "rf_531.h"
#include "myqueue.h"
#include "user_proxr.h"


#define     NO_PWM           0x0                // PWM not used
#define     LOAD_VALUE       4000                 //16M??8??,20000 reload value for 100ms
                                                //32M/8??=  4M,?????10      , 4000000/10 = 400kHz, 
                                                //(T = 1/400kHz * LOAD_VALUE = 1/400000(s) * 400 = 0.001s=1ms) 

extern volatile char    rxbuf[1];
extern volatile uint8_t txbuf[1];
extern uint8_t uartBuf_index;
extern volatile uint8_t uartBuf[];

//transmission mode
uint8_t TransmissionMode = PASSTHROUGH_MODE;


pQueue Uart1RxQueue;
pQueue Uart1TxQueue;     
extern bool Uart1RxFlag;

static tim0_2_clk_div_config_t clk_div_config =
{
    .clk_div  = TIM0_2_CLK_DIV_8
};


static void timer0ProcessCallback(void)
{
#if 1    
    if (!Uart1RxFlag)
    {
        if (uartBuf_index)
        {        
            queueEnqueue(&UartRxQueue, (void*)uartBuf, uartBuf_index);
            uartBuf_index = 0;
            if(!queueIsEmpty(&UartRxQueue))
            {    
                app_uart_deal();  
            }
        }    

    }
#endif

}

void timer0Init(void)
{
     // Stop timer for enter settings
    timer0_stop();

    // register callback function for SWTIM_IRQn irq
    timer0_register_callback(timer0ProcessCallback);

    // Enable the Timer0/Timer2 input clock
    timer0_2_clk_enable();

    // Set the Timer0/Timer2 input clock division factor to 8, so 16 MHz / 8 = 2 MHz input clock
    timer0_2_clk_div_set(&clk_div_config);

    // clear PWM settings register to not generate PWM
    timer0_set_pwm_high_counter(NO_PWM);
    timer0_set_pwm_low_counter(NO_PWM);

    // Set timer with 2MHz source clock divided by 10 so Fclk = 2MHz/10 = 200kHz
    timer0_init(TIM0_CLK_FAST, PWM_MODE_ONE, TIM0_CLK_DIV_BY_10);

    // reload value for 100ms (T = 1/200kHz * RELOAD_100MS = 0,000005 * 20000 = 100ms)
    timer0_set_pwm_on_counter(LOAD_VALUE); // 4000=>20ms 200=>1ms

    // Enable SWTIM_IRQn irq
    timer0_enable_irq();

    // Start Timer0
    timer0_start();
}


void userAPPInit(void)
{        
    timer0Init();
}


