/**
 ****************************************************************************************
 *
 * @file user_proxr.c
 *
 * @brief Proximity reporter project source code.
 *
 * Copyright (C) 2015-2020 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APP
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"
#include "gapc_task.h"
#include "user_periph_setup.h"
#include "wkupct_quadec.h"
#include "app_easy_msg_utils.h"
#include "gpio.h"
#include "app_security.h"
#include "user_proxr.h"
#include "arch.h"
#include "arch_api.h"
#if defined (__DA14531__) && (defined (CFG_APP_GOTO_HIBERNATION) || defined (CFG_APP_GOTO_STATEFUL_HIBERNATION))
#include "arch_hibernation.h"
#endif
#include "app_task.h"
#include "app_proxr.h"

#if defined (__DA14531__)
#include "rtc.h"
#include "timer1.h"
#endif

#if (BLE_SUOTA_RECEIVER)
#include "app_suotar.h"
#endif

#if defined (CFG_SPI_FLASH_ENABLE)
#include "spi_flash.h"
#endif

//chen 2021-7-7
#include "user_custs1_def.h"
#include "custs1_task.h"
#include "gattc_task.h"
#include "myqueue.h"	
#include "uart.h"
#include "uart_utils.h"
#include "myprintf.h"

//chen 2021-7-7
timer_hnd app_param_update_request_timer_used   __SECTION_ZERO("retention_mem_area0");
timer_hnd app_sleep_set_timer                   __SECTION_ZERO("retention_mem_area0");
uint8_t app_connection_idx                      __SECTION_ZERO("retention_mem_area0");

#define APP_PARAM_UPDATE_REQUEST_TO         (300) //1 (1000)   // 1000*10ms = 10sec, The maximum allowed value is 41943sec (4194300 * 10ms)
#define APP_SLEEP_SET_TIMER_REQUEST_TO        (1000)
extern void userAPPInit(void);
extern int queueIsEmpty(pQueue q);

#define MAX_CMD_LEN 20
#define MAX_PARAM_LEN 256

static bool ble_notify_app(uint8_t* pdata, int16_t len);
/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles APP_WAKEUP_MSG sent when device exits deep sleep. Triggered by button press.
 ****************************************************************************************
*/
static void app_wakeup_cb(void)
{
    // If state is not idle, ignore the message
    if (ke_state_get(TASK_APP) == APP_CONNECTABLE)
    {
        default_advertise_operation();
    }
}

/**
 ****************************************************************************************
 * @brief Routine to resume system from sleep state.
 ****************************************************************************************
 */

void app_sleep_set_timer_cb(void)
{
#if defined(GPIO_LED_PORT) && defined(GPIO_LED_PIN)
    GPIO_SetInactive( GPIO_LED_PORT, GPIO_LED_PIN);
#endif
    app_sleep_set_timer = EASY_TIMER_INVALID_TIMER;
    if(!arch_get_sleep_mode())
    {
        arch_restore_sleep_mode();
        arch_set_sleep_mode(app_default_sleep_mode);
        MyPrintf("arch_set_extended_sleep\n"); 
    }
}

static void app_resume_system_from_sleep(void)
{
#if !defined (__DA14531__)
    if (GetBits16(SYS_STAT_REG, PER_IS_DOWN))
    {
        periph_init();
    }
#endif

    if (arch_ble_ext_wakeup_get())
    {
        arch_set_sleep_mode(app_default_sleep_mode);
        arch_ble_force_wakeup();
        arch_ble_ext_wakeup_off();
        app_easy_wakeup();
    }
    //chen 2021-7-7
    if (arch_get_sleep_mode())
    {
        arch_restore_sleep_mode();
        arch_disable_sleep();
    }
    //stop the current running timer
    if(app_sleep_set_timer != EASY_TIMER_INVALID_TIMER)
        app_easy_timer_cancel(app_sleep_set_timer);
    app_sleep_set_timer=app_easy_timer(APP_SLEEP_SET_TIMER_REQUEST_TO, app_sleep_set_timer_cb);
    userAPPInit();

#if defined(GPIO_LED_PORT) && defined(GPIO_LED_PIN)
    GPIO_SetActive( GPIO_LED_PORT, GPIO_LED_PIN );
#endif
}

/**
 ****************************************************************************************
 * @brief Button press callback function. Registered in WKUPCT driver. 按键回调函数
 ****************************************************************************************
 */
static void app_button_press_cb(void)
{
#if (BLE_PROX_REPORTER)
    if (alert_state.lvl != PROXR_ALERT_NONE)
    {
        app_proxr_alert_stop();
    }
#endif
    app_resume_system_from_sleep();

    app_button_enable();
}

void app_button_enable(void)
{
    app_easy_wakeup_set(app_wakeup_cb);
    wkupct_register_callback(app_button_press_cb);

    if (GPIO_GetPinStatus(GPIO_BUTTON_PORT, GPIO_BUTTON_PIN))
    {
        wkupct_enable_irq(WKUPCT_PIN_SELECT(GPIO_BUTTON_PORT, GPIO_BUTTON_PIN), // select pin (GPIO_BUTTON_PORT, GPIO_BUTTON_PIN)
                          WKUPCT_PIN_POLARITY(GPIO_BUTTON_PORT, GPIO_BUTTON_PIN, WKUPCT_PIN_POLARITY_LOW), // polarity low 按键按下后松开
                          1, // 1 event
                          0); // debouncing time = 0
    }
}

#if (BLE_SUOTA_RECEIVER)
void on_suotar_status_change(const uint8_t suotar_event)
{
#if (!SUOTAR_SPI_DISABLE)
    uint8_t dev_id;

    // Release Flash from power down
    spi_flash_release_from_power_down();

    // Try to auto-detect the device
    spi_flash_auto_detect(&dev_id);

    if (suotar_event == SUOTAR_END)
    {
        // Power down SPI Flash
        spi_flash_power_down();
    }
#endif
}
#endif


//chen 2021-7-7
static void userGattcExcMtuCMD(uint8_t conidx)
{
	struct gattc_exc_mtu_cmd *cmd = KE_MSG_ALLOC(GATTC_EXC_MTU_CMD,
													KE_BUILD_ID(TASK_GATTC, conidx),
													TASK_APP, gattc_exc_mtu_cmd);
	cmd->operation = GATTC_MTU_EXCH;

	ke_msg_send(cmd);
}
//chen
static void param_update_request_timer_cb()
{
    app_easy_gap_param_update_start(app_connection_idx);
    app_param_update_request_timer_used = EASY_TIMER_INVALID_TIMER;
    
}


//static void user_gattc_exc_mtu_cmd(uint8_t conidx)
//{
//    struct gattc_exc_mtu_cmd *cmd =  KE_MSG_ALLOC(GATTC_EXC_MTU_CMD,
//                                                  KE_BUILD_ID(TASK_GATTC, conidx),
//                                                  TASK_APP,
//                                                  gattc_exc_mtu_cmd);

//    cmd->operation=GATTC_MTU_EXCH;
//    cmd->seq_num = 0;
//    ke_msg_send(cmd);
//}

//void user_app_on_data_length_change(uint8_t conidx, struct gapc_le_pkt_size_ind *ind)
//{
//		MyPrintf("Data length set: TX %u, RX %u for coninx %u.\r\n", (int)ind->max_tx_octets, ind->max_rx_octets, conidx);
//		
////		uint8_t role = gapc_get_role(conidx);
////		if (role == MASTER_ROLE) 
////		{	
//				//Start MTU exchange
//				user_gattc_exc_mtu_cmd(conidx);
////		}
//}

void user_app_on_connection(uint8_t conidx, struct gapc_connection_req_ind const *param)
{
	 if (app_env[conidx].conidx != GAP_INVALID_CONIDX)
	 {
		   app_connection_idx=conidx;
//		   app_easy_gap_set_data_packet_length(conidx, user_gapm_conf.max_txoctets, user_gapm_conf.max_txtime);
		   userGattcExcMtuCMD(conidx);
		   app_param_update_request_timer_used = app_easy_timer(APP_PARAM_UPDATE_REQUEST_TO, param_update_request_timer_cb);
	 }
	
	default_app_on_connection(conidx, param);
}



void user_app_on_disconnect(struct gapc_disconnect_ind const *param)
{
    default_app_on_disconnect(NULL);

#if (BLE_BATT_SERVER)
    app_batt_poll_stop();
#endif

#if (BLE_SUOTA_RECEIVER)
    // Issue a platform reset when it is requested by the suotar procedure
    if (suota_state.reboot_requested)
    {
        // Reboot request will be served
        suota_state.reboot_requested = 0;

        // Platform reset
        platform_reset(RESET_AFTER_SUOTA_UPDATE);
    }
#endif

#if BLE_PROX_REPORTER
    app_proxr_alert_stop();
#endif
}

#if defined (__DA14531__)

#if defined (CFG_EXT_SLEEP_WAKEUP_RTC) || defined (CFG_DEEP_SLEEP_WAKEUP_RTC)
/**
 ****************************************************************************************
 * @brief RTC interrupt handler routine for wakeup.
 ****************************************************************************************
*/

static void rtc_interrupt_hdlr(uint8_t event)
{
#if defined (CFG_EXT_SLEEP_WAKEUP_RTC)
    app_resume_system_from_sleep();
#endif
}

/**
 ****************************************************************************************
 * @brief Configure RTC to generate an interrupt after 10 seconds.
 ****************************************************************************************
*/
static void configure_rtc_wakeup(void)
{
    rtc_time_t alarm_time;

    // Init RTC
    rtc_reset();

    // Configure the RTC clock; RCX is the RTC clock source (14420 Hz)
    rtc_clk_config(RTC_DIV_DENOM_1000, 14420);
    rtc_clock_enable();

    rtc_config_t cfg = {.hour_clk_mode = RTC_HOUR_MODE_24H, .keep_rtc = 0};

    rtc_time_t time = {.hour_mode = RTC_HOUR_MODE_24H, .pm_flag = 0, .hour = 11,
                       .minute = 55, .sec = 30, .hsec = 00};

    // Alarm interrupt in ten seconds
    alarm_time = time;
    alarm_time.sec += 10;

    // Initialize RTC, set time and data, register interrupt handler callback function and enable seconds interrupt
    rtc_init(&cfg);

    // Start RTC
    rtc_set_time_clndr(&time, NULL);
    rtc_set_alarm(&alarm_time, NULL, RTC_ALARM_EN_SEC);

    // Clear pending interrupts
    rtc_get_event_flags();
    rtc_register_intr(rtc_interrupt_hdlr, RTC_INTR_ALRM);
#if defined (CFG_EXT_SLEEP_WAKEUP_RTC)
    app_easy_wakeup_set(app_wakeup_cb);
#endif
}
#endif

#if defined (CFG_EXT_SLEEP_WAKEUP_TIMER1) || defined (CFG_DEEP_SLEEP_WAKEUP_TIMER1)
/**
 ****************************************************************************************
 * @brief Timer1 interrupt handler routine for wakeup.
 ****************************************************************************************
*/

static void timer1_interrupt_hdlr(void)
{
#if defined (CFG_EXT_SLEEP_WAKEUP_TIMER1)
    app_resume_system_from_sleep();
#endif
}

/**
 ****************************************************************************************
 * @brief Configure Timer1 to generate an interrupt when it reaches its max value.
 ****************************************************************************************
*/
static void configure_timer1_wakeup(void)
{
    timer1_count_options_t count_options = {.input_clk = TIM1_CLK_SRC_LP,
                                            .free_run = TIM1_FREE_RUN_ON,
                                            .irq_mask = TIM1_IRQ_MASK_OFF,
                                            .count_dir = TIM1_CNT_DIR_UP,
                                            .reload_val = TIM1_RELOAD_MAX,
    };
    // Enable Timer1 interrupt
    timer1_enable_irq();
#if defined (CFG_EXT_SLEEP_WAKEUP_TIMER1)
    app_easy_wakeup_set(app_wakeup_cb);
#endif
    timer1_count_config(&count_options, timer1_interrupt_hdlr);

    // Start the Timer
    timer1_start();
}
#endif

#if defined (CFG_APP_GOTO_DEEP_SLEEP)
/**
 ****************************************************************************************
 * @brief Put the system into deep sleep mode. It demonstrates the deep sleep mode usage
 *        and how the system can wake up from it. The exit from the deep sleep state causes 
 *        a system reboot.
 * @note  The system can wake up from deep sleep by:
 *          - external wake up interrupt, caused e.g. by button press (properly configured GPIO pin)
 *          - power on reset, caused e.g. by button press (properly configured GPIO pin)
 *          - interrupt generated from RTC
 *          - interrupt generated from Timer1
 *        When the system exits deep sleep state, the boot process is triggered.
 *        The application code has to be programmed in an external memory resource or
 *        in the OTP memory, in order for the system to reboot properly.
 ****************************************************************************************
*/
static void put_system_into_deep_sleep(void)
{
#if defined (CFG_DEEP_SLEEP_WAKEUP_POR)
    // Configure button for POR
    GPIO_EnablePorPin(GPIO_BUTTON_PORT, GPIO_BUTTON_PIN, GPIO_POR_PIN_POLARITY_LOW, GPIO_GetPorTime());
#endif

#if defined (CFG_DEEP_SLEEP_WAKEUP_GPIO)
    wkupct_enable_irq(WKUPCT_PIN_SELECT(GPIO_BUTTON_PORT, GPIO_BUTTON_PIN), // Select pin
                      WKUPCT_PIN_POLARITY(GPIO_BUTTON_PORT, GPIO_BUTTON_PIN, WKUPCT_PIN_POLARITY_LOW), // Polarity low
                      1, // 1 event
                      0); // debouncing time = 0
#endif

#if defined (CFG_DEEP_SLEEP_WAKEUP_RTC)
    configure_rtc_wakeup();
#endif

#if defined (CFG_DEEP_SLEEP_WAKEUP_TIMER1)
    configure_timer1_wakeup();
#endif

    // Go to deep sleep
    arch_set_deep_sleep(CFG_DEEP_SLEEP_RAM1,
                        CFG_DEEP_SLEEP_RAM2,
                        CFG_DEEP_SLEEP_RAM3,
                        CFG_DEEP_SLEEP_PAD_LATCH_EN);
}
#endif // (CFG_APP_GOTO_DEEP_SLEEP)

#else //__DA14531__

#if defined (CFG_APP_GOTO_DEEP_SLEEP)

/**
 ****************************************************************************************
 * @brief Put the system into deep sleep mode. It demonstrates the deep sleep mode usage
 *        and how the system can wake up from it. Once the system enters deep sleep state
 *        it retains NO RAM blocks. The exit from the deep sleep state causes a system
 *        reboot.
 * @note  The system can wake up from deep sleep by:
 *          - external wake up interrupt, caused e.g. by button press (properly configured GPIO pin)
 *          - power on reset, caused e.g. by button press (properly configured GPIO pin)
 *          - H/W reset button press or power cycle (at any time)
 *        When the system exits deep sleep state, the boot process is triggered.
 *        The application code has to be programmed in an external memory resource or
 *        in the OTP memory, in order for the system to reboot properly.
 ****************************************************************************************
*/
static void put_system_into_deep_sleep(void)
{
#if defined (CFG_DEEP_SLEEP_WAKEUP_GPIO)
    // Configure button for wake-up interrupt
    app_button_enable();

    // Set deep sleep - external interrupt wake up
    arch_set_deep_sleep(true);

#elif defined (CFG_DEEP_SLEEP_WAKEUP_POR)
    // Configure button for POR
    GPIO_EnablePorPin(GPIO_BUTTON_PORT, GPIO_BUTTON_PIN, GPIO_POR_PIN_POLARITY_LOW, GPIO_GetPorTime());

    // Set deep sleep - POR wake up
    arch_set_deep_sleep(false);

#else
    // Do nothing.
    // The system will eventually enter the selected Extended sleep state.
    // A button press will wake up the system if the respective GPIO is configured as a wake up interrupt.
#endif
}

#endif //(CFG_APP_GOTO_DEEP_SLEEP)

#endif

void app_advertise_complete(const uint8_t status)
{
    if ((status == GAP_ERR_NO_ERROR) || (status == GAP_ERR_CANCELED))
    {

#if (BLE_PROX_REPORTER)
        app_proxr_alert_stop();
#endif
    }

    if (status == GAP_ERR_CANCELED)
    {
        arch_ble_ext_wakeup_on();

#if defined (__DA14531__)
    // Configure PD_TIM
#if defined (CFG_EXT_SLEEP_WAKEUP_RTC) || defined (CFG_EXT_SLEEP_WAKEUP_TIMER1) || \
    defined (CFG_DEEP_SLEEP_WAKEUP_RTC) || defined (CFG_DEEP_SLEEP_WAKEUP_TIMER1)
        // Ensure PD_TIM is open
        SetBits16(PMU_CTRL_REG, TIM_SLEEP, 0);
        // Wait until PD_TIM is opened
        while ((GetWord16(SYS_STAT_REG) & TIM_IS_UP) != TIM_IS_UP);
#else
        // Close PD_TIM
        SetBits16(PMU_CTRL_REG, TIM_SLEEP, 1);
        // Wait until PD_TIM is closed
        while ((GetWord16(SYS_STAT_REG) & TIM_IS_DOWN) != TIM_IS_DOWN);
#endif
#endif

#if defined (CFG_APP_GOTO_DEEP_SLEEP)
        // Put system into deep sleep
        put_system_into_deep_sleep();
#elif defined (__DA14531__) && defined (CFG_APP_GOTO_HIBERNATION)
        // Put system into hibernation
        arch_set_hibernation(HIB_WAKE_UP_PIN_MASK,
                             CFG_HIBERNATION_RAM1,
                             CFG_HIBERNATION_RAM2,
                             CFG_HIBERNATION_RAM3,
                             CFG_HIBERNATION_REMAP,
                             CFG_HIBERNATION_PAD_LATCH_EN);
#elif defined (__DA14531__) && defined (CFG_APP_GOTO_STATEFUL_HIBERNATION)
        // Put system into stateful hibernation
        arch_set_stateful_hibernation(HIB_WAKE_UP_PIN_MASK,
                                      CFG_STATEFUL_HIBERNATION_RAM1,
                                      CFG_STATEFUL_HIBERNATION_RAM2,
                                      CFG_STATEFUL_HIBERNATION_RAM3,
                                      CFG_STATEFUL_HIBERNATION_REMAP,
                                      CFG_STATEFUL_HIBERNATION_PAD_LATCH_EN);
        #if (DEVELOPMENT_DEBUG)
            // Turn on the orange LED (D5 on the 376-18-B Motherboard)
            SetWord16(P09_MODE_REG, ((uint32_t) OUTPUT) | ((uint32_t) PID_GPIO));
            SetWord16(P0_SET_DATA_REG, 1 << GPIO_ALERT_LED_PIN);
            // Keep it on for a couple of seconds
            for (uint32_t i = 4*2000000; i != 0; i--)
            {
                __NOP();
            }
            // Turn it off
            SetWord16(P0_RESET_DATA_REG, 1 << GPIO_ALERT_LED_PIN);
        #endif // DEVELOPMENT_DEBUG

        // Configure button to trigger wake-up interrupt from extended sleep
        app_button_enable();
#elif defined (__DA14531__) && defined (CFG_EXT_SLEEP_WAKEUP_RTC)
        // Configure RTC to trigger wake-up interrupt from extended sleep
        configure_rtc_wakeup();
#elif defined (__DA14531__) && defined (CFG_EXT_SLEEP_WAKEUP_TIMER1)
        // Configure TIMER1 to trigger wake-up interrupt from extended sleep
        configure_timer1_wakeup();
#else
        // Configure button to trigger wake-up interrupt from extended sleep
        // app_button_enable();
#endif
    }
}



//chen 2021-7-7
void user_custs1_server_rx_ind_handler(ke_msg_id_t const msgid,
                                     struct custs1_val_write_ind const *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
		uint8_t conidx = param->conidx;
		if(conidx != GAP_INVALID_CONIDX)
		{
			//TODO: here we notify data back to the host for test purpose.
			//Note: the SDK notify all the connected host by default!
//		  custs1_notify_data_req(conidx, param->length, (uint8_t *)param->value);
//			custs1_write_data_req(conidx, param->length, (uint8_t *)param->value);	
//        uart_outdata_printf(&param->value[0], param->length);		//chen 2021-7-8
        MyPrintf("the app send data to BLE device.\r\n"); 
			
		}
}

#define Cmp(cmd, str)  (strncmp(cmd, str, strlen(str)) == 0)

struct bd_addr ble_device_mac   __SECTION_ZERO("retention_mem_area0");;


static bool app_at_cmd_deal(char* cmd)
{   
    char rsp[256] = {0};
    char rsp_value[256] = {0};
    bool result = 1;

    if (Cmp(cmd, "ADVERTISE")) {
        llm_util_set_public_addr(&ble_device_mac);
        app_easy_gap_undirected_advertise_start();
    } else if (Cmp(cmd, "DISADVERTISE")) {
        app_easy_gap_advertise_stop();
    } else if (Cmp(cmd, "DISCON")) {
        app_easy_gap_disconnect(app_connection_idx);
    } else if (Cmp(cmd, "NAME")) {
        struct app_device_name device_name;
        default_app_on_get_dev_name(&device_name);
        strncpy(rsp_value, device_name.name, device_name.length);
    } else if (Cmp(cmd, "MAC")) {
        sprintf(rsp_value, "%02X%02X%02X%02X%02X%02X", 
            ble_device_mac.addr[5], ble_device_mac.addr[4], ble_device_mac.addr[3], 
            ble_device_mac.addr[2], ble_device_mac.addr[1], ble_device_mac.addr[0]);
    }

    if (strlen(rsp_value) > 0) {
        sprintf(rsp, "+%s:%d, %s\r\nOK\r\n", cmd, result, rsp_value);
    } else {
        sprintf(rsp, "+%s:%d\r\nOK\r\n", cmd, result);
    }
    
    uart_outdata_printf(rsp, strlen(rsp));
    return true;
}

static bool app_at_config_deal(char* cmd, char* param)
{
    char rsp[256] = {0};
    sprintf(rsp, "cmd: %s, param: %s\r\n", cmd, param);
    uart_outdata_printf(rsp, strlen(rsp));

    if (Cmp(cmd, "WRITE")) {
        ble_notify_app(param, strlen(param));
    } else if (Cmp(cmd, "MAC")) {
        if (strlen(param) == 12) {
            uint8_t new_mac[6];
            sscanf(param, "%02hhx%02hhx%02hhx%02hhx%02hhx%02hhx", &new_mac[5], &new_mac[4], &new_mac[3], &new_mac[2], &new_mac[1], &new_mac[0]);
            for (int i = 0; i < 6; i++) {
                ble_device_mac.addr[i] = new_mac[i];
            }
        }
    } else if (Cmp(cmd, "NAME")) {
        struct gapc_set_dev_info_req_ind dev_info;
        dev_info.req = GAPC_DEV_NAME;
        uint8_t status;
        memcpy(dev_info.info.name.value, param, strlen(param));
        dev_info.info.name.length = strlen(param);
        default_app_on_set_dev_info(&dev_info, &status);
    }
    return true;
}

// 处理接收到的串口数据
void app_uart_deal(void)
{
    uint32_t state = 0;
    state = ke_get_max_mem_usage();
    if( state <= 2988 ) 
    {    
        if(!queueIsEmpty(&UartRxQueue))
        {       
            int16_t len;    
            uint8_t pData[256] = {0};
            bool result = false;

            len = queueDequeue(&UartRxQueue, &pData);

            if (len > 5) {
                if (pData[len - 2] == '\r' && pData[len - 1] == '\n') 
                    pData[len - 2] = '\0'; // 去掉\r\n

                char cmd[MAX_CMD_LEN];
                char param[MAX_PARAM_LEN];
                int num_args = sscanf(pData, "AT+%[^=]=%s", cmd, param);
                if (num_args == 1) {
                    // 处理仅带命令的情况
                    app_at_cmd_deal(cmd);
                    return;
                } else if (num_args == 2) {
                    // 处理带参数的情况
                    app_at_config_deal(cmd, param);
                    return;
                }
            }
            uart_outdata_printf("ERR", 3);
            uart_outdata_printf(pData, len);
        }
    }
}

// 数据notify到app
static bool ble_notify_app(uint8_t* pData, int16_t len) 
{   
    struct custs1_val_ntf_ind_req *req = KE_MSG_ALLOC_DYN(CUSTS1_VAL_NTF_REQ,
                    prf_get_task_from_id(TASK_ID_CUSTS1),
                    TASK_APP,
                    custs1_val_ntf_ind_req,
                    len);

    req->handle = CUST1_IDX_VAL;
    req->length = len;   
    req->notification = true; 
    memcpy(req->value, pData, len);
    ke_msg_send(req);

    return true;
}


//chen 2021-7-7
void user_catch_rest_hndl(ke_msg_id_t const msgid,
                          void const *param,
                          ke_task_id_t const dest_id,
                          ke_task_id_t const src_id)
{
    switch(msgid)
    {
        case CUSTS1_VAL_WRITE_IND:
        {
            struct custs1_val_write_ind const *msg_param = (struct custs1_val_write_ind const *)(param);

            switch (msg_param->handle)
            {
               case CUST1_IDX_VAL:
                uart_outdata_printf((char *)&msg_param->value[0], msg_param->length);// 将接收到的APP数据通过串口发送
                break;

                default:
                    break;
            }
        } break;

        case CUSTS1_VAL_NTF_CFM:
        {
            struct custs1_val_ntf_cfm const *msg_param = (struct custs1_val_ntf_cfm const *)(param);

            switch (msg_param->handle)
            {
                case CUST1_IDX_VAL:
                    // user_custs1_server_tx_ntf_cfm_handler(msgid, msg_param, dest_id, src_id);
                    break;

                default:
                    break;
            }
            }
          break;

        case CUSTS1_VAL_IND_CFM:
        {
            struct custs1_val_ind_cfm const *msg_param = (struct custs1_val_ind_cfm const *)(param);

            switch (msg_param->handle)
            {
                default:
                break;
             }
        } break;

        default:
            break;
    }
}



/// @} APP
