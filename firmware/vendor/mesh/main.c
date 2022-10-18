/********************************************************************************************************
 * @file     main.c 
 *
 * @brief    for TLSR chips
 *
 * @author	 telink
 * @date     Sep. 30, 2010
 *
 * @par      Copyright (c) 2010, Telink Semiconductor (Shanghai) Co., Ltd.
 *           All rights reserved.
 *           
 *			 The information contained herein is confidential and proprietary property of Telink 
 * 		     Semiconductor (Shanghai) Co., Ltd. and is available under the terms 
 *			 of Commercial License Agreement between Telink Semiconductor (Shanghai) 
 *			 Co., Ltd. and the licensee in separate contract or the terms described here-in. 
 *           This heading MUST NOT be removed from this file.
 *
 * 			 Licensees are granted free, non-transferable use of the information in this 
 *			 file under Mutual Non-Disclosure Agreement. NO WARRENTY of ANY KIND is provided. 
 *           
 *******************************************************************************************************/
#include "proj/tl_common.h"
#include "proj/mcu/watchdog_i.h"
#include "vendor/common/user_config.h"
#include "proj_lib/rf_drv.h"
#include "proj_lib/pm.h"
#include "proj_lib/ble/blt_config.h"
#include "proj_lib/ble/ll/ll.h"
#include "proj_lib/sig_mesh/app_mesh.h"
#include "vendor/Rang_Dong/RD_Light_Sensor.h"
#include "vendor/Rang_Dong/RD_type_device.h"
#include "vendor/Rang_Dong/RD_control.h"

#include "vendor/Rang_Dong/SoftUart.h"
#include "vendor/Rang_Dong/RD_pir_DC_Control.h"
#include "vendor/Rang_Dong/RD_flash.h"
#include "proj/drivers/uart.h"


unsigned int rdTimeLine = 0;



extern void user_init();
extern void main_loop ();
void blc_pm_select_none();
#if (HCI_ACCESS==HCI_USE_UART)
extern my_fifo_t hci_rx_fifo;
extern unsigned int clock_time_read;
u16 uart_tx_irq=0, uart_rx_irq=0;



unsigned int unFreezeTime = 0;
unsigned char checkFreeze = 0;
unsigned long sleepUnprov = 0;
unsigned char sendLuxFlag  = 0;
unsigned char btnStateRead = 0;
unsigned int workingTime  = 0;

extern unsigned char jointState;
extern unsigned char confirmState;
extern u16 rd_node_addr;
extern u16 rd_box_addr;
extern sUartTypeDef sUart1;
extern pirPinTypeDef rdPir;
extern unsigned char confirmDone;
extern unsigned long endConfigTime;
extern otaTypeDef rdOta;



_attribute_ram_code_ void irq_uart_handle()
{
	unsigned char irqS = reg_dma_rx_rdy0;
	if(irqS & FLD_DMA_CHN_UART_RX)	//rx
	{
		uart_rx_irq++;
		reg_dma_rx_rdy0 = FLD_DMA_CHN_UART_RX;
		u8* w = hci_rx_fifo.p + (hci_rx_fifo.wptr & (hci_rx_fifo.num-1)) * hci_rx_fifo.size;
		if(w[0]!=0)
		{
			my_fifo_next(&hci_rx_fifo);
			u8* p = hci_rx_fifo.p + (hci_rx_fifo.wptr & (hci_rx_fifo.num-1)) * hci_rx_fifo.size;
			reg_dma0_addr = (u16)((u32)p);
		}
	}

	if(irqS & FLD_DMA_CHN_UART_TX)	//tx
	{
		uart_tx_irq++;
		reg_dma_rx_rdy0 = FLD_DMA_CHN_UART_TX;
	}
}
#endif

#if IRQ_TIMER1_ENABLE
_attribute_ram_code_ void irq_timer_handle()
{
    u32 src = reg_irq_src;
    static u32 A_debug_irq_cnt =0;
    if(src & FLD_IRQ_TMR1_EN){
       A_debug_irq_cnt++;
       reg_tmr_sta = FLD_TMR_STA_TMR1;
       gpio_write(GPIO_PA1,A_debug_irq_cnt%2);
    }
}
#endif

#if	IRQ_GPIO_ENABLE
static u32 irq_gpio_user_cnt=0;
void gpio_irq_user_handle()
{
	irq_gpio_user_cnt++;
	return;
}

void gpio_risc0_user_handle()
{
	return;
}

void gpio_risc1_user_handle()
{
	return;
}

void gpio_risc2_user_handle()
{
	return;
}

void irq_gpio_handle()
{
	u32 src = reg_irq_src;
	if(src & FLD_IRQ_GPIO_EN){
		gpio_irq_user_handle();
		reg_irq_src = FLD_IRQ_GPIO_EN;        // clear irq_gpio irq flag		
	}

	/************* gpio irq risc0 *************/
	if(src & FLD_IRQ_GPIO_RISC0_EN){
		gpio_risc0_user_handle();
		reg_irq_src = FLD_IRQ_GPIO_RISC0_EN;        // clear irq_gpio irq flag				
	}

	/************* gpio irq risc1 *************/
	if(src & FLD_IRQ_GPIO_RISC1_EN){
		gpio_risc1_user_handle();
		reg_irq_src = FLD_IRQ_GPIO_RISC1_EN;        // clear irq_gpio irq flag		
	}
	#if (!(__TL_LIB_8258__ || (MCU_CORE_TYPE && MCU_CORE_TYPE == MCU_CORE_8258) || (MCU_CORE_TYPE == MCU_CORE_8278)))
	if(src & FLD_IRQ_GPIO_RISC2_EN){
		gpio_risc2_user_handle();
		reg_irq_src = FLD_IRQ_GPIO_RISC2_EN;
	}
	#endif
}
#endif

_attribute_ram_code_ void irq_handler(void)
{
	#if DUAL_MODE_ADAPT_EN
	if(rf_mode == RF_MODE_ZIGBEE){
		irq_zigbee_sdk_handler();
	}else
	#endif
	{
		irq_blt_sdk_handler ();  //ble irq proc
	}

#if IRQ_TIMER1_ENABLE
	irq_timer_handle();
#endif

#if (HCI_ACCESS==HCI_USE_UART)
	irq_uart_handle();
#endif

#if	IRQ_GPIO_ENABLE
	irq_gpio_handle();
#endif
}

FLASH_ADDRESS_DEFINE;
#if(MCU_CORE_TYPE == MCU_CORE_8269)
int main (void) {
	FLASH_ADDRESS_CONFIG;
	cpu_wakeup_init();

	clock_init();
	set_tick_per_us(CLOCK_SYS_CLOCK_HZ/1000000);

	gpio_init();

	rf_drv_init(CRYSTAL_TYPE);

	user_init ();

    irq_enable();

	while (1) {
#if (MODULE_WATCHDOG_ENABLE)
		wd_clear(); //clear watch dog
#endif
		main_loop ();
	}
}
#elif((MCU_CORE_TYPE == MCU_CORE_8258) || (MCU_CORE_TYPE == MCU_CORE_8278))
_attribute_ram_code_ int main (void)    //must run in ramcode
{
	FLASH_ADDRESS_CONFIG;
#if (PINGPONG_OTA_DISABLE && (0 == FW_START_BY_BOOTLOADER_EN))
    ota_fw_check_over_write();  // must at first for main_
#endif

#if SLEEP_FUNCTION_DISABLE
    blc_pm_select_none();
#else
	blc_pm_select_internal_32k_crystal();
#endif
#if(MCU_CORE_TYPE == MCU_CORE_8258)
	cpu_wakeup_init();
#elif(MCU_CORE_TYPE == MCU_CORE_8278)
	cpu_wakeup_init(LDO_MODE,EXTERNAL_XTAL_24M);
#endif

	int deepRetWakeUp = pm_is_MCU_deepRetentionWakeup();  //MCU deep retention wakeUp
	rf_drv_init(RF_MODE_BLE_1M);
	gpio_init( !deepRetWakeUp );  //analog resistance will keep available in deepSleep mode, so no need initialize again
	pirInit();
	rdPir.state.pirHighState = gpio_read(PIR_DC_HIGH);
	rdPir.state.pirLowState = gpio_read(PIR_DC_LOW);
	btnStateRead = gpio_read(BUTTON_PIN);

	//LogInit();
	#if (CLOCK_SYS_CLOCK_HZ == 16000000)
		clock_init(SYS_CLK_16M_Crystal);
	#elif (CLOCK_SYS_CLOCK_HZ == 24000000)
		clock_init(SYS_CLK_24M_Crystal);
	#elif (CLOCK_SYS_CLOCK_HZ == 32000000)
		clock_init(SYS_CLK_32M_Crystal);
	#elif (CLOCK_SYS_CLOCK_HZ == 48000000)
		clock_init(SYS_CLK_48M_Crystal);
	#endif

	#if	(PM_DEEPSLEEP_RETENTION_ENABLE)
			if( pm_is_MCU_deepRetentionWakeup() ){
				user_init_deepRetn ();
			}
			else
	#endif

	// RD_EDIT: Tac dung de chi khoi tao lan dau.
	if(jointState != JOIN_DONE && jointState != JOIN_FAIL){
		rdFlashReadData(&jointState,&confirmState,&rd_node_addr,&rd_box_addr);
		if(jointState != JOIN_DONE){
			jointState = JOIN_FAIL;
		}
	}
	//sUartInit(&sUart1);
	rd_factory_reset(8,btnStateRead,jointState);
	if(jointState == JOIN_DONE){
		pirDetect_poll_edge(&rdPir);
		if(!sendLuxFlag){
			sendLuxFlag = LuxDetect(30);
		}
		else {
			LuxDetect(30);
		}
		if(!rdPir.flag.motionFlag && !sendLuxFlag){
			rd_deep_sleep_setup();
			cpu_sleep_wakeup(DEEPSLEEP_MODE_RET_SRAM_LOW32K, PM_WAKEUP_TIMER|PM_WAKEUP_PAD,clock_time() + 180000*CLOCK_SYS_CLOCK_1MS);
			start_reboot();
		}
	}
	user_init();
    irq_enable();
	if(jointState == JOIN_DONE ){
		if( confirmState == CONFIRM_DONE){
				sensorSendData(rdPir.flag.motionFlag,sendLuxFlag);
				workingTime = clock_time_ms();
				if(sendLuxFlag){
					rdTimeLine = clock_time_ms();
				}
		}
		else {
			kick_out();
		}
	}

	sleepUnprov = clock_time_ms();
	//unFreezeTime = clock_time_ms();
	led_init();
	while (1) {
		ledShowProvState();
		wd_clear(); //clear watch dog

		if(sendLuxFlag){
			rdTimeLine = 180;
		}
		if(rdPir.flag.motionFlag){
			rdTimeLine = 120;
		}

		//rdTimeLine = 160;

		if((rdPir.flag.motionFlag || sendLuxFlag ) && is_clock_time_done(workingTime,rdTimeLine)){
			workingTime = clock_time_ms();
			if(rdPir.flag.motionFlag) {
				rdPir.flag.motionFlag = 0;
				cpu_set_gpio_wakeup (BUTTON_PIN, 0, 1);
				gpio_setup_up_down_resistor(BUTTON_PIN,PM_PIN_PULLUP_1M);
				gpio_core_wakeup_enable_all (1);

				cpu_sleep_wakeup(DEEPSLEEP_MODE_RET_SRAM_LOW32K, PM_WAKEUP_TIMER | PM_WAKEUP_PAD,clock_time() + 20000*CLOCK_SYS_CLOCK_1MS);
				start_reboot();
			}
			else if(sendLuxFlag){
				sendLuxFlag = 0;
				rd_deep_sleep_setup();
				cpu_sleep_wakeup(DEEPSLEEP_MODE_RET_SRAM_LOW32K, PM_WAKEUP_TIMER,clock_time() + 1000*CLOCK_SYS_CLOCK_1MS);
				start_reboot();
			}
		}

		if(!rdOta.otaFlag && (confirmDone &&  is_clock_time_done(endConfigTime,1000))){
			confirmDone = 0;
			if(confirmState != CONFIRM_DONE){
				confirmState = CONFIRM_DONE;
				rdFlashWriteData(confirmType,&confirmState);
				for(unsigned char i=0;i<100;i++){
					main_loop();
				}
				start_reboot();
				sleepAfterSend();
			}
		}

		if((confirmState == CONFIRM_DONE) && is_clock_time_done(unFreezeTime, 500)){
			unFreezeTime = clock_time_ms();
			cpu_set_gpio_wakeup (BUTTON_PIN, 0, 1);
			gpio_setup_up_down_resistor(BUTTON_PIN,PM_PIN_PULLUP_1M);
			gpio_core_wakeup_enable_all (1);
			cpu_sleep_wakeup(DEEPSLEEP_MODE_RET_SRAM_LOW32K, PM_WAKEUP_TIMER | PM_WAKEUP_PAD,clock_time() + 5000*CLOCK_SYS_CLOCK_1MS);
			start_reboot();
		}

		main_loop();
		if(!is_provision_success() && (clock_time_ms() - sleepUnprov >= 60000)){
			sleepMode();
		}

		if(confirmState != CONFIRM_DONE){
			rd_get_provision_state();
		}

		Join_confirm_check();
		confirm_receive_check();
		otaConcol(&rdOta);
	}
}
#endif
