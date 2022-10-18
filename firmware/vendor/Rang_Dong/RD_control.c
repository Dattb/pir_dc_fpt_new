/*
 * RD_control.c
 *
 *  Created on: Mar 12, 2022
 *      Author: PC5
 */

#include "RD_control.h"
#include "RD_pir_DC_Control.h"
#include "SoftUart.h"
#include "RD_Light_Sensor.h"
#include "RD_flash.h"


extern u16 rd_node_addr;
extern u16 rd_box_addr;
extern pirPinTypeDef rdPir;
unsigned int btn_cnt = 0;
unsigned char btn_status = 1;
unsigned char factory_reset_cnt = 0;
unsigned char btn_flag_old = 1,btn_flag_new = 1;

unsigned int power_cnt_reset = 0;
unsigned char check_prov_status_old = 0;
extern pirPinTypeDef rdPir;

otaTypeDef rdOta;

unsigned int resetCntTime = 0;
unsigned char rd_factory_reset(unsigned int cnt, unsigned char btnState,unsigned char joinState){
		if(!btnState){
			led_init();
			if(joinState == JOIN_DONE){
				gpio_write(LED_BLUE_PIN,0);
				gpio_write(LED_RED_PIN,1);
			}
			else{
				gpio_write(LED_BLUE_PIN,1);
				gpio_write(LED_RED_PIN,0);
			}
			if(SYS_32K_TICK_S - resetCntTime >= 3){
				factory_reset_cnt = 0;
			}
			else if(resetCntTime > SYS_32K_TICK_S){
				resetCntTime = SYS_32K_TICK_S;
			}
			resetCntTime = SYS_32K_TICK_S;
			factory_reset_cnt++;

			if(factory_reset_cnt >= cnt){
				factory_reset_cnt = 0;
				kick_out();
			}
			sleep_ms(10);
			unsigned char otaFlagOld = 0;
			while(!gpio_read(BUTTON_PIN)){
				wd_clear();
				if(factory_reset_cnt == 5 || factory_reset_cnt == 6){
					if(joinState == JOIN_DONE){
						 if(otaDetect()){
							 rdOta.otaFlag = 1;
							 rdOta.otaTime = clock_time_ms();
						 }
						 if(otaFlagOld != rdOta.otaFlag){
							 otaFlagOld = rdOta.otaFlag;
							 if(otaFlagOld){
								 RD_light_ota_with_sleep(8,100*1000);
							 }
						 }
					}
				}
			}
			cpu_set_gpio_wakeup(PIR_DC_HIGH, 1, 0);     // level : 1 (high); 0 (low)
			cpu_set_gpio_wakeup(PIR_DC_LOW,1, 0);     // level : 1 (high); 0 (low)
			cpu_set_gpio_wakeup(BUTTON_PIN,0,1);
			gpio_setup_up_down_resistor(BUTTON_PIN,PM_PIN_PULLUP_1M);
			gpio_core_wakeup_enable_all (1);
			if(!rdOta.otaFlag){
				if(joinState == JOIN_DONE)
					cpu_sleep_wakeup(DEEPSLEEP_MODE_RET_SRAM_LOW32K, PM_WAKEUP_TIMER|PM_WAKEUP_PAD,clock_time() + 5000*CLOCK_SYS_CLOCK_1MS);
				else
					cpu_sleep_wakeup(DEEPSLEEP_MODE_RET_SRAM_LOW32K,PM_WAKEUP_PAD|PM_WAKEUP_TIMER,clock_time() + 180000*CLOCK_SYS_CLOCK_1MS);
			}
		}
	return 0;
}


unsigned char btn_detect(unsigned char btn_threshold){
	 if (!gpio_read(BUTTON_PIN)){
		 btn_cnt++;
		 if(btn_cnt > btn_threshold){
			 btn_status = 0;
		 }
	 }
	 else {
		 btn_cnt = 0;
		 btn_status = 1;
	 }
	 return btn_status;
}



void led_init(){
	gpio_set_func(LED_RED_PIN, AS_GPIO);
	gpio_set_output_en(LED_RED_PIN, 1);
	gpio_set_input_en(LED_RED_PIN, 0);
	gpio_write(LED_RED_PIN, 1);

	gpio_set_func(LED_BLUE_PIN, AS_GPIO);
	gpio_set_output_en(LED_BLUE_PIN, 1);
	gpio_set_input_en(LED_BLUE_PIN, 0);
	gpio_write(LED_BLUE_PIN, 0);
}


void btn_init(){
	gpio_set_func(BUTTON_PIN, AS_GPIO);
	gpio_set_output_en(BUTTON_PIN,0);
	gpio_set_input_en(BUTTON_PIN,1);
	gpio_setup_up_down_resistor(BUTTON_PIN,PM_PIN_PULLUP_10K);
}

unsigned long addr[5] = {0x7f000,0x7e000,0x7d000,0x7c000,0x7b000};
unsigned long flash_addr_power = 0x7f000;

void flash_test_data(unsigned long addr, unsigned long data){
	unsigned char test_buff[4] = {0};
	test_buff[0] = data>>16;
	test_buff[1] = data>>8;
	test_buff[2] = data;
	flash_erase_sector(addr);
	flash_write_page(flash_addr_power,4,test_buff);
}



void led_show_provision_success(unsigned char cycle, unsigned int cnt,unsigned char *flag){
	static unsigned int loop_cnt = 0;
	static unsigned int effect_cycle = 0;
	if(*flag){
		if(effect_cycle < cycle){
			loop_cnt++;
			if(loop_cnt < cnt){
				gpio_write(LED_BLUE_PIN,0);
				gpio_write(LED_RED_PIN,0);
			}
			else if(loop_cnt < cnt){
				gpio_write(LED_BLUE_PIN,0);
				gpio_write(LED_RED_PIN,0);
			}
			else{
				effect_cycle ++;
				loop_cnt = 0;
			}
		}
		else {
			*flag = 0;
		}
	}
}

void pirInit(){
	gpio_set_input_en(PIR_DC_HIGH,1);
	gpio_setup_up_down_resistor(PIR_DC_HIGH,PM_PIN_UP_DOWN_FLOAT);
	gpio_set_input_en(PIR_DC_LOW,1);
	gpio_setup_up_down_resistor(PIR_DC_LOW,PM_PIN_UP_DOWN_FLOAT);
	gpio_set_input_en(BUTTON_PIN,1);
	//gpio_setup_up_down_resistor(BUTTON_PIN,PM_PIN_UP_DOWN_FLOAT);
}



void LogInit(){
	gpio_set_input_en(LOG_ENABLE_PIN,1);
	gpio_setup_up_down_resistor(LOG_ENABLE_PIN,PM_PIN_PULLUP_1M);
}

void rd_get_provision_state(){
	if(check_prov_status_old != is_provision_success()){
		check_prov_status_old = is_provision_success();
		RD_light_prov_success_with_sleep(5,200*1000);
		//RD_light_ev_with_sleep(10, 100*1000);	//1Hz shine for  2.5 second
	}
}

void sleepBeforSend(unsigned char pirMotionFlag, unsigned char luxFlag){
	if(!pirMotionFlag && !luxFlag) {
		rdPrintf("sleep_1\n");
		cpu_set_gpio_wakeup (PIR_DC_HIGH, 1, 1);     // level : 1 (high); 0 (low)
		cpu_set_gpio_wakeup (PIR_DC_LOW,1, 1);     // level : 1 (high); 0 (low)
		if(gpio_read(PIR_DC_HIGH)) cpu_set_gpio_wakeup (PIR_DC_HIGH, 1, 0);     // level : 1 (high); 0 (low)
		if(gpio_read(PIR_DC_LOW)) cpu_set_gpio_wakeup (PIR_DC_LOW, 1, 0);     // level : 1 (high); 0 (low)
		cpu_set_gpio_wakeup (BUTTON_PIN, 0, 1);
		gpio_setup_up_down_resistor(BUTTON_PIN,PM_PIN_PULLUP_1M);
		gpio_core_wakeup_enable_all (1);
		cpu_sleep_wakeup(DEEPSLEEP_MODE_RET_SRAM_LOW32K, PM_WAKEUP_TIMER|PM_WAKEUP_PAD,clock_time() + 120000*CLOCK_SYS_CLOCK_1MS);
	}
	else {
		cpu_sleep_wakeup(DEEPSLEEP_MODE_RET_SRAM_LOW32K, PM_WAKEUP_TIMER,clock_time() + 5000*CLOCK_SYS_CLOCK_1MS);
	}
}

void sensorSendData(unsigned char pirMotionFlag, unsigned char luxSendFlag){
	unsigned char powerRead = powerUpdate();
	if(pirMotionFlag){
		led_init();
		main_loop();
		pirSendData(powerRead);
	}
	else {
		if(luxSendFlag){
			main_loop();
			lightSendData(powerRead);
		}
	}

}



unsigned char powerUpdate(){
	unsigned char powerPercent = RD_power_read();
	unsigned char powerSave = getPowerSaveReg() ;
	if((powerSave > powerPercent) && (powerSave - powerPercent) >= 5){
		setPowerSaveReg(powerPercent);
		return powerPercent;
	}
	else if((powerSave < powerPercent) && (powerPercent - powerSave) >= 5){
		if(powerPercent >100) powerPercent = 100;
		setPowerSaveReg(powerPercent);
		return powerPercent;
	}
	return powerSave;
}


//void lightSendData(unsigned short lux){
//	lux_send_buff[0] = 0x0d;
//	lux_send_buff[1] = lux;
//	lux_send_buff[2] = lux >>8;
//	lux_send_buff[3] = powerData;
//	mesh_tx_cmd2normal(0xe5,(u8 *) lux_send_buff,4, rd_node_addr ,rd_box_addr, 2);
//}

void pirSendData(unsigned char power){
	unsigned char pir_send_buff[3] = {0x0c,0x01,0x00};
	pir_send_buff[2] = power;
	mesh_tx_cmd2normal(0xe5,(u8 *) pir_send_buff, 3 , rd_node_addr , rd_box_addr , 2 );
	rdPrintf("S:%x-%x\n",rd_node_addr,rd_box_addr);
}

void sleepAfterSend(){
	if(!rdPir.flag.motionFlag){
		cpu_set_gpio_wakeup (PIR_DC_HIGH, 1, 1);     // level : 1 (high); 0 (low)
		cpu_set_gpio_wakeup (PIR_DC_LOW,1, 1);     // level : 1 (high); 0 (low)
		if(gpio_read(PIR_DC_HIGH)) cpu_set_gpio_wakeup (PIR_DC_HIGH, 1, 0);     // level : 1 (high); 0 (low)
		if(gpio_read(PIR_DC_LOW)) cpu_set_gpio_wakeup (PIR_DC_LOW, 1, 0);     // level : 1 (high); 0 (low)
	}
	cpu_set_gpio_wakeup (BUTTON_PIN, 0, 1);
	gpio_setup_up_down_resistor(BUTTON_PIN,PM_PIN_PULLUP_1M);
	gpio_core_wakeup_enable_all (1);
	cpu_sleep_wakeup(DEEPSLEEP_MODE_RET_SRAM_LOW32K,PM_WAKEUP_TIMER|PM_WAKEUP_PAD,clock_time() + 5000*CLOCK_SYS_CLOCK_1MS);
}



void sleepMode(){
	cpu_set_gpio_wakeup (BUTTON_PIN, 0, 1);
	gpio_setup_up_down_resistor(BUTTON_PIN,PM_PIN_PULLUP_1M);
	gpio_core_wakeup_enable_all (1);
	cpu_sleep_wakeup(DEEPSLEEP_MODE_RET_SRAM_LOW32K,PM_WAKEUP_PAD,clock_time() + 5000*CLOCK_SYS_CLOCK_1MS);
}


void ledTestInit(){
	gpio_set_func(LED_RED_PIN, AS_GPIO);
	gpio_set_output_en(LED_RED_PIN, 1);
	gpio_set_input_en(LED_RED_PIN, 0);
	gpio_write(LED_RED_PIN, 1);

	gpio_set_func(LED_BLUE_PIN, AS_GPIO);
	gpio_set_output_en(LED_BLUE_PIN, 1);
	gpio_set_input_en(LED_BLUE_PIN, 0);
	gpio_write(LED_BLUE_PIN, 0);
}


unsigned char otaDetect(){
	static unsigned char otaModeCnt = 0;
	sleep_ms(100);
	otaModeCnt++;
	if(otaModeCnt > 30){
		return 1;
	}
	return 0;
}

void otaConcol(otaTypeDef *ota){
	if(ota->otaFlag && (clock_time_ms() - ota->otaTime) >= 120000){
		ota->otaFlag = 0;
	}
}

/*
 * function put mcu in suspend mode to save power
 * param is time : ms
 */
void LP_Suspend(unsigned int time){
	cpu_sleep_wakeup(SUSPEND_MODE, PM_WAKEUP_TIMER|PM_WAKEUP_PAD,clock_time() + time*CLOCK_SYS_CLOCK_1MS);
}

void rd_deep_sleep_setup(){
	cpu_set_gpio_wakeup (PIR_DC_HIGH, 1, 1);     // level : 1 (high); 0 (low)
	cpu_set_gpio_wakeup (PIR_DC_HIGH, 0, 1);     // level : 1 (high); 0 (low)
	cpu_set_gpio_wakeup (PIR_DC_LOW,1, 1);     // level : 1 (high); 0 (low)
	cpu_set_gpio_wakeup (PIR_DC_LOW,0, 1);     // level : 1 (high); 0 (low)
	if(gpio_read(PIR_DC_HIGH)){
		cpu_set_gpio_wakeup (PIR_DC_HIGH, 1, 0);     // level : 1 (high); 0 (low)
		cpu_set_gpio_wakeup (PIR_DC_HIGH, 0, 1);     // level : 1 (high); 0 (low)
	}
	if(!gpio_read(PIR_DC_HIGH)) {
		cpu_set_gpio_wakeup (PIR_DC_HIGH, 0, 0);     // level : 1 (high); 0 (low)
		cpu_set_gpio_wakeup (PIR_DC_HIGH, 1, 1);     // level : 1 (high); 0 (low)
	}

	if(gpio_read(PIR_DC_LOW)){
		cpu_set_gpio_wakeup (PIR_DC_LOW,1, 0);     // level : 1 (high); 0 (low)
		cpu_set_gpio_wakeup (PIR_DC_LOW,0, 1);     // level : 1 (high); 0 (low)
	}
	if(!gpio_read(PIR_DC_LOW)){
		cpu_set_gpio_wakeup (PIR_DC_LOW,0, 0);     // level : 1 (high); 0 (low)
		cpu_set_gpio_wakeup (PIR_DC_LOW,1, 1);     // level : 1 (high); 0 (low)
	}
	cpu_set_gpio_wakeup (BUTTON_PIN, 0, 1);

	gpio_core_wakeup_enable_all (1);
}


int is_clock_time_done(unsigned int time,unsigned int period){
	if(clock_time_ms() - time >= period){
		return 1;
	}
	else if(clock_time_ms() < time){
		time = clock_time_ms();
		return 0;
	}
	return 0;
}
