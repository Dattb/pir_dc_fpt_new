/*
 * RD_flash.c
 *
 *  Created on: Apr 26, 2022
 *      Author: PC5
 */


#include "RD_flash.h"
#include "SoftUart.h"



unsigned char jointState = 0;
unsigned char confirmState = 0;
extern u16 rd_node_addr;
extern u16 rd_box_addr;


void rd_flash_read_data(unsigned long addr, u16 *out){
	u8 buff_read[2] = {0};
	flash_read_page(addr,2,buff_read);
	*out = buff_read[1]<<8|buff_read[0];
}



unsigned char  flashReadJoinFlag(){
	unsigned char flag = 0;
	flash_read_page(0x7f000,1,&flag);
	return flag;
}

void flashWriteJoinFlag(unsigned char data){
}






void rdFlashWriteData(unsigned char type,unsigned char *data){
	unsigned char readBuff[FLASH_LENGHT];
	flash_read_page(RD_FLASH_ADDR,FLASH_LENGHT,readBuff);

	if(type == confirmType){
		readBuff[1] = *data;
		confirmState = *data;
	}
	else if(type == nodeAddrType){
		readBuff[0] = JOIN_DONE;  // joined !
		jointState = readBuff[0];

		readBuff[2] = data[0];
		readBuff[3] = data[1];
		rd_node_addr = *(unsigned short *)&readBuff[2];
	}
	else if(type == boxAddrType){
		readBuff[4] = data[0];
		readBuff[5] = data[1];
		rd_box_addr = *(unsigned short *)&readBuff[4];
	}
	flash_erase_sector(RD_FLASH_ADDR);
	flash_write_page(RD_FLASH_ADDR,FLASH_LENGHT,readBuff);
}


void rdFlashReadData(unsigned char *jointStatus, unsigned char *confirmStatus, unsigned short *nodeAddr, unsigned short *boxAddr){
	unsigned char readBuff[FLASH_LENGHT];
	flash_read_page(RD_FLASH_ADDR,FLASH_LENGHT,readBuff);
	*jointStatus = readBuff[0];
	*confirmStatus = readBuff[1];
	*nodeAddr = *(unsigned short *)&readBuff[2];
	*boxAddr = *(unsigned short *)&readBuff[4];
//
//	sUartInit(&sUart1);
//	sleep_ms(2);
//	rdPrintf("join:%x - confirm:%x - node:%x - box:%x\n",jointState,confirmState,rd_node_addr,rd_box_addr);
}
