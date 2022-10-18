/*
 * RD_flash.h
 *
 *  Created on: Apr 26, 2022
 *      Author: PC5
 */

#ifndef RD_FLASH_H_
#define RD_FLASH_H_
#include "proj/tl_common.h"
#include "vendor/mesh/app.h"
#include "vendor/mesh_lpn/app.h"
#include "vendor/mesh_provision/app.h"
#include "vendor/mesh_switch/app.h"
#include "vendor/common/sensors_model.h"
#include "proj_lib/mesh_crypto/sha256_telink.h"
#include "vendor/common/app_heartbeat.h"


enum {
	confirmType = 0,
	nodeAddrType,
	boxAddrType,
};


#define JOIN_DONE			0x55
#define JOIN_FAIL			0xfe
#define CONFIRM_DONE		0xaa

#define RD_FLASH_ADDR			0x78000
#define FLASH_LENGHT			6




void rd_flash_read_data(unsigned long addr, u16 *out);
unsigned char  flashReadJoinFlag();
void flashWriteJoinFlag(unsigned char data);




void rdFlashWriteData(unsigned char type,unsigned char *data);
void rdFlashReadData(unsigned char *jointStatus, unsigned char *confirmStatus, unsigned short *nodeAddr, unsigned short *boxAddr);

#endif /* RD_FLASH_H_ */
