################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../proj/app/usbaud.c \
../proj/app/usbcdc.c \
../proj/app/usbkb.c \
../proj/app/usbmouse.c 

OBJS += \
./proj/app/usbaud.o \
./proj/app/usbcdc.o \
./proj/app/usbkb.o \
./proj/app/usbmouse.o 


# Each subdirectory must supply rules for building sources it contributes
proj/app/%.o: ../proj/app/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: TC32 Compiler'
	tc32-elf-gcc -ffunction-sections -fdata-sections -I"E:\D\Rang_Dong_tren_cty\CODE_Telink_BLE_FPT\FW_gui_TT_Digital\pir_DC\pir_DC_OTA_SRAM\firmware" -I"E:\D\Rang_Dong_tren_cty\CODE_Telink_BLE_FPT\FW_gui_TT_Digital\pir_DC\pir_DC_OTA_SRAM\firmware\vendor\common\mi_api\libs" -I"E:\D\Rang_Dong_tren_cty\CODE_Telink_BLE_FPT\FW_gui_TT_Digital\pir_DC\pir_DC_OTA_SRAM\firmware\vendor\common\mi_api\mijia_ble_api" -D__PROJECT_MESH__=1 -DCHIP_TYPE=CHIP_TYPE_8258 -Wall -O2 -fpack-struct -fshort-enums -finline-small-functions -std=gnu99 -fshort-wchar -fms-extensions -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


