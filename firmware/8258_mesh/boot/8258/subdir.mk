################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_UPPER_SRCS += \
../boot/8258/cstartup_8258.S \
../boot/8258/cstartup_8258_RET_16K.S \
../boot/8258/cstartup_8258_RET_32K.S 

OBJS += \
./boot/8258/cstartup_8258.o \
./boot/8258/cstartup_8258_RET_16K.o \
./boot/8258/cstartup_8258_RET_32K.o 


# Each subdirectory must supply rules for building sources it contributes
boot/8258/%.o: ../boot/8258/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: TC32 CC/Assembler'
	tc32-elf-gcc -Xassembler"E:\D\Rang_Dong_tren_cty\CODE_Telink_BLE_FPT\FW_gui_TT_Digital\pir_DC\pir_DC_OTA_SRAM\firmware" -DMCU_STARTUP_8258_RET_16K -D__PROJECT_MESH__=1 -DCHIP_TYPE=CHIP_TYPE_8258 -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


