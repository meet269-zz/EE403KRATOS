################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/fsl_clock.c \
../drivers/fsl_common.c \
../drivers/fsl_dma.c \
../drivers/fsl_emc.c \
../drivers/fsl_flexcomm.c \
../drivers/fsl_gpio.c \
../drivers/fsl_i2c.c \
../drivers/fsl_i2c_dma.c \
../drivers/fsl_i2s.c \
../drivers/fsl_i2s_dma.c \
../drivers/fsl_inputmux.c \
../drivers/fsl_power.c \
../drivers/fsl_reset.c \
../drivers/fsl_usart.c \
../drivers/fsl_usart_dma.c 

OBJS += \
./drivers/fsl_clock.o \
./drivers/fsl_common.o \
./drivers/fsl_dma.o \
./drivers/fsl_emc.o \
./drivers/fsl_flexcomm.o \
./drivers/fsl_gpio.o \
./drivers/fsl_i2c.o \
./drivers/fsl_i2c_dma.o \
./drivers/fsl_i2s.o \
./drivers/fsl_i2s_dma.o \
./drivers/fsl_inputmux.o \
./drivers/fsl_power.o \
./drivers/fsl_reset.o \
./drivers/fsl_usart.o \
./drivers/fsl_usart_dma.o 

C_DEPS += \
./drivers/fsl_clock.d \
./drivers/fsl_common.d \
./drivers/fsl_dma.d \
./drivers/fsl_emc.d \
./drivers/fsl_flexcomm.d \
./drivers/fsl_gpio.d \
./drivers/fsl_i2c.d \
./drivers/fsl_i2c_dma.d \
./drivers/fsl_i2s.d \
./drivers/fsl_i2s_dma.d \
./drivers/fsl_inputmux.d \
./drivers/fsl_power.d \
./drivers/fsl_reset.d \
./drivers/fsl_usart.d \
./drivers/fsl_usart_dma.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/%.o: ../drivers/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -DCR_INTEGER_PRINTF -DDEBUG -D__USE_CMSIS -DCPU_LPC54608J512ET180=1 -DSDK_DEBUGCONSOLE=0 -D__MCUXPRESSO -DCPU_LPC54608J512ET180_cm4 -DCPU_LPC54608J512ET180 -D__REDLIB__ -I"C:\Users\Meet - HP\Documents\MCUXpressoIDE_10.0.0_344\workspace\lpcxpresso54608_driver_examples_i2s_dma_transfer\source" -I"C:\Users\Meet - HP\Documents\MCUXpressoIDE_10.0.0_344\workspace\lpcxpresso54608_driver_examples_i2s_dma_transfer" -I"C:\Users\Meet - HP\Documents\MCUXpressoIDE_10.0.0_344\workspace\lpcxpresso54608_driver_examples_i2s_dma_transfer\drivers" -I"C:\Users\Meet - HP\Documents\MCUXpressoIDE_10.0.0_344\workspace\lpcxpresso54608_driver_examples_i2s_dma_transfer\startup" -I"C:\Users\Meet - HP\Documents\MCUXpressoIDE_10.0.0_344\workspace\lpcxpresso54608_driver_examples_i2s_dma_transfer\utilities" -I"C:\Users\Meet - HP\Documents\MCUXpressoIDE_10.0.0_344\workspace\lpcxpresso54608_driver_examples_i2s_dma_transfer\board" -I"C:\Users\Meet - HP\Documents\MCUXpressoIDE_10.0.0_344\workspace\lpcxpresso54608_driver_examples_i2s_dma_transfer\codec" -I"C:\Users\Meet - HP\Documents\MCUXpressoIDE_10.0.0_344\workspace\lpcxpresso54608_driver_examples_i2s_dma_transfer\CMSIS" -O0 -fno-common -g -Wall -c  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


