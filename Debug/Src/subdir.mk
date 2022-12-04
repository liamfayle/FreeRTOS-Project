################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/arducam.c \
../Src/camera.c \
../Src/dma.c \
../Src/dma2d.c \
../Src/fmc.c \
../Src/freertos.c \
../Src/gpio.c \
../Src/i2c.c \
../Src/ltdc.c \
../Src/main.c \
../Src/ov5642_registers.c \
../Src/picojpeg.c \
../Src/spi.c \
../Src/stb_image.c \
../Src/stm32f4xx_hal_msp.c \
../Src/stm32f4xx_hal_timebase_TIM.c \
../Src/stm32f4xx_it.c \
../Src/system_stm32f4xx.c \
../Src/usb_device.c \
../Src/usbd_cdc_if.c \
../Src/usbd_conf.c \
../Src/usbd_desc.c 

OBJS += \
./Src/arducam.o \
./Src/camera.o \
./Src/dma.o \
./Src/dma2d.o \
./Src/fmc.o \
./Src/freertos.o \
./Src/gpio.o \
./Src/i2c.o \
./Src/ltdc.o \
./Src/main.o \
./Src/ov5642_registers.o \
./Src/picojpeg.o \
./Src/spi.o \
./Src/stb_image.o \
./Src/stm32f4xx_hal_msp.o \
./Src/stm32f4xx_hal_timebase_TIM.o \
./Src/stm32f4xx_it.o \
./Src/system_stm32f4xx.o \
./Src/usb_device.o \
./Src/usbd_cdc_if.o \
./Src/usbd_conf.o \
./Src/usbd_desc.o 

C_DEPS += \
./Src/arducam.d \
./Src/camera.d \
./Src/dma.d \
./Src/dma2d.d \
./Src/fmc.d \
./Src/freertos.d \
./Src/gpio.d \
./Src/i2c.d \
./Src/ltdc.d \
./Src/main.d \
./Src/ov5642_registers.d \
./Src/picojpeg.d \
./Src/spi.d \
./Src/stb_image.d \
./Src/stm32f4xx_hal_msp.d \
./Src/stm32f4xx_hal_timebase_TIM.d \
./Src/stm32f4xx_it.d \
./Src/system_stm32f4xx.d \
./Src/usb_device.d \
./Src/usbd_cdc_if.d \
./Src/usbd_conf.d \
./Src/usbd_desc.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc -c "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F429xx -c -I../Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/BSP/STM32F429I-Discovery -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/arducam.d ./Src/arducam.o ./Src/arducam.su ./Src/camera.d ./Src/camera.o ./Src/camera.su ./Src/dma.d ./Src/dma.o ./Src/dma.su ./Src/dma2d.d ./Src/dma2d.o ./Src/dma2d.su ./Src/fmc.d ./Src/fmc.o ./Src/fmc.su ./Src/freertos.d ./Src/freertos.o ./Src/freertos.su ./Src/gpio.d ./Src/gpio.o ./Src/gpio.su ./Src/i2c.d ./Src/i2c.o ./Src/i2c.su ./Src/ltdc.d ./Src/ltdc.o ./Src/ltdc.su ./Src/main.d ./Src/main.o ./Src/main.su ./Src/ov5642_registers.d ./Src/ov5642_registers.o ./Src/ov5642_registers.su ./Src/picojpeg.d ./Src/picojpeg.o ./Src/picojpeg.su ./Src/spi.d ./Src/spi.o ./Src/spi.su ./Src/stb_image.d ./Src/stb_image.o ./Src/stb_image.su ./Src/stm32f4xx_hal_msp.d ./Src/stm32f4xx_hal_msp.o ./Src/stm32f4xx_hal_msp.su ./Src/stm32f4xx_hal_timebase_TIM.d ./Src/stm32f4xx_hal_timebase_TIM.o ./Src/stm32f4xx_hal_timebase_TIM.su ./Src/stm32f4xx_it.d ./Src/stm32f4xx_it.o ./Src/stm32f4xx_it.su ./Src/system_stm32f4xx.d ./Src/system_stm32f4xx.o ./Src/system_stm32f4xx.su ./Src/usb_device.d ./Src/usb_device.o ./Src/usb_device.su ./Src/usbd_cdc_if.d ./Src/usbd_cdc_if.o ./Src/usbd_cdc_if.su ./Src/usbd_conf.d ./Src/usbd_conf.o ./Src/usbd_conf.su ./Src/usbd_desc.d ./Src/usbd_desc.o ./Src/usbd_desc.su

.PHONY: clean-Src

