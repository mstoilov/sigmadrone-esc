#!/bin/sh -x

STM32CUBE=$1
MCU=$2


echo ${STM32CUBE}
echo ${MCU}

rm -rf system/src/stm32f4-hal/*
rm -rf system/inc/stm32f4-hal/*

cp -Ra ${STM32CUBE}/Drivers/STM32F4xx_HAL_Driver/Src/* system/src/stm32f4-hal/
cp -Ra ${STM32CUBE}/Drivers/STM32F4xx_HAL_Driver/Inc/* system/inc/stm32f4-hal/

# Update CMSIS .h
cp -Ra ${STM32CUBE}/Drivers/CMSIS/Device/ST/STM32F4xx/Include/system_stm32f4xx.h system/inc/cmsis/system_stm32f4xx.h
cp -Ra ${STM32CUBE}/Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f${MCU}[a-x][a-x].h system/inc/cmsis/
cp -Ra ${STM32CUBE}/Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f4xx.h system/inc/cmsis/
cp -Ra ${STM32CUBE}/Drivers/CMSIS/Include/* system/inc/cmsis/

# Update CMSIS .c
cp -Ra ${STM32CUBE}/Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c system/src/cmsis/

#update stm32_assert.h
cp -Ra ${STM32CUBE}/Projects/STM32F411RE-Nucleo/Templates_LL/Inc/stm32_assert.h inc/
