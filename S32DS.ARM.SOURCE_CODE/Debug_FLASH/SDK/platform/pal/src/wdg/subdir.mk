################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/NXP/S32DS_ARM_v2.2/S32DS/software/S32SDK_S32K1xx_RTM_3.0.0/platform/pal/src/wdg/wdg_pal.c 

OBJS += \
./SDK/platform/pal/src/wdg/wdg_pal.o 

C_DEPS += \
./SDK/platform/pal/src/wdg/wdg_pal.d 


# Each subdirectory must supply rules for building sources it contributes
SDK/platform/pal/src/wdg/wdg_pal.o: C:/NXP/S32DS_ARM_v2.2/S32DS/software/S32SDK_S32K1xx_RTM_3.0.0/platform/pal/src/wdg/wdg_pal.c
	@echo 'Building file: $<'
	@echo 'Invoking: Standard S32DS C Compiler'
	arm-none-eabi-gcc "@SDK/platform/pal/src/wdg/wdg_pal.args" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


