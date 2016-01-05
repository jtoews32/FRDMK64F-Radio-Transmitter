################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Sources/Events.c \
../Sources/Packet.c \
../Sources/main.c 

OBJS += \
./Sources/Events.o \
./Sources/Packet.o \
./Sources/main.o 

C_DEPS += \
./Sources/Events.d \
./Sources/Packet.d \
./Sources/main.d 


# Each subdirectory must supply rules for building sources it contributes
Sources/%.o: ../Sources/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -I"/home/jtoews/Desktop/keplercpp/workspace_keplercpp/FRDMK64F-Radio-Transmitter/Static_Code/Peripherals" -I"/home/jtoews/Desktop/keplercpp/workspace_keplercpp/FRDMK64F-Radio-Transmitter/Static_Code/System" -I"/home/jtoews/Desktop/keplercpp/workspace_keplercpp/FRDMK64F-Radio-Transmitter/Static_Code/PDD" -I"/home/jtoews/Desktop/keplercpp/workspace_keplercpp/FRDMK64F-Radio-Transmitter/Static_Code/IO_Map" -I"/home/jtoews/Desktop/keplercpp/ProcessorExpert/lib/Kinetis/pdd/inc" -I"/home/jtoews/Desktop/keplercpp/workspace_keplercpp/FRDMK64F-Radio-Transmitter/Sources" -I"/home/jtoews/Desktop/keplercpp/workspace_keplercpp/FRDMK64F-Radio-Transmitter/Generated_Code" -I"/home/jtoews/Desktop/keplercpp/workspace_keplercpp/FRDMK64F-Radio-Transmitter/Static_Code\Peripherals" -I"/home/jtoews/Desktop/keplercpp/workspace_keplercpp/FRDMK64F-Radio-Transmitter/Static_Code\System" -I"/home/jtoews/Desktop/keplercpp/workspace_keplercpp/FRDMK64F-Radio-Transmitter/Static_Code\PDD" -I"/home/jtoews/Desktop/keplercpp/workspace_keplercpp/FRDMK64F-Radio-Transmitter/Static_Code\IO_Map" -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


