################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
Satellitt\ Design\ ADCS.cpp: ../Satellitt\ Design\ ADCS.ino
	@echo 'Building file: "$<"'
	@echo 'Invoking: Resource Custom Build Step'
	
	@echo 'Finished building: "$<"'
	@echo ' '

Satellitt\ Design\ ADCS.o: ./Satellitt\ Design\ ADCS.cpp $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: GNU Compiler'
	"/home/sew/Documents/energia-1.8.10E23/hardware/tools/msp430/bin/msp430-gcc-4.6.3" -c -mmcu=msp430f5529 -DF_CPU=25000000L -DENERGIA_MSP_EXP430F5529LP -DENERGIA_ARCH_MSP430 -DENERGIA=23 -DARDUINO=10610 -I"/home/sew/Documents/energia-1.8.10E23/hardware/energia/msp430/cores/msp430" -I"/home/sew/Documents/energia-1.8.10E23/hardware/energia/msp430/variants/MSP-EXP430F5529LP" -I"/home/sew/workspace_v10/Satellitt Design ADCS" -I"/home/sew/Documents/energia-1.8.10E23/hardware/energia/msp430/libraries/Wire" -I"/home/sew/Documents/energia-1.8.10E23/hardware/tools/msp430/msp430/include" -Os -ffunction-sections -fdata-sections -g -gdwarf-3 -gstrict-dwarf -w -Wall -MMD -MP -MF"Satellitt Design ADCS.d_raw" -MT"Satellitt\ Design\ ADCS.o"   -fno-threadsafe-statics -fno-exceptions $(GEN_OPTS__FLAG) -o"$@" "$(shell echo $<)"
	@echo 'Finished building: "$<"'
	@echo ' '


