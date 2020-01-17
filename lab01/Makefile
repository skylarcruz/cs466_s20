PART=TM4C123GH6PM

TivaDriverlib=../../TivaDriver

TARGET=blink

SIZE=arm-none-eabi-size

FLASHOPTS=-v
FLASH=lm4flash
CP=/bin/cp

#
# Echo full command lines.. Comment out for terse
VERBOSE=1
DEBUG=1

#
# Include the common make definitions.
#
include makedefs

CFLAGS += -Wno-unused-value  
#CFLAGS += -save-temps  

LDFLAGS += --stats

#
# Cheasy makefile way to create a target directory
#
ODIR = ${COMPILER}
dummy_build_folder := $(shell mkdir -p ${COMPILER})


#
# Where to find header files that do not live in the source directory.
#
IPATH += .
IPATH += ${TivaDriverlib}

#
# The default rule, which causes the FreeRTOS example to be built.
#
all: ${COMPILER}
all: ${COMPILER}/${TARGET}.axf

#
# The rule to clean out all the build products.
#
clean:
	@rm -rf ${COMPILER} ${wildcard *~}

#
# Rules for building the application
#
${COMPILER}/${TARGET}.axf: ${COMPILER}/blinky.o
${COMPILER}/${TARGET}.axf: ${COMPILER}/startup_${COMPILER}.o

#
# load map and device memory description
#
${COMPILER}/${TARGET}.axf: ti_tm4c123g.ld

SCATTERgcc_${TARGET}=ti_tm4c123g.ld
ENTRY_${TARGET}=ResetISR
CFLAGSgcc=-DTARGET_IS_BLIZZARD_RB1

#
# Include the automatically generated dependency files.
#
ifneq (${MAKECMDGOALS},clean)
-include ${wildcard ${COMPILER}/*.d} __dummy__
endif

#
# A rule to flash and restarrt the program
#
flash: ${COMPILER}/${TARGET}.axf
	${SIZE} ${COMPILER}/${TARGET}.axf
	${FLASH} ${FLASHOPTS} ${COMPILER}/${TARGET}.bin
