#
# stm32f4 Codec2 test programs
#
# CMake configuration contributed by Richard Shaw (KF5OIM)
# Please report questions, comments, problems, or patches to the freetel
# mailing list: https://lists.sourceforge.net/lists/listinfo/freetel-codec2
#
set(ARM_GCC_BIN "" CACHE STRING "Path to the bin directory of your arm-eabi-none-gcc (optional)")
project(stm32f4 C ASM)

if(CMAKE_CROSSCOMPILING)
    message(STATUS "We are cross compiling...")
else()
    message(STATUS "Performing standard host build...")
endif()

cmake_minimum_required(VERSION 2.8)

include(gencodebooks.cmake)
#include(codec2/stm32/cmake/gencodebooks.cmake)

#
# Prevent in-source builds
# If an in-source build is attempted, you will still need to clean up a few
# files manually.
#
set(CMAKE_DISABLE_SOURCE_CHANGES ON)
set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)
if("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
  message(FATAL_ERROR "In-source builds in ${CMAKE_BINARY_DIR} are not "
   "allowed, please remove ./CMakeCache.txt and ./CMakeFiles/, create a "
   "separate build directory and run cmake from there.")
endif("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")

#include(cmake/STM32_Toolchain.cmake)

###################################################

set(FLOAT_TYPE "hard" CACHE STRING "Floating point: defaults to hard.")
set(CMAKE_TOOLCHAIN_FILE "../stm32/cmake/STM32_Toolchain.cmake" CACHE STRING "Toolchain defs")

###################################################

# Set default C flags.
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -std=gnu11 -mlittle-endian -mthumb -mthumb-interwork --specs=nano.specs -u_printf_float -mcpu=cortex-m4 -ffunction-sections -fdata-sections -O3")

add_definitions(-DSTM32F40_41xxx -DCORTEX_M4 -D__EMBEDDED__)
#add_definitions(-DFREEDV_MODE_EN_DEFAULT=0 -DFREEDV_MODE_1600_EN=1 -DFREEDV_MODE_700D_EN=1 -DCODEC2_MODE_EN_DEFAULT=0 -DCODEC2_MODE_1300_EN=1 -DCODEC2_MODE_700C_EN=1)

#add_definitions(-DCODEC2_MODE_450_EN=0 -DCODEC2_MODE_450PWB_EN=0)

if(FLOAT_TYPE STREQUAL "hard")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsingle-precision-constant -Wdouble-promotion -mfpu=fpv4-sp-d16 -mfloat-abi=hard -D__FPU_PRESENT=1 -D__FPU_USED=1")
    #CFLAGS += -fsingle-precision-constant
else()
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -msoft-float")
endif()

option(VERIFY_OPT "Enable this for dump files to help verify optimization" OFF)
if(VERIFY_OPT)
    add_definitions(-DDUMP)
endif()

# Set default build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    link_libraries(g m)
else()
    link_libraries(c m)
endif()

# Setup defaults that can't be set in the toolchain file
set(CMAKE_EXE_LINKER_FLAGS "-u_init -T${CMAKE_SOURCE_DIR}/stm32_flash.ld -Xlinker --gc-sections")
set(CMAKE_EXECUTABLE_SUFFIX_C ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_ASM ".elf")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp")


# Check build flags

message(STATUS "CMAKE_C_FLAGS: " ${CMAKE_C_FLAGS})
message(STATUS "CMAKE_ASM_FLAGS: " ${CMAKE_ASM_FLAGS})


###################################################

# STM32F4 Standard Peripheral Library

include(codec2/stm32//cmake/STM32_Lib.cmake)

###################################################

# Macro for elf->bin
macro(elf2bin target)
    add_custom_command(TARGET ${target}
    POST_BUILD COMMAND ${CMAKE_OBJCOPY} -O binary ${target}.elf ${target}.bin
    COMMENT "Creating binary for ${target}")
    set_source_files_properties(${target}.bin PROPERTIES GENERATED TRUE)
    set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" APPEND PROPERTY
    ADDITIONAL_MAKE_CLEAN_FILES ${target}.bin)
endmacro()

macro(elf2hex target)
    add_custom_command(TARGET ${target}
    POST_BUILD COMMAND ${CMAKE_OBJCOPY} -O ihex ${target}.elf ${target}.hex
    COMMENT "Creating ihex for ${target}")
    set_source_files_properties(${target}.hex PROPERTIES GENERATED TRUE)
    set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" APPEND PROPERTY
    ADDITIONAL_MAKE_CLEAN_FILES ${target}.hex)
endmacro()

# Macro for elf->bin
macro(elf2dfu target)
    add_custom_command(TARGET ${target}
            POST_BUILD COMMAND ${CMAKE_OBJCOPY} -O ihex ${target}.elf ${target}.hex && ${CMAKE_SOURCE_DIR}/support/hex2dfu.py ${target}.hex ${target}.dfu
    COMMENT "Creating dfu file for ${target}")
    set_source_files_properties(${target}.bin PROPERTIES GENERATED TRUE)
    set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" APPEND PROPERTY
    ADDITIONAL_MAKE_CLEAN_FILES ${target}.hex ${target}.dfu)
endmacro()

# This macro just adds generation of a map file with the same name as the executable and .map suffix
# to the linker command line. This works in older Cmake version (versions >= 3.13 have target_link_options)
# it should be a one to one replacement for add_executable
macro(add_mapped_executable target)
    add_executable(${target} ${ARGN})
    target_link_libraries(${target} "-Wl,-Map=$<TARGET_PROPERTY:NAME>.map")
    set_source_files_properties(${target}.map PROPERTIES GENERATED TRUE)
    set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" APPEND PROPERTY
    ADDITIONAL_MAKE_CLEAN_FILES ${target}.map)
endmacro()

include(CTest)
include_directories(codec2/src ../unittest codec2/stm32/inc ${PROJECT_BINARY_DIR})

#add_subdirectory(unittest/src)



# Codec 2

# Output path is such that #include <codec2/version.h> in codec2.h works
set(CODEC2_VERSION_PATH "${PROJECT_BINARY_DIR}/codec2")
#configure_file ("${PROJECT_SOURCE_DIR}/../cmake/version.h.in"
#                "${CODEC2_VERSION_PATH}/version.h" )
configure_file ("${PROJECT_SOURCE_DIR}/codec2/cmake/version.h.in"
                "${CODEC2_VERSION_PATH}/version.h" )


set(CODEC2_SRC ../codec2/src)
set(CODEC2_GEN_CODEBOOK_SRC ../build/src)
set(CODEC2_SRCS
${CODEC2_SRC}/lpc.c
${CODEC2_SRC}/nlp.c
${CODEC2_SRC}/postfilter.c
${CODEC2_SRC}/sine.c
${CODEC2_SRC}/codec2.c
${CODEC2_SRC}/codec2_fft.c
${CODEC2_SRC}/gp_interleaver.c
${CODEC2_SRC}/interldpc.c
${CODEC2_SRC}/kiss_fft.c
${CODEC2_SRC}/kiss_fftr.c
${CODEC2_SRC}/interp.c
${CODEC2_SRC}/lsp.c
${CODEC2_SRC}/ofdm.c
${CODEC2_SRC}/phase.c
${CODEC2_SRC}/quantise.c
${CODEC2_SRC}/pack.c
${CODEC2_SRC}/dump.c
${CODEC2_SRC}/cohpsk.c
${CODEC2_SRC}/fdmdv.c
${CODEC2_SRC}/freedv_api.c
${CODEC2_SRC}/filter.c
${CODEC2_SRC}/varicode.c
${CODEC2_SRC}/golay23.c
${CODEC2_SRC}/freedv_data_channel.c
${CODEC2_SRC}/newamp1.c
${CODEC2_SRC}/newamp2.c
${CODEC2_SRC}/mbest.c
${CODEC2_SRC}/HRA_112_112.c
${CODEC2_SRC}/linreg.c
${CODEC2_SRC}/mpdecode_core.c
${CODEC2_SRC}/phi0.c
${CODEC2_SRC}/HRAb_396_504.c
codebook.c
codebookd.c
codebookjvm.c
codebookge.c
codebooknewamp1_energy.c
codebooknewamp1.c
codebooknewamp2.c
codebooknewamp2_energy.c
)

add_library(codec2 STATIC ${CODEC2_SRCS})
add_library(codec2_prof STATIC ${CODEC2_SRCS})
target_compile_definitions(codec2_prof PRIVATE PROFILE)

set(SYSTEM_SRCS
system_stm32f4xx.c
startup_stm32f4xx.s
)

set(PROFILE_SYSTEM_SRCS
src/stm32f4_machdep.c
)

list(APPEND PROFILE_SYSTEM_SRCS ${SYSTEM_SRCS})

set(ADCDAC_UT_SRCS
src/adcdac_ut.c
)

set(POWER_UT_SRCS
src/power_ut.c
)

list(APPEND POWER_UT_SRCS ${CODEC2_SRCS})

set(USB_VCP
usb_conf/usb_bsp.c
usb_conf/usbd_desc.c
usb_conf/usbd_usr.c
usb_lib/cdc/usbd_cdc_core.c
usb_lib/cdc/usbd_cdc_vcp.c
usb_lib/core/usbd_core.c
usb_lib/core/usbd_ioreq.c
usb_lib/core/usbd_req.c
usb_lib/otg/usb_core.c
usb_lib/otg/usb_dcd.c
usb_lib/otg/usb_dcd_int.c)

add_definitions(-DUSE_USB_OTG_FS -DUSE_ULPI_PHY -DVBUS_SENSING_ENABLED)
include_directories(usb_conf usb_lib/cdc usb_lib/core usb_lib/otg)

set(FDMDV_PROFILE_SRCS
src/fdmdv_profile.c
)

list(APPEND FDMDV_PROFILE_SRCS ${CODEC2_SRCS} ${SYSTEM_SRCS})

set(FDMDV_DUMP_RT_SRCS
src/fdmdv_dump_rt.c
)

list(APPEND FDMDV_DUMP_RT_SRCS ${CODEC2_SRCS} ${SYSTEM_SRCS})

# -----------------------------------------------------------------------------

include_directories(Utilities/STM32F4-Discovery pdm_fir)

set(VCP_SRCS
    usb_lib/cdc/usbd_cdc_vcp.c
    usb_lib/cdc/usbd_cdc_core.c
    usb_lib/otg/usb_dcd.c
    usb_lib/otg/usb_core.c
    usb_lib/core/usbd_req.c
    usb_lib/core/usbd_ioreq.c
    usb_lib/core/usbd_core.c
    usb_lib/otg/usb_dcd_int.c
    usb_conf/usbd_desc.c
    usb_conf/usbd_usr.c
    usb_conf/usb_bsp.c
)

add_library(vcp STATIC ${VCP_SRCS})

set(PASSTHRU_SRCS
    Utilities/STM32F4-Discovery/stm32f4_discovery_audio_codec.c
    passthru/passthru.c
    passthru/speaker.c
    passthru/microphone.c
    pdm_fir/pdm_fir.c
)

add_mapped_executable(passthru ${PASSTHRU_SRCS} ${SYSTEM_SRCS})
target_link_libraries(passthru stm32f4 vcp)
elf2bin(passthru)
elf2hex(passthru)

set(VOCODER_PASSTHRU_SRCS
    Utilities/STM32F4-Discovery/stm32f4_discovery_audio_codec.c
    vocoder_passthru/main.c
    vocoder_passthru/speaker.c
    vocoder_passthru/microphone.c
    pdm_fir/pdm_fir.c
)

include_directories(vocoder_passthru ${PROJECT_SOURCE_DIR})

add_mapped_executable(vocoder_passthru ${VOCODER_PASSTHRU_SRCS} ${SYSTEM_SRCS})
target_link_libraries(vocoder_passthru stm32f4 vcp codec2)
elf2bin(vocoder_passthru)
elf2hex(vocoder_passthru)

set(CODEC2_SX1262_SRCS
    Utilities/STM32F4-Discovery/stm32f4_discovery_audio_codec.c
    lora_codec2/main.c
    lora_codec2/speaker.c
    lora_codec2/microphone.c
    lora_codec2/radio.c
    lora_codec2/app_sx126x.c
    pdm_fir/pdm_fir.c
    sx12xx_hal/radio_sx126x.c
    sx126x/sx126x.c
)


add_mapped_executable(lora_codec2 ${CODEC2_SX1262_SRCS} ${SYSTEM_SRCS})
target_compile_definitions(lora_codec2 PRIVATE -DIS_SX1262=1)
#target_compile_definitions(lora_codec2 PRIVATE -DENABLE_VCP_PRINT)
target_compile_definitions(lora_codec2 PRIVATE -DLORA_BW_KHZ=500)
#target_compile_definitions(lora_codec2 PRIVATE -DLORA_BW_KHZ=250)
#target_compile_definitions(lora_codec2 PRIVATE -DLORA_BW_KHZ=125)
#target_compile_definitions(lora_codec2 PRIVATE -DCODEC2_MODE=CODEC2_MODE_3200)
#target_compile_definitions(lora_codec2 PRIVATE -DCODEC2_MODE=CODEC2_MODE_2400)
#target_compile_definitions(lora_codec2 PRIVATE -DCODEC2_MODE=CODEC2_MODE_1600)
#target_compile_definitions(lora_codec2 PRIVATE -DCODEC2_MODE=CODEC2_MODE_1400)
#target_compile_definitions(lora_codec2 PRIVATE -DCODEC2_MODE=CODEC2_MODE_1300)
target_compile_definitions(lora_codec2 PRIVATE -DCODEC2_MODE=CODEC2_MODE_1200)
#target_compile_definitions(lora_codec2 PRIVATE -DCODEC2_MODE=CODEC2_MODE_700C)
#target_compile_definitions(lora_codec2 PRIVATE -DCODEC2_MODE=CODEC2_MODE_450)
include_directories(vocoder_passthru sx12xx_hal sx126x)
target_link_libraries(lora_codec2 stm32f4 vcp codec2)
elf2bin(lora_codec2)
elf2hex(lora_codec2)

