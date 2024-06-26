# set minimum version of CMake.
cmake_minimum_required(VERSION 3.13)

# The Generic system name is used for embedded targets (targets without OS) in
# CMake
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR ARM)

# Supress Error when trying to test the compiler
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(BUILD_SHARED_LIBS OFF)

# set project name and version
project(flash_loader VERSION 0.0.1)

# enable assembly
enable_language(ASM)

set(SOURCES
    ${CMAKE_SOURCE_DIR}/entry/entry.c
    ${CMAKE_SOURCE_DIR}/entry/cortex-vector.cpp
    ${CMAKE_SOURCE_DIR}/flash/main.cpp
    ${CMAKE_SOURCE_DIR}/flash/flash_device.cpp
)

set(HEADERS
    ${CMAKE_SOURCE_DIR}/entry/entry.hpp
)

# add our executable
add_executable(flash_loader
    ${SOURCES} ${HEADERS} 
)

# add the include directories for klib, the target and the arm files
target_include_directories(flash_loader PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../klib/")
target_include_directories(flash_loader PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../klib/targets/chip/lpc1756/")
target_include_directories(flash_loader PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../klib/targets/arm/")

# set the interrupt method, the target and if we we support low power sleep
target_compile_definitions(flash_loader PUBLIC "KLIB_IRQ=irq_ram")
target_compile_definitions(flash_loader PUBLIC "TARGET_CPU=lpc1756")

# enable C++20 support for the library
target_compile_features(flash_loader PUBLIC cxx_std_20)

# set the output filename
set_target_properties(flash_loader PROPERTIES OUTPUT_NAME "flash_loader" SUFFIX ".elf")

# compiler optimisations
target_compile_options(flash_loader PRIVATE "-g")
target_compile_options(flash_loader PRIVATE "-Os")

# set the cpu options for the compiler
target_compile_options(flash_loader PRIVATE "-march=armv7-m")
target_compile_options(flash_loader PRIVATE "-mcpu=cortex-m3")
target_compile_options(flash_loader PRIVATE "-mthumb")

# other compiler settings
target_compile_options(flash_loader PRIVATE "-Wno-attributes")
target_compile_options(flash_loader PRIVATE "-fno-non-call-exceptions")
target_compile_options(flash_loader PRIVATE "-fno-common")
target_compile_options(flash_loader PRIVATE "-ffunction-sections")
target_compile_options(flash_loader PRIVATE "-fdata-sections")
target_compile_options(flash_loader PRIVATE "-fno-exceptions")
target_compile_options(flash_loader PRIVATE "-Wno-maybe-uninitialized")
target_compile_options(flash_loader PRIVATE "-Wno-unused-local-typedefs")
target_compile_options(flash_loader PRIVATE "-Wno-unused-but-set-variable")
target_compile_options(flash_loader PRIVATE "-Wno-unused-function")
target_compile_options(flash_loader PRIVATE "-fomit-frame-pointer")
target_compile_options(flash_loader PRIVATE "-Wall")
target_compile_options(flash_loader PRIVATE "-Werror")

# set the c++ only options
target_compile_options(flash_loader PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>)
target_compile_options(flash_loader PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>)
target_compile_options(flash_loader PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-fconcepts>)
target_compile_options(flash_loader PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-fno-use-cxa-get-exception-ptr>)
target_compile_options(flash_loader PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-fno-use-cxa-atexit>)
target_compile_options(flash_loader PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-Wno-volatile>)

# set the cpu options for the linker
target_link_options(flash_loader PRIVATE "-march=armv7-m")
target_link_options(flash_loader PRIVATE "-mcpu=cortex-m3")
target_link_options(flash_loader PRIVATE "-mthumb")

# other linker options
target_link_options(flash_loader PUBLIC "-ffunction-sections")
target_link_options(flash_loader PUBLIC "-fdata-sections")
target_link_options(flash_loader PUBLIC "-nostdlib")
target_link_options(flash_loader PUBLIC "-nodefaultlibs")
target_link_options(flash_loader PUBLIC "-nostartfiles")
target_link_options(flash_loader PUBLIC "-Wl,--gc-sections")
target_link_options(flash_loader PUBLIC "-Wl,-fatal-warnings")
target_link_options(flash_loader PUBLIC "-Wl,-cref,-Map=flash_loader.map")
target_link_options(flash_loader PUBLIC "-Wl,--print-memory-usage")
target_link_options(flash_loader PUBLIC "-Wl,--no-warn-rwx-segments")

# link to the linkerscript of the target cpu
target_link_options(flash_loader PUBLIC "-T${CMAKE_SOURCE_DIR}/linkerscript.ld")

# Custom commands for processing the build binary and show some statistics and debug info 
add_custom_command(TARGET flash_loader DEPENDS ${CMAKE_BINARY_DIR}/flash_loader.elf POST_BUILD COMMAND arm-none-eabi-objcopy ARGS -O binary -R .bss -R .stack flash_loader.elf flash_loader.bin)
add_custom_command(TARGET flash_loader DEPENDS ${CMAKE_BINARY_DIR}/flash_loader.elf POST_BUILD COMMAND arm-none-eabi-objdump ARGS -C -S flash_loader.elf > flash_loader.lss)
add_custom_command(TARGET flash_loader DEPENDS ${CMAKE_BINARY_DIR}/flash_loader.elf POST_BUILD COMMAND arm-none-eabi-objdump ARGS -C -sj PrgCode -sj DevDscr -sj .bss -sj .data -sj .rodata -S flash_loader.elf > flash_loader.memory)
add_custom_command(TARGET flash_loader DEPENDS ${CMAKE_BINARY_DIR}/flash_loader.elf POST_BUILD COMMAND arm-none-eabi-size ARGS -A flash_loader.elf -x)