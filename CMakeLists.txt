cmake_minimum_required(VERSION 3.20)

set (PROJECT_NAME             "firmware_liquids")
# Used by compiler flags:
set (LINKER_SCRIPT            "${CMAKE_SOURCE_DIR}/Linker/STM32G491METX_FLASH.ld")
set (MCPU                     "-mcpu=Cortex-M4")
set (MFPU                     "-mfpu=fpv4-sp-d16")
set (MFLOAT_ABI               "-mfloat-abi=hard")
set (RUNTIME_LIBRARY          "--specs=nano.specs")
set (RUNTIME_LIBRARY_SYSCALLS "--specs=nosys.specs")

# These are the files that will be compiled.
# GLOB_RECURSE is used to find all files matching the pattern in the specified directory and its subdirectories.
# CONFIGURE_DEPENDS is used to re-check globbed files when rebuilding.
file(GLOB_RECURSE PROJECT_SOURCES CONFIGURE_DEPENDS
	"Core/Startup/startup_stm32g491metx.s"
	"Core/Src/*.cpp"
	"Core/Src/*.c"
	"Drivers/STM32G474/*.c"
	)

project(${PROJECT_NAME})
enable_language(ASM)

# Target to re-generate the git hash file
add_custom_target(
    generate_git_hash
    COMMAND bash gen_git_hash.sh ./
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Generating Git hash information"
)

add_executable(${PROJECT_NAME} ${PROJECT_SOURCES})
# Always re-generate the git hash file
add_dependencies(${PROJECT_NAME} generate_git_hash)
# For printing the size of the binary
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>)

add_compile_definitions (
	# Definitions for the preprocessor
	STM32G474xx
	USE_FULL_LL_DRIVER
	$<$<CONFIG:Debug>:DEBUG> # Only define DEBUG in Debug mode
)

include_directories (
	# Include directories
	Core/Inc
	Core/Inc/can_houbolt
	Drivers/STM32G474/STRHAL/Inc
	Drivers/STM32G474/CMSIS/Include
	Drivers/STM32G474/LL_Driver/Inc
	Drivers/STM32G474/CMSIS/Device/ST/STM32G4xx/Include
)

set (CMAKE_EXECUTABLE_SUFFIX ".elf")

# Flags: Those were the flags used by the STM32CubeIDE.
# Some flags are initially set in the CMake preset, check CMakePresets.json (e.g. ECU_LAMARR_BOARD define)

# Flags for the C compiler
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${MCPU} -std=gnu11 ${MCPU} ${MFPU} ${MFLOAT_ABI} ${RUNTIME_LIBRARY} -mthumb -Wall -ffunction-sections -fdata-sections -fstack-usage")
# Flags for the C++ compiler
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${MCPU} -std=gnu++17 ${MCPU} ${MFPU} ${MFLOAT_ABI} ${RUNTIME_LIBRARY} -mthumb -Wall -ffunction-sections -fdata-sections -fstack-usage")
# Flags for the C compiler in Debug mode (include more debugging information than normal)
set (CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g3 -ggdb")
# Flags for the C++ compiler in Debug mode (include more debugging information than normal)
set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g3 -ggdb")
# Flags for the linker
set (CMAKE_EXE_LINKER_FLAGS "-T${LINKER_SCRIPT} ${RUNTIME_LIBRARY_SYSCALLS} -Wl,-Map=${PROJECT_NAME}.map -Wl,--gc-sections -static -u _printf_float  -Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group")
# Flags for the assembler
set (CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp")
