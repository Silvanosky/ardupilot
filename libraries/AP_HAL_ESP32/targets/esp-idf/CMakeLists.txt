cmake_minimum_required(VERSION 3.5)

set(CMAKE_TOOLCHAIN_FILE $ENV{IDF_PATH}/tools/cmake/toolchain-esp32.cmake CACHE STRING "")

project(ardupilot)

# Include for ESP-IDF build system functions
include($ENV{IDF_PATH}/tools/cmake/idf.cmake)

# Create idf::esp32 and idf::freertos static libraries
idf_build_process(esp32
    # try and trim the build; additional components
    # will be included as needed based on dependency tree
    #
    # although esptool_py does not generate static library,
    # processing the component is needed for flashing related
    # targets and file generation
    COMPONENTS esp32
               freertos
               tcpip_adapter
               fatfs
               esp_adc_cal
               nvs_flash
               esptool_py
               app_update
    SDKCONFIG ${CMAKE_CURRENT_LIST_DIR}/sdkconfig
    BUILD_DIR ${CMAKE_BINARY_DIR})

set(elf_file ${CMAKE_PROJECT_NAME}.bin)
add_executable(${elf_file} main.c)

add_custom_target(showinc ALL
        COMMAND echo -e
        "$<TARGET_PROPERTY:${elf_file},INCLUDE_DIRECTORIES>"
        > includes.list
    VERBATIM
    BYPRODUCTS includes.list
    COMMAND_EXPAND_LISTS
)

#Find files name lib to link
MACRO(SUBLINK target curdir)
  FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
  FOREACH(child ${children})
    IF(NOT IS_DIRECTORY ${curdir}/${child})
      SET(PATH "${curdir}/${child}")
      message("Linking ${PATH}")
      target_link_libraries(${target} "${PATH}")
    ENDIF()
  ENDFOREACH()
ENDMACRO()

SUBLINK(${elf_file} ${ARDUPILOT_BIN})
SUBLINK(${elf_file} ${ARDUPILOT_LIB})

# Link the static libraries to the executable
target_link_libraries(${elf_file}
                idf::esp32
                idf::freertos
                idf::fatfs
                idf::esp_adc_cal
                idf::nvs_flash
                idf::spi_flash
                idf::tcpip_adapter
                idf::app_update
                )

# Attach additional targets to the executable file for flashing,
# linker script generation, partition_table generation, etc.
idf_build_executable(${elf_file})

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

