# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set(IDF_TARGET esp32)

# keep in mind 'led_strip' added to EXTRA_COMPONENT_DIRS and COMPONENTS
# set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/examples/common_components/led_strip)
# set(COMPONENTS freertos newlib heap log soc esp_rom esp_common xtensa cxx
#                                                   esptool_py main led_strip)



# add app_trace to test gcov
# set(COMPONENTS freertos newlib heap log soc esp_rom esp_common xtensa esptool_py
#                                                         app_trace esp_wifi main)
set(COMPONENTS freertos newlib heap log soc esp_rom esp_common xtensa esptool_py
                                                    esp_wifi esp_http_client main)


include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(base)

# CMakeCache.txt should define CMAKE_C_COMPILER otherwise intellisense not working
# CMAKE_C_COMPILER:FILEPATH=IDF_XTENSA_ESP32_ELF_GCC

message(STATUS "target: ${IDF_TARGET}")
message(STATUS "idf: ${IDF_VER}")
message(STATUS "c-compiler: ${CMAKE_C_COMPILER}")
idf_build_get_property(e EXECUTABLE)
message(STATUS "elf exe is: ${e}")
message(STATUS "COMPONENT_DIR: ${COMPONENT_DIR}")
message(STATUS "EXTRA_COMPONENT_DIRS: ${EXTRA_COMPONENT_DIRS}")
message(STATUS "COMPONENTS: ${COMPONENTS}")
message(STATUS "BUILD_COMPONENTS: ${BUILD_COMPONENTS}")
