# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/Jimmy/.espressif/frameworks/esp-idf-v4.4.2/components/bootloader/subproject"
  "C:/ESP32/Motor_esp32_Pos/cmake-build-debug/bootloader"
  "C:/ESP32/Motor_esp32_Pos/cmake-build-debug/bootloader-prefix"
  "C:/ESP32/Motor_esp32_Pos/cmake-build-debug/bootloader-prefix/tmp"
  "C:/ESP32/Motor_esp32_Pos/cmake-build-debug/bootloader-prefix/src/bootloader-stamp"
  "C:/ESP32/Motor_esp32_Pos/cmake-build-debug/bootloader-prefix/src"
  "C:/ESP32/Motor_esp32_Pos/cmake-build-debug/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/ESP32/Motor_esp32_Pos/cmake-build-debug/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
