# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "E:/ESP32/Espressif/frameworks/esp-idf-v4.4.2/components/bootloader/subproject"
  "G:/Esp_Project/Mortor_ESP32/build/bootloader"
  "G:/Esp_Project/Mortor_ESP32/build/bootloader-prefix"
  "G:/Esp_Project/Mortor_ESP32/build/bootloader-prefix/tmp"
  "G:/Esp_Project/Mortor_ESP32/build/bootloader-prefix/src/bootloader-stamp"
  "G:/Esp_Project/Mortor_ESP32/build/bootloader-prefix/src"
  "G:/Esp_Project/Mortor_ESP32/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "G:/Esp_Project/Mortor_ESP32/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
