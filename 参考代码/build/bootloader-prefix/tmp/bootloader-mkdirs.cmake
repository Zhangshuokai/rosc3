# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Espressif/frameworks/esp-idf-v5.3.1/components/bootloader/subproject"
  "C:/code_sum/esp32/i2c_oled/i2c_oled/build/bootloader"
  "C:/code_sum/esp32/i2c_oled/i2c_oled/build/bootloader-prefix"
  "C:/code_sum/esp32/i2c_oled/i2c_oled/build/bootloader-prefix/tmp"
  "C:/code_sum/esp32/i2c_oled/i2c_oled/build/bootloader-prefix/src/bootloader-stamp"
  "C:/code_sum/esp32/i2c_oled/i2c_oled/build/bootloader-prefix/src"
  "C:/code_sum/esp32/i2c_oled/i2c_oled/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/code_sum/esp32/i2c_oled/i2c_oled/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/code_sum/esp32/i2c_oled/i2c_oled/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
