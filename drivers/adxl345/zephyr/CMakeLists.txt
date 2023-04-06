# SPDX-License-Identifier: Apache-2.0

if(CONFIG_ADXL345) 
  zephyr_include_directories(.)

  zephyr_library()
  zephyr_library_sources(
    adxl345.c
    )
endif()
