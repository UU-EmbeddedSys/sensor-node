# SPDX-License-Identifier: Apache-2.0

if(CONFIG_BME680)
  zephyr_include_directories(.)

  zephyr_library()
  zephyr_library_sources(
    bme680.c
    )
endif()
