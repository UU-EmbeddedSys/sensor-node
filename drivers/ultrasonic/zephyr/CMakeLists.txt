# SPDX-License-Identifier: Apache-2.0

if(CONFIG_ULTRASONIC)
  zephyr_include_directories(.)

  zephyr_library()
  zephyr_library_sources(
    ultrasonic.c
    )
endif()
