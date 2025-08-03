cmake_minimum_required(VERSION 3.20.0)

if(NOT ZEPHYR_BASE)
  message(FATAL_ERROR "This is not a standalone app")
endif()

add_subdirectory(drivers)
