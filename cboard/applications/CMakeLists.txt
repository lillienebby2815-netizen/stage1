target_sources(cboard PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/buzzer/buzzer.c
  ${CMAKE_CURRENT_SOURCE_DIR}/led/led.c
)

target_include_directories(cboard PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/buzzer
  ${CMAKE_CURRENT_SOURCE_DIR}/led
)
