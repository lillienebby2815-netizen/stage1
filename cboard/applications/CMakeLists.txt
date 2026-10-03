target_sources(cboard PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/buzzer/buzzer.c
)

target_include_directories(cboard PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/buzzer
)
