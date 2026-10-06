target_sources(cboard PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/buzzer/buzzer.c
  ${CMAKE_CURRENT_SOURCE_DIR}/led/led.c
  ${CMAKE_CURRENT_SOURCE_DIR}/plotter_task.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/imu_task.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/imu/bmi088.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/imu/yaw_estimator.cpp
)

target_include_directories(cboard PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/buzzer
  ${CMAKE_CURRENT_SOURCE_DIR}/led
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/imu
)
