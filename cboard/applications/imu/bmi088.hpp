#ifndef CBOARD_APPLICATIONS_IMU_BMI088_HPP_
#define CBOARD_APPLICATIONS_IMU_BMI088_HPP_

#include <cstdint>

#include "gpio.h"
#include "spi.h"

namespace cboard
{

class BMI088
{
public:
  BMI088(
    SPI_HandleTypeDef * hspi, GPIO_TypeDef * accel_cs_port, std::uint16_t accel_cs_pin,
    GPIO_TypeDef * gyro_cs_port, std::uint16_t gyro_cs_pin, const float rotation[3][3]);

  // Values are expressed in the selected robot coordinate frame.
  volatile float acc[3] = {0.0F, 0.0F, 0.0F};  // m/s^2
  volatile float gyro[3] = {0.0F, 0.0F, 0.0F};  // rad/s
  volatile float temperature = 0.0F;  // degrees Celsius
  volatile bool initialized = false;
  volatile std::uint8_t last_error = 0U;

  bool init();
  bool update();

private:
  SPI_HandleTypeDef * hspi_;
  GPIO_TypeDef * accel_cs_port_;
  GPIO_TypeDef * gyro_cs_port_;
  std::uint16_t accel_cs_pin_;
  std::uint16_t gyro_cs_pin_;
  float rotation_[3][3];
  std::uint8_t tx_buffer_[8] = {};
  std::uint8_t rx_buffer_[8] = {};

  bool accel_init();
  bool gyro_init();
  bool accel_read(std::uint8_t reg, std::uint8_t * data, std::uint8_t length);
  bool gyro_read(std::uint8_t reg, std::uint8_t * data, std::uint8_t length);
  bool accel_write(std::uint8_t reg, std::uint8_t value);
  bool gyro_write(std::uint8_t reg, std::uint8_t value);
  bool read_register(bool accel, std::uint8_t reg, std::uint8_t & value);
  bool write_and_verify(bool accel, std::uint8_t reg, std::uint8_t value);
  void select_accel(bool selected);
  void select_gyro(bool selected);
};

extern BMI088 bmi088;

}  // namespace cboard

#endif  // CBOARD_APPLICATIONS_IMU_BMI088_HPP_
