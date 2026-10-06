#include "bmi088.hpp"

#include <cstring>

#include "cmsis_os.h"
#include "bmi088_defs.h"

namespace
{
constexpr std::uint32_t SPI_TIMEOUT_MS = 100U;
constexpr std::uint32_t RESET_DELAY_MS = 2U;

void delay_ms(std::uint32_t milliseconds)
{
  // BMI088 is initialized from the FreeRTOS task, so yield during the
  // mandatory device delays instead of busy-waiting at high task priority.
  (void)osDelay(milliseconds);
}
}

namespace cboard
{

BMI088::BMI088(
  SPI_HandleTypeDef * hspi, GPIO_TypeDef * accel_cs_port, std::uint16_t accel_cs_pin,
  GPIO_TypeDef * gyro_cs_port, std::uint16_t gyro_cs_pin, const float rotation[3][3])
: hspi_(hspi),
  accel_cs_port_(accel_cs_port),
  gyro_cs_port_(gyro_cs_port),
  accel_cs_pin_(accel_cs_pin),
  gyro_cs_pin_(gyro_cs_pin)
{
  for (std::uint8_t row = 0U; row < 3U; ++row) {
    for (std::uint8_t column = 0U; column < 3U; ++column) {
      rotation_[row][column] = rotation[row][column];
    }
  }
}

bool BMI088::init()
{
  initialized = false;
  last_error = 0U;

  // The BMI088 accelerometer starts in I2C mode after reset.  A read with
  // CS high followed by the normal SPI transactions switches it to SPI.
  if (!accel_init()) {
    last_error = 1U;
    return false;
  }
  if (!gyro_init()) {
    last_error = 2U;
    return false;
  }

  initialized = true;
  return true;
}

bool BMI088::update()
{
  if (!initialized) {
    return false;
  }

  std::uint8_t accel_data[6] = {};
  std::uint8_t gyro_data[6] = {};
  if (!accel_read(BMI088_ACC_DATA, accel_data, sizeof(accel_data)) ||
      !gyro_read(BMI088_GYRO_DATA, gyro_data, sizeof(gyro_data))) {
    last_error = 3U;
    return false;
  }

  const auto decode = [](const std::uint8_t * bytes) -> std::int16_t {
    return static_cast<std::int16_t>(
      static_cast<std::uint16_t>(bytes[0]) |
      (static_cast<std::uint16_t>(bytes[1]) << 8U));
  };

  const float accel_sensor[3] = {
    static_cast<float>(decode(&accel_data[0])) * BMI088_ACCEL_INT_TO_MPS2,
    static_cast<float>(decode(&accel_data[2])) * BMI088_ACCEL_INT_TO_MPS2,
    static_cast<float>(decode(&accel_data[4])) * BMI088_ACCEL_INT_TO_MPS2};
  const float gyro_sensor[3] = {
    static_cast<float>(decode(&gyro_data[0])) * BMI088_GYRO_INT_TO_RADS,
    static_cast<float>(decode(&gyro_data[2])) * BMI088_GYRO_INT_TO_RADS,
    static_cast<float>(decode(&gyro_data[4])) * BMI088_GYRO_INT_TO_RADS};

  // Keep the sensor-to-robot coordinate transform identical to course3.
  for (std::uint8_t row = 0U; row < 3U; ++row) {
    acc[row] = rotation_[row][0] * accel_sensor[0] +
      rotation_[row][1] * accel_sensor[1] + rotation_[row][2] * accel_sensor[2];
    gyro[row] = rotation_[row][0] * gyro_sensor[0] +
      rotation_[row][1] * gyro_sensor[1] + rotation_[row][2] * gyro_sensor[2];
  }

  return true;
}

bool BMI088::accel_init()
{
  std::uint8_t chip_id = 0U;
  // This first transaction creates the CS rising edge required by the datasheet.
  (void)accel_read(BMI088_ACC_CHIP_ID, &chip_id, 1U);
  delay_ms(1U);

  if (!accel_write(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE)) {
    return false;
  }
  delay_ms(RESET_DELAY_MS);
  (void)accel_read(BMI088_ACC_CHIP_ID, &chip_id, 1U);
  delay_ms(1U);

  if (!accel_read(BMI088_ACC_CHIP_ID, &chip_id, 1U) ||
      chip_id != BMI088_ACC_CHIP_ID_VALUE) {
    return false;
  }

  const bool configured =
    write_and_verify(true, BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON) &&
    write_and_verify(true, BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE) &&
    write_and_verify(
      true, BMI088_ACC_CONF,
      static_cast<std::uint8_t>(BMI088_ACC_BWP_NORMAL | BMI088_ACC_ODR_800_HZ |
                                BMI088_ACC_CONF_MUST_SET)) &&
    write_and_verify(true, BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G);
  return configured;
}

bool BMI088::gyro_init()
{
  if (!gyro_write(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE)) {
    return false;
  }
  delay_ms(RESET_DELAY_MS);

  std::uint8_t chip_id = 0U;
  if (!gyro_read(BMI088_GYRO_CHIP_ID, &chip_id, 1U) ||
      chip_id != BMI088_GYRO_CHIP_ID_VALUE) {
    return false;
  }

  return
    write_and_verify(false, BMI088_GYRO_RANGE, BMI088_GYRO_2000_DPS) &&
    write_and_verify(
      false, BMI088_GYRO_BANDWIDTH,
      static_cast<std::uint8_t>(BMI088_GYRO_BANDWIDTH_230_HZ |
                                BMI088_GYRO_BANDWIDTH_MUST_SET)) &&
    write_and_verify(false, BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE);
}

bool BMI088::accel_read(std::uint8_t reg, std::uint8_t * data, std::uint8_t length)
{
  if (length > 6U) {
    return false;
  }
  std::memset(tx_buffer_, 0, sizeof(tx_buffer_));
  std::memset(rx_buffer_, 0, sizeof(rx_buffer_));
  tx_buffer_[0] = static_cast<std::uint8_t>(reg | 0x80U);

  select_accel(true);
  const HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(
    hspi_, tx_buffer_, rx_buffer_, static_cast<std::uint16_t>(length + 2U), SPI_TIMEOUT_MS);
  select_accel(false);
  if (status != HAL_OK) {
    return false;
  }
  std::memcpy(data, &rx_buffer_[2], length);
  return true;
}

bool BMI088::gyro_read(std::uint8_t reg, std::uint8_t * data, std::uint8_t length)
{
  if (length > 6U) {
    return false;
  }
  std::memset(tx_buffer_, 0, sizeof(tx_buffer_));
  std::memset(rx_buffer_, 0, sizeof(rx_buffer_));
  tx_buffer_[0] = static_cast<std::uint8_t>(reg | 0x80U);

  select_gyro(true);
  const HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(
    hspi_, tx_buffer_, rx_buffer_, static_cast<std::uint16_t>(length + 1U), SPI_TIMEOUT_MS);
  select_gyro(false);
  if (status != HAL_OK) {
    return false;
  }
  std::memcpy(data, &rx_buffer_[1], length);
  return true;
}

bool BMI088::accel_write(std::uint8_t reg, std::uint8_t value)
{
  tx_buffer_[0] = reg;
  tx_buffer_[1] = value;
  select_accel(true);
  const HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi_, tx_buffer_, 2U, SPI_TIMEOUT_MS);
  select_accel(false);
  return status == HAL_OK;
}

bool BMI088::gyro_write(std::uint8_t reg, std::uint8_t value)
{
  tx_buffer_[0] = reg;
  tx_buffer_[1] = value;
  select_gyro(true);
  const HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi_, tx_buffer_, 2U, SPI_TIMEOUT_MS);
  select_gyro(false);
  return status == HAL_OK;
}

bool BMI088::read_register(bool accel, std::uint8_t reg, std::uint8_t & value)
{
  return accel ? accel_read(reg, &value, 1U) : gyro_read(reg, &value, 1U);
}

bool BMI088::write_and_verify(bool accel, std::uint8_t reg, std::uint8_t value)
{
  const bool write_ok = accel ? accel_write(reg, value) : gyro_write(reg, value);
  if (!write_ok) {
    return false;
  }
  delay_ms(1U);
  std::uint8_t readback = 0U;
  return read_register(accel, reg, readback) && readback == value;
}

void BMI088::select_accel(bool selected)
{
  HAL_GPIO_WritePin(accel_cs_port_, accel_cs_pin_, selected ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void BMI088::select_gyro(bool selected)
{
  HAL_GPIO_WritePin(gyro_cs_port_, gyro_cs_pin_, selected ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

}  // namespace cboard
