/**
 * @file pinout.hpp
 * @author Alper Tunga Güven (alpert.guven@gmail.com)
 * @brief All the pins used by components.
 * @version 0.1
 * @date 2024-09-09
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef PINS_HPP
#define PINS_HPP

#include <etl/array.h>
#include <hardware/pio.h>
namespace pinout {

inline constexpr uint led = 25;

inline constexpr uint i2cSda = 0;
inline constexpr uint i2cScl = 1;

inline constexpr etl::array<uint, 3> gripperMotorPwmL{14, 10, 12};
inline constexpr etl::array<uint, 3> gripperMotorPwmR{15, 11, 13};

inline constexpr etl::array<uint, 3> armStepperPul{ 2, 6, 8 };
inline constexpr etl::array<uint, 3> armStepperDir{ 3, 7, 9 };

// inline const PIO encoderPio = pio0;
// inline constexpr etl::array<uint, 4> encoderA{ 10, 14, 12, 20 };
// inline constexpr etl::array<uint, 4> encoderB{ 11, 15, 13, 21 };
// inline constexpr etl::array<uint, 4> encoderPioSm{ 0, 1, 2, 3 };
} // namespace pinout

#endif // PINS_HPP