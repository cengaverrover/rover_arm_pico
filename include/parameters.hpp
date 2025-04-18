/**
 * @file parameters.hpp
 * @author Alper Tunga Güven (alpert.guven@gmail.com)
 * @brief Header file of MicroROS parameters and server.
 * @version 0.1
 * @date 2024-09-09
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

#include <pico/types.h>
#include <rclc_parameter/rclc_parameter.h>
#include <etl/array.h>
#include <etl/string_view.h>

namespace ros {

namespace parameter {

class Server {
public:
    Server(rcl_node_t* node, rclc_parameter_options_t* serverOptions = nullptr);

    Server(rcl_node_t* node, bool notify_changed_over_dds, uint32_t max_params,
        bool allow_undeclared_parameters, bool low_mem_mode);

    rcl_ret_t addToExecutor(rclc_executor_t* executor);

    rcl_ret_t addParameter(etl::string_view paramName, rclc_parameter_type_t paramType);
    rcl_ret_t addParameter(etl::string_view paramName, int32_t value);
    rcl_ret_t addParameter(etl::string_view paramName, float value);
    rcl_ret_t addParameter(etl::string_view paramName, bool value);

    rcl_ret_t addParameterConstraint(
        etl::string_view paramName, int32_t lower, int32_t upper, int32_t step = 0);
    rcl_ret_t addParameterConstraint(
        etl::string_view paramName, float lower, float upper, float step = 0.0f);

    rcl_ret_t setParameter(etl::string_view paramName, int32_t paramValue);
    rcl_ret_t setParameter(etl::string_view paramName, float paramValue);
    rcl_ret_t setParameter(etl::string_view paramName, bool paramValue);

private:
    rclc_parameter_server_t paramServer_{};

    rcl_ret_t initParameters();
};

inline constexpr int paramCount = 13;

inline int32_t gripperMotorMaxRpm = 100;

inline float gripperMotorMaxDutyCycle = 100.0f;
constexpr inline float maxMotorDutyCycleUpperConstraint = 100.0f;
constexpr inline float maxMotorDutyCycleLowerConstraint = 0.0f;

inline float gripperMotorMaxCurrent = 50.0f;

inline int32_t motorTimeoutMs = 1000;
inline int32_t stepperTimeoutMs = 1000;

inline int32_t feedbackPeriodMs = 50;

inline int32_t executorSpinPeriodMs = 20;

inline etl::array<float, 3> stepperGearRatios{ 50.0f, 68.18181818f, 2.0f };
inline etl::array<int32_t, 3> stepperStepsPerRev{ 400, 400, 2000 };


} // namespace parameter

} // namespace ros

#endif // PARAMETERS_HPP