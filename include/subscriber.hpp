/**
 * @file subscriber.hpp
 * @author Alper Tunga Güven (alpert.guven@gmail.com)
 * @brief Header file for MicroROS subscriber related functions.
 * @version 0.1
 * @date 2024-09-09
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef SUBSCRIBER_HPP
#define SUBSCRIBER_HPP

#include <rcl/subscription.h>
#include <rclc/executor.h>
#include <rover_drive_interfaces/msg/motor_drive.h>
#include <rover_arm_interfaces/msg/arm_stepper.h>

#include <etl/string_view.h>

namespace ros {

extern "C" void armStepperSubscriberCallback(const void* msgin, void* context);

extern "C" void gripperSubscriberCallback(const void* msgin, void* context);


class Subscriber {
public:
    Subscriber(rcl_node_t* node, etl::string_view name, const rosidl_message_type_support_t* type);

    rcl_ret_t addToExecutor(rclc_executor_t* executor, void* msg,
        rclc_subscription_callback_with_context_t callback, void* context,
        rclc_executor_handle_invocation_t event);

private:
    rcl_subscription_t subscriber_{};
};

etl::array<Subscriber, 3> createGripperSubscribers(rcl_node_t * node);
etl::array<Subscriber, 3> createStepperSubscribers(rcl_node_t * node);

} // namespace ros

#endif // SUBSCRIBER_HPP