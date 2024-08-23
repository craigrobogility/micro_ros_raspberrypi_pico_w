#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico_uart_transports.h"

const uint LED_PIN = 25;

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    msg.data++;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, msg.data % 2);
    // printf("callback was hit. Value: %u", msg.data);
}

int main()
{
    const int flash_delay_ms = 250;
    const int flash_fast_delay_ms = 100;
    const int flash_slow_delay_ms = 500;
    // printf("Booting");

    stdio_init_all();
    if (cyw43_arch_init())
    {
        // printf("Wi-Fi init failed");
        return -1;
    }

    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read);

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping
    const int timeout_ms = 1000;
    const uint8_t attempts = 5;

    // show that we woke up/alive
    for (int i = 0; i < 3; i++)
    {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        sleep_ms(flash_fast_delay_ms);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        sleep_ms(flash_fast_delay_ms);
    }

    while (true)
    {
        // printf("try to connect to the agent");
        rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);
        if (ret == RCL_RET_OK)
            break;

        for (int i = 0; i < 3; i++)
        {
            // Unreachable agent, flash fast
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
            sleep_ms(flash_fast_delay_ms);
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
            sleep_ms(flash_fast_delay_ms);
        }
    }

        msg.data = 0;

        rclc_support_init(&support, 0, NULL, &allocator);

        rclc_node_init_default(&node, "pico_node", "", &support);
        rclc_publisher_init_default(
            &publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "pico_publisher");

        rclc_timer_init_default(
            &timer,
            &support,
            RCL_MS_TO_NS(1000),
            timer_callback);

        rclc_executor_init(&executor, &support.context, 1, &allocator);
        rclc_executor_add_timer(&executor, &timer);

        while (true)
        {
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(flash_slow_delay_ms));
        }

    return 0;
}
