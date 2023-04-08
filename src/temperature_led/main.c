/*
 * Copyright (c) 2023 Todd Buiten.
 * Portions copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * 
 * This example uses OTAA to join the LoRaWAN network and then sends the 
 * internal temperature sensors value up as an uplink message periodically 
 * and the first byte of any uplink messages received controls the boards
 * built-in LED.
 */

// NOTE: The max packet size that we allow for is 11 bytes corresponding to DR0 but we prefer three bytes where possible.
//       See https://lora-developers.semtech.com/documentation/tech-papers-and-guides/the-book/packet-size-considerations/
//       for more info

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/watchdog.h"
#include "hardware/rtc.h"

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/critical_section.h"
#include "pico/lorawan.h"
#include "tusb.h"

// edit with LoRaWAN Node Region and OTAA settings 
#include "config.h"

#define REMAINING_RETRIES 3

// pin configuration for SX1276 radio module
const struct lorawan_sx1276_settings sx1276_settings = {
    .spi = {
        .inst = PICO_DEFAULT_SPI_INSTANCE,
        .mosi = PICO_DEFAULT_SPI_TX_PIN,
        .miso = PICO_DEFAULT_SPI_RX_PIN,
        .sck  = PICO_DEFAULT_SPI_SCK_PIN,
        .nss  = 8
    },
    .reset = 9,
    .dio0  = 7,
    .dio1  = 10
};

// OTAA settings
const struct lorawan_otaa_settings otaa_settings = {
    .device_eui   = LORAWAN_DEVICE_EUI,
    .app_eui      = LORAWAN_APP_EUI,
    .app_key      = LORAWAN_APP_KEY,
    .channel_mask = LORAWAN_CHANNEL_MASK
};

//
// Message queue - 11 bytes of transmitted data total
//
// header format
// +---------+-----------+-----------+----------------+
// | version | timestamp |   type    | content_length |
// +---------+-----------+-----------+----------------+
// | 0-2 (3) | 3-22 (20) | 23-27 (5) |   28-31 (4)    |
// +---------+-----------+-----------+----------------+
//
//   version - 0..7 (message version)
//   timestamp - 0..0xFFFFF (see below)
//   type - (0..31) (user-defined messge type)
//   content_length - (0..11) (length of the message content)
//
// timestamp format
// +---------+-----------+
// |   DOW   |   time    |
// +---------+-----------+
// | 0-2 (3) | 3-19 (20) |
// +---------+-----------+
//
//   DOW - 0..6, 0 is Sunday
//   time - 0..86400 (seconds past midnight)
//
struct message_entry {
  uint32_t header;
  uint8_t content[7];
  /* extra data that's not transmitted */
  uint8_t f_port;
  uint8_t content_length;
  struct message_entry* next;
  bool guaranteed_delivery;
  uint8_t remaining_retries;
  uint8_t dow;
};
struct message_entry* message_queue = NULL;
static critical_section_t message_queue_cri_sec;

// functions used in main
void internal_temperature_init();
float internal_temperature_get();

void machine_reset() {
    watchdog_enable(1, 1);
    while (1);
}

void erase_nvm( void ) {
    printf("Erasing NVM ... ");
    if (lorawan_erase_nvm() < 0) {
        printf("failed to erase NVM!!!\n");
        return;
    }

    printf("success erasing NVM!\n");
}

void join( void ) {
    // initialize the LoRaWAN stack
    printf("Initilizating LoRaWAN ... ");
    if (lorawan_init_otaa(&sx1276_settings, LORAWAN_REGION, &otaa_settings) < 0) {
        printf("failed to initialize OTAA - retarting!!!\n");
        erase_nvm();
        machine_reset();
    }

    printf("success!\n");

    // Start the join process and wait
    printf("Joining LoRaWAN network ...");
    lorawan_join();

    int seconds = 0;
    while (!lorawan_is_joined()) {
        lorawan_process_timeout_ms(1000);
        printf(".");
        if (++seconds > 120) {
            // Give up and restart the Pico
            printf("failed to join (timeout) - restarting!!!\n");
            erase_nvm();
            machine_reset();
        }
    }

    printf(" joined successfully!\n");
}

void cleanup_message( struct message_entry* message ) {
    if (message == NULL) {
        return;
    }

    critical_section_enter_blocking(&message_queue_cri_sec);
    if (message_queue == message) {
        message_queue = message->next;
    } else {
        struct message_entry* previous = message_queue;
        struct message_entry* current = message_queue->next;
        while (current != NULL) {
            if (current == message) {
                previous->next = current->next;
                break;
            }

            previous = current;
            current = current->next;
        }
    }
    critical_section_exit(&message_queue_cri_sec);

    free(message);
}

uint32_t create_message_timestamp() {
    datetime_t current_time;

    rtc_get_datetime(&current_time);

    return (current_time.dotw << 17) +
           current_time.hour * 3600 +
           current_time.min * 60 +
           current_time.sec;
}

void create_message_entry(uint8_t f_port, uint8_t type, uint8_t* content, uint8_t content_length, bool guaranteed_delivery) {
    uint8_t version = 0;
    uint32_t timestamp = create_message_timestamp();

    if (content_length > 7) {
        content_length = 7;
    }

    struct message_entry* message = (struct message_entry*) malloc(sizeof(struct message_entry));
    message->header =
        ((version & 0x07) << 29) |
        ((timestamp & 0xFFFFF) << 9) |
        ((type & 0x1F) << 4) |
        (content_length & 0x0F);

    message->f_port = f_port;
    message->content_length = content_length;
    message->guaranteed_delivery = guaranteed_delivery;
    message->remaining_retries = 0;
    message->dow = (timestamp >> 17) & 0x07;
    memcpy(&message->content[0], content, message->content_length);

    // Hook our message into the queue
    critical_section_enter_blocking(&message_queue_cri_sec);
    message->next = message_queue;
    message_queue = message;
    critical_section_exit(&message_queue_cri_sec);

    // uint8_t time_components[4 + content_length];
    // memcpy(&time_components[0], message, 4 + content_length);
    // int i;
    // for (i = 0; i < 4 + content_length; i++) {
    //     printf("header byte %d: 0x%02x\n", i, time_components[i]);
    // }
    // sleep_ms(300000);
    // continue;
}

struct message_entry* match_message_by_timestamp( uint32_t response_timestamp ) {
    struct message_entry* current = message_queue;

    while (current != NULL) {
        uint32_t message_timestamp = (current->header >> 9) & 0xFFFFF;

        if (response_timestamp == message_timestamp) {
            break;
        }

        current = current->next;
    }

    return current;
}

uint32_t queued_message_count() {
    struct message_entry* current = message_queue;
    uint32_t count = 0;

    while (current != NULL) {
        count++;
        current = current->next;
    }

    return count;
}

bool is_leap_year(int16_t year) {
    return (((year % 4 == 0) && (year % 100) != 0) || ((year % 400) == 0));
}

int8_t time_component_limits_min[5] = { 0, 0, 0, 1, 1 };
int8_t getTimeComponentLimitMin(int component_number) {
    return time_component_limits_min[component_number];
}

int8_t time_component_limits_max[5] = { 60, 60, 24, 31 /* Unused, see getTimeComponentLimitMax */, 12 };
int8_t time_component_month_limits_max[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
int8_t getTimeComponentLimitMax(int component_number, int8_t month, int16_t year) {
    //$ printf("here0 %d\n", component_number);
    if (component_number == 3) {
        //$ printf("here1 %d %d\n", component_number, 3);
        if (month == 2) {
            //$ printf("here2 %d %d %d\n", component_number, 3, month);
            int time_component_limit_max = time_component_month_limits_max[month - 1];
            if (is_leap_year(year)) {
                time_component_limit_max++;
            }

            return time_component_limit_max;
        }

        return time_component_month_limits_max[month - 1];
    }

    return time_component_limits_max[component_number];
}

int8_t month_key[12] = { 1, 4, 4, 0, 2, 5, 0, 3, 6, 1, 4, 6 };
int8_t calculate_dow(int8_t day, int8_t month, int16_t year) {
    int8_t day_of_week =
        day +
        month_key[month - 1] +
        ((month == 1 || month == 2) && is_leap_year(year) ? -1 : 0) -
        1 - // Take one off from 2000 - 2099
        1;  // Shift from Sat = 0 to Sun = 0
    year = year % 100;
    day_of_week += year + (year / 4);
    day_of_week = day_of_week % 7;
    //$ printf("day = %d, m = %d, y = %d\n", day, month, year);

    return day_of_week;
}

void sync_time_on_timestamp( uint8_t* receive_buffer) {
    datetime_t current_time;

    rtc_get_datetime(&current_time);
    for (int i = 4; i < 11; i++) {
        receive_buffer[i] -= 128;
    }

    current_time.year += (receive_buffer[4] * 100) + receive_buffer[5];
    current_time.month += receive_buffer[6];
    current_time.day += receive_buffer[7];
    current_time.hour += receive_buffer[8];
    current_time.min += receive_buffer[9];
    current_time.sec += receive_buffer[10];
    current_time.dotw = 0;

    int8_t* time_components[5] = {
        &current_time.sec,
        &current_time.min,
        &current_time.hour,
        &current_time.day,
        &current_time.month
    };

    for (int i = 0; i < 5; i++) {
        int8_t time_component_limit_min = getTimeComponentLimitMin(i);
        int8_t time_component_limit_max = getTimeComponentLimitMax(i, current_time.month, current_time.year);

        //$ printf("%d, min = %d, max = %d, mon = %d, year = %d, tc[i] = %d,\n", i, getTimeComponentLimitMin(i), getTimeComponentLimitMax(i, current_time.month, current_time.year), current_time.month, current_time.year, *time_components[i]);
        if (*time_components[i] < time_component_limit_min) {
            i < 4 ? (*time_components[i+1])-- : current_time.year--;
            *time_components[i] += time_component_limit_max;
        } else if (*time_components[i] >= (time_component_limit_max + time_component_limit_min)) {
            //$ printf("< %d: *time_components[i+1] = %d\n", i, *time_components[i+1]);
            i < 4 ? (*time_components[i+1])++ : current_time.year++;
            //$ printf("> %d: *time_components[i+1] = %d\n", i, *time_components[i+1]);
            *time_components[i] -= time_component_limit_max;
        }
    }

    // Calculate the day of week
    current_time.dotw = calculate_dow(current_time.day, current_time.month, current_time.year);

    printf("Setting time to %02d-%02d-%02d %02d:%02d:%02d (%d)\n",
        current_time.year,
        current_time.month,
        current_time.day,
        current_time.hour,
        current_time.min,
        current_time.sec,
        current_time.dotw
    );

    rtc_set_datetime(&current_time);
    sleep_ms(1); // Let the RTC stabiize

    rtc_get_datetime(&current_time);
    printf("Date updated to %02d-%02d-%02d %02d:%02d:%02d (%d)\n",
        current_time.year,
        current_time.month,
        current_time.day,
        current_time.hour,
        current_time.min,
        current_time.sec,
        current_time.dotw
    );
}

bool skip_first_received_messages = true;
bool transfer_data() {
    int receive_length = 0;
    uint8_t receive_buffer[242];
    uint8_t receive_port = 0;

    struct message_entry* message = message_queue;

    // Since we're a Class A device, if there are no uplinks then there are no downlinks either
    if (message == NULL) {
        return true;
    }

    while (message) {
        if (message->remaining_retries-- <= 0) {
            if (message->f_port == 1) {
                printf("sending internal temperature: %d °C (0x%02x)... ", message->content[0], message->content[0]);
            } else {
                printf("sending time sync message... ");
            }

            // send the message as a series of unsigned bytes in an unconfirmed uplink message
            if (lorawan_send_unconfirmed(message, sizeof(uint32_t) /* header length */ + message->content_length, message->f_port) < 0) {
                printf("failed!!!\n");

                if (!message->guaranteed_delivery) {
                    cleanup_message(message);
                }

                return false;
            }

            if (!message->guaranteed_delivery) {
                cleanup_message(message);
            }

            printf("success!\n");

            message->remaining_retries = REMAINING_RETRIES;
        }

        while (true) {
            // wait for up to 30 seconds for an event
            if (lorawan_process_timeout_ms(30000) == 0) {
                // check if a downlink message was received
                receive_length = lorawan_receive(receive_buffer, sizeof(receive_buffer) / sizeof(receive_buffer[0]), &receive_port);
                if (receive_length >= 0) {
                    // If the application restarts we could have a leftover time sync downlink
                    // message being held at the gateway. If the restart happens during the initial time sync
                    // then we can have two "gross" adjustments where we change the year by many multiples.
                    // For example, receiving two adjustments for the date 3/2/2023 will push our clock out
                    // to June 2046. To combat this problem, we drain any messages in the gateway when the
                    // app first starts up. If we do happen to discard something important then it should be
                    // retransmitted
                    if (skip_first_received_messages) {
                        printf("Skipping buffered receive message from previous session\n");
                        continue;
                    }

                    printf("received a %d byte message on port %d: ", receive_length, receive_port);

                    for (int i = 0; i < receive_length; i++) {
                        printf("%02x", receive_buffer[i]);
                    }
                    printf("\n");

                    uint32_t receive_header =
                        (receive_buffer[3] << 24) |
                        (receive_buffer[2] << 16) |
                        (receive_buffer[1] << 8) |
                        receive_buffer[0];
                    uint32_t receive_timestamp = (receive_header >> 9) & 0xFFFFF;
                    printf("receive message timestamp = %d, dow = %d, time = %d\n", receive_timestamp, receive_timestamp >> 17, receive_timestamp & 0x1FFFF);

                    cleanup_message(match_message_by_timestamp(receive_timestamp));

                    uint32_t type = (receive_header >> 4) & 0x1F;
                    uint32_t time_offset;
                    switch (receive_port) {
                        case 222:
                            switch (type) {
                                case 0:
                                    sync_time_on_timestamp(&receive_buffer[0]);
                                    break;

                                default:
                                    printf("Unknown system message (port 222), type = %d", type);
                                    break;
                            }
                            break;

                        case 1:
                            // the first byte of the received message controls the on board LED
                            gpio_put(PICO_DEFAULT_LED_PIN, receive_buffer[0]);
                            break;

                        default:
                            printf("unknown message type ack: %d\n", type);
                            break;
                    }
                }

                continue;
            }

            skip_first_received_messages = false;

            break;
        }

        message = message->next;
    }

    return true;
}

void populate_time_sync( uint8_t* time_sync ) {
    datetime_t current_time;

    rtc_get_datetime(&current_time);
    time_sync[0] = current_time.year / 100;
    time_sync[1] = current_time.year % 100;
    time_sync[2] = current_time.month;
    time_sync[3] = current_time.day;
    time_sync[4] = current_time.hour;
    time_sync[5] = current_time.min;
    time_sync[6] = current_time.sec;
}

void populate_time_sync_nop( uint8_t* time_sync ) {
    for (int i = 0; i < 7; i++) {
        time_sync[i] = 0;
    }
}

void sync_time( bool initialize ) {
    datetime_t current_time;

    if (initialize) {
        // Initialize the realtime clock. We arbitrarily set it to Jan 1, 2000
        current_time.year = 2000;
        current_time.month = 1;
        current_time.day = 1;
        current_time.dotw = 6;
        current_time.hour = 0;
        current_time.min = 0;
        current_time.sec = 0;
        rtc_init();
        rtc_set_datetime(&current_time);
        sleep_ms(1); // Let the RTC stabiize
    }

    while (1) {
        // First, send the message with our current time
        uint8_t time_sync[7];
        populate_time_sync(&time_sync[0]);
        create_message_entry(222, 0, &time_sync[0], 4 + (sizeof(time_sync) / sizeof(time_sync[0])), false);

        // If we're initializing then we want to block the sync_time call until we can set
        // the time for the first time. Otherwise we just wait until the downlink response
        // would normally be processed. And what if that normal processing response is lost?
        // Oh well, we'll retry again soon as part of our regular time sync so no big deal
        if (initialize) {
            if (!transfer_data()) {
                join();
                continue;
            }

            // Wait for the server to send back a downlink with the offset
            sleep_ms(10000);

            // Go pick up the new timestamp
            populate_time_sync_nop(&time_sync[0]);
            create_message_entry(222, 0, &time_sync[0], 4 + (sizeof(time_sync) / sizeof(time_sync[0])), false);
            if (!transfer_data()) {
                join();
                continue;
            }

            // Check to see if our clock was adjusted beyond its initial value. If it was then
            // we'll assume that the value was received correctly or that the time adjustment
            // value was lost
            bool rtc_ready = rtc_get_datetime(&current_time);
            if (current_time.year > 2000) {
                break;
            }

            // If we get here then we'll automatically retry in the while loop above
        } else {
            break;
        }
    }
}

void service_messages() {
    datetime_t current_time;

    bool rejoin = false;
    // loop forever
    while (1) {
        bool rtc_ready = rtc_get_datetime(&current_time);
        printf("(%d) current time: %04d-%02d-%02d %02d:%02d:%02d, queued message count: %d\n",
            rtc_ready,
            current_time.year,
            current_time.month,
            current_time.day,
            current_time.hour,
            current_time.min,
            current_time.sec,
            queued_message_count()
        );
        if (rejoin) {
            rejoin = false;
            join();
        }

        rejoin = !transfer_data();

        if (rejoin) {
            continue;
        }

        // now sleep for 10 seconds
        sleep_ms(10000);
    }
}

bool scheduled_daily_tasks( repeating_timer_t* time_sync_timer ) {
    datetime_t current_time;
    uint8_t expired_dow;
    struct message_entry* current = message_queue;


    // Walk the list of unacknowledged messages, looking for those that are expired.
    // We don't need to worry about new messages showing up on the message queue
    // since new messsages always appear at the head of the queue. We do, however,
    // need to worry about deleting messages in the list since in theory we could get
    // an ACK back on a message at the same time that we are about to delete it. The
    // probablility of this happening is extremely slim, however, so we'll just
    // let the watchdog deal with this case for now
    printf("Cleaning up dead messages\n");
    rtc_get_datetime(&current_time);
    expired_dow = (current_time.dotw + 2) % 7;
    while (current != NULL) {
        if (expired_dow == current->dow) {
            cleanup_message(current);
        }

        current = current->next;
    }


    printf("Daily time sync\n");
    sync_time(false);
}

void service_interrupts( void ) {
    static struct repeating_timer time_sync_timer;

    // Once per day, schedule a time sync
    //$ add_repeating_timer_ms(86400000, scheduled_daily_tasks, NULL, &time_sync_timer);
    add_repeating_timer_ms(3600000, scheduled_daily_tasks, NULL, &time_sync_timer);

    while (true) {
        // get the internal temperature
        uint8_t adc_temperature_byte = internal_temperature_get();
        uint8_t content_length = sizeof(adc_temperature_byte);

        printf("Writing temperature to message queue\n");
        create_message_entry(1, 1, &adc_temperature_byte, content_length, true);

        // now sleep for five minutes
        sleep_ms(300000);
    }
}

int main( void )
{
    // initialize stdio and wait for USB CDC connect
    stdio_init_all();
    //$ while (!tud_cdc_connected()) {
    //$     tight_loop_contents();
    //$ }

    printf("Pico LoRaWAN - OTAA - Temperature + LED\n\n");

    // If your device can't seem to connect then uncomment the line
    // below to remove any existing device info
    // erase_nvm();

    // initialize the LED pin and internal temperature ADC
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    critical_section_init(&message_queue_cri_sec);

    internal_temperature_init();

    // uncomment next line to enable debug
    // lorawan_debug(true);

    // Join the server
    join();

    // Get the current date and time from the remote end
    sync_time( true );

    printf("Hooking in service_interrupts\n");
    multicore_launch_core1(&service_interrupts);

    service_messages();

    // If we get here then there must have been an error
    return 1;
}

void internal_temperature_init()
{
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);
}

float internal_temperature_get()
{
    const float v_ref = 3.3;

    // select and read the ADC
    adc_select_input(4);
    uint16_t adc_raw = adc_read();

    // convert the raw ADC value to a voltage
    float adc_voltage = adc_raw * v_ref / 4095.0f;

    // convert voltage to temperature, using the formula from 
    // section 4.9.4 in the RP2040 datasheet
    //   https://datasheets.raspberrypi.org/rp2040/rp2040-datasheet.pdf
    float adc_temperature = 27.0 - ((adc_voltage - 0.706) / 0.001721);

    return adc_temperature;
}
