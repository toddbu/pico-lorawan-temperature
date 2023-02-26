/*
 * Copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
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
#include "pico/lorawan.h"
#include "tusb.h"

// edit with LoRaWAN Node Region and OTAA settings 
#include "config.h"

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

// Message queue - 11 bytes total
//
// header format
// +---------+-----------+-----------+----------------+
// | version |    id     |   type    | content_length |
// +---------+-----------+-----------+----------------+
// | 0-2 (3) | 3-22 (20) | 23-27 (5) |   28-31 (4)    |
// +---------+-----------+-----------+----------------+
//
// id calculation
//
//
struct message_entry {
  uint32_t header;
  int8_t content[7];
};

// variables for receiving data
int receive_length = 0;
uint8_t receive_buffer[242];
uint8_t receive_port = 0;

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
    free(message);
}

uint32_t message_id = 0;
int main( void )
{
    // initialize stdio and wait for USB CDC connect
    stdio_init_all();
    while (!tud_cdc_connected()) {
        tight_loop_contents();
    }

    printf("Pico LoRaWAN - OTAA - Temperature + LED\n\n");

    // initialize the LED pin and internal temperature ADC
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    internal_temperature_init();

    datetime_t current_time;
    current_time.year = 2023;
    current_time.month = 2;
    current_time.day = 26;
    current_time.dotw = 0;
    current_time.hour = 0;
    current_time.min = 0;
    current_time.sec = 0;

    rtc_init();
    rtc_set_datetime(&current_time);
    sleep_ms(1); // Let the RTC stabiize
    bool rtc_ready = rtc_get_datetime(&current_time);
    printf("(%d) current_year: %d\n", rtc_ready, current_time.year);

    // uncomment next line to enable debug
    // lorawan_debug(true);

    bool rejoin = true;
    // loop forever
    while (1) {
        if (rejoin) {
            rejoin = false;
            join();
        }

        uint8_t version = 1;
        uint32_t id = ++message_id;
        uint8_t type = 1;
        uint8_t content_length = sizeof(int8_t);

        // get the internal temperature
        int8_t adc_temperature_byte = internal_temperature_get();

        struct message_entry* message = (struct message_entry*) malloc(sizeof(struct message_entry));
        message->header =
            ((version & 0x07) << 29) |
            ((id & 0xFFFFF) << 9) |
            ((type & 0x1F) << 4) |
            (content_length & 0x0F);

        // uint8_t xyzzy[5];
        // memcpy(&xyzzy[0], message, 5);
        // int i;
        // for (i = 0; i < 4; i++) {
        //     printf("header byte %d: 0x%02x\n", i, xyzzy[i]);
        // }
        // sleep_ms(300000);
        // continue;

        memcpy(&message->content[0], &adc_temperature_byte, content_length);

        // send the internal temperature as a (signed) byte in an unconfirmed uplink message
        printf("sending internal temperature: %d °C (0x%02x)... ", message->content[0], message->content[0]);
        if (lorawan_send_unconfirmed(message, sizeof(uint32_t) /* header length */ + content_length, 2) < 0) {
            printf("failed!!!\n");
            rejoin = true;
            cleanup_message(message);
            continue;
        } else {
            printf("success!\n");
            cleanup_message(message);
        }

        // wait for up to 30 seconds for an event
        if (lorawan_process_timeout_ms(30000) == 0) {
            // check if a downlink message was received
            receive_length = lorawan_receive(receive_buffer, sizeof(receive_buffer), &receive_port);
            if (receive_length > -1) {
                printf("received a %d byte message on port %d: ", receive_length, receive_port);

                for (int i = 0; i < receive_length; i++) {
                    printf("%02x", receive_buffer[i]);
                }
                printf("\n");

                // the first byte of the received message controls the on board LED
                gpio_put(PICO_DEFAULT_LED_PIN, receive_buffer[0]);

                uint32_t receive_header =
                    (receive_buffer[3] << 24) |
                    (receive_buffer[2] << 16) |
                    (receive_buffer[1] << 8) |
                    receive_buffer[0];
                uint32_t receive_id = (receive_header >> 9) & 0xFFFFF;
                printf("receive message id = %d\n", receive_id);
            }
        }

        // now sleep for five minutes
        sleep_ms(300000);
    }

    return 0;
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
