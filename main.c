/**
 * Copyright (c) 2021 Jonathan Foucher
 *
 * SPDX-License-Identifier: MIT
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define PICO_SCANVIDEO_SCANLINE_BUFFER_COUNT 64
#define INPUT_PIN_BASE 0
#define DMA_CHAN 0

#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/time.h"
#include "pico/sync.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "pico/multicore.h"

// #include "scanvideo.h"
// #include "composable_scanline.h"
#include "input.pio.h"

// If this is active, then an overclock will be applied
// #define OVERCLOCK

// Delay startup by so many seconds
#define START_DELAY 3

absolute_time_t start;

bool running = 1;
uint total_sample_bits = 14;
uint buf_size_words;

uint16_t *capture_buf;

void core1_func();
static semaphore_t video_initted;

static void __time_critical_func(input_received)() {
    printf(".");
}

static void __time_critical_func(dma_handler)() {
    dma_hw->ints0 = 1u << DMA_CHAN;
    // pio_sm_set_enabled(pio1, 0, false);
    // pio_sm_clear_fifos(pio1, 0);
    // // pio_sm_exec(pio1, 0, pio_encode_wait_gpio(0, 5));
    // pio_sm_set_enabled(pio1, 0, true);
    printf("0x%X\n", (capture_buf[0]>>2) & 0xFF);
    // printf("%04X\n", *(capture_buf + sizeof(uint16_t)));

    dma_channel_set_write_addr(DMA_CHAN, capture_buf, true);
}

void init_input() {
    // gpio_init(5);
    // gpio_pull_up(5);
    // gpio_set_dir(5, 0);
    // gpio_init(4);
    // gpio_set_dir(3, 0);
    // gpio_init(2);
    // gpio_set_dir(2, 0);

    PIO pio = pio1;
    uint offset = pio_add_program(pio, &input_program);
    uint sm = 0;
    input_program_init(pio, sm, offset, INPUT_PIN_BASE);

    // dma_channel_config c = dma_channel_get_default_config(DMA_CHAN);
    
    // total_sample_bits += bits_packed_per_word(14) - 1;
    // buf_size_words = total_sample_bits / bits_packed_per_word(14);

    // printf("buf_size_words %d", buf_size_words);

    // capture_buf = malloc(buf_size_words * sizeof(uint16_t));

    // channel_config_set_read_increment(&c, false);
    // channel_config_set_write_increment(&c, true);
    // channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    // channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));
    // dma_channel_configure(DMA_CHAN, &c,
    //     capture_buf,
    //     &pio->rxf[sm],
    //     buf_size_words,
    //     true
    // );

    // dma_channel_set_irq0_enabled(DMA_CHAN, true);

    // irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    // irq_set_enabled(DMA_IRQ_0, true);

    pio_sm_set_enabled(pio, sm, true);
}

static unsigned char lookup[16] = {
0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe,
0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf, };

uint8_t reverse(uint8_t n) {
   // Reverse the top and bottom nibble then swap them.
   return (lookup[n & 0xF] << 4) | lookup[n>>4];
}

char buffer[33];

int main() {
#ifdef OVERCLOCK
    vreg_set_voltage(VREG_VOLTAGE_1_20);
    set_sys_clock_khz(240000, true);
#endif
    stdio_init_all();
    // time_init();


#ifdef START_DELAY
    for(uint8_t i = START_DELAY; i > 0; i--) {
        printf("Starting in %d \r\n", i);
        sleep_ms(1000);
    }
#endif
    //printf("Starting\n");

    // gpio_init(PICO_DEFAULT_LED_PIN);
    // gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    // gpio_put(PICO_DEFAULT_LED_PIN, 1);

    // Load the ps2 program, and configure a free state machine
    // to run the program.
    PIO pio = pio1;

    uint sm = 0;

    init_input();

    printf("ready\r\n");



    //hookexternal(callback);

    //dma_handler();

    while(running) {
        if (pio_sm_is_rx_fifo_empty(pio, sm) == false) {
        //     // Get data from 6502
        
            uint32_t val = pio_sm_get(pio, sm);
            uint8_t t = ((val) >> 4);
            printf("%02X\n", reverse(t));
        }
    }

    return 0;
}



void core1_func() {
    // initialize video and interrupts on core 1
    // scanvideo_setup(&vga_mode);
    // scanvideo_timing_enable(true);
    sem_release(&video_initted);

    // while (true) {
    //     scanvideo_scanline_buffer_t *scanline_buffer = scanvideo_begin_scanline_generation(true);

    //     vic_draw_color_bar(scanline_buffer);
    //     scanvideo_end_scanline_generation(scanline_buffer);
    // }
}
