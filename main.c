/**
 * Copyright (c) 2021 Jonathan Foucher
 *
 * SPDX-License-Identifier: MIT
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h> 

#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/time.h"
#include "pico/sync.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "pico/multicore.h"

#define vga_mode vga_mode_320x240_60
#define PICO_SCANVIDEO_SCANLINE_BUFFER_COUNT 64
#define INPUT_PIN_BASE 0
#define RW_PIN 22
// #define DMA_CHAN 0
#define PICO_SCANVIDEO_COLOR_PIN_BASE 14
#define PICO_SCANVIDEO_SYNC_PIN_BASE 26

#define VIDEO_BASE 0
#define VIDEO_CTRL VIDEO_BASE
#define VIDEO_ADDR_LOW (VIDEO_BASE + 1) 
#define VIDEO_ADDR_HIGH (VIDEO_BASE + 2)
#define VIDEO_DATA (VIDEO_BASE + 3)
#define VIDEO_IEN (VIDEO_BASE + 4)
#define VIDEO_INTR (VIDEO_BASE + 5)
#define VIDEO_HSCROLL = (VIDEO_BASE + 6)
#define VIDEO_VSCROLL = (VIDEO_BASE + 7)

#define MODE_CHAR 1
#define MODE_PIXEL 0

#include "scanvideo.h"
#include "composable_scanline.h"
#include "input.pio.h"
#include "font.h"

// If this is active, then an overclock will be applied
#define OVERCLOCK

// Delay startup by so many seconds
#define START_DELAY 3
#define DISPLAY_WIDTH 320
#define DISPLAY_HEIGHT 240
#define BUF_LENGTH_PIXEL_MODE  (DISPLAY_WIDTH * DISPLAY_HEIGHT)
#define BUF_LENGTH_CHAR_MODE  ((DISPLAY_WIDTH / 8) * (DISPLAY_HEIGHT / 8))

absolute_time_t start;

// uint8_t font[128*8];

bool running = 1;
uint total_sample_bits = 14;
uint buf_size_words;

uint16_t *capture_buf;

void core1_func();
static semaphore_t video_initted;
uint8_t* px_buf;
uint16_t multiplier=1;
uint8_t pxbuf[DISPLAY_WIDTH * DISPLAY_HEIGHT];
uint8_t mode = MODE_PIXEL;

PIO pio;
uint sm;
uint write_address=0;
uint8_t fg_color = 0xFF;
uint8_t bg_color = 0;

void video_init(uint8_t* buf) {
    px_buf = buf;
    multiplier = vga_mode.width / DISPLAY_WIDTH;
}

// static void __time_critical_func(input_received)() {
//     printf(".");
// }

// static void __time_critical_func(dma_handler)() {
//     dma_hw->ints0 = 1u << DMA_CHAN;
//     // pio_sm_set_enabled(pio1, 0, false);
//     // pio_sm_clear_fifos(pio1, 0);
//     // // pio_sm_exec(pio1, 0, pio_encode_wait_gpio(0, 5));
//     // pio_sm_set_enabled(pio1, 0, true);
//     printf("0x%X\n", (capture_buf[0]>>2) & 0xFF);
//     // printf("%04X\n", *(capture_buf + sizeof(uint16_t)));

//     dma_channel_set_write_addr(DMA_CHAN, capture_buf, true);
// }

void init_input() {
    gpio_init(RW_PIN);
    gpio_pull_down(RW_PIN);
    gpio_set_dir(RW_PIN, 0);

    uint offset = pio_add_program(pio, &input_program);
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
uint increment = 1;

int main() {

#ifdef OVERCLOCK
    vreg_set_voltage(VREG_VOLTAGE_1_15);

    if (!set_sys_clock_khz(250000, false)) {
        printf("overclock fail \n");
    }
#endif
    stdio_init_all();
// #ifdef START_DELAY
//     for(uint8_t i = START_DELAY; i > 0; i--) {
//         printf("Starting in %d \r\n", i);
//         sleep_ms(1000);
//     }
// #endif
    // for (int i = 0; i < 128*8; i++) {
    //     font[i] = font8x8[i];
    // }

    //printf("Starting\n");

    // gpio_init(PICO_DEFAULT_LED_PIN);
    // gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    // gpio_put(PICO_DEFAULT_LED_PIN, 1);

    

    pio = pio1;

    sm = pio_claim_unused_sm(pio, true);

    init_input();

    //printf("ready\r\n");
    mode = MODE_CHAR;

    for (int i = 0; i < DISPLAY_WIDTH * DISPLAY_HEIGHT; i++) {
        if (mode==MODE_PIXEL) {
            pxbuf[i] = 0;
        } else {
            pxbuf[i] = 0x20;
        }
        
    }
    // pxbuf[0] = 0x43;
    // pxbuf[1] = 0x42;
    // pxbuf[2] = 0x41;
    // pxbuf[200] = 0x41;
    

    video_init(pxbuf);

    sem_init(&video_initted, 0, 1);

    multicore_launch_core1(core1_func);

    sem_acquire_blocking(&video_initted);

    //hookexternal(callback);

    //dma_handler();


    while(running) {
        if (pio_sm_is_rx_fifo_empty(pio, sm) == false) {
            // Get data from 6502

            uint32_t val = pio_sm_get(pio, sm);
            uint8_t reg = lookup[(val) & 0xF];
            uint8_t data = reverse((val >> 4) & 0xFF);
            //printf("%02X ", reg);
            if (reg == VIDEO_CTRL) {
                mode = data & 1;
                increment = ((int8_t) data) >> 2;
            } else if (reg == VIDEO_ADDR_LOW) {
                write_address = (write_address & 0xFFE0) | (data & 0x1F);
            } else if (reg == VIDEO_ADDR_HIGH) {
                write_address = (write_address & 0x00FF) | (data << 5);
            } else if (reg == VIDEO_DATA) {
                if (mode == MODE_CHAR && write_address == 0x1FFE) {
                    bg_color = reverse(data);
                } else if (mode == MODE_CHAR && write_address == 0x1FFF) {
                    fg_color = reverse(data);
                } else {
                    pxbuf[write_address] = data;
                }
                
                write_address += increment;
                uint l = mode == MODE_CHAR ? BUF_LENGTH_CHAR_MODE : BUF_LENGTH_PIXEL_MODE;
                if (write_address > l) {
                    write_address = 0;
                }
            }
        
        }
    }

    return 0;
}


void __time_critical_func(draw_color_bar)(scanvideo_scanline_buffer_t *buffer) {
    uint line_num = scanvideo_scanline_number(buffer->scanline_id);
    uint line_index = (line_num * DISPLAY_HEIGHT) / vga_mode.height;
    uint16_t *p = (uint16_t *) buffer->data;

    uint dp = line_index * DISPLAY_WIDTH;
    uint dpc = (line_index /8) * (DISPLAY_WIDTH /8);
    uint dpn = line_index & 0x7;
    
    uint16_t color = px_buf[dp];
    uint16_t old_color = px_buf[dp];
    if (mode == MODE_CHAR) {
        color = 0;
        old_color = 0;
    }
    uint16_t color_count = 0;
    uint16_t tokens = 0;

    // *p++ = COMPOSABLE_COLOR_RUN;
    // *p++ = (uint16_t) color;
    // *p++ = (uint16_t) ((vga_mode.width - DISPLAY_WIDTH)/2 - 3);
    // tokens += 3;

    //printf("dpc %d\n", dpc);
    for (uint px = 0; px < DISPLAY_WIDTH; px++) {
        if (mode == MODE_PIXEL) {
            color = px_buf[dp + px];
        } else {
            uint8_t character = px_buf[dpc + (px >> 3)];
            //printf(" %0d", (character << 3) + dpn);
            uint8_t char_line_data = font8x8[8 * character + dpn];
            //printf(" %02X", char_line_data);
            uint8_t p = px & 0x7;
            uint8_t b = (char_line_data) & (1 << p);
            color = b ? fg_color : bg_color;
        }
        
        if (color == old_color) {
            color_count += multiplier;
        } else {
            color_count += multiplier;
            //write old_color color_count times
            //printf("cnt %ld px %ld color %ld\n", color_count, px, old_color);
            if (color_count == 1) {
                *p++ = COMPOSABLE_RAW_1P;
                *p++ = (uint16_t) old_color;
                tokens += 2;
            } else if (color_count == 2) {
                *p++ = COMPOSABLE_RAW_2P;
                *p++ = (uint16_t) old_color;
                *p++ = (uint16_t) old_color;
                tokens += 3;
            } else if (color_count >= 3) {
                *p++ = COMPOSABLE_COLOR_RUN;
                *p++ = (uint16_t) old_color;
                *p++ = (uint16_t) (color_count - 3);
                tokens += 3;
            }
            old_color = color;
            color_count = 0;
        }
    }

    // Write last color
    if (color_count == 1) {
        *p++ = COMPOSABLE_RAW_1P;
        *p++ = (uint16_t) color;
        tokens += 2;
    } else if (color_count == 2) {
        *p++ = COMPOSABLE_RAW_2P;
        *p++ = (uint16_t) color;
        *p++ = (uint16_t) color;
        tokens += 3;
    } else if (color_count >= 3) {
        *p++ = COMPOSABLE_COLOR_RUN;
        *p++ = (uint16_t) color;
        *p++ = (uint16_t) (color_count - 3);
        tokens += 3;
    }

    // Write last part of border
    // *p++ = COMPOSABLE_COLOR_RUN;
    // *p++ = (uint16_t) color;
    // *p++ = (uint16_t) ((vga_mode.width - DISPLAY_WIDTH*multiplier)/2 - 4);
    // tokens += 3;
    // black pixel to end line
    *p++ = COMPOSABLE_RAW_1P;
    *p++ = 0;
    tokens += 2;
    // // end of line with alignment padding
    if (tokens % 2 == 0) {
        *p++ = COMPOSABLE_EOL_SKIP_ALIGN;
        *p++ = 0;
    } else {
        *p++ = COMPOSABLE_EOL_ALIGN;
        // *p++ = 0;
    }


    buffer->data_used = ((uint32_t *) p) - buffer->data;
    assert(buffer->data_used < buffer->data_max);

    buffer->status = SCANLINE_OK;
}

void core1_func() {
    // initialize video and interrupts on core 1
    scanvideo_setup(&vga_mode);
    scanvideo_timing_enable(true);
    sem_release(&video_initted);
    // printf("core1 ready");
    while (true) {
        scanvideo_scanline_buffer_t *scanline_buffer = scanvideo_begin_scanline_generation(true);

        draw_color_bar(scanline_buffer);
        scanvideo_end_scanline_generation(scanline_buffer);
    }
}


