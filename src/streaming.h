#pragma once


namespace streaming
{

void test();

void init();

void set_rx_format(uint32_t sample_frequency, uint32_t bits, uint8_t channels);
void push_rx_data(size_t (*fn)(uint8_t*, size_t), size_t data_size);
void close_rx();

void print_debug_stats();

}