#pragma once
#include <stdint.h>
#include <Arduino.h>

#define DMA_ADC_BUFFER_SIZE 8 

extern uint16_t dma_adc_buffer[DMA_ADC_BUFFER_SIZE];
void dma_adc_init();
uint16_t dma_adc_get_latest_average();