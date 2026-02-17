#include "dma_manager.h"
#include "config.h"
#include <hardware/adc.h>
#include <hardware/dma.h>

uint16_t dma_adc_buffer[DMA_ADC_BUFFER_SIZE] __attribute__((aligned(16)));

static int dma_chan;

void dma_adc_init() {
    adc_init();
    adc_gpio_init(ADC_DIFFPRESS_PIN);
    adc_select_input(ADC_CHANNEL_PDUS);
    
    adc_fifo_setup(
        true,    
        true,
        1,       
        false,   
        false    
    );
    
    adc_set_clkdiv(9600); 

    dma_chan = dma_claim_unused_channel(true);
    dma_channel_config cfg = dma_channel_get_default_config(dma_chan);

    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);
    channel_config_set_ring(&cfg, true, 4);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
    channel_config_set_dreq(&cfg, DREQ_ADC);

    dma_channel_configure(
        dma_chan,
        &cfg,
        dma_adc_buffer,
        &adc_hw->fifo,     
        -1,                
        true               
    );

    adc_run(true);
}

uint16_t dma_adc_get_latest_average() {
    uint32_t sum = 0;
    for (int i = 0; i < DMA_ADC_BUFFER_SIZE; i++) {
        sum += dma_adc_buffer[i];
    }
    return (uint16_t)(sum / DMA_ADC_BUFFER_SIZE);
}