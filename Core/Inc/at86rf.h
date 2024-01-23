#ifndef __AT86RF_H
#define __AT86RF_H

#include "stm32l4xx_hal.h"

uint16_t options;
uint8_t seq_nr;
int frame_len;
int idle_state0;
int idle_state;

uint8_t readRegister(const uint8_t addr);

void writeRegister(const uint8_t addr,
        const uint8_t value);

uint8_t get_status(void);

void set_csma_max_retries(int8_t retries);

void set_csma_backoff_exp(uint8_t min, uint8_t max);

void set_csma_seed(uint8_t entropy[2]);

void set_option(uint16_t option, int state);

void _set_state(uint8_t state_);

void set_state(uint8_t state_);

void set_addr_short(uint16_t addr);

void set_addr_long(uint64_t addr);

void set_pan(uint16_t pan);

void set_chan(uint8_t channel);

void at86rf233_init();

void tx_prepare(void);

size_t tx_load(uint8_t *data,
         size_t len, size_t offset);

void tx_exec();

int send(uint8_t *data, size_t len);

void fb_read(uint8_t *data,
                       const size_t len);

size_t rx_len(void);

void sram_read(const uint8_t offset,
                          uint8_t *data,
                          const size_t len);

void rx_read(uint8_t *data, size_t len, size_t offset);

void at86rf2xx_receive_data();

void sleepMode(void);

#endif
