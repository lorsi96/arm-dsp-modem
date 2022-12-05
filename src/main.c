/**
 * @file main.c
 * @author Lucas Orsi (lorsi@itba.edu.ar)
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 */

/* ************************************************************************* */
/*                             Public Inclusions                             */
/* ************************************************************************* */
#include "arm_const_structs.h"
#include "arm_math.h"
#include "sapi.h"
#include "rrc_fir.h"

/* ************************************************************************* */
/*                               Configuration                               */
/* ************************************************************************* */
#define PROG_LOOP_HZ 8000
#define PROG_FREQ_CYCLES (EDU_CIAA_NXP_CLOCK_SPEED / PROG_LOOP_HZ)

#define MODEM_NBYTES 4
#define MODEM_PRE_BITS 8
#define MODEM_SFD 1
#define MODEM_PACKET_BITS MODEM_PRE_BITS + (8 * MODEM_NBYTES)

#define MOD_SYMB_LEN_BITS 16
#define MOD_RRC_SZ  RCC_SIZE 
#define MOD_BUFFER_LEN MOD_SYMB_LEN_BITS * MODEM_PACKET_BITS
#define MOD_OUT_SYM_F_HZ PROG_LOOP_HZ / MOD_SYMB_LEN_BITS
#define MOD_FILT_DATA_SZ  MOD_BUFFER_LEN + MOD_RRC_SZ + 1

#define ADC_BUFFER_LEN 128

#define UART_BAUDRATE 460800

/* ********************************* System ******************************** */
static enum modem_error_t {
    MODEM_ERR_OK = 0,
    MODEM_BUFFER_FULL = 1,
    MODEM_MOD_GET_SAMPLE_WITHOUT_DATA_VALID = 2,
} __modem_err = MODEM_ERR_OK;

/* ******************************* Modulator ******************************* */
static struct modulator_t {
    q15_t* filter_coeffs;
    q15_t mapped_data[MOD_BUFFER_LEN];
    q15_t filtered_data[MOD_FILT_DATA_SZ];
    uint16_t buff_i;
    uint16_t out_i;
} mod;

void modulator_init(struct modulator_t* self, q15_t* filter_coeffs) {
    self->filter_coeffs = filter_coeffs;
    self->buff_i = 0;
    self->out_i = 0;
}

void modulator_data_add(struct modulator_t* self, bool bit) {
    if (self->buff_i >= MOD_SYMB_LEN_BITS) {
        __modem_err = MODEM_BUFFER_FULL;
        return;
    }
    self->mapped_data[MOD_SYMB_LEN_BITS * self->buff_i] = bit ? 0x7FFF : 0x8000;
    self->buff_i++;
}

void __modulator_filter_data(struct modulator_t* self) {
    arm_conv_q15(self->mapped_data, MOD_BUFFER_LEN, 
                 self->filter_coeffs, MOD_RRC_SZ, self->filtered_data);
}

bool modulator_is_data_valid(struct modulator_t* self) {
    return self->buff_i == MOD_SYMB_LEN_BITS;
}

q15_t modulator_get_out_sample(struct modulator_t* self) {
    q15_t ret;
    if (self->buff_i != MOD_SYMB_LEN_BITS) {
        __modem_err = MODEM_MOD_GET_SAMPLE_WITHOUT_DATA_VALID;
        return 0x00;
    }
    if (self->out_i == 0) {
        __modulator_filter_data(self);
    }
    ret = self->filtered_data[self->out_i];
    if (++self->out_i == MOD_FILT_DATA_SZ) {
        self->out_i = 0;
        self->buff_i = 0;  // Ready to receive more data.
    }
    return ret;
}

/* ******************************* ADC / DAC ******************************* */

uint16_t adc_sample = 0;
uint16_t dac_sample = 512;

/* ***************************** Data Input Sim **************************** */
bool data_in = false;
const uint16_t DATA_IN_RESET = 1024;
uint16_t data_in_count = 0;

/* ***************************** Filter Coeffs ***************************** */
q15_t sq_coeffs[] = {
    0x2000, 0x2000, 0x2000, 0x2000, 0x2000, 0x2000, 0x2000, 0x2000,
    0x2000, 0x2000, 0x2000, 0x2000, 0x2000, 0x2000, 0x2000, 0x2000,
};

/* ***************************** Data Transfer ***************************** */
static struct header_struct {
    char head[4];
    uint32_t id;
    uint16_t N;
    uint16_t fs;
    uint16_t dbg1;
    uint16_t dbg2;
    uint16_t dbg3;
    char tail[4];
} header = {"head", 0, DATA_IN_RESET, PROG_LOOP_HZ, 0, 0, 0, "tail"};

/* ************************************************************************* */
/*                                    Code                                   */
/* ************************************************************************* */

int main(void) {
    boardConfig();
    uartConfig(UART_USB, UART_BAUDRATE);
    adcConfig(ADC_ENABLE);
    dacConfig(DAC_ENABLE);
    cyclesCounterInit(EDU_CIAA_NXP_CLOCK_SPEED);
    /* Modulator init */
    modulator_init(&mod, rrc_coeffs);
    for (;;) {
        cyclesCounterReset();

        /* Send adc data through UART */
        adc_sample = (((adcRead(CH1) - 512)) << 6);
        uartWriteByteArray(UART_USB, (uint8_t*)&adc_sample, sizeof(adc_sample));

        /* Handle data in */
        data_in_count = (data_in_count + 1) % DATA_IN_RESET;
        if (data_in_count == 0) {
            data_in_count = DATA_IN_RESET;
            modulator_data_add(&mod, data_in);
            data_in = !data_in;
            uartWriteByteArray(UART_USB, (uint8_t*)&header, sizeof(header));
            header.dbg1 = 0;
            header.dbg2 = 0;
            header.dbg3 = 0;
        }

        /* Modulator */
        if (modulator_is_data_valid(&mod)) {
            header.dbg1++;
            dacWrite(DAC, (modulator_get_out_sample(&mod) >> 6) + 512);
        } else {
            header.dbg2++;
            dacWrite(DAC, 512);  // a.k.a 0
        }

        while (cyclesCounterRead() < PROG_FREQ_CYCLES)
            ;
    }
}
