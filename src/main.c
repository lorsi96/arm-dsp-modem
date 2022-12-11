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

#define DEMOD_TH_SIGNAL_LEVEL  (1<<5)

#define ADC_BUFFER_LEN 128

#define UART_BAUDRATE 460800

/* ***************************** Data Transfer ***************************** */
const uint16_t DATA_IN_RESET =1024;
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

/* ********************************* System ******************************** */
static enum modem_error_t {
    MODEM_ERR_OK = 0,
    MODEM_BUFFER_FULL = 1,
    MODEM_MOD_GET_SAMPLE_WITHOUT_DATA_VALID = 2,
    MODEM_DEMOD_BUFFER_FULL = 3,
    MODEM_DEMOD_DATA_NOT_READY = 4,
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

    /* Create Preamble and SFD.*/
    bool reverse = false;
    bool bit = true;
    for(;self->buff_i<MODEM_PRE_BITS; self->buff_i++) {
        if(self->buff_i == (MODEM_PRE_BITS - MODEM_SFD)) {
            reverse = true;
        }
        self->mapped_data[MOD_SYMB_LEN_BITS * self->buff_i] = (bit ^ reverse) ? 0x7FFF : 0x8000;  
        bit = !bit;
    }
}



void modulator_data_add(struct modulator_t* self, bool bit) {
    if (self->buff_i >= MOD_SYMB_LEN_BITS) {
        __modem_err = MODEM_BUFFER_FULL;
        return;
    }
    self->mapped_data[MOD_SYMB_LEN_BITS * self->buff_i] = bit ? 0x7FFF : 0x8000;
    self->buff_i++;
}

void modulator_send_by(struct modulator_t* self, uint8_t byte) {
    for(uint8_t i=0; i < 8; i++) {
        modulator_data_add(self, (byte >> i) & 0b1);
    }
}

void __modulator_filter_data(struct modulator_t* self) {
    arm_conv_q15(self->mapped_data, MOD_BUFFER_LEN, 
                 self->filter_coeffs, MOD_RRC_SZ, self->filtered_data);
}

bool modulator_is_data_valid(struct modulator_t* self) {
    return self->buff_i == MOD_SYMB_LEN_BITS;
}

bool modulator_is_rfd(struct modulator_t* self) {
    return self->out_i == 0;
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
        self->buff_i = MODEM_PRE_BITS;  // Ready to receive more data.
    }
    return ret;
}

/* ****************************** Demodulator ****************************** */
enum demodulator_state_t {
    DEMOD_NO_SIGNAL = 0,
    DEMOD_PREAMB = 1,
    DEMOD_SFD = 2,
    DEMOD_DATA_SAMPLING = 3,
    DEMOD_DATA_READY = 4,
};

static struct demodulator_t {
    q15_t* mf_coeffs;
    q15_t* bp_coeffs;
    q15_t window_buffer[MOD_SYMB_LEN_BITS];
    q15_t mf_data[MOD_FILT_DATA_SZ];
    q15_t pf_data[MOD_FILT_DATA_SZ];
    q15_t pf_sq_data[MOD_FILT_DATA_SZ];
    q15_t pf_sq_bp_data[MOD_FILT_DATA_SZ + MOD_RRC_SZ + 1];
    bool bits[MODEM_PACKET_BITS];
    uint16_t bit_i;
    q15_t det_th;
    uint16_t wbuff_i;
    bool out_bit;
    bool data_valid;
    enum demodulator_state_t state;
} demod = {0};


void demod_init(struct demodulator_t* self, q15_t det_th, q15_t* mf_coeffs) {
    self->mf_coeffs = mf_coeffs;
    self->bp_coeffs = mf_coeffs;
    self->det_th = det_th;
    self->wbuff_i = 0;
    self->data_valid = false;
    self->bit_i = 0;
}

bool demod_is_data_available(struct demodulator_t* self) {
    return self->data_valid;
}

bool demod_get_bit(struct demodulator_t* self) {
    if(!demod_is_data_available(self)) {
        __modem_err = MODEM_DEMOD_DATA_NOT_READY;
        return false;
    }
    self->data_valid = false;
    return self->out_bit;
}

bool __demod_is_symbol_detectable(struct demodulator_t* self) {
    q63_t pow;
    volatile q15_t min, max, minneg, absmax;
    uint32_t min_i, max_i;
    arm_conv_q15(self->window_buffer, MOD_BUFFER_LEN, 
                 self->mf_coeffs, MOD_RRC_SZ, self->mf_data);
    arm_power_q15(self->mf_data, MOD_FILT_DATA_SZ, &pow);
    arm_shift_q15(self->mf_data, 4, self->mf_data,  MOD_FILT_DATA_SZ);

    /* Estimation. */
    arm_min_q15(self->mf_data, MOD_FILT_DATA_SZ, &min, &min_i);
    arm_max_q15(self->mf_data, MOD_FILT_DATA_SZ, &max, &max_i);
    minneg = -min;
    self->out_bit = max > minneg;

    /* Detection. */
    header.dbg3 = (pow >> 32) & 0xFFFF;
    return ((pow >> 32) & 0xFFFF) > self->det_th;
}

void demod_feed_sample(struct demodulator_t* self, q15_t sample) {
    bool bit = false;
    if(self->data_valid) {
        __modem_err = MODEM_DEMOD_BUFFER_FULL;
        return;
    }

    self->window_buffer[self->wbuff_i++] = sample;
    if (self->wbuff_i == MOD_SYMB_LEN_BITS) {
        if (__demod_is_symbol_detectable(self)) {
            self->data_valid = true;
        }
        self->wbuff_i = 0; // Start capturing another window.
    }
}

/* ******************************* ADC / DAC ******************************* */

q15_t adc_sample = 0;
uint16_t dac_sample = 512;

/* ***************************** Data Input Sim **************************** */
bool data_in = false;
uint16_t data_in_count = DATA_IN_RESET;

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
    demod_init(&demod, DEMOD_TH_SIGNAL_LEVEL, rrc_coeffs);
    dacWrite(DAC, 512);  // Start at 0.
    for (;;) {
        cyclesCounterReset();

        /* Send ADC data through UART. */
        volatile uint16_t raw_adc = adcRead(CH1);
        adc_sample = (((raw_adc - 512)) << 6);
        uartWriteByteArray(UART_USB, (uint8_t*)&adc_sample, sizeof(adc_sample));

        /* Send Header & Data Packet every DATA_IN_RESET cycles. */
        data_in_count = data_in_count - 1;
        if (data_in_count == 0) {
            data_in_count = DATA_IN_RESET;
            uartWriteByteArray(UART_USB, (uint8_t*)&header, sizeof(header));
            header.dbg1 = 0;
            header.dbg2 = 0;
            header.dbg3 = 0;
        }

        /* Receive UART Requests end enqueue pulses. */
        if(modulator_is_rfd(&mod)) {
            uint8_t recv_by;
            if (uartReadByte(UART_USB, &recv_by)) {
                // header.dbg3 = recv_by;
                modulator_send_by(&mod, recv_by);
            }
        }

        /* Send pulses through DAC when modulator is ready to send. */
        if (modulator_is_data_valid(&mod)) {
            dacWrite(DAC, (modulator_get_out_sample(&mod) >> 6) + 512);
        }  // Else, line is kept at 0.


        demod_feed_sample(&demod, adc_sample);
        if(demod_is_data_available(&demod)) {  // Symbol was detected.
            header.dbg2 |= (demod_get_bit(&demod) << header.dbg1++);
        }

        while (cyclesCounterRead() < PROG_FREQ_CYCLES)
            ;
    }
}
