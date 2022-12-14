# CMSIS DSP Modulator
## Overview
The original goal of this project was to implement a simple MODEM using the CMSIS-DSP library. The general architecture would respond to the following diagram.

```mermaid
flowchart LR
HostPC <--UART--> MODEM
subgraph EmbeddedSystem
    MODEM <--> ADC/DAC
end
ADC/DAC <--Analog--> Channel
```

However, due to a number of issues, only the modulator was implemented.

## Modem
### Modulator
The modulator is responsible for the following tasks:
- Receiving raw data and building "packets" that are comprised of three elements:
   - Message length 
   - Preamble length
   - Start of frame length 
- Generating modulated pulses from data:
  - Generating a zero padded impulse train signal from input data
  - Applying a modulation filter to the aforementioned impulse train signal

### Demodulator
The demodulator is responsible for the following tasks: 
- Applying the matched filter to the modulated signal captured by the ADC
- Detecting whether there's signal level or not
- Syncronizing the modulated input signal 
- Sampling the matched filter output signal converting it to either 1s or 0s

### Demo
![demo](images/SDC_ScopeCap.gif)
