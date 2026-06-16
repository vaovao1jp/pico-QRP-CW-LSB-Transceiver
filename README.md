# pico_40m_CW_LSB_v1_1EN

40m band CW/LSB dual-mode transceiver (for Raspberry Pi Pico2)

---

## Overview

Firmware for a homebrew 7MHz (40m) band transceiver that leverages the dual cores and the
built-in ADC/PWM of the Raspberry Pi Pico (RP2040). A single unit can switch between CW
(Morse) and LSB (lower-sideband SSB) transmit/receive operation. Reception uses an SDR
(software-defined radio) approach, with a band scope, waterfall, and waveform display on an OLED.

- Designer callsign: **JR3XNW**
- Target MCU: Raspberry Pi Pico2 (RP2350, dual-core Cortex-M0+)
- Display: SSD1306 128×64 OLED (I2C)
- Frequency synthesis: Si5351 (I2C, 3 outputs CLK0/CLK1/CLK2)

---

## Main features

| Feature | Description |
|---------|-------------|
| CW receive | IQ mixer + Hilbert transform + narrow IIR BPF (700Hz center, ±100Hz) |
| CW transmit | Supports electronic keyer / straight key; keying via Si5351 CLK2 ON/OFF |
| CW auto-decode | Converts received CW to characters and shows them at the bottom of the screen |
| LSB receive | Tayloe detector (IQ mixer) + HPF 500Hz (2nd-order) + 4th-order Butterworth LPF 2400Hz (500–2400Hz band) |
| LSB transmit | Si5351 phase-modulation method (same principle as uSDX/QCX-SSB); transmits microphone audio |
| Band scope | ±15kHz spectrum display via IQ-FFT |
| Waterfall | Time-history display of the spectrum |
| Waveform display | Bottom row shows the demodulated audio on LSB receive, and the microphone input waveform on LSB transmit |
| S-meter | Bar display of the band scope peak power |
| Settings storage | Frequency, step, WPM, keyer mode, volume, and operating mode saved to EEPROM |

---

## Hardware

### Pin assignment

| GPIO | Function |
|------|----------|
| 0, 1 | Rotary encoder A/B |
| 2 | STEP_BUTTON (frequency step / long press: WPM setting) |
| 3 | KEY_MODE_BUTTON (keyer mode toggle / long press: volume) |
| 6 | PADDLE_DOT (dot paddle, CW) |
| 7 | PADDLE_DASH (dash paddle / straight key, CW) |
| 8 | MODE_BUTTON (CW/LSB mode switch) |
| 9 | PTT_BUTTON (LSB push-to-talk) |
| 15 | RX_SW (TX/RX switch, High=TX) |
| 16 | speakerPin (PWM audio output, 44100Hz) |
| 17 | PA_BIAS_PIN (PA bias control PWM, LSB TX amplitude control) |
| 25 | LED_INDICATOR (processing indicator) |
| 26 | I-ch analog input (ADC0) |
| 27 | Q-ch analog input (ADC1) |
| 28 | MIC_IN (microphone input, ADC2, for LSB TX) |

### Peripheral circuit assumptions

- **Receive front-end**: A Tayloe detector (IQ mixer) generates the I/Q baseband signals, fed into GPIO26/27.
- **Si5351**: CLK0/CLK1 are used as the receive LO (90-degree phase difference), and CLK2 as the transmit carrier.
- **PA (power amplifier)**: The GPIO17 PWM is used for the gate bias, providing amplitude control proportional to the audio amplitude (carrier-suppressed SSB) during LSB transmit.
- **Microphone**: An electret microphone, etc., connected to GPIO28 (its output is weak, so adjust with `MIC_GAIN`).

---

## Software architecture (dual-core)

This firmware divides the work between the two RP2040 cores.

### Core0 (`loop()`) — UI / control / centralized I2C management
- Button input handling, mode switching, frequency setting
- FFT computation and band scope / waterfall / waveform drawing
- **Centralizes all I2C access (OLED drawing + Si5351 frequency updates)**

### Core1 (`loop1()`) — audio / DSP
- Samples I/Q via ADC at 40kHz → filter → AGC → PWM audio output (receive)
- During LSB transmit, samples the microphone at 4kHz and computes the instantaneous frequency via the Hilbert transform
- **Never calls I2C**. The transmit frequency is passed to Core0 via `txFreqShared`

> **Key design point**: Initially Core1 controlled the Si5351 directly over I2C, but because `Wire` is
> initialized on Core0 this was unstable and caused in-band noise to remain after transmission.
> This is resolved by completely removing I2C from Core1 and concentrating it on Core0.
> `volatile` is mandatory for the inter-core shared variables (especially `transmitting`).

---

## Signal processing details

### CW receive
- An IQ Hilbert transform (Q delayed by 14 samples) generates the 90-degree phase difference around 700Hz
- LSB side is selected with `I − Q_delayed`
- A 2-stage cascaded IIR 2nd-order BPF (700Hz center, ±100Hz). The band is set somewhat wide to tolerate the Si5351 frequency error (±10ppm ≈ ±70Hz)

### LSB receive
- The Tayloe detector's I/Q has a 90-degree phase difference at all audio frequencies, so LSB is selected by direct subtraction `I − Q`
- Q is corrected with `lsbQGain` (the I/Q amplitude ratio computed by Core0 from the FFT frame) to improve image rejection
- Soft clipper → HPF 500Hz (2nd-order) → LPF 2400Hz (4th-order); the high side has a steep 24dB/oct rolloff that greatly reduces out-of-band noise

### LSB transmit (uSDX/QCX-SSB method)
1. Sample the GPIO28 microphone at 4kHz; remove DC offset, apply gain, soft clip
2. Generate I/Q with a 15-tap FIR Hilbert transform
3. Obtain the instantaneous phase with `atan2`, convert the phase difference → instantaneous frequency deviation [Hz]
4. Write the target frequency to `txFreqShared`; Core0 updates Si5351 CLK2 every 250µs
5. At the same time, control the PA bias PWM from the I/Q vector magnitude (alpha-max plus beta-min approximation) to achieve carrier-suppressed SSB

### AGC
- Uses independent gain variables/parameters for CW and LSB (prevents excessive gain when switching modes)
- CW: maxGain=40, fast attack / slow decay
- LSB: maxGain=10, fast response (prevents ADC saturation distortion on strong signals)

---

## Operation

| Action | Behavior |
|--------|----------|
| Rotate the rotary encoder | Move the frequency up/down by STEP (7.000–7.200MHz) |
| MODE_BUTTON short press | Toggle CW ⇔ LSB (corrects FREQ by ±700Hz so the same station is always heard at 700Hz) |
| STEP_BUTTON short press | Toggle frequency step (1kHz → 100Hz → 10Hz) |
| STEP_BUTTON long press | WPM setting mode |
| KEY_MODE_BUTTON short press (in CW) | Toggle electronic keyer ⇔ straight key |
| KEY_MODE_BUTTON long press | Volume setting mode (×1.0 to ×3.0) |
| Paddle (in CW) | CW transmit |
| PTT_BUTTON (in LSB) | LSB transmit while held (push-to-talk) |

---

## Screen layout (128×64 OLED)

```
y= 0  Frequency display / STEP / S-meter frame / mode (CW/LSB)
y=26  ───────── Scope bottom-edge line ─────────
y=27  Band scope (IQ-FFT spectrum, ±15kHz)
y=48  ───────── Waterfall bottom-edge line ─────────
y=27–47  Waterfall
y=49  Frequency scale (left / center / right)
y=54–63  CW decoded text (in CW) / audio waveform (in LSB)
```

---

## Key tuning parameters

Most constants are `#define`s at the top of the `.ino` (only `WAVE_SCALE` is a local constant
inside `displayLsbWaveform()`). The values currently set in the source are shown as the "current value".

| Constant | Current value | Description |
|----------|---------------|-------------|
| `MIC_GAIN` | 10.0f | Microphone input gain factor (match to the mic circuit) |
| `PA_TX_DRIVE` | 2 | PA drive 0–8 (higher = more output, PA protection required) |
| `PA_BIAS_SCALE` | 2.5f | Amplitude→PWM duty scale (match to the PA circuit) |
| `CW_PA_BIAS_LEVEL` | 4095 | PA bias during CW transmit (fixed full bias) |
| `LSB_AGC_MAX_GAIN` | 10.0f | LSB receive AGC maximum gain |
| `CW_DETECT_THRESHOLD` | 1.5f | CW auto-decode detection sensitivity (SNR threshold) |
| `SCOPE_SPAN_HZ` | 15000.0f | Band scope horizontal display range (±Hz) |
| `SCOPE_SENSITIVITY` | 16.0f | Band scope / waterfall amplitude gain |
| `SCOPE_OFFSET` | 3.0f | Band scope / waterfall noise-floor lift amount |
| `WAVE_SCALE` (in function) | 8.0f | Audio waveform display sensitivity |

> ⚠ **For the transmit-power constants (`PA_TX_DRIVE`, `PA_BIAS_SCALE`), raise them gradually while checking distortion on the actual hardware so as not to exceed the PA rating.**

### Band scope / waterfall drawing adjustment

The band scope and waterfall share the same amplitude data (`waterfallHistory`), so changing the
three constants below alters the appearance of both at once.

- **`SCOPE_SPAN_HZ`** (default 15000.0f): the frequency range shown on the horizontal axis (±Hz).
  Smaller zooms in to separate close signals; larger gives a band overview.
- **`SCOPE_SENSITIVITY`** (default 16.0f): the vertical amplitude gain. Larger draws weak signals
  taller but also darkens the noise floor and the waterfall background. Adjust if signals clip or are too low.
- **`SCOPE_OFFSET`** (default 3.0f): the noise-floor lift amount, used in `barLength()` as
  `log10(amplitude)+OFFSET`. Smaller lowers the noise-floor display and reduces waterfall background noise.
- **`THRESHOLD`** (default 2, `#define`): the amplitude threshold for plotting a waterfall dot. Larger
  hides weak signals for a cleaner background; smaller shows weak signals but more noise.
- **`WATERFALL_HEIGHT`** (default 21): the vertical pixel count of the waterfall (length of time history).

### Microphone sensitivity (LSB transmit) adjustment

- **`MIC_GAIN`** (default 10.0f): the microphone input amplification factor; match it to the mic circuit.
  Guideline: amplified mic module = 5–10, bare electret mic direct = 20–30, low-sensitivity mic = 40–50.
  Too small gives shallow modulation (weak transmit); too large distorts.
- During transmit the mic waveform is shown in the bottom row, so set a value that swings appropriately.

### PA (power amplifier) related adjustment

- **`PA_TX_DRIVE`** (default 2, range 0–8): the transmit power step; each sample is left-shifted by 2^drive.
  1–2 = QRP (minimal distortion), 3–4 = low power, 5–6 = high power, 7–8 = maximum (PA protection mandatory).
- **`PA_BIAS_SCALE`** (default 2.5f): the scale factor converting the instantaneous mic amplitude into PA
  gate bias (PWM duty 0–4095). Smaller = less output (PA protection); larger = more output (watch the rating).
  Adjust from around 2.5f while checking distortion on the actual PA.
- **`CW_PA_BIAS_LEVEL`** (default 4095): the fixed PA bias during CW transmit. CW needs no amplitude
  modulation, so the PA is kept at full bias and RF ON/OFF is done via Si5351 CLK2. No effect if GPIO17 is unconnected.
- **`PA_BIAS_ENV_ATTACK` / `PA_BIAS_ENV_DECAY`**: unused, because the current transmit path converts
  amplitude directly to PWM (kept as reference values in case envelope smoothing is added later).

### CW auto-decode sensitivity adjustment

- **`CW_DETECT_THRESHOLD`** (default 1.5f): a key-down is registered when the received SNR ratio
  (`cwEnvelope`) exceeds this value. Smaller picks up weaker signals but increases false detection (noise);
  larger is safer but drops characters more easily. Recommended range 1.5–2.5.

### AGC (receive gain) adjustment

- **CW**: in `applyAGC()`, `maxGain=40`, `targetAmp=0.5`, `attackRate=0.01`, `decayRate=0.001`.
  Configured to amplify weak signals as much as possible.
- **LSB**: `LSB_AGC_MAX_GAIN=10`, `LSB_AGC_TARGET=0.35`, `LSB_AGC_ATTACK=0.08`, `LSB_AGC_DECAY=0.002`.
  A lower max gain and faster attack than CW prevent ADC saturation distortion on strong signals.
  Raise `maxGain` if receive audio is too quiet; lower it if strong signals distort.

### Other adjustment constants

- **`WAVE_SCALE`** (in `displayLsbWaveform()`, default 8.0f): the sensitivity of the bottom-row audio
  waveform. 3.0 = swings only on loud/strong signals, 8.0 = appropriate at normal conversation, 15.0 =
  swings even on weak signals (noise visible too). `WAVE_MAX_AMP` (default 3px) limits the swing height.
- **`LSB_SOFT_CLIP_THRESH`** (default 0.7f): the soft-clip threshold for the LSB receive filter input,
  preventing the IIR filter from oscillating on strong signals. Lower it if it distorts.
- **`DEFAULT_WPM`** (default 20): the keyer's initial WPM (the EEPROM-saved value takes priority in practice).
- **`LOW_FREQ` / `HI_FREQ`** (default 7000000 / 7200000): the VFO tuning range (Hz).
- **`CW_TONE` / `CW_AUDIO_OFFSET`** (default 700Hz / 70000 centihz): the CW monitor tone and beat frequency.
  Change both together to shift the CW pitch.

---

## How to build

### Required development environment
- Arduino IDE (or arduino-cli)
- **arduino-pico** board package (Earle Philhower version, for Raspberry Pi Pico2)

### Required libraries
| Library | Source |
|---------|--------|
| Rotary | https://github.com/brianlow/Rotary |
| U8g2 | https://github.com/olikraus/U8g2_Arduino |
| arduinoFFT | arduinoFFT v2.0.4 |
| Si5351 (Etherkit) | https://github.com/etherkit/Si5351Arduino |
| EEPROM / Wire | Bundled with the Pico board package |

### Steps
1. Install arduino-pico in the Arduino IDE
2. Add the libraries above via the Library Manager, etc.
3. Select "Raspberry Pi Pico2" as the board
4. Open `pico_40m_CW_LSB_v1_1EN.ino` and upload

---

## File structure

```
pico_40m_CW_LSB_v1_1J/
├── pico_40m_CW_LSB_v1_1J.ino   Source with Japanese comments
└── ドキュメント_日本語.md       Documentation (Japanese)

pico_40m_CW_LSB_v1_1EN/
├── pico_40m_CW_LSB_v1_1EN.ino  Source with English comments
└── documentation_EN.md         This document (English)
```

---

## License / acknowledgments

- The LSB transmit method is a port to the Raspberry Pi Pico of the Si5351 phase-modulation method of **uSDX / QCX-SSB** (Guido PE1NNZ and others).
- The libraries used are subject to the rights of their respective authors.

---

*Author: JR3XNW — 40m CW/LSB Transceiver firmware v1.1*
