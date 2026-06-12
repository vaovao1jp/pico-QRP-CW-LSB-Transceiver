/*
  ============================================================
  pico_40m_CW_LSB_v1_1EN.ino
  40m CW/LSB Dual-Mode Transceiver
  For Raspberry Pi Pico2
  ============================================================

  [Basic specifications]
  - CW mode  : IQ band scope + keyer + CW auto-decode + waterfall
  - LSB mode : SSB transmit/receive (Si5351 phase modulation TX / Hilbert transform RX)
  - Mode switch : MODE_BUTTON (button on the rotary encoder) short press

  [Pin assignment]
  GPIO 0,1  : Rotary encoder A/B
  GPIO 2    : STEP_BUTTON  (frequency step / WPM setting)
  GPIO 3    : KEY_MODE_BUTTON (keyer mode / volume)
  GPIO 6    : PADDLE_DOT
  GPIO 7    : PADDLE_DASH
  GPIO 8    : MODE_BUTTON  (CW/LSB mode switch)
  GPIO 9    : PTT_BUTTON   (LSB push-to-talk)
  GPIO 15   : RX_SW (TX/RX switch, High=TX)
  GPIO 16   : speakerPin (PWM audio output)
  GPIO 17   : PA_BIAS_PIN (PA bias control PWM, LSB TX amplitude control)
  GPIO 25   : LED_INDICATOR
  GPIO 26   : I-ch analog input (ADC0)
  GPIO 27   : Q-ch analog input (ADC1)
  GPIO 28   : MIC_IN  microphone input (ADC2)

  [LSB TX method]
  The same Si5351 phase-modulation method as uSDX/QCX-SSB, ported to the Pico.
  The microphone audio is processed with a Hilbert transform to obtain I/Q, the phase
  difference is converted into an instantaneous frequency deviation, and CLK2 is updated
  at a 4kHz rate to generate the LSB carrier.

  [Libraries used]
  Rotary.h     : https://github.com/brianlow/Rotary
  U8g2lib.h    : https://github.com/olikraus/U8g2_Arduino
  arduinoFFT.h : v2.0.4
  si5351.h     : https://github.com/etherkit/Si5351Arduino
  EEPROM.h     (built-in for Pico)
  Wire.h
*/

#include <Arduino.h>
#include <Rotary.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <arduinoFFT.h>
#include <si5351.h>
#include <EEPROM.h>

// --- Forward declarations (declared explicitly rather than relying on the Arduino IDE auto-generation) ---
void Freq_Set();
void Freq_Set_CW();
void Freq_Set_LSB();
void startTransmit();
void stopTransmit();
void handleKeyer();
void handleLsbPTT();
void handleModeButton();
void handleKeyModeButton();
void Fnc_Stp();
void handleCWDecoder();
void displayCWText();
void displayLsbWaveform();
void handleLsbTxCore1();
void handleRxCore1(float& lastRxPwm);
float applyLsbAGC(float input);
inline float lsbSoftClip(float x);
void updateCwDetector(float demodulated);
void addCWDecodedChar(char c);
char decodeMorse(const char* code);

// ============================================================
// [1] Pin definitions and basic constants
// ============================================================

// --- Analog input pins ---
#define I_IN              26   // I-ch analog input (ADC0)
#define Q_IN              27   // Q-ch analog input (ADC1)
#define MIC_IN            28   // Microphone input (ADC2) *for LSB TX
#define inputPinI         26
#define inputPinQ         27

// --- Digital pins ---
#define speakerPin        16   // PWM audio output
#define PIN_IN1           0    // Rotary encoder A
#define PIN_IN2           1    // Rotary encoder B
#define STEP_BUTTON       2    // Frequency step / WPM setting (long press)
#define KEY_MODE_BUTTON   3    // Keyer mode / volume adjust (long press)
#define MODE_BUTTON       8    // CW/LSB mode switch button
#define PTT_BUTTON        9    // LSB PTT button (push-to-talk)
#define PADDLE_DOT        6    // Dot paddle (CW mode)
#define PADDLE_DASH       7    // Dash paddle / straight key (CW mode)
#define PA_BIAS_PIN       17   // *PA bias control PWM output (GPIO17, amplitude control for LSB TX)
                               //   Shares the same PWM slice (slice0) as GPIO16 (speakerPin).
                               //   The duty cycle can be controlled independently while the 44100Hz frequency is shared.
                               //   By controlling the PA final-stage gate bias with this PWM,
                               //   RF output amplitude control proportional to the audio amplitude (= carrier-suppressed SSB) is achieved.
#define RX_SW             15   // TX/RX switch (High=TX)
#define LED_INDICATOR     25   // Processing indicator LED

// --- SDR / signal processing ---
#define SAMPLES           256      // FFT sample count
#define sampleRate        40000    // Effective sampling frequency [Hz]
#define pwmFrequency      44100    // PWM carrier frequency [Hz]
#define CW_AUDIO_OFFSET   70000ULL // BFO offset in CW mode [centihz/100 -> Hz = 700Hz]
#define CW_TONE           700      // CW monitor tone frequency [Hz]

// --- LSB TX settings ---
// Update Si5351 CLK2 every 250us (4kHz) per sample.
// One update at 400kHz I2C takes ~180us, so 250us provides sufficient margin.
#define LSB_TX_SAMPLE_RATE   4000  // LSB TX sampling rate [Hz]
#define LSB_TX_INTERVAL_US   250   // LSB TX sampling interval [us]
#define LSB_TX_HILBERT_TAPS  15    // Hilbert transform FIR tap count

// --- Microphone gain and PA drive ---
// The output of an electret microphone is very weak (+-10 to 50mV).
// In uSDX, MORE_MIC_GAIN gives 2x amplification + the drive parameter provides 0-8 step gain adjustment.
// On the Pico, the same accurate flow of microphone input -> amplitude calculation -> PA gain reflection is essential.
//
// * Adjust here to match the actual microphone circuit *
//   MIC_GAIN:
//     5-10:   when using an amplified microphone module
//     20-30:  when connecting a bare electret microphone directly
//     40-50:  when using a low-sensitivity microphone
//
//   PA_TX_DRIVE: PA maximum output power (equivalent to the uSDX drive parameter)
//     1-2:   QRP (low power, minimal distortion)
//     3-4:   low power
//     5-6:   high power (stronger RF output)
//     7-8:   maximum power (PA protection circuit is mandatory)
//
#define MIC_GAIN             10.0f  // Microphone input gain factor (current value 10.0 = high-sensitivity; uSDX standard is 2.0)
#define PA_TX_DRIVE           2     // PA drive 0-8 (current value 2 = QRP; left-shifts each sample by 2^drive)

// --- PA bias control (GPIO17 PWM) ---
// During LSB transmit, handleLsbTxCore1() converts the instantaneous audio amplitude
// (I/Q magnitude) directly into PA gate bias (PWM duty), achieving carrier suppression
// and amplitude control.
//
// [PA_BIAS_SCALE]: scale factor from amplitude to PWM duty (used by the transmit path)
//   * Adjust here to match the PA circuit *
//   Smaller value: less RF output power (PA protection, prevents overdrive)
//   Larger value:  more RF output power (be careful not to exceed the PA maximum rating)
//   Recommended initial value: 2.5f -> adjust while checking distortion on the actual PA
//
// [PA_BIAS_ENV_ATTACK / PA_BIAS_ENV_DECAY]
//   Envelope-follower coefficients. The current transmit path converts amplitude
//   directly to PWM, so these are unused (kept as reference values in case envelope
//   smoothing is added later).
//
// PWM output: 12bit (0-4095), 44100Hz
//   0    = PA fully OFF (carrier suppression, when silent / during RX)
//   4095 = maximum bias (full power)
#define PA_BIAS_SCALE        2.5f   // *Adjust to match the PA circuit (amplitude->PWM scale)
#define PA_BIAS_ENV_ATTACK   0.30f  // Unused (reference: envelope rise coefficient)
#define PA_BIAS_ENV_DECAY    0.008f // Unused (reference: envelope fall coefficient)

// PA bias level during CW TX (fixed value)
// In CW mode, audio modulation is not needed, so the PA is kept at full bias at all times,
// and RF output ON/OFF is controlled by enabling/disabling Si5351 CLK2.
// Adjust within a range that does not exceed the PA maximum rating, according to the PA circuit.
// If GPIO17 is unconnected, this has no effect (does not affect CW TX operation).
#define CW_PA_BIAS_LEVEL     4095   // *PA bias during CW TX (12bit maximum value = full bias)

// --- LSB RX filter coefficients (HPF 500Hz 2nd-order + LPF 2400Hz 4th-order = 500-2400Hz band @ fs=40kHz) ---
// From the I-Q LSB audio, the HPF removes low-frequency hum / DC and the LPF removes
// high-frequency noise / aliasing.
//
// Stage1: 2nd-order Butterworth HPF 500Hz
//   K = tan(pi*500/40000) = 0.039293
//   norm = 1 + K*sqrt(2) + K^2 = 1.057144
//   b = [1/norm, -2/norm, 1/norm] = [0.9460, -1.8919, 0.9460]
//   a = [1, 2(K^2-1)/norm, (1-K*sqrt(2)+K^2)/norm] = [1, -1.8896, 0.8949]
//
// Stage2+3: 4th-order Butterworth LPF 2400Hz (two Biquads cascaded)

// --- Stage1: HPF 500Hz 2nd-order Butterworth (low-cut) ---
#define LSB_HPF_B0   0.9460f
#define LSB_HPF_B1  -1.8919f  // = -2 * b0
#define LSB_HPF_B2   0.9460f  // = b0
#define LSB_HPF_A1  -1.8896f
#define LSB_HPF_A2   0.8949f

// --- Stage2: LPF 2400 Hz 4th-order Butterworth (first half: Biquad 1, Q=0.54) ---
#define LSB_LPF1_B0   0.02620f
#define LSB_LPF1_B1   0.05240f
#define LSB_LPF1_B2   0.02620f
#define LSB_LPF1_A1  -1.38763f
#define LSB_LPF1_A2   0.49243f

// --- Stage3: LPF 2400 Hz 4th-order Butterworth (second half: Biquad 2, Q=1.31) ---
#define LSB_LPF2_B0   0.03078f
#define LSB_LPF2_B1   0.06156f
#define LSB_LPF2_B2   0.03078f
#define LSB_LPF2_A1  -1.63000f
#define LSB_LPF2_A2   0.75305f

// LSB AGC settings (independent from the CW settings)
// CW : maxGain=40, attackRate=0.01 (maximize gain for weak signals)
// LSB: maxGain=10, attackRate=0.08 (fast response and low gain ceiling to prevent distortion on strong signals)
#define LSB_AGC_MAX_GAIN    10.0f   // ~1/4 of CW: prevents strong-signal ADC distortion
#define LSB_AGC_MIN_GAIN     0.1f
#define LSB_AGC_TARGET       0.35f  // Lower than CW (0.5): secures clipping margin
#define LSB_AGC_ATTACK       0.08f  // 8x of CW (0.01): reacts quickly to strong signals
#define LSB_AGC_DECAY        0.002f

// LSB soft clipper threshold
// When the filter input exceeds +-this value, smooth limiting is applied
// Prevents the IIR filter from oscillating due to a sudden signal from ADC overinput
#define LSB_SOFT_CLIP_THRESH 0.7f

// --- Default settings ---
const long LOW_FREQ     = 7000000;
const long HI_FREQ      = 7200000;
#define DEFAULT_WPM       20

// --- EEPROM addresses ---
const int EEPROM_ADDR_FREQ    = 0;
const int EEPROM_ADDR_STEP    = 4;
const int EEPROM_ADDR_WPM     = 8;
const int EEPROM_ADDR_KEYMODE = 12;
const int EEPROM_ADDR_VOL     = 16;
const int EEPROM_ADDR_MODE    = 20;   // Stores CW(0)/LSB(1) mode

// ============================================================
// [2] Global variables
// ============================================================

// --- Frequency / VFO ---
unsigned long FREQ     = 7000000;
unsigned long FREQ_OLD = FREQ;
unsigned long long FREQ_ULL  = 700000000ULL;  // centihz representation
unsigned long long pll_freq  = 86400000000ULL; // PLL VCO clock [centihz]
int STEP     = 1000;
int stepMode = 0;  // 0=1kHz, 1=100Hz, 2=10Hz

// --- Mode ---
volatile bool lsbMode = false;   // false=CW, true=LSB

// --- Band scope settings ---
// SCOPE_SPAN_HZ    : horizontal display range (+-Hz). Smaller = more zoomed-in.
// SCOPE_SENSITIVITY: vertical amplitude gain. Larger draws weak signals taller (more noise too).
// SCOPE_OFFSET     : noise-floor lift amount. Smaller lowers the noise-floor display.
// Note: the waterfall shares the same amplitude data (waterfallHistory), so changing
//       these three also changes the waterfall density in the lower section.
#define SCOPE_SPAN_HZ     15000.0f  // Display range +-15kHz
#define SCOPE_SENSITIVITY 16.0f     // Amplitude gain
#define SCOPE_OFFSET      3.0f      // Noise floor lift amount

// --- TX/RX state ---
// ** volatile is mandatory **
// Core0 (loop) sets it to true in startTransmit(), and
// Core1 (loop1) reads it in the handleLsbTxCore1() call condition "lsbMode && transmitting".
// Without volatile, compiler optimization makes Core1 keep using a cached register value,
// so it can never read the change from Core0 -> the root cause of LSB TX not working at all.
// (CW TX works without volatile because it is completed entirely on Core0)
volatile bool transmitting = false;
volatile int muteCounter = 0;

// --- Audio processing ---
volatile float volumeMultiplier = 1.0f;
float agcGain  = 1.0f;
float dcOffsetI = 0.0f;
float dcOffsetQ = 0.0f;

// --- CW IIR BPF filter state variables (2-stage 4th-order, +-100Hz @700Hz) ---
float iir_x1[3] = {0}, iir_x2[3] = {0};
float iir_y1[3] = {0}, iir_y2[3] = {0};

// --- LSB RX filter state variables ---
// Stage1: HPF 500Hz 2nd-order
float lsb_hpf_x1 = 0.0f, lsb_hpf_x2 = 0.0f;
float lsb_hpf_y1 = 0.0f, lsb_hpf_y2 = 0.0f;

// Stage2: LPF 2400Hz 4th-order (first half)
float lsb_lpf1_x1 = 0.0f, lsb_lpf1_x2 = 0.0f;
float lsb_lpf1_y1 = 0.0f, lsb_lpf1_y2 = 0.0f;

// Stage3: LPF 2400Hz 4th-order (second half)
float lsb_lpf2_x1 = 0.0f, lsb_lpf2_x2 = 0.0f;
float lsb_lpf2_y1 = 0.0f, lsb_lpf2_y2 = 0.0f;

// LSB-dedicated AGC gain (managed independently for CW/LSB -> the previous mode's gain is not carried over when switching)
float lsbAgcGain = 1.0f;

// IQ amplitude balance correction factor (shared Core0->Core1)
// Core0's FFT processing computes qGain from the I/Q RMS ratio and writes it here.
// Core1's lsbDemodulate() uses it to correct the Q channel amplitude.
// If the I/Q amplitudes do not match, image rejection degrades and the opposite sideband (USB)
// is heard as noise or beat tones.
// Since float can be read/written in one instruction on the 32bit Pico, it can be shared safely with volatile.
volatile float lsbQGain = 1.0f;

// --- Q signal 90-degree delay buffer (Hilbert transform approximation) ---
// Optimal value to delay 700Hz by 90 degrees at 40kHz = 14 samples
#define Q_DELAY_SAMPLES 14
float qDelayBuffer[Q_DELAY_SAMPLES] = {0};
int   qDelayIndex = 0;

// --- Inter-core data sharing (Core1 -> Core0, for FFT) ---
float sharedBufferI[SAMPLES];
float sharedBufferQ[SAMPLES];
volatile bool sharedBufferReady = false;
int sharedIndex = 0;

// --- For CW decoding ---
volatile float cwEnvelope = 0.0f;     // SNR ratio (CW_DETECT_THRESHOLD or higher = signal present)
// CW decode detection sensitivity: a key-down is registered when cwEnvelope (SNR ratio) exceeds this.
// Smaller picks up weaker signals but increases false detection (noise); larger is safer but may drop characters.
#define CW_DETECT_THRESHOLD 1.5f      // CW detection SNR threshold (recommended 1.5-2.5)
#define CW_DECODED_MAX      36
char cwDecodedBuf[CW_DECODED_MAX + 1] = "";
int  cwDecodedLen = 0;

// CW event buffer (Core1->Core0, ring buffer)
struct CWEvent {
  uint8_t  type;    // 0=space end (key down), 1=mark end (key up)
  uint16_t durMs;
};
#define CW_EVBUF_SIZE 16
volatile CWEvent cwEvBuf[CW_EVBUF_SIZE];
volatile uint8_t cwEvWr = 0;
volatile uint8_t cwEvRd = 0;

// --- LSB TX frequency shared variable (Core1 -> Core0) ---
// Core1 writes the instantaneous frequency it computed via the Hilbert transform here.
// Core0 reads this value and updates Si5351 CLK2 over I2C (to centralize I2C management).
//
// Design policy:
//   All I2C is managed on Core0. If Core1 directly calls si5351.set_freq_manual() (I2C),
//   the calls become unstable because Wire.begin() is initialized on Core0, and a collision
//   with the I2C in stopTransmit() (Core0) leaves CLK2 not disabled correctly, causing
//   fixed noise in the band after transmission. Therefore Core1 never calls I2C and instead
//   passes the computed frequency to Core0 through this shared variable.
//
// Why uint32_t is sufficient:
//   The 40m band centihz representation = 700000000 to 720500000 ~= 720M < 2^30 -> fits in uint32_t
//   On Cortex-M0+, a 32bit store is one instruction = atomic write is guaranteed
volatile uint32_t txFreqShared = 0;  // Core1 writes, Core0 reads

// --- LSB waveform display shared buffer (Core1 -> Core0) ---
// Core1 writes audio samples and Core0 draws them in the bottom 10px of the screen
#define LSB_WAVE_BUF_SIZE 128
volatile float lsbWaveBuf[LSB_WAVE_BUF_SIZE];
volatile uint8_t lsbWaveWrIdx = 0;   // Write index (updated by Core1)

// --- For FFT / scope ---
double vReal[SAMPLES];
double vImag[SAMPLES];
static uint8_t peakR[64];
static uint8_t peakL[64];
static uint8_t peakDecayDiv = 0;
const uint8_t  PEAK_DECAY_FRAMES = 1;
const uint8_t  DC_BLANK_BINS = 0;
const int VFO_MARKER_X   = 63;
const int PEAK_Y_OFFSET  = 10;
const int MARKER_TOP_MARGIN = 22;

// --- CW TX envelope (Core1 state, managed inside loop1) ---
// Kept global to separate the CW TX fade in/out from the transmitter control.
// Only Core1 writes to it.
volatile float txEnvelope = 0.0f;

// --- Keyer ---
int  wpm = DEFAULT_WPM;
bool straightKeyMode = false;
bool sending = false;
bool sendingDot = false;
bool sendingDash = false;
unsigned long dotDuration, dashDuration;
unsigned long elementSpace, charSpace, wordSpace;

// --- Waterfall ---
#define WATERFALL_HEIGHT 21   // Displayed at screen y=27 to 47
#define THRESHOLD        2
uint8_t waterfallHistory[WATERFALL_HEIGHT][128] = {0};
int waterfallIndex = 0;

// --- Objects ---
Rotary r = Rotary(PIN_IN1, PIN_IN2);
Si5351 si5351;
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
ArduinoFFT<double> FFT;

// ============================================================
// [3] EEPROM / hardware control
// ============================================================

void saveToEEPROM(int address, int data) {
  for (size_t i = 0; i < sizeof(data); i++) {
    EEPROM.write(address + i, (data >> (8 * i)) & 0xFF);
  }
  EEPROM.commit();
}

int readFromEEPROM(int address) {
  int data = 0;
  for (size_t i = 0; i < sizeof(data); i++) {
    data |= EEPROM.read(address + i) << (8 * i);
  }
  return data;
}

void saveFrequencyToEEPROM(unsigned long freq)  { saveToEEPROM(EEPROM_ADDR_FREQ,    (int)freq); }
unsigned long readFrequencyFromEEPROM()          { return (unsigned long)readFromEEPROM(EEPROM_ADDR_FREQ); }

void saveStepToEEPROM(int step)                 { saveToEEPROM(EEPROM_ADDR_STEP,    step); }
int readStepFromEEPROM()                         { return readFromEEPROM(EEPROM_ADDR_STEP); }

void saveWPMToEEPROM(int wpmVal)                { saveToEEPROM(EEPROM_ADDR_WPM,     wpmVal); }
int readWPMFromEEPROM() {
  int val = readFromEEPROM(EEPROM_ADDR_WPM);
  return (val >= 5 && val <= 40) ? val : DEFAULT_WPM;
}

void saveKeyModeToEEPROM(bool mode)             { saveToEEPROM(EEPROM_ADDR_KEYMODE, mode ? 1 : 0); }
bool readKeyModeFromEEPROM()                     { return readFromEEPROM(EEPROM_ADDR_KEYMODE) == 1; }

void saveVolToEEPROM(int volVal)                { saveToEEPROM(EEPROM_ADDR_VOL,     volVal); }
int readVolFromEEPROM() {
  int val = readFromEEPROM(EEPROM_ADDR_VOL);
  return (val >= 10 && val <= 30) ? val : 10;
}

// Save/load CW/LSB mode
void saveModeToEEPROM(bool isLsb)               { saveToEEPROM(EEPROM_ADDR_MODE, isLsb ? 1 : 0); }
bool readModeFromEEPROM()                        { return readFromEEPROM(EEPROM_ADDR_MODE) == 1; }

// ============================================================
// [4] Si5351 frequency control
// ============================================================

// For CW mode: shift CLK0/CLK1 down by the BFO offset so the received signal is heard as a 700Hz tone
void Freq_Set_CW() {
  unsigned long long lo_freq = FREQ_ULL - CW_AUDIO_OFFSET;  // centihz
  si5351.set_freq_manual(lo_freq, pll_freq, SI5351_CLK0);
  si5351.set_freq_manual(lo_freq, pll_freq, SI5351_CLK1);
  // Set 90-degree phase difference (for IQ mixer)
  int phase = (int)((float)pll_freq / (float)lo_freq + 0.5f);
  si5351.set_phase(SI5351_CLK0, 0);
  si5351.set_phase(SI5351_CLK1, phase);
  si5351.pll_reset(SI5351_PLLA);
}

// For LSB mode: set CLK0/CLK1 to the carrier frequency (no offset)
// LSB is selected from I-Q for the received audio
void Freq_Set_LSB() {
  si5351.set_freq_manual(FREQ_ULL, pll_freq, SI5351_CLK0);
  si5351.set_freq_manual(FREQ_ULL, pll_freq, SI5351_CLK1);
  // Set 90-degree phase difference
  int phase = (int)((float)pll_freq / (float)FREQ_ULL + 0.5f);
  si5351.set_phase(SI5351_CLK0, 0);
  si5351.set_phase(SI5351_CLK1, phase);
  si5351.pll_reset(SI5351_PLLA);
}

// Frequency setting according to the mode
void Freq_Set() {
  if (lsbMode) Freq_Set_LSB();
  else         Freq_Set_CW();
}

// Start transmission
// CW mode : sets the PA bias to CW_PA_BIAS_LEVEL (full bias).
//           Since setup1() initializes GPIO17 to 0, without this setting the PA stays
//           in the bias-OFF state -> the cause of extremely weak RF output.
//           RF output ON/OFF is controlled by enabling/disabling Si5351 CLK2.
// LSB mode: starts the PA bias from 0; during transmit, handleLsbTxCore1() updates the PWM
//           directly according to the instantaneous microphone amplitude (carrier-suppressed SSB operation).
void startTransmit() {
  if (!transmitting) {
    transmitting = true;
    si5351.output_enable(SI5351_CLK0, 0);  // RX LO OFF
    si5351.output_enable(SI5351_CLK1, 0);
    digitalWrite(RX_SW, HIGH);             // Switch the antenna to the TX side
    si5351.set_freq_manual(FREQ_ULL, pll_freq, SI5351_CLK2);
    si5351.output_enable(SI5351_CLK2, 1);  // TX carrier ON
    if (lsbMode) {
      analogWrite(PA_BIAS_PIN, 0);              // LSB: start from 0, the envelope raises it
    } else {
      analogWrite(PA_BIAS_PIN, CW_PA_BIAS_LEVEL); // CW: set the PA to full bias
    }
  }
}

// Stop transmission
// In both CW/LSB modes, immediately set the PA bias to 0 to cut off RF output.
// In LSB mode, additional cleanup:
//   1. Clear txFreqShared (so Core0's update loop does not use a stale frequency)
//   2. Fully reconfigure Si5351 for RX (so CLK2 does not linger, reset it reliably)
//   3. Set muteCounter generously (to reliably suppress the TX->RX transition noise)
void stopTransmit() {
  if (transmitting) {
    transmitting = false;
    txFreqShared = 0;                      // Clear the LSB TX frequency shared buffer
    analogWrite(PA_BIAS_PIN, 0);           // PA OFF (common to CW/LSB)
    si5351.output_enable(SI5351_CLK2, 0);  // TX carrier OFF <- this works reliably (no Core1 I2C)
    si5351.output_enable(SI5351_CLK0, 1);  // RX LO ON
    si5351.output_enable(SI5351_CLK1, 1);
    digitalWrite(RX_SW, LOW);             // Switch the antenna to the RX side
    if (lsbMode) {
      // Countermeasure for residual noise after LSB:
      // If an I2C collision occurs during set_freq_manual, CLK2 may end up in an invalid state, so
      // Freq_Set_LSB() reconfigures the entire Si5351 into a clean RX state.
      // (CLK2 was turned off above, but this is insurance in case the internal registers were disturbed)
      muteCounter = 1200;  // ~30ms mute: fully suppress the TX->RX switch noise
      Freq_Set_LSB();      // Reliably return Si5351 CLK0/CLK1 to the RX frequency
    }
  }
}

// ============================================================
// [5] Signal processing (DSP)
// ============================================================

// --- CW demodulation ---
// Extracts the CW signal with an IQ Hilbert transform + a 2-stage cascaded IIR 2nd-order BPF
// (700Hz center, +-100Hz).
//
// The Si5351 frequency accuracy is +-10ppm (+-70Hz error in the 7MHz band), so the band is
// kept somewhat wide at +-100Hz to suppress sensitivity variation due to frequency deviation.
//
// Filter coefficient quick reference (700Hz center, fs=40kHz):
//   +-100Hz: b0=0.024122, a1=1.939356, a2=0.951756 (current value)
//   +-50Hz : b0=0.015134, a1=1.957930, a2=0.969710 (narrow band)
//   +-30Hz : b0=0.009137, a1=1.970800, a2=0.981730 (ultra-narrow band)
float cwDemodulate(float iSig, float qSig) {
  // Delaying the Q signal by 14 samples gives a 90-degree phase difference around 700Hz
  float qDelayed = qDelayBuffer[qDelayIndex];
  qDelayBuffer[qDelayIndex] = qSig;
  qDelayIndex = (qDelayIndex + 1 >= Q_DELAY_SAMPLES) ? 0 : qDelayIndex + 1;

  // LSB selection: I - Q_delayed
  float audio = iSig + qDelayed;

  // 2-stage cascaded IIR 2nd-order BPF (700Hz center, +-100Hz band)
  // The somewhat wide band balances sensitivity and tolerance to frequency deviation
  float out = audio;
  for (int i = 0; i < 2; i++) {
    float y = 0.024122f * out - 0.024122f * iir_x2[i]
              + 1.939356f * iir_y1[i] - 0.951756f * iir_y2[i];
    iir_x2[i] = iir_x1[i];  iir_x1[i] = out;
    iir_y2[i] = iir_y1[i];  iir_y1[i] = y;
    out = y;
  }
  return out;
}

// --- LSB soft clipper ---
// Smoothly limits the input amplitude to the filter within LSB_SOFT_CLIP_THRESH.
// Prevents the IIR filter from momentarily oscillating on a steep, large-amplitude input,
// greatly suppressing the high-frequency noise (distortion components) on strong signals.
// Unlike hard clipping, it is a smooth curve, so there is little sound-quality degradation.
inline float lsbSoftClip(float x) {
  const float th = LSB_SOFT_CLIP_THRESH;
  if (x >  th) return  th + (x - th) / (1.0f + (x - th) * (x - th));
  if (x < -th) return -th - (-x - th) / (1.0f + (-x - th) * (-x - th));
  return x;
}

// --- LSB receive demodulation ---
// A Tayloe detector (IQ mixer) has an accurate 90-degree phase difference between I/Q at all
// audio frequencies, so no delay buffer is needed; LSB is selected with just I - Q.
//
// Processing flow:
//   I-Q -> soft clipper -> HPF500Hz(2nd-order) -> LPF2400Hz(4th-order) -> output
//   The 4th-order LPF (24dB/oct) on the high side greatly reduces out-of-band noise
float lsbDemodulate(float iSig, float qSig) {
  float audio = iSig - qSig * lsbQGain;
  audio = lsbSoftClip(audio);

  // --- Stage1: HPF 500Hz 2nd-order Butterworth ---
  float hpf_out = LSB_HPF_B0 * audio
                + LSB_HPF_B1 * lsb_hpf_x1
                + LSB_HPF_B2 * lsb_hpf_x2
                - LSB_HPF_A1 * lsb_hpf_y1
                - LSB_HPF_A2 * lsb_hpf_y2;
  lsb_hpf_x2 = lsb_hpf_x1;  lsb_hpf_x1 = audio;
  lsb_hpf_y2 = lsb_hpf_y1;  lsb_hpf_y1 = hpf_out;

  // --- Stage2: LPF 2400Hz 4th-order Butterworth (first half) ---
  float lpf1_out = LSB_LPF1_B0 * hpf_out
                 + LSB_LPF1_B1 * lsb_lpf1_x1
                 + LSB_LPF1_B2 * lsb_lpf1_x2
                 - LSB_LPF1_A1 * lsb_lpf1_y1
                 - LSB_LPF1_A2 * lsb_lpf1_y2;
  lsb_lpf1_x2 = lsb_lpf1_x1;  lsb_lpf1_x1 = hpf_out;
  lsb_lpf1_y2 = lsb_lpf1_y1;  lsb_lpf1_y1 = lpf1_out;

  // --- Stage3: LPF 2400Hz 4th-order Butterworth (second half) ---
  float lpf2_out = LSB_LPF2_B0 * lpf1_out
                 + LSB_LPF2_B1 * lsb_lpf2_x1
                 + LSB_LPF2_B2 * lsb_lpf2_x2
                 - LSB_LPF2_A1 * lsb_lpf2_y1
                 - LSB_LPF2_A2 * lsb_lpf2_y2;
  lsb_lpf2_x2 = lsb_lpf2_x1;  lsb_lpf2_x1 = lpf1_out;
  lsb_lpf2_y2 = lsb_lpf2_y1;  lsb_lpf2_y1 = lpf2_out;

  return lpf2_out;
}

// --- LSB-dedicated AGC ---
// Uses a gain variable (lsbAgcGain) independent from the CW AGC.
// Solves the problem where a high-gain state is carried over when switching CW->LSB, causing overamplification.
//
// Differences from CW:
//   - maxGain 40->10: prevents ADC saturation noise on strong signals
//   - attackRate 0.01->0.08: quickly lowers the gain when a strong signal arrives
//   - targetAmp 0.5->0.35: secures clipping margin
float applyLsbAGC(float input) {
  float error = LSB_AGC_TARGET - fabsf(input);
  lsbAgcGain += (error > 0) ? LSB_AGC_ATTACK * error : LSB_AGC_DECAY * error;
  lsbAgcGain = constrain(lsbAgcGain, LSB_AGC_MIN_GAIN, LSB_AGC_MAX_GAIN);
  return input * lsbAgcGain;
}

// --- AGC (automatic gain control) ---
// Tracks the gain toward a target amplitude. Asymmetric control with fast attack and slow decay.
float applyAGC(float input) {
  const float targetAmp  = 0.5f;
  const float maxGain    = 40.0f;
  const float minGain    = 0.1f;
  const float attackRate = 0.01f;
  const float decayRate  = 0.001f;

  float error = targetAmp - fabsf(input);
  agcGain += (error > 0) ? attackRate * error : decayRate * error;
  agcGain = constrain(agcGain, minGain, maxGain);
  return input * agcGain;
}

// ============================================================
// [6] CW keyer processing
// ============================================================

void calculateTiming() {
  dotDuration   = 1200 / wpm;
  dashDuration  = dotDuration * 3;
  elementSpace  = dotDuration;
  charSpace     = dotDuration * 3;
  wordSpace     = dotDuration * 7;
}

void initKeyer() {
  calculateTiming();
  sending = sendingDot = sendingDash = transmitting = false;
}

// CW keyer (supports both electronic keyer and straight key)
// CW mode only: not called in LSB mode since transmission uses the microphone
void handleKeyer() {
  static unsigned long stateStart = 0;
  static int keyerState = 0;  // 0=idle, 1=transmitting, 2=inter-element space
  unsigned long now = millis();

  // Straight key mode
  if (straightKeyMode) {
    bool keyDown = (digitalRead(PADDLE_DASH) == LOW);
    if (keyDown  && !transmitting) startTransmit();
    if (!keyDown &&  transmitting) stopTransmit();
    return;
  }

  // Electronic keyer mode
  bool dotPressed  = (digitalRead(PADDLE_DOT)  == LOW);
  bool dashPressed = (digitalRead(PADDLE_DASH) == LOW);

  if (keyerState == 0) {
    if (dotPressed) {
      keyerState = 1;  sendingDot = true;  sendingDash = false;
      stateStart = now; startTransmit();
    } else if (dashPressed) {
      keyerState = 1;  sendingDot = false; sendingDash = true;
      stateStart = now; startTransmit();
    }
  } else if (keyerState == 1) {
    // Check the transmission duration
    if (now - stateStart >= (sendingDash ? dashDuration : dotDuration)) {
      stopTransmit();
      keyerState = 2;
      stateStart = now;
    }
  } else if (keyerState == 2) {
    // Wait for the inter-element space
    if (now - stateStart >= elementSpace) keyerState = 0;
  }
}

// ============================================================
// [7] LSB transmit PTT processing
// ============================================================

// LSB mode PTT: uses GPIO9 (PTT_BUTTON) as a dedicated push-to-talk switch.
// It is a dedicated pin independent of PADDLE_DASH, so it does not interfere with the CW paddle.
// Transmits only while pressed (push-to-talk operation).
void handleLsbPTT() {
  bool pttPressed = (digitalRead(PTT_BUTTON) == LOW);
  if (pttPressed && !transmitting) startTransmit();
  if (!pttPressed &&  transmitting) stopTransmit();
}

// ============================================================
// [8] User interface
// ============================================================

// Convert a frequency to a string in "7.000.000" format
static String fmtMHz(unsigned long f_hz) {
  char buf[16];
  unsigned long mhz = f_hz / 1000000UL;
  unsigned long khz = (f_hz % 1000000UL) / 1000UL;
  snprintf(buf, sizeof(buf), "%lu.%03lu", mhz, khz);
  return String(buf);
}

// Band scope amplitude[dB] -> pixel height conversion
int barLength(double d) {
  float fy = SCOPE_SENSITIVITY * (log10(d) + SCOPE_OFFSET);
  return constrain((int)fy, 0, 20);
}

// Rotary encoder interrupt handler
void rotary_encoder() {
  unsigned char result = r.process();
  if (result) {
    if (result == DIR_CW) FREQ += STEP;
    else                  FREQ -= STEP;
  }
  FREQ     = constrain(FREQ, (long)LOW_FREQ, (long)HI_FREQ);
  FREQ_ULL = (unsigned long long)FREQ * 100ULL;
}

// --- Volume adjustment mode ---
void changeVolume() {
  detachInterrupt(0); detachInterrupt(1);
  int currentVolInt = constrain((int)(volumeMultiplier * 10), 10, 30);

  auto drawVol = [&](int val) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_8x13B_tr);
    u8g2.setCursor(18, 6);
    u8g2.print("VOLUME: ");
    u8g2.print(val / 10.0, 1);
    u8g2.print(" x");
    u8g2.drawFrame(14, 20, 100, 8);
    u8g2.drawBox(14, 20, (val - 10) * 5, 8);  // 10->0%, 30->100%
    u8g2.sendBuffer();
  };

  drawVol(currentVolInt);
  bool btnPrev = HIGH;

  while (true) {
    unsigned char res = r.process();
    if (res) {
      currentVolInt += (res == DIR_CW) ? 1 : -1;
      currentVolInt = constrain(currentVolInt, 10, 30);
      volumeMultiplier = currentVolInt / 10.0f;
      drawVol(currentVolInt);
      delay(10);
    }
    bool btn = (digitalRead(KEY_MODE_BUTTON) == LOW);
    if (!btn && btnPrev == LOW) {
      saveVolToEEPROM(currentVolInt);
      break;
    }
    btnPrev = btn;
    delay(5);
  }

  attachInterrupt(0, rotary_encoder, CHANGE);
  attachInterrupt(1, rotary_encoder, CHANGE);
}

// --- WPM setting mode ---
void changeWPM() {
  detachInterrupt(0); detachInterrupt(1);
  int newWpm = wpm;

  auto draw = [&](int val) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_8x13B_tr);
    u8g2.setCursor(38, 6);
    u8g2.print("WPM: "); u8g2.print(val);
    u8g2.setFont(u8g2_font_micro_tr);
    u8g2.setCursor(15, 20); u8g2.print("Rotate: +/-");
    u8g2.setCursor(15, 26); u8g2.print("Press: Save  Hold: Cancel");
    u8g2.sendBuffer();
  };

  draw(newWpm);
  unsigned long holdStart = 0;
  bool btnPrev = HIGH;

  while (true) {
    unsigned char res = r.process();
    if (res) {
      newWpm += (res == DIR_CW) ? 1 : -1;
      newWpm = constrain(newWpm, 5, 40);
      draw(newWpm);
      delay(10);
    }
    bool btn = (digitalRead(STEP_BUTTON) == LOW);
    unsigned long now = millis();
    if (btn && !btnPrev) holdStart = now;
    else if (!btn && btnPrev) {
      if (holdStart && (now - holdStart) >= 800) break;  // Long press = cancel
      wpm = newWpm;
      saveWPMToEEPROM(wpm);
      initKeyer();
      break;
    }
    btnPrev = btn;
    delay(5);
  }

  attachInterrupt(0, rotary_encoder, CHANGE);
  attachInterrupt(1, rotary_encoder, CHANGE);
}

// --- KEY_MODE_BUTTON handler (in CW mode: keyer toggle / long press: volume) ---
void handleKeyModeButton() {
  static unsigned long pressStart = 0;
  static bool isPressing      = false;
  static bool longHandled     = false;
  static bool lastReading     = HIGH;
  const unsigned long DEBOUNCE   = 30;
  const unsigned long LONG_PRESS = 800;

  bool reading = digitalRead(KEY_MODE_BUTTON);

  if (reading == LOW && lastReading == HIGH) {
    pressStart = millis();  isPressing = true;  longHandled = false;
  } else if (reading == LOW && isPressing) {
    if (!longHandled && (millis() - pressStart) > LONG_PRESS) {
      longHandled = true;
      changeVolume();
    }
  } else if (reading == HIGH && lastReading == LOW) {
    isPressing = false;
    if (!longHandled && (millis() - pressStart) > DEBOUNCE) {
      // Toggle keyer mode only in CW mode (not needed in LSB mode since it uses microphone transmission)
      if (!lsbMode) {
        straightKeyMode = !straightKeyMode;
        saveKeyModeToEEPROM(straightKeyMode);
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_8x13B_tr);
        u8g2.setCursor(20, 4);  u8g2.print("KEY MODE");
        u8g2.setCursor(40, 18); u8g2.print(straightKeyMode ? "STRAIGHT" : "IAMBIC");
        u8g2.sendBuffer();
        delay(600);
      }
    }
  }
  lastReading = reading;
}

// --- STEP_BUTTON handler (frequency step toggle / long press: WPM setting) ---
void Fnc_Stp() {
  unsigned long pressStart = millis();
  while (digitalRead(STEP_BUTTON) == LOW) {
    if (millis() - pressStart > 1000) { changeWPM(); return; }
    delay(10);
  }
  if      (stepMode == 0) { stepMode = 1; STEP = 100; }
  else if (stepMode == 1) { stepMode = 2; STEP = 10; }
  else                    { stepMode = 0; STEP = 1000; }
  saveStepToEEPROM(STEP);
  delay(10);
}

// --- *MODE_BUTTON handler (CW/LSB mode switch) ---
// A short press toggles CW<->LSB and saves it to EEPROM.
// On switching, it resets the filter states and reconfigures the VFO frequency.
void handleModeButton() {
  static bool lastReading = HIGH;
  static unsigned long pressTime = 0;
  const unsigned long DEBOUNCE = 40;

  bool reading = digitalRead(MODE_BUTTON);

  if (reading == LOW && lastReading == HIGH) {
    pressTime = millis();
  } else if (reading == HIGH && lastReading == LOW) {
    if (millis() - pressTime >= DEBOUNCE) {

      // ============================================================
      // VFO frequency correction (performed before the mode switch)
      // ============================================================
      // Since the BFO offset differs between CW and LSB, the "meaning" of the VFO display frequency shifts.
      //
      //   CW  : LO = FREQ - 700Hz  -> the station at FREQ is heard as a 700Hz tone
      //   LSB : LO = FREQ          -> the station at FREQ + 700Hz is heard as 700Hz
      //
      // Example: in CW, FREQ=7.001MHz -> the station at 7.001MHz is a 700Hz tone
      //   Switching to LSB as-is -> the same station is (7001000 - 7001000) = 0Hz(DC) -> inaudible!
      //   The 700Hz heard in LSB is the station at FREQ + 700Hz = 7.001700MHz
      //
      // Correction: when switching modes, move FREQ by +-700Hz so the same station is always heard at 700Hz
      //   CW -> LSB: FREQ -= 700Hz  (no LO change: FREQ_new = FREQ_old - 700)
      //   LSB -> CW : FREQ += 700Hz  (no LO change: FREQ_new - 700 = FREQ_old)
      //
      // Verification:
      //   CW FREQ=7.001MHz, station=7.001MHz, tone=700Hz
      //   ->LSB FREQ=7.000300MHz, LO=7.000300MHz, station=7.001MHz -> tone=700Hz OK
      //
      //   LSB FREQ=7.000300MHz, station=7.001MHz, tone=700Hz
      //   ->CW FREQ=7.001MHz, LO=7.001MHz-700Hz=7.000300MHz, station=7.001MHz -> tone=700Hz OK
      if (!lsbMode) {
        // Currently CW -> switching to LSB: lower FREQ by 700Hz
        FREQ = (unsigned long)constrain((long)FREQ - 700, (long)LOW_FREQ, (long)HI_FREQ);
      } else {
        // Currently LSB -> switching to CW: raise FREQ by 700Hz
        FREQ = (unsigned long)constrain((long)FREQ + 700, (long)LOW_FREQ, (long)HI_FREQ);
      }
      FREQ_ULL = (unsigned long long)FREQ * 100ULL;
      FREQ_OLD = FREQ;
      saveFrequencyToEEPROM(FREQ);

      // Switch mode
      lsbMode = !lsbMode;
      saveModeToEEPROM(lsbMode);

      // Reset filter / AGC state variables
      // If the CW high gain is carried over when switching CW->LSB, a pop sound occurs, so reset both
      memset(iir_x1, 0, sizeof(iir_x1));
      memset(iir_x2, 0, sizeof(iir_x2));
      memset(iir_y1, 0, sizeof(iir_y1));
      memset(iir_y2, 0, sizeof(iir_y2));
      memset(qDelayBuffer, 0, sizeof(qDelayBuffer));
      qDelayIndex    = 0;
      lsb_lpf1_x1 = lsb_lpf1_x2 = lsb_lpf1_y1 = lsb_lpf1_y2 = 0.0f;
      lsb_lpf2_x1 = lsb_lpf2_x2 = lsb_lpf2_y1 = lsb_lpf2_y2 = 0.0f;
      agcGain    = 1.0f;
      lsbAgcGain = 1.0f;

      // Reconfigure the VFO (with the corrected FREQ)
      muteCounter = 500;
      Freq_Set();

      // Mode display (new mode name + corrected frequency)
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_8x13B_tr);
      u8g2.setCursor(30, 4);  u8g2.print("MODE:");
      u8g2.setCursor(40, 18); u8g2.print(lsbMode ? "LSB" : "CW");
      u8g2.sendBuffer();
      delay(600);
    }
  }
  lastReading = reading;
}

// ============================================================
// [9] Screen drawing
// ============================================================

// --- Band scope drawing ---
// Upper scope (y=0 to 26): displays the IQ-FFT result as a spectrum
void showScope() {
  const int BASE_Y = 26;
  const int PX_PER_SIDE = 63;
  const int MAX_D = 25;
  const float BIN_HZ = (float)sampleRate / (float)SAMPLES;
  int binsTarget = constrain((int)(SCOPE_SPAN_HZ / BIN_HZ + 0.5f), 0, SAMPLES / 2);
  float binsPerPixel = (float)binsTarget / (float)PX_PER_SIDE;

  peakDecayDiv++;
  bool doDecay = (peakDecayDiv >= PEAK_DECAY_FRAMES);
  if (doDecay) peakDecayDiv = 0;

  float offsetBins = (float)CW_TONE / BIN_HZ;

  for (int i = 0; i < 128; i++) waterfallHistory[waterfallIndex][i] = 0;

  // --- Right half (positive frequencies) ---
  int prevX_now = -1, prevY_now = -1, prevX_pk = -1, prevY_pk = -1;
  for (int xi = 0; xi <= PX_PER_SIDE; xi++) {
    float exactBin = xi * binsPerPixel + offsetBins;
    int bin = constrain((int)(exactBin + 0.5f), 0, SAMPLES / 2 - 1);
    int d = (bin > DC_BLANK_BINS)
            ? constrain((barLength(vReal[bin]) + barLength(vImag[bin])) / 2, 0, MAX_D)
            : constrain((barLength(vReal[DC_BLANK_BINS + 1]) + barLength(vImag[DC_BLANK_BINS + 1])) / 2, 0, MAX_D);

    uint8_t pk = peakR[xi];
    if ((uint8_t)d >= pk) pk = d;
    else if (doDecay && pk > 0) pk--;
    peakR[xi] = pk;

    int x     = 63 + xi;
    waterfallHistory[waterfallIndex][x] = d;
    int y_now = BASE_Y - d;
    int y_pk  = constrain(BASE_Y - (int)pk - 1 + PEAK_Y_OFFSET, 0, BASE_Y);
    if (prevX_now >= 0) u8g2.drawLine(prevX_now, prevY_now, x, y_now);
    if (prevX_pk  >= 0) u8g2.drawLine(prevX_pk,  prevY_pk,  x, y_pk);
    prevX_now = x; prevY_now = y_now; prevX_pk = x; prevY_pk = y_pk;
  }

  // --- Left half (negative frequencies) ---
  prevX_now = -1; prevY_now = -1; prevX_pk = -1; prevY_pk = -1;
  for (int xi = 1; xi <= PX_PER_SIDE; xi++) {
    float exactBin = -(xi * binsPerPixel) + offsetBins;
    int bin = constrain((int)(fabsf(exactBin) + 0.5f), 0, SAMPLES / 2 - 1);
    int d = 0;
    if (bin > DC_BLANK_BINS) {
      d = (exactBin >= 0)
          ? constrain((barLength(vReal[bin]) + barLength(vImag[bin])) / 2, 0, MAX_D)
          : constrain((barLength(vReal[SAMPLES - bin]) + barLength(vImag[SAMPLES - bin])) / 2, 0, MAX_D);
    } else {
      int nb = SAMPLES - (DC_BLANK_BINS + 1);
      d = constrain((barLength(vReal[nb]) + barLength(vImag[nb])) / 2, 0, MAX_D);
    }

    uint8_t pk = peakL[xi];
    if ((uint8_t)d >= pk) pk = d;
    else if (doDecay && pk > 0) pk--;
    peakL[xi] = pk;

    int x     = 63 - xi;
    waterfallHistory[waterfallIndex][x] = d;
    int y_now = BASE_Y - d;
    int y_pk  = constrain(BASE_Y - (int)pk - 1 + PEAK_Y_OFFSET, 0, BASE_Y);
    if (prevX_now >= 0) u8g2.drawLine(prevX_now, prevY_now, x, y_now);
    if (prevX_pk  >= 0) u8g2.drawLine(prevX_pk,  prevY_pk,  x, y_pk);
    prevX_now = x; prevY_now = y_now; prevX_pk = x; prevY_pk = y_pk;
  }

  // --- Frequency scale (y=49) ---
  unsigned long leftHz  = (FREQ > 15000) ? FREQ - 15000 : 0;
  unsigned long rightHz = FREQ + 15000;
  u8g2.setFont(u8g2_font_micro_tr);
  u8g2.drawStr(0, 49, fmtMHz(leftHz).c_str());
  String midS = fmtMHz(FREQ);
  u8g2.drawStr(constrain(64 - (int)(midS.length() * 2), 34, 80), 49, midS.c_str());
  String rS = fmtMHz(rightHz);
  u8g2.drawStr(constrain(128 - (int)(rS.length() * 4), 98, 128), 49, rS.c_str());

  // --- Frequency / step display (upper) ---
  u8g2.setFont(u8g2_font_8x13B_tr);
  String freqt = String(FREQ);
  u8g2.setCursor(2, 1);
  u8g2.print(freqt.substring(0, 1) + "." + freqt.substring(1, 4) + "." + freqt.substring(4));

  u8g2.setFont(u8g2_font_micro_tr);
  // TX status display (CW mode only)
  if (transmitting && !lsbMode) {
    u8g2.setCursor(90, 6);
    u8g2.print("TX ");
    if      (sendingDot)  u8g2.print("DOT");
    else if (sendingDash) u8g2.print("DASH");
    else                  u8g2.print("KEY");
    u8g2.print(" "); u8g2.print(wpm); u8g2.print("W");
  }
  // TX status display (LSB mode)
  if (transmitting && lsbMode) {
    u8g2.setCursor(90, 6);
    u8g2.print("LSB TX");
  }

  u8g2.setCursor(78, 0);  u8g2.print("STEP:");
  if      (STEP == 1000) u8g2.drawStr(102, 0, "1000");
  else if (STEP == 100)  u8g2.drawStr(102, 0, " 100");
  else                   u8g2.drawStr(106, 0, "  10");
}

// --- S-meter drawing ---
// Computes the peak power from the band scope data and displays it as a bar
void showS_meter() {
  for (int xi = 1; xi < 64; xi++) {
    int d = (barLength(vReal[xi * 2]) + barLength(vImag[xi * 2 + 1])) * 2;
    u8g2.drawBox(86, 6, d, 6);
  }
}

// --- Waterfall drawing (y=27 to 47) ---
void displayWaterfall() {
  for (int y = 0; y < WATERFALL_HEIGHT; y++) {
    int histY = (waterfallIndex - y + WATERFALL_HEIGHT) % WATERFALL_HEIGHT;
    for (int x = 0; x < 128; x++) {
      if (waterfallHistory[histY][x] > THRESHOLD) u8g2.drawPixel(x, 27 + y);
    }
  }
  waterfallIndex = (waterfallIndex + 1) % WATERFALL_HEIGHT;
}

// --- Fixed graphics frame / label drawing ---
void showGraphics() {
  u8g2.drawHLine(0,   26, 128);   // Scope bottom edge
  u8g2.drawHLine(0,   48, 128);   // Waterfall bottom edge
  u8g2.drawFrame(86,  6,  42, 6); // S-meter frame
  u8g2.drawBox(0,   26, 2, 2);
  u8g2.drawBox(126, 26, 2, 2);

  u8g2.setFont(u8g2_font_micro_tr);
  u8g2.drawStr(78, 6, "S:");
  u8g2.drawStr(78, 0, "STEP:");
  u8g2.drawStr(122, 0, "Hz");

  // *Mode display: just below the S-meter bar (y=13), at the right edge in the smallest font, "CW" or "LSB"
  // y=6 is the S-meter top (6px tall), and y=12 is just below it, so draw at y=13
  u8g2.drawStr(lsbMode ? 117 : 121, 13, lsbMode ? "LSB" : "CW");

  // Frequency (large font)
  String freqt = String(FREQ);
  u8g2.setFont(u8g2_font_8x13B_tr);
  u8g2.setCursor(2, 1);
  u8g2.print(freqt.substring(0, 1) + "." + freqt.substring(1, 4) + "." + freqt.substring(4));
}

// --- CW decoded text display (y=54 to 63, 10px) ---
// Fixes the newest character at the right edge; older characters scroll out to the left
void displayCWText() {
  const int CHAR_W   = 5;
  const int CHAR_GAP = 2;
  const int STEP_W   = CHAR_W + CHAR_GAP;  // 7px/character
  const int MAX_VIS  = 128 / STEP_W;       // Maximum 18 characters
  const int TEXT_Y   = 55;                 // Within y=54 to 63, 1px top margin

  u8g2.setFont(u8g2_font_5x8_tr);
  int count  = (cwDecodedLen < MAX_VIS) ? cwDecodedLen : MAX_VIS;
  int start  = cwDecodedLen - count;
  int xStart = (MAX_VIS - count) * STEP_W;  // Right-aligned
  for (int i = 0; i < count; i++) {
    char buf[2] = { cwDecodedBuf[start + i], '\0' };
    u8g2.drawStr(xStart + i * STEP_W, TEXT_Y, buf);
  }
}

// --- *LSB waveform display (y=56 to 63, 8px) ---
// Displays the audio waveform as an oscilloscope in the bottom 8 pixels of the screen. No divider line is drawn.
// During transmit: microphone input waveform (4kHz / 4 = 1kHz display sample rate)
// During receive: AGC-processed demodulated audio waveform (40kHz / 32 = 1250Hz display sample rate)
//
// Display area: y=56 (top) to y=63 (bottom), center y=59
// Buffer: 128-element ring buffer -> maps 1:1 to 128 pixels
void displayLsbWaveform() {
  const int CENTER_Y = 59;   // Center of the 8px area (y=56-63)

  // ================================================================
  // ** Adjust the waveform sensitivity here **
  // WAVE_SCALE : a larger value makes the waveform display larger
  //   Recommended range: 3.0 to 20.0
  //   3.0  -> swings only on a loud voice / strong signal (low sensitivity)
  //   8.0  -> swings appropriately at normal conversation volume (recommended default)
  //   15.0 -> swings even on weak signals (high sensitivity, noise is also visible)
  // WAVE_MAX_AMP: maximum waveform swing [px] (clamped to +-this value)
  //   3 -> 3px up and down each = 6px total (fits within the 8px area)
  // ================================================================
  const float WAVE_SCALE   = 8.0f;  // <- change this to adjust sensitivity
  const int   WAVE_MAX_AMP = 3;     // <- clamp width [px]

  // Take a snapshot of the wrIdx updated by Core1
  // wrIdx is the "next write position", so read out from oldest to newest
  uint8_t wrSnap = lsbWaveWrIdx;

  int prevX = -1, prevY = CENTER_Y;
  for (int x = 0; x < 128; x++) {
    uint8_t bufIdx = (uint8_t)((wrSnap + x) % LSB_WAVE_BUF_SIZE);
    float sample   = lsbWaveBuf[bufIdx];

    // Amplitude -> pixel conversion (clamped to +-WAVE_MAX_AMP)
    int dy = constrain((int)(sample * WAVE_SCALE), -WAVE_MAX_AMP, WAVE_MAX_AMP);
    int y  = CENTER_Y - dy;

    if (prevX >= 0) u8g2.drawLine(prevX, prevY, x, y);
    prevX = x;  prevY = y;
  }
}

// ============================================================
// [10] Setup / main loop (Core0: UI / control)
// ============================================================

void setup() {
  // Pin initialization
  pinMode(LED_INDICATOR,  OUTPUT);
  pinMode(RX_SW,          OUTPUT);
  pinMode(PADDLE_DOT,     INPUT_PULLUP);
  pinMode(PADDLE_DASH,    INPUT_PULLUP);
  pinMode(KEY_MODE_BUTTON,INPUT_PULLUP);
  pinMode(STEP_BUTTON,    INPUT_PULLUP);
  pinMode(MODE_BUTTON,    INPUT_PULLUP);  // CW/LSB mode button
  pinMode(PTT_BUTTON,     INPUT_PULLUP);  // LSB PTT button

  EEPROM.begin(512);
  Wire.begin();

  // Si5351 initialization
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 25001042, 0);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_8MA);
  si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_8MA);

  // *Explicit PLL initialization (LSB TX fix)
  // The etherkit Si5351 library assigns CLK0/CLK1 to PLLA and CLK2 to PLLB.
  // set_freq_manual(CLK2) assumes "PLLB is running at pll_freq", but
  // if PLLB is uninitialized, CLK2's output frequency is completely off -> the cause of no LSB TX output.
  // Initialize both PLLs explicitly at pll_freq, then configure the CLK2 multisynth.
  si5351.set_pll(pll_freq, SI5351_PLLA);   // Initialize PLLA for CLK0/CLK1
  si5351.set_pll(pll_freq, SI5351_PLLB);   // Initialize PLLB for CLK2 *important
  // Initialize the CLK2 multisynth (preprocessing so set_freq_manual works correctly)
  si5351.set_freq(FREQ_ULL, SI5351_CLK2);
  si5351.output_enable(SI5351_CLK2, 0);  // TX initial state is OFF
  digitalWrite(RX_SW, LOW);

  // Rotary encoder interrupt setup
  r.begin();
  attachInterrupt(0, rotary_encoder, CHANGE);
  attachInterrupt(1, rotary_encoder, CHANGE);

  // Restore settings from EEPROM
  unsigned long savedFreq = readFrequencyFromEEPROM();
  FREQ = (savedFreq >= (unsigned long)LOW_FREQ && savedFreq <= (unsigned long)HI_FREQ) ? savedFreq : 7000000UL;
  wpm             = readWPMFromEEPROM();
  straightKeyMode = readKeyModeFromEEPROM();
  volumeMultiplier= readVolFromEEPROM() / 10.0f;
  lsbMode         = readModeFromEEPROM();  // *Restore CW/LSB mode

  int s = readStepFromEEPROM();
  if (s == 10 || s == 100 || s == 1000) {
    STEP = s;
    stepMode = (STEP == 1000) ? 0 : (STEP == 100) ? 1 : 2;
  } else {
    STEP = 1000;
  }

  initKeyer();
  FREQ_OLD = FREQ;
  FREQ_ULL = (unsigned long long)FREQ * 100ULL;
  Freq_Set();

  analogReadResolution(12);

  // OLED initialization
  u8g2.begin();
  u8g2.setFlipMode(0);
  u8g2.setFont(u8g2_font_micro_tr);
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();

  // Startup splash
  u8g2.clearBuffer();
  u8g2.drawStr(26, 14, "7MHz CW/LSB TRX v1.1");
  u8g2.drawStr(24, 24, "BandScope & Waterfall");
  u8g2.drawStr(54, 34, "JR3XNW");
  u8g2.drawStr(50, 44, lsbMode ? "Mode: LSB" : "Mode: CW");
  u8g2.sendBuffer();
  delay(1500);
}

void loop() {
  static unsigned long lastUIUpdate = 0;

  // Button processing (every loop)
  handleKeyModeButton();
  handleModeButton();   // *CW/LSB mode switch

  if (lsbMode) {
    // === LSB mode ===
    handleLsbPTT();     // Use PTT_BUTTON (GPIO9) as PTT

    // Detect frequency change
    if (FREQ != FREQ_OLD) {
      muteCounter = 500;
      Freq_Set_LSB();
      FREQ_OLD = FREQ;
      saveFrequencyToEEPROM(FREQ);
    }

    if (!transmitting) {
      if (digitalRead(STEP_BUTTON) == LOW) Fnc_Stp();

      // When the FFT data is ready, update the screen
      if (sharedBufferReady) {
        static float avgI2 = 0.01f, avgQ2 = 0.01f;
        for (int i = 0; i < SAMPLES; i++) {
          float si = sharedBufferI[i], sq = sharedBufferQ[i];
          avgI2 = avgI2 * 0.9995f + si * si * 0.0005f;
          avgQ2 = avgQ2 * 0.9995f + sq * sq * 0.0005f;
        }
        float qGain = (avgQ2 > 0.0001f) ? constrain(sqrtf(avgI2 / avgQ2), 0.8f, 1.25f) : 1.0f;
        // *Share the IQ amplitude correction factor with Core1's LSB demodulation processing
        // Core1 multiplies the Q channel by this value inside lsbDemodulate() to balance the amplitude
        // This improves image rejection and reduces the opposite-sideband (USB) beat tone
        lsbQGain = qGain;
        for (int i = 0; i < SAMPLES; i++) {
          vReal[i] = sharedBufferI[i];
          vImag[i] = sharedBufferQ[i] * qGain;
        }
        sharedBufferReady = false;

        if (millis() - lastUIUpdate >= 50) {
          digitalWrite(LED_INDICATOR, HIGH);
          FFT.windowing(vReal, SAMPLES, FFTWindow::Hamming, FFTDirection::Forward);
          FFT.windowing(vImag, SAMPLES, FFTWindow::Hamming, FFTDirection::Forward);
          FFT.compute(vReal, vImag, SAMPLES, FFTDirection::Reverse);
          FFT.complexToMagnitude(vReal, vImag, SAMPLES);

          u8g2.clearBuffer();
          showS_meter();
          showScope();
          displayWaterfall();
          showGraphics();
          displayLsbWaveform();  // *Display the LSB receive waveform in the bottom row
          u8g2.sendBuffer();
          digitalWrite(LED_INDICATOR, LOW);
          lastUIUpdate = millis();
        }
      }
    } else {
      // =====================================================
      // During LSB TX: Core0 updates Si5351 CLK2 at 4kHz
      // =====================================================
      // All I2C is concentrated on Core0 (Core1 never calls I2C):
      //   Core1: ADC -> Hilbert transform -> PA bias (GPIO) + write txFreqShared (no I2C)
      //   Core0: read txFreqShared and update Si5351 CLK2 at 4kHz (I2C is concentrated here)
      //   The OLED is not updated (to keep Core0's I2C dedicated to the Si5351 and not disturb TX modulation)
      // Note: Wire is begin()-ed on Core0, so I2C calls from Core1 are unstable, and a collision
      //   with the I2C in stopTransmit() leaves CLK2 not disabled, causing noise after transmission.
      {
        static unsigned long lastTxUpdate = 0;
        unsigned long nowUs = micros();
        if (nowUs - lastTxUpdate >= (unsigned long)LSB_TX_INTERVAL_US) {
          lastTxUpdate = nowUs;
          uint32_t f = txFreqShared;  // Atomic read (32bit)
          if (f > 0) {
            // Core0 updates the Si5351 CLK2 frequency (no I2C contention)
            si5351.set_freq_manual((unsigned long long)f, pll_freq, SI5351_CLK2);
          }
        }
      }

      // --- *Microphone waveform display during LSB TX ---
      // A full-screen sendBuffer() occupies I2C for too long and disturbs TX modulation, so it cannot be used.
      // updateDisplayArea() transfers only the bottom tile row (y=56-63, 128 bytes) to
      // minimize the I2C occupation time. The update rate is held to 10fps to minimize TX interruption.
      {
        static unsigned long lastTxWaveDraw = 0;
        if (millis() - lastTxWaveDraw >= 100) {   // 10fps
          lastTxWaveDraw = millis();
          u8g2.setDrawColor(0);
          u8g2.drawBox(0, 56, 128, 8);            // Clear the bottom 8px
          u8g2.setDrawColor(1);
          displayLsbWaveform();                    // Draw the microphone waveform
          u8g2.updateDisplayArea(0, 7, 16, 1);     // Transfer only tile row 7 (y=56-63)
        }
      }

      lastUIUpdate = millis();  // Timer reset: prevent a batch update after PTT release
    }

  } else {
    // === CW mode ===
    handleKeyer();
    handleCWDecoder();  // CW auto-decode

    if (!transmitting) {
      if (FREQ != FREQ_OLD) {
        muteCounter = 500;
        Freq_Set_CW();
        FREQ_OLD = FREQ;
        saveFrequencyToEEPROM(FREQ);
      }
      if (digitalRead(STEP_BUTTON) == LOW) Fnc_Stp();

      if (sharedBufferReady) {
        static float avgI2 = 0.01f, avgQ2 = 0.01f;
        for (int i = 0; i < SAMPLES; i++) {
          float si = sharedBufferI[i], sq = sharedBufferQ[i];
          avgI2 = avgI2 * 0.9995f + si * si * 0.0005f;
          avgQ2 = avgQ2 * 0.9995f + sq * sq * 0.0005f;
        }
        float qGain = (avgQ2 > 0.0001f) ? constrain(sqrtf(avgI2 / avgQ2), 0.8f, 1.25f) : 1.0f;
        for (int i = 0; i < SAMPLES; i++) {
          vReal[i] = sharedBufferI[i];
          vImag[i] = sharedBufferQ[i] * qGain;
        }
        sharedBufferReady = false;

        if (millis() - lastUIUpdate >= 50) {
          digitalWrite(LED_INDICATOR, HIGH);
          FFT.windowing(vReal, SAMPLES, FFTWindow::Hamming, FFTDirection::Forward);
          FFT.windowing(vImag, SAMPLES, FFTWindow::Hamming, FFTDirection::Forward);
          FFT.compute(vReal, vImag, SAMPLES, FFTDirection::Reverse);
          FFT.complexToMagnitude(vReal, vImag, SAMPLES);

          u8g2.clearBuffer();
          showS_meter();
          showScope();
          displayWaterfall();
          showGraphics();
          displayCWText();   // Display the CW decoded text in the bottom row
          u8g2.sendBuffer();
          digitalWrite(LED_INDICATOR, LOW);
          lastUIUpdate = millis();
        }
      }
    } else {
      // During CW TX
      if (millis() - lastUIUpdate >= 50) {
        u8g2.clearBuffer();
        showGraphics();
        displayCWText();
        u8g2.sendBuffer();
        lastUIUpdate = millis();
      }
    }
  }
}

// ============================================================
// [11] Core1 setup / loop (audio / DSP processing)
// ============================================================

void setup1() {
  pinMode(speakerPin,  OUTPUT);
  pinMode(PA_BIAS_PIN, OUTPUT);  // *PA bias control PWM output
  pinMode(inputPinI,   INPUT);
  pinMode(inputPinQ,   INPUT);
  pinMode(MIC_IN,      INPUT);   // Microphone input

  analogReadResolution(12);
  analogWriteResolution(12);
  // speakerPin(GPIO16) and PA_BIAS_PIN(GPIO17) share the same PWM slice (slice0), so
  // the 44100Hz set by analogWriteFreq() applies to both.
  // 44100Hz is high enough for PA bias use as well, and is smoothed by the PA-side RC filter.
  analogWriteFreq(pwmFrequency);  // 44100Hz -> applied to both GPIO16/17

  analogWrite(PA_BIAS_PIN, 0);   // PA bias is 0 at startup (PA OFF)
}

void loop1() {
  unsigned long now = micros();

  // * LSB TX processing: phase-modulate Si5351 CLK2 at a 4kHz rate
  // Operates on timing independent from the normal 40kHz RX loop
  if (lsbMode && transmitting) {
    static unsigned long lastLsbTxTime = 0;
    if (now - lastLsbTxTime < (unsigned long)LSB_TX_INTERVAL_US) return;
    lastLsbTxTime = now;
    handleLsbTxCore1();
    return;
  }

  // === 40kHz RX / CW TX common timing control ===
  static bool prevTxState  = false;
  static unsigned long lastSampleTime = 0;
  static float toneAngle   = 0.0f;   // Phase accumulator for the CW monitor tone
  static float lastRxPwm   = 2047.5f;// The final RX output value for TX->RX crossfade

  const float  ENV_STEP      = 0.002f; // Fade completes in ~12.5ms (@40kHz)
  const float  FADE_WIN      = 0.05f;  // Crossfade interval
  const unsigned long TARGET = 25;     // 40kHz = 25us

  // Reset the timestamp at TX start (to prevent catch-up due to accumulated drift during RX)
  if (transmitting && !prevTxState) lastSampleTime = now;
  prevTxState = transmitting;

  if (now - lastSampleTime < TARGET) return;
  lastSampleTime += TARGET;

  // --- CW TX fade in/out processing ---
  // txEnvelope operates independently of the transmitting flag, and
  // continues to output a sine wave even after stopTransmit() until the envelope reaches 0.
  txEnvelope += transmitting ? ENV_STEP : -ENV_STEP;
  txEnvelope  = constrain(txEnvelope, 0.0f, 1.0f);

  if (!lsbMode && txEnvelope > 0.0f) {
    // Generate a 700Hz sine wave
    toneAngle += 2.0f * (float)PI * (float)CW_TONE / (float)sampleRate;
    if (toneAngle >= 2.0f * (float)PI) toneAngle -= 2.0f * (float)PI;
    float sine = sinf(toneAngle);

    // Crossfade right after TX start (smoothly transition from the final RX output value to the sine wave)
    float blend = (txEnvelope < FADE_WIN) ? (txEnvelope / FADE_WIN) : 1.0f;
    float txPwm = (sine * 0.1f * txEnvelope + 1.0f) * 2047.5f;
    float pwm   = lastRxPwm * (1.0f - blend) + txPwm * blend;

    analogWrite(speakerPin, (uint16_t)constrain((int)pwm, 0, 4095));
    return;  // Skip RX processing
  }

  // --- RX processing (common to CW/LSB) ---
  // Pass lastRxPwm by argument (equivalent to pass-by-reference) so it gets updated
  handleRxCore1(lastRxPwm);
}

// --- RX processing (common to CW/LSB modes) ---
// I/Q sampling -> mode-specific digital filter -> AGC -> PWM output
// At the same time, accumulates data into the shared buffer for the FFT.
// lastRxPwm: writes back the final output value for the CW TX crossfade in loop1()
void handleRxCore1(float& lastRxPwm) {
  static bool wasTransmitting = false;
  static float smoothMute  = 1.0f;

  // Cleanup at the TX->RX transition: reset the residual IIR filter state
  if (wasTransmitting) {
    dcOffsetI = dcOffsetQ = 0.0f;
    sharedIndex = 0;
    memset(iir_x1, 0, sizeof(iir_x1));
    memset(iir_x2, 0, sizeof(iir_x2));
    memset(iir_y1, 0, sizeof(iir_y1));
    memset(iir_y2, 0, sizeof(iir_y2));
    memset(qDelayBuffer, 0, sizeof(qDelayBuffer));
    qDelayIndex = 0;
    lsb_lpf1_x1 = lsb_lpf1_x2 = lsb_lpf1_y1 = lsb_lpf1_y2 = 0.0f;
    lsb_lpf2_x1 = lsb_lpf2_x2 = lsb_lpf2_y1 = lsb_lpf2_y2 = 0.0f;
    lsbAgcGain  = 1.0f;       // Reset LSB AGC: prevents excessive gain after TX
    smoothMute  = 0.0f;       // Fade in when RX resumes
    lastRxPwm   = 2047.5f;    // Reset for the next TX-start crossfade
    wasTransmitting = false;
  }
  if (transmitting) { wasTransmitting = true; return; }

  // 1. 4x oversampling (+3dB SNR)
  long sumI = 0, sumQ = 0;
  for (int i = 0; i < 4; i++) {
    sumI += analogRead(inputPinI);
    sumQ += analogRead(inputPinQ);
  }
  float rawI = ((float)sumI / 4.0f / 2047.5f) - 1.0f;
  float rawQ = ((float)sumQ / 4.0f / 2047.5f) - 1.0f;

  // 2. Mute / DC offset correction
  if (muteCounter > 0) {
    muteCounter--;
    smoothMute *= 0.80f;
    rawI -= dcOffsetI;
    rawQ -= dcOffsetQ;
  } else {
    smoothMute  = smoothMute * 0.80f + 0.20f;
    dcOffsetI   = dcOffsetI * 0.999f + rawI * 0.001f;
    dcOffsetQ   = dcOffsetQ * 0.999f + rawQ * 0.001f;
    rawI -= dcOffsetI;
    rawQ -= dcOffsetQ;
  }

  float mutedI = rawI * smoothMute;
  float mutedQ = rawQ * smoothMute;

  // 3. Accumulate into the shared buffer for the FFT
  sharedBufferI[sharedIndex] = mutedI;
  sharedBufferQ[sharedIndex] = mutedQ;
  sharedIndex++;
  if (sharedIndex >= SAMPLES) {
    sharedIndex = 0;
    sharedBufferReady = true;
  }

  // 4. Demodulate with the mode-specific filter
  float demodulated;
  if (lsbMode) {
    // LSB mode: I-Q + 500-2400Hz BPF to select LSB
    demodulated = lsbDemodulate(mutedI, mutedQ);
  } else {
    // CW mode: 700Hz center +-100Hz narrow BPF
    demodulated = cwDemodulate(mutedI, mutedQ);
  }

  // 5. Only in CW mode: SNR detection + event generation (for auto-decode)
  if (!lsbMode) {
    updateCwDetector(demodulated);
  }

  // 6. Apply AGC (uses independent gain variables / parameters for CW and LSB)
  float agcOut;
  if (lsbMode) {
    // LSB-dedicated AGC: low maxGain(10) + fast response(0.08) to prevent ADC saturation distortion
    agcOut = (smoothMute < 0.9f) ? demodulated * lsbAgcGain : applyLsbAGC(demodulated);
  } else {
    // CW AGC: high maxGain(40) + weak-signal support (without losing sensitivity)
    agcOut = (smoothMute < 0.9f) ? demodulated * agcGain : applyAGC(demodulated);
  }
  agcOut *= volumeMultiplier;

  // 7. LSB mode: write the AGC-processed signal to the waveform display buffer with decimation
  // Decimate 40kHz to 1/32 -> a 1250Hz display sample rate
  // Displays about 102ms of waveform across 128 pixels.
  // Since it is post-AGC, the amplitude is normalized and is easy to view as a waveform.
  if (lsbMode) {
    static uint8_t waveDecim = 0;
    if (++waveDecim >= 32) {
      waveDecim = 0;
      lsbWaveBuf[lsbWaveWrIdx] = agcOut;  // Use the post-AGC value
      lsbWaveWrIdx = (lsbWaveWrIdx + 1) % LSB_WAVE_BUF_SIZE;
    }
  }

  // 8. PWM output (update lastRxPwm so it can be used for the crossfade in loop1())
  uint16_t pwmOut = (uint16_t)constrain((int)((agcOut + 1.0f) * 2047.5f), 0, 4095);
  lastRxPwm = (float)pwmOut;  // Hold for the TX-start crossfade
  analogWrite(speakerPin, pwmOut);
}

// ============================================================
// [12] *LSB TX phase modulation processing (Core1)
// ============================================================
// Implements the same principle as uSDX/QCX-SSB for the Pico.
// Applies a Hilbert transform to the microphone audio to obtain I/Q, converts the phase
// difference (dp) into an instantaneous frequency deviation, and updates the Si5351 CLK2
// frequency to generate the LSB carrier.
//
// [Update rate] 4000Hz (250us interval)
// [Si5351 update time] ~180us @400kHz I2C -> fits within the 250us interval
// [Audio band] 0-2kHz (Nyquist 2kHz)
//
// Hilbert transform (15-tap FIR): same coefficients as uSDX
//   q = ((v[0]-v[14])*2 + (v[2]-v[12])*8 + (v[4]-v[10])*21 + (v[6]-v[8])*16) / 64
//       + (v[6]-v[8])
//   i = v[7]  (center tap = 7-sample delay)
void handleLsbTxCore1() {
  // Microphone FIR buffer (15 taps)
  static float micBuf[LSB_TX_HILBERT_TAPS] = {0};
  static float prevPhase = 0.0f;

  // Sample the microphone (ADC2 = GPIO28)
  float micRaw = (float)analogRead(MIC_IN) / 2047.5f - 1.0f;

  // Remove DC offset (equivalent to uSDX's dc = (in + dc) / 2)
  // *[Important] Accurately tracking the DC offset of the microphone input
  // greatly improves the stability of the phase calculation and the accuracy of amplitude detection.
  static float micDC = 0.0f;
  micDC = micDC * 0.99609375f + micRaw * 0.00390625f;  // LPF: fc ~= 10Hz @ 4kHz
  float micAC = micRaw - micDC;  // Remove the DC component

  // Apply gain (equivalent to uSDX's * 2)
  // + amplification by PA_TX_DRIVE (equivalent to uSDX's << drive)
  float micGained = micAC * MIC_GAIN;
  micGained = micGained * (1 << PA_TX_DRIVE);  // PA_TX_DRIVE: left shift by 0-8 (x1, x2, x4, ... x256)

  // Soft clip: saturate at +-1.0 (minimizes distortion while preventing IIR oscillation from large overinput)
  // Equivalent to uSDX's lut[] table mapping
  if (micGained >  1.0f) micGained =  1.0f;
  if (micGained < -1.0f) micGained = -1.0f;

  float micAC_scaled = micGained;  // Input to the Hilbert transform

  // Write to the microphone waveform display buffer (decimate the 4kHz TX samples by 4 -> a 1kHz display rate)
  // Displays about 128ms of microphone waveform across 128 pixels.
  static uint8_t txWaveDecim = 0;
  if (++txWaveDecim >= 4) {
    txWaveDecim = 0;
    lsbWaveBuf[lsbWaveWrIdx] = micAC_scaled;  // Store the post-gain/post-shift value as-is
    lsbWaveWrIdx = (lsbWaveWrIdx + 1) % LSB_WAVE_BUF_SIZE;
  }

  // Shift the FIR buffer (append the newest sample at the end)
  // Hilbert transform: uses the same 15-tap coefficients as uSDX
  for (int j = 0; j < LSB_TX_HILBERT_TAPS - 1; j++) micBuf[j] = micBuf[j + 1];
  micBuf[LSB_TX_HILBERT_TAPS - 1] = micAC_scaled;

  // Generate I/Q with the Hilbert transform (15-tap FIR, uSDX coefficients)
  // I = center tap (= v[7], 7-sample delay)
  float i_val = micBuf[7];

  // Q = Hilbert transform output (odd-symmetric FIR)
  // Q = ((v[0]-v[14])*2 + (v[2]-v[12])*8 + (v[4]-v[10])*21 + (v[6]-v[8])*16) / 64
  //     + (v[6]-v[8])
  float q_val = ((micBuf[0]  - micBuf[14]) * 2.0f
               + (micBuf[2]  - micBuf[12]) * 8.0f
               + (micBuf[4]  - micBuf[10]) * 21.0f
               + (micBuf[6]  - micBuf[8])  * 16.0f) / 64.0f
               + (micBuf[6]  - micBuf[8]);

  // =========================================================
  // PA bias control (GPIO17 PWM) - equivalent to uSDX's amp (vibration amplitude)
  // =========================================================
  // [Implementation synchronized with uSDX]
  //   uSDX: _amp = magn(i/2, q/2)
  //         _amp = _amp << drive           (amplification by PA_TX_DRIVE)
  //         _amp = (_amp > 255) ? 255 : _amp  (clip)
  //         amp = (tx) ? lut[_amp] : 0
  //
  // Pico: magnitude of the I/Q vector = instantaneous audio amplitude
  //      alpha-max plus beta-min approximation (no sqrt needed, error < 12%):
  //      amplitude ~= max(|I|, |Q|) + 0.5 x min(|I|, |Q|)
  //
  // Since the microphone gain + PA_TX_DRIVE are already reflected in micAC_scaled,
  // the value can be converted to a PWM value immediately after computing the amplitude.
  {
    float absI = fabsf(i_val);
    float absQ = fabsf(q_val);
    float amplitude = (absI > absQ) ? absI + 0.5f * absQ
                                    : absQ + 0.5f * absI;

    // Convert directly to a PWM value (no envelope follower).
    // As in uSDX, this achieves PA gate control that responds directly to the amplitude.
    // This accurately reflects changes in the instantaneous audio amplitude in the RF envelope.
    //
    // Amplitude 0.0-1.0 (normalized by the clip below) -> converted to PWM 0-4095
    // Scaled by PA_BIAS_SCALE (to match the gain characteristics of the PA circuit)
    int amplitude_clipped = (int)constrain(amplitude * 1024.0f, 0.0f, 1024.0f);  // Normalize to 0-1024
    int paBias = (int)constrain((float)amplitude_clipped * PA_BIAS_SCALE, 0.0f, 4095.0f);

    // *Keep a tiny bias even at zero amplitude for VOX (voice activation) (comment out if not needed)
    // if (paBias < 10) paBias = 0;  // Off below 10

    analogWrite(PA_BIAS_PIN, paBias);
  }

  // =========================================================
  // Phase modulation - instantaneous phase difference -> write txFreqShared (no I2C)
  // Core0's loop() reads txFreqShared and updates Si5351 CLK2
  // =========================================================
  // Compute the instantaneous phase
  float phase = atan2f(q_val, i_val);

  // Find the phase difference dp and normalize it to [-pi, pi]
  float dp = phase - prevPhase;
  prevPhase = phase;
  if (dp >  (float)PI) dp -= 2.0f * (float)PI;
  if (dp < -(float)PI) dp += 2.0f * (float)PI;

  // Phase difference -> instantaneous frequency deviation [Hz]
  // df = dp / (2pi) x sample rate
  // LSB is the lower sideband of the carrier -> invert the sign to modulate downward
  float df = dp / (2.0f * (float)PI) * (float)LSB_TX_SAMPLE_RATE;
  long long freqShift = (long long)(df * 100.0f);  // Convert to centihz

  // Compute the target frequency and share it with Core0 (Core0 updates the Si5351 over I2C)
  // * Do not call si5351.set_freq_manual() here * (I2C is concentrated on Core0)
  long long txFreqLL = (long long)FREQ_ULL - freqShift;

  // Clamp the frequency range (within +-5kHz)
  long long minF = (long long)FREQ_ULL - 500000LL;
  long long maxF = (long long)FREQ_ULL + 500000LL;
  txFreqLL = constrain(txFreqLL, minF, maxF);

  // uint32_t atomic write (Cortex-M0+ does a 32bit store in one instruction = safe)
  // The 40m band centihz value is ~700M < 2^30, so it fits in uint32_t
  txFreqShared = (uint32_t)txFreqLL;
}

// ============================================================
// [13] CW signal detection / event generation (called from Core1)
// ============================================================

// Computes the SNR ratio of the CW signal and writes to the event buffer.
// Called in CW mode inside handleRxCore1().
void updateCwDetector(float demodulated) {
  // --- SNR ratio calculation ---
  static float cwEnvLP = 0.0f;
  static float cwNoise = 0.05f;
  float absD = fabsf(demodulated);

  // Asymmetric LPF: attack ~0.5ms, decay ~12ms
  cwEnvLP = (absD > cwEnvLP)
            ? cwEnvLP * 0.95f  + absD * 0.05f
            : cwEnvLP * 0.998f + absD * 0.002f;

  // Noise floor tracking (not updated while a signal is present)
  float ratio = cwEnvLP / max(cwNoise, 0.0001f);
  if (ratio < 1.5f)
    cwNoise = cwNoise * 0.9995f + cwEnvLP * 0.0005f;  // Noise convergence ~50ms
  else
    cwNoise = cwNoise * 0.99999f + cwEnvLP * 0.00001f; // No change while a signal is present

  cwEnvelope = ratio;  // Share with Core0 (CW_DETECT_THRESHOLD or higher = CW signal present)

  // --- CW event generation (debounce 80 samples = 2ms) ---
  static bool     cwSt  = false;
  static uint32_t cwCnt = 0;
  static uint8_t  cwDB  = 0;
  const  uint8_t  DB_TH = 80;

  bool raw = (cwEnvelope > CW_DETECT_THRESHOLD);

  if (raw != cwSt) {
    if (++cwDB >= DB_TH) {
      // Debounce confirmed -> write the event to the ring buffer
      uint16_t ms = (uint16_t)min(cwCnt / 40UL, 60000UL);
      uint8_t nw = (cwEvWr + 1) % CW_EVBUF_SIZE;
      if (nw != cwEvRd) {
        cwEvBuf[cwEvWr].type  = cwSt ? (uint8_t)1 : (uint8_t)0;
        cwEvBuf[cwEvWr].durMs = ms;
        cwEvWr = nw;
      }
      cwSt = raw;  cwCnt = 0;  cwDB = 0;
    }
  } else {
    cwDB = 0;
  }
  cwCnt++;
}

// ============================================================
// [14] CW auto-decode (runs on Core0)
// ============================================================

// Morse code table
struct MorseEntry { const char* code; char ch; };
const MorseEntry MORSE_TABLE[] = {
  {".-",   'A'}, {"-...", 'B'}, {"-.-.", 'C'}, {"-..",  'D'}, {".",    'E'},
  {"..-.", 'F'}, {"--.",  'G'}, {"....", 'H'}, {"..",   'I'}, {".---", 'J'},
  {"-.-",  'K'}, {".-..", 'L'}, {"--",   'M'}, {"-.",   'N'}, {"---",  'O'},
  {".--.", 'P'}, {"--.-", 'Q'}, {".-.",  'R'}, {"...",  'S'}, {"-",    'T'},
  {"..-",  'U'}, {"...-", 'V'}, {".--",  'W'}, {"-..-", 'X'}, {"-.--", 'Y'},
  {"--..", 'Z'},
  {"-----",'0'}, {".----",'1'}, {"..---",'2'}, {"...--",'3'}, {"....-",'4'},
  {".....", '5'}, {"-....", '6'}, {"--...", '7'}, {"---..", '8'}, {"----.", '9'},
  {".-.-.-",'.'}, {"--..--",','}, {"..--..",'?'}, {"-..-.", '/'},
  {"-....-",'-'}, {".----.",'\''},{".-..-.",'"'}, {"---...",':'},
  {"-.-.--",'!'}, {".--.-.",'@'},
  {".-.-.",  '+'}, {"-...-",  '='}, {"-.--.",  '('}, {".-...",  '~'}, {"...-.-", '*'},
  {nullptr, '\0'}
};

char decodeMorse(const char* code) {
  for (int i = 0; MORSE_TABLE[i].code != nullptr; i++) {
    if (strcmp(code, MORSE_TABLE[i].code) == 0) return MORSE_TABLE[i].ch;
  }
  return '?';
}

void addCWDecodedChar(char c) {
  if (c == '\0') return;
  if (cwDecodedLen < CW_DECODED_MAX) {
    cwDecodedBuf[cwDecodedLen++] = c;
    cwDecodedBuf[cwDecodedLen]   = '\0';
  } else {
    memmove(cwDecodedBuf, cwDecodedBuf + 1, CW_DECODED_MAX - 1);
    cwDecodedBuf[CW_DECODED_MAX - 1] = c;
    cwDecodedBuf[CW_DECODED_MAX]     = '\0';
  }
}

// CW decoder core (event-driven method)
// Consumes the mark/space length events measured by Core1 to confirm characters.
void handleCWDecoder() {
  if (transmitting || lsbMode) { cwEvRd = cwEvWr; return; }

  static float dotEst      = 60.0f;   // Estimated dot length [ms] (initial = 20WPM)
  static char  morse[10]   = "";
  static int   morseLen    = 0;
  static bool  charDecoded = false;
  static bool  wordAdded   = false;
  static unsigned long keyUpMs = 0;
  static bool  keyIsUp = true;

  unsigned long nowMs = millis();

  // Consume all events from Core1
  while (cwEvRd != cwEvWr) {
    CWEvent ev;
    ev.type  = cwEvBuf[cwEvRd].type;
    ev.durMs = cwEvBuf[cwEvRd].durMs;
    cwEvRd   = (cwEvRd + 1) % CW_EVBUF_SIZE;

    if (ev.type == 1) {
      // Mark end -> dot/dash determination
      uint16_t dur = ev.durMs;
      if (dur >= (uint16_t)(dotEst * 0.3f) && morseLen < 8) {
        if (dur < (uint16_t)(dotEst * 2.0f)) {
          morse[morseLen++] = '.';
          dotEst = dotEst * 0.85f + (float)dur * 0.15f;
        } else {
          morse[morseLen++] = '-';
          dotEst = dotEst * 0.85f + ((float)dur / 3.0f) * 0.15f;
        }
        morse[morseLen] = '\0';
        dotEst = constrain(dotEst, 30.0f, 240.0f);
      }
      keyUpMs = nowMs;  keyIsUp = true;
      charDecoded = wordAdded = false;
    } else {
      keyIsUp = false;
    }
  }

  // Character/word confirmation by silence timeout
  if (keyIsUp) {
    unsigned long silMs = nowMs - keyUpMs;

    // Inter-character space: dot length x2.5 or more -> confirm one character
    if (!charDecoded && morseLen > 0 && silMs >= (unsigned long)(dotEst * 2.5f)) {
      addCWDecodedChar(decodeMorse(morse));
      morseLen = 0;  morse[0] = '\0';
      charDecoded = true;
    }

    // Inter-word space: dot length x6 or more -> add a space
    if (charDecoded && !wordAdded && silMs >= (unsigned long)(dotEst * 6.0f)) {
      addCWDecodedChar(' ');
      wordAdded = true;
    }
  }
}
