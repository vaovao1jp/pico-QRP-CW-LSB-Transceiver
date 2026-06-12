/*
  ============================================================
  pico_40m_CW_LSB_v1_1J.ino
  40m CW/LSB デュアルモード トランシーバー
  Raspberry Pi Pico2 向け
  ============================================================

  【基本仕様】
  ・CWモード  : IQバンドスコープ + キーヤー + CW自動解読 + ウォーターフォール
  ・LSBモード : SSB送受信（Si5351位相変調TX / ヒルベルト変換RX）
  ・モード切替 : MODE_BUTTON（ロータリーエンコーダー付属ボタン）短押し

  【ピン配置】
  GPIO 0,1  : ロータリーエンコーダー A/B
  GPIO 2    : STEP_BUTTON  (周波数ステップ / WPM設定)
  GPIO 3    : KEY_MODE_BUTTON (キーヤーモード / 音量)
  GPIO 6    : PADDLE_DOT
  GPIO 7    : PADDLE_DASH
  GPIO 8    : MODE_BUTTON  (CW/LSBモード切替)
  GPIO 9    : PTT_BUTTON   (LSB プッシュトーク)
  GPIO 15   : RX_SW (送受信切替, High=TX)
  GPIO 16   : speakerPin (PWM音声出力)
  GPIO 17   : PA_BIAS_PIN (PAバイアス制御 PWM, LSB TX 振幅制御)
  GPIO 25   : LED_INDICATOR
  GPIO 26   : I-ch アナログ入力 (ADC0)
  GPIO 27   : Q-ch アナログ入力 (ADC1)
  GPIO 28   : MIC_IN  マイク入力 (ADC2)

  【LSB TX方式】
  uSDX/QCX-SSBと同じ Si5351位相変調方式をPico向けに移植。
  マイク音声にヒルベルト変換を施してI/Q取得、位相差→瞬時周波数偏移に変換し
  CLK2の周波数を4kHzレートで更新することでLSBキャリアを生成する。

  【使用ライブラリ】
  Rotary.h     : https://github.com/brianlow/Rotary
  U8g2lib.h    : https://github.com/olikraus/U8g2_Arduino
  arduinoFFT.h : v2.0.4
  si5351.h     : https://github.com/etherkit/Si5351Arduino
  EEPROM.h     (Pico用内蔵)
  Wire.h
*/

#include <Arduino.h>
#include <Rotary.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <arduinoFFT.h>
#include <si5351.h>
#include <EEPROM.h>

// --- 前方宣言 (Arduino IDE の自動生成に頼らず明示的に宣言) ---
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
// [1] ピン定義と基本定数
// ============================================================

// --- アナログ入力ピン ---
#define I_IN              26   // I-ch アナログ入力 (ADC0)
#define Q_IN              27   // Q-ch アナログ入力 (ADC1)
#define MIC_IN            28   // マイク入力 (ADC2) ★LSB TX用
#define inputPinI         26
#define inputPinQ         27

// --- デジタルピン ---
#define speakerPin        16   // PWM音声出力
#define PIN_IN1           0    // ロータリーエンコーダー A
#define PIN_IN2           1    // ロータリーエンコーダー B
#define STEP_BUTTON       2    // 周波数ステップ / WPM設定 (長押し)
#define KEY_MODE_BUTTON   3    // キーヤーモード / 音量調節 (長押し)
#define MODE_BUTTON       8    // CW/LSBモード切替ボタン
#define PTT_BUTTON        9    // LSB PTTボタン (プッシュトーク)
#define PADDLE_DOT        6    // ドット パドル (CWモード)
#define PADDLE_DASH       7    // ダッシュ パドル / 縦振り電鍵 (CWモード)
#define PA_BIAS_PIN       17   // ★PAバイアス制御 PWM出力 (GPIO17, LSB TX時の振幅制御)
                               //   GPIO16(speakerPin)と同じPWMスライス(slice0)を共有する。
                               //   デューティサイクルは独立制御可能で周波数は44100Hzを共用。
                               //   PA最終段のゲートバイアスをこのPWMで制御することで
                               //   音声振幅に応じたRF出力振幅制御(=キャリア抑圧SSB)を実現する。
#define RX_SW             15   // 送受信切替 (High=TX)
#define LED_INDICATOR     25   // 処理インジケーターLED

// --- SDR・信号処理 ---
#define SAMPLES           256      // FFTサンプル数
#define sampleRate        40000    // 実効サンプリング周波数 [Hz]
#define pwmFrequency      44100    // PWM搬送周波数 [Hz]
#define CW_AUDIO_OFFSET   70000ULL // CWモード時のBFOオフセット [centihz/100 → Hz = 700Hz]
#define CW_TONE           700      // CWモニター音周波数 [Hz]

// --- LSB TX 設定 ---
// 1サンプルあたり250µs(4kHz)で Si5351 CLK2を更新する。
// I2C 400kHz時の1回更新所要時間 ≒ 180µs のため250µsで十分な余裕がある。
#define LSB_TX_SAMPLE_RATE   4000  // LSB TX サンプリングレート [Hz]
#define LSB_TX_INTERVAL_US   250   // LSB TX サンプリング間隔 [µs]
#define LSB_TX_HILBERT_TAPS  15    // ヒルベルト変換FIRタップ数

// --- マイクゲインと PA 駆動力 ---
// エレクトレットマイクの出力は非常に微弱（±10〜50mV）。
// uSDXでは MORE_MIC_GAIN で 2倍増幅 + drive パラメータで 0-8段階のゲイン調整を行う。
// Pico でも同様に、マイク入力→振幅計算→PAゲイン反映 という正確な流れが必須。
//
// ★ 実際のマイク回路に合わせてここを調整する ★
//   MIC_GAIN:
//     5-10:   アンプ内蔵マイクモジュール使用時
//     20-30:  裸のエレクトレットマイク直結時
//     40-50:  感度の低いマイク使用時
//
//   PA_TX_DRIVE: PA最大出力パワー（uSDXの drive パラメータに相当）
//     1-2:   QRP（低パワー，歪み最小）
//     3-4:   低パワー
//     5-6:   高パワー（より強い RF 出力）
//     7-8:   最大パワー（PA保護回路が必須）
//
#define MIC_GAIN              10.0f  // マイク入力ゲイン倍率（現在値 10.0 = 高感度設定。uSDX標準は 2.0）
#define PA_TX_DRIVE            2     // PA 駆動力 0-8（現在値 2 = QRP。1サンプルあたり 2^drive 倍に左シフト）

// --- PAバイアス制御 (GPIO17 PWM) ---
// LSB送信中は handleLsbTxCore1() がマイク音声の瞬時振幅(I/Q magnitude)を
// PAゲートバイアス(PWMデューティ)へ直接変換し、キャリア抑圧と振幅制御を実現する。
//
// 【PA_BIAS_SCALE】: 振幅→PWMデューティのスケール係数（送信処理で使用）
//   ★ PA回路に合わせてここを調整する ★
//   小さい値: RF出力パワー小（PA保護、オーバードライブ防止）
//   大きい値: RF出力パワー大（PAの最大定格を超えないように注意）
//   推奨初期値: 2.5f →実際のPAで歪みを確認しながら調整
//
// 【PA_BIAS_ENV_ATTACK / PA_BIAS_ENV_DECAY】
//   エンベロープフォロワー用の係数。現在の送信処理は振幅を直接PWMへ
//   変換する方式のため未使用（エンベロープ平滑を行う場合の参考値として残置）。
//
// PWM出力: 12bit(0-4095), 44100Hz
//   0    = PA完全OFF (キャリア抑圧, 無音時・RX時)
//   4095 = 最大バイアス (フルパワー)
#define PA_BIAS_SCALE        2.5f   // ★PA回路に合わせて調整（振幅→PWMスケール）
#define PA_BIAS_ENV_ATTACK   0.30f  // 未使用（エンベロープ立ち上がり係数の参考値）
#define PA_BIAS_ENV_DECAY    0.008f // 未使用（エンベロープ立ち下がり係数の参考値）

// CW TX 時のPAバイアスレベル (固定値)
// CWモードでは音声変調が不要なためPAを常時フルバイアスにし、
// RF出力のON/OFFはSi5351 CLK2のenable/disableで制御する。
// PA回路に応じて最大定格を超えない範囲で調整すること。
// GPIO17が未接続の場合は効果なし（CW TX動作に影響しない）。
#define CW_PA_BIAS_LEVEL     4095   // ★CW TX時のPAバイアス (12bit最大値=フルバイアス)

// --- LSB RX フィルタ係数 (HPF 500Hz 2次 + LPF 2400Hz 4次 = 帯域 500-2400Hz @ fs=40kHz) ---
// I−Q で得たLSB音声を、低域ハム・DC成分をHPFで、高域ノイズ・エイリアスをLPFで除去する。
//
// Stage1: 2次バターワース HPF 500Hz
//   K = tan(π×500/40000) = 0.039293
//   norm = 1 + K√2 + K² = 1.057144
//   b = [1/norm, -2/norm, 1/norm] = [0.9460, -1.8919, 0.9460]
//   a = [1, 2(K²-1)/norm, (1-K√2+K²)/norm] = [1, -1.8896, 0.8949]
//
// Stage2+3: 4次バターワース LPF 2400Hz (2つのBiquadを直列接続)

// --- Stage1: HPF 500Hz 2次バターワース (低域カット用) ---
#define LSB_HPF_B0   0.9460f
#define LSB_HPF_B1  -1.8919f  // = -2 * b0
#define LSB_HPF_B2   0.9460f  // = b0
#define LSB_HPF_A1  -1.8896f
#define LSB_HPF_A2   0.8949f

// --- Stage2: LPF 2400Hz 4次バターワース (前半 Biquad 1, Q=0.54) ---
#define LSB_LPF1_B0   0.02620f
#define LSB_LPF1_B1   0.05240f
#define LSB_LPF1_B2   0.02620f
#define LSB_LPF1_A1  -1.38763f
#define LSB_LPF1_A2   0.49243f

// --- Stage3: LPF 2400Hz 4次バターワース (後半 Biquad 2, Q=1.31) ---
#define LSB_LPF2_B0   0.03078f
#define LSB_LPF2_B1   0.06156f
#define LSB_LPF2_B2   0.03078f
#define LSB_LPF2_A1  -1.63000f
#define LSB_LPF2_A2   0.75305f

// LSB AGC 設定 (CWとは独立した設定)
// CW : maxGain=40, attackRate=0.01 (微弱信号を最大増幅)
// LSB: maxGain=10, attackRate=0.08 (強信号での歪みを防ぐため速い応答・低ゲイン上限)
#define LSB_AGC_MAX_GAIN    10.0f   // CWの約1/4: 強信号ADC歪みを防ぐ
#define LSB_AGC_MIN_GAIN     0.1f
#define LSB_AGC_TARGET       0.35f  // CW(0.5)より低め: クリッピングマージン確保
#define LSB_AGC_ATTACK       0.08f  // CW(0.01)の8倍: 強信号に素早く反応
#define LSB_AGC_DECAY        0.002f

// LSB ソフトクリッパー閾値
// フィルタ入力が±この値を超えた場合に滑らかに制限をかける
// ADC過入力による急激な信号でIIRフィルタが発振するのを防ぐ
#define LSB_SOFT_CLIP_THRESH 0.7f

// --- デフォルト設定 ---
const long LOW_FREQ     = 7000000;
const long HI_FREQ      = 7200000;
#define DEFAULT_WPM       20

// --- EEPROM アドレス ---
const int EEPROM_ADDR_FREQ    = 0;
const int EEPROM_ADDR_STEP    = 4;
const int EEPROM_ADDR_WPM     = 8;
const int EEPROM_ADDR_KEYMODE = 12;
const int EEPROM_ADDR_VOL     = 16;
const int EEPROM_ADDR_MODE    = 20;   // CW(0)/LSB(1) モード保存

// ============================================================
// [2] グローバル変数
// ============================================================

// --- 周波数・VFO ---
unsigned long FREQ     = 7000000;
unsigned long FREQ_OLD = FREQ;
unsigned long long FREQ_ULL  = 700000000ULL;  // centihz 表現
unsigned long long pll_freq  = 86400000000ULL; // PLLVCOクロック [centihz]
int STEP     = 1000;
int stepMode = 0;  // 0=1kHz, 1=100Hz, 2=10Hz

// --- モード ---
volatile bool lsbMode = false;   // false=CW, true=LSB

// --- バンドスコープ設定 ---
// SCOPE_SPAN_HZ    : 横軸の表示範囲(±Hz)。小さくすると拡大表示になる。
// SCOPE_SENSITIVITY: 振幅方向のゲイン。大きくすると弱い信号も高く描画される(ノイズも増える)。
// SCOPE_OFFSET     : ノイズフロアの持ち上げ量。小さくするとノイズフロアの表示が下がる。
// ※ ウォーターフォールも同じ振幅データ(waterfallHistory)を共有するため、
//    これら3つを変えると下段のウォーターフォール濃度も連動して変化する。
#define SCOPE_SPAN_HZ     15000.0f  // 表示範囲 ±15kHz
#define SCOPE_SENSITIVITY 16.0f     // 振幅ゲイン
#define SCOPE_OFFSET      3.0f      // ノイズフロア持ち上げ量

// --- 送受信状態 ---
// ★★ volatile 必須 ★★
// Core0 (loop) が startTransmit() で true にセットし、
// Core1 (loop1) の handleLsbTxCore1() 呼び出し条件 "lsbMode && transmitting" で読む。
// volatile がないとコンパイラの最適化でCore1がレジスタキャッシュ値を使い続け、
// Core0 からの変更を永遠に読み取れない → LSB TX が一切動作しない根本原因。
// (CW TX はCore0だけで完結するため volatile なしでも動作する)
volatile bool transmitting = false;
volatile int muteCounter = 0;

// --- オーディオ処理 ---
volatile float volumeMultiplier = 1.0f;
float agcGain  = 1.0f;
float dcOffsetI = 0.0f;
float dcOffsetQ = 0.0f;

// --- CW用 IIR BPFフィルタ状態変数 (2段4次, ±100Hz @700Hz) ---
float iir_x1[3] = {0}, iir_x2[3] = {0};
float iir_y1[3] = {0}, iir_y2[3] = {0};

// --- LSB RX用 フィルタ状態変数 ---
// Stage1: HPF 500Hz 2次
float lsb_hpf_x1 = 0.0f, lsb_hpf_x2 = 0.0f;
float lsb_hpf_y1 = 0.0f, lsb_hpf_y2 = 0.0f;

// Stage2: LPF 2400Hz 4次 (前半)
float lsb_lpf1_x1 = 0.0f, lsb_lpf1_x2 = 0.0f;
float lsb_lpf1_y1 = 0.0f, lsb_lpf1_y2 = 0.0f;

// Stage3: LPF 2400Hz 4次 (後半)
float lsb_lpf2_x1 = 0.0f, lsb_lpf2_x2 = 0.0f;
float lsb_lpf2_y1 = 0.0f, lsb_lpf2_y2 = 0.0f;

// LSB専用AGCゲイン (CW/LSBで独立管理 → 切替時に前のモードのゲインが引き継がれない)
float lsbAgcGain = 1.0f;

// IQ振幅バランス補正係数 (Core0→Core1 共有)
// Core0のFFT処理でI/Qの実効値比からqGainを計算し、ここに書き込む。
// Core1のlsbDemodulate()でQチャンネルの振幅補正に使用する。
// I/Q振幅が一致していないと「image rejection」が低下し、対側波帯(USB)が
// ノイズやビート音として聞こえる原因になる。
// float は32bitのPicoでは1命令で読み書き可能なため volatile で安全に共有できる。
volatile float lsbQGain = 1.0f;

// --- Q信号90度遅延バッファ (ヒルベルト変換近似) ---
// 40kHz で 700Hz を90度遅延させる最適値 = 14サンプル
#define Q_DELAY_SAMPLES 14
float qDelayBuffer[Q_DELAY_SAMPLES] = {0};
int   qDelayIndex = 0;

// --- コア間データ共有 (Core1 → Core0, FFT用) ---
float sharedBufferI[SAMPLES];
float sharedBufferQ[SAMPLES];
volatile bool sharedBufferReady = false;
int sharedIndex = 0;

// --- CW解読用 ---
volatile float cwEnvelope = 0.0f;     // SNR比 (CW_DETECT_THRESHOLD 以上 = 信号あり)
// CW解読の検出感度: cwEnvelope(SNR比)がこの値を超えるとキーダウンと判定する。
// 小さくすると弱い信号でも拾うが誤検出(ノイズ)が増える。大きくすると確実だが取りこぼす。
#define CW_DETECT_THRESHOLD 1.5f      // CW検出SNRしきい値 (推奨 1.5〜2.5)
#define CW_DECODED_MAX      36
char cwDecodedBuf[CW_DECODED_MAX + 1] = "";
int  cwDecodedLen = 0;

// CWイベントバッファ (Core1→Core0, リングバッファ)
struct CWEvent {
  uint8_t  type;    // 0=スペース終了(キーダウン), 1=マーク終了(キーアップ)
  uint16_t durMs;
};
#define CW_EVBUF_SIZE 16
volatile CWEvent cwEvBuf[CW_EVBUF_SIZE];
volatile uint8_t cwEvWr = 0;
volatile uint8_t cwEvRd = 0;

// --- LSB TX 周波数共有変数 (Core1 → Core0) ---
// Core1がヒルベルト変換で計算した瞬時周波数をここに書き込む。
// Core0がこの値を読んでSi5351 CLK2をI2C更新する（I2Cを一元管理するため）。
//
// 設計方針:
//   I2CはすべてCore0で管理する。Core1から直接 si5351.set_freq_manual()（I2C）を
//   呼ぶと、Wire.begin()がCore0で初期化されているため呼び出しが不安定になり、
//   stopTransmit()のI2C（Core0）と衝突してCLK2が正しく無効化されず、
//   送信後にバンド内に固定ノイズが残る。そのためCore1はI2Cを一切呼ばず、
//   計算した周波数をこの共有変数経由でCore0へ渡す。
//
// uint32_t で十分な理由:
//   40m帯の centihz 表現 = 700000000〜720500000 ≒ 720M < 2^30 → uint32_t に収まる
//   Cortex-M0+ では 32bit ストアが1命令 = アトミック書き込みが保証される
volatile uint32_t txFreqShared = 0;  // Core1が書く、Core0が読む

// --- LSB 波形表示用共有バッファ (Core1 → Core0) ---
// Core1がオーディオサンプルを書き込み、Core0が画面下10pxに描画する
#define LSB_WAVE_BUF_SIZE 128
volatile float lsbWaveBuf[LSB_WAVE_BUF_SIZE];
volatile uint8_t lsbWaveWrIdx = 0;   // 書き込みインデックス (Core1更新)

// --- FFT・スコープ用 ---
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

// --- CW TX エンベロープ (Core1 状態, loop1内で管理) ---
// CW TX フェードイン/アウトをトランスミッタ制御から分離するため
// グローバルに保持する。Core1のみが書き込む。
volatile float txEnvelope = 0.0f;

// --- キーヤー ---
int  wpm = DEFAULT_WPM;
bool straightKeyMode = false;
bool sending = false;
bool sendingDot = false;
bool sendingDash = false;
unsigned long dotDuration, dashDuration;
unsigned long elementSpace, charSpace, wordSpace;

// --- ウォーターフォール ---
#define WATERFALL_HEIGHT 21   // 画面y=27〜47に表示
#define THRESHOLD        2
uint8_t waterfallHistory[WATERFALL_HEIGHT][128] = {0};
int waterfallIndex = 0;

// --- オブジェクト ---
Rotary r = Rotary(PIN_IN1, PIN_IN2);
Si5351 si5351;
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
ArduinoFFT<double> FFT;

// ============================================================
// [3] EEPROM・ハードウェア制御
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

// CW/LSBモード保存・読み込み
void saveModeToEEPROM(bool isLsb)               { saveToEEPROM(EEPROM_ADDR_MODE, isLsb ? 1 : 0); }
bool readModeFromEEPROM()                        { return readFromEEPROM(EEPROM_ADDR_MODE) == 1; }

// ============================================================
// [4] Si5351 周波数制御
// ============================================================

// CWモード用: CLK0/CLK1 を BFOオフセット分下にずらし、受信信号が700Hzトーンとして聞こえるようにする
void Freq_Set_CW() {
  unsigned long long lo_freq = FREQ_ULL - CW_AUDIO_OFFSET;  // centihz
  si5351.set_freq_manual(lo_freq, pll_freq, SI5351_CLK0);
  si5351.set_freq_manual(lo_freq, pll_freq, SI5351_CLK1);
  // 90度位相差設定 (IQミキサー用)
  int phase = (int)((float)pll_freq / (float)lo_freq + 0.5f);
  si5351.set_phase(SI5351_CLK0, 0);
  si5351.set_phase(SI5351_CLK1, phase);
  si5351.pll_reset(SI5351_PLLA);
}

// LSBモード用: CLK0/CLK1 を搬送波周波数に設定 (オフセットなし)
// 受信音声は I−Q でLSBが選択される
void Freq_Set_LSB() {
  si5351.set_freq_manual(FREQ_ULL, pll_freq, SI5351_CLK0);
  si5351.set_freq_manual(FREQ_ULL, pll_freq, SI5351_CLK1);
  // 90度位相差設定
  int phase = (int)((float)pll_freq / (float)FREQ_ULL + 0.5f);
  si5351.set_phase(SI5351_CLK0, 0);
  si5351.set_phase(SI5351_CLK1, phase);
  si5351.pll_reset(SI5351_PLLA);
}

// モードに応じた周波数設定
void Freq_Set() {
  if (lsbMode) Freq_Set_LSB();
  else         Freq_Set_CW();
}

// 送信開始
// CWモード : PAバイアスをCW_PA_BIAS_LEVEL（フルバイアス）に設定する。
//            setup1()でGPIO17を0に初期化しているため、この設定がないとPA
//            がバイアスOFF状態のまま → RF出力が極端に微弱になる原因。
//            RF出力のON/OFFはSi5351 CLK2のenable/disableで制御する。
// LSBモード: PAバイアスを0から開始し、送信中は handleLsbTxCore1() が
//            マイク音声の瞬時振幅に応じてPWMを直接更新する（キャリア抑圧SSB動作）。
void startTransmit() {
  if (!transmitting) {
    transmitting = true;
    si5351.output_enable(SI5351_CLK0, 0);  // RX LO OFF
    si5351.output_enable(SI5351_CLK1, 0);
    digitalWrite(RX_SW, HIGH);             // アンテナをTX側へ切替
    si5351.set_freq_manual(FREQ_ULL, pll_freq, SI5351_CLK2);
    si5351.output_enable(SI5351_CLK2, 1);  // TX搬送波 ON
    if (lsbMode) {
      analogWrite(PA_BIAS_PIN, 0);              // LSB: 0から開始、エンベロープが立ち上げる
    } else {
      analogWrite(PA_BIAS_PIN, CW_PA_BIAS_LEVEL); // CW: PAをフルバイアスにする
    }
  }
}

// 送信停止
// CW/LSB両モードでPAバイアスをただちに0にしてRF出力を遮断する。
// LSBモードでは追加クリーンアップ:
//   1. txFreqSharedをクリア（Core0の更新ループが古い周波数を使わないよう）
//   2. Si5351をRX用に完全再設定（CLK2が残留しないよう確実にリセット）
//   3. muteCounterを大きめに設定（TX→RX遷移のノイズを確実に抑制）
void stopTransmit() {
  if (transmitting) {
    transmitting = false;
    txFreqShared = 0;                      // LSB TX周波数共有バッファをクリア
    analogWrite(PA_BIAS_PIN, 0);           // PA OFF（CW/LSB共通）
    si5351.output_enable(SI5351_CLK2, 0);  // TX搬送波 OFF ← これが確実に動く（Core1 I2C不在）
    si5351.output_enable(SI5351_CLK0, 1);  // RX LO ON
    si5351.output_enable(SI5351_CLK1, 1);
    digitalWrite(RX_SW, LOW);             // アンテナをRX側へ切替
    if (lsbMode) {
      // LSB後の残留ノイズ対策:
      // set_freq_manual中にI2C衝突が起きるとCLK2が不正な状態になる可能性があるため、
      // Freq_Set_LSB()で Si5351全体をクリーンなRX状態に再設定する。
      // （CLK2は上でoffにしたが、内部レジスタが乱れた場合の保険）
      muteCounter = 1200;  // ~30ms ミュート: TX→RX切替ノイズを完全に抑制
      Freq_Set_LSB();      // Si5351 CLK0/CLK1 を確実にRX周波数に戻す
    }
  }
}

// ============================================================
// [5] 信号処理 (DSP)
// ============================================================

// --- CW復調 ---
// IQヒルベルト変換 + 2段直列 IIR 2次BPF でCW信号を抽出する（700Hz中心 ±100Hz）。
//
// Si5351 の周波数精度は ±10ppm（7MHz帯で ±70Hz の誤差）があるため、
// ±100Hz とやや広めの帯域にして周波数偏差による感度変動を抑えている。
//
// フィルタ係数早見表 (中心700Hz, fs=40kHz):
//   ±100Hz: b0=0.024122, a1=1.939356, a2=0.951756 (現在値)
//   ±50Hz : b0=0.015134, a1=1.957930, a2=0.969710 (狭帯域)
//   ±30Hz : b0=0.009137, a1=1.970800, a2=0.981730 (超狭帯域)
float cwDemodulate(float iSig, float qSig) {
  // Q信号を14サンプル遅延させることで700Hz付近の90度位相差を得る
  float qDelayed = qDelayBuffer[qDelayIndex];
  qDelayBuffer[qDelayIndex] = qSig;
  qDelayIndex = (qDelayIndex + 1 >= Q_DELAY_SAMPLES) ? 0 : qDelayIndex + 1;

  // LSB選択: I − Q_delayed
  float audio = iSig + qDelayed;

  // 2段直列 IIR 2次BPF (700Hz中心, ±100Hz帯域)
  // やや広めの帯域で感度と周波数偏差への耐性を両立する
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

// --- LSBソフトクリッパー ---
// フィルタへの入力振幅を LSB_SOFT_CLIP_THRESH 以内に滑らかに制限する。
// 急峻な大振幅入力でIIRフィルタが一時的に発振するのを防ぎ、
// 強信号時の高音ノイズ(歪み成分)を大幅に抑制する。
// ハードクリップと違い滑らかな曲線なので音質劣化が少ない。
inline float lsbSoftClip(float x) {
  const float th = LSB_SOFT_CLIP_THRESH;
  if (x >  th) return  th + (x - th) / (1.0f + (x - th) * (x - th));
  if (x < -th) return -th - (-x - th) / (1.0f + (-x - th) * (-x - th));
  return x;
}

// --- LSB受信復調 ---
// Tayloe検波器(IQミキサー)はすべての音声周波数で I/Q が正確に90度位相差を持つため、
// 遅延バッファは不要で I − Q だけでLSBが選択される。
//
// 処理フロー:
//   I−Q → ソフトクリッパー → HPF500Hz(2次) → LPF2400Hz(4次) → 出力
//   高域側はLPF4次 = 24dB/oct の急峻な減衰で帯域外ノイズを大幅低減
float lsbDemodulate(float iSig, float qSig) {
  float audio = iSig - qSig * lsbQGain;
  audio = lsbSoftClip(audio);

  // --- Stage1: HPF 500Hz 2次バターワース ---
  float hpf_out = LSB_HPF_B0 * audio
                + LSB_HPF_B1 * lsb_hpf_x1
                + LSB_HPF_B2 * lsb_hpf_x2
                - LSB_HPF_A1 * lsb_hpf_y1
                - LSB_HPF_A2 * lsb_hpf_y2;
  lsb_hpf_x2 = lsb_hpf_x1;  lsb_hpf_x1 = audio;
  lsb_hpf_y2 = lsb_hpf_y1;  lsb_hpf_y1 = hpf_out;

  // --- Stage2: LPF 2400Hz 4次バターワース (前半) ---
  float lpf1_out = LSB_LPF1_B0 * hpf_out
                 + LSB_LPF1_B1 * lsb_lpf1_x1
                 + LSB_LPF1_B2 * lsb_lpf1_x2
                 - LSB_LPF1_A1 * lsb_lpf1_y1
                 - LSB_LPF1_A2 * lsb_lpf1_y2;
  lsb_lpf1_x2 = lsb_lpf1_x1;  lsb_lpf1_x1 = hpf_out;
  lsb_lpf1_y2 = lsb_lpf1_y1;  lsb_lpf1_y1 = lpf1_out;

  // --- Stage3: LPF 2400Hz 4次バターワース (後半) ---
  float lpf2_out = LSB_LPF2_B0 * lpf1_out
                 + LSB_LPF2_B1 * lsb_lpf2_x1
                 + LSB_LPF2_B2 * lsb_lpf2_x2
                 - LSB_LPF2_A1 * lsb_lpf2_y1
                 - LSB_LPF2_A2 * lsb_lpf2_y2;
  lsb_lpf2_x2 = lsb_lpf2_x1;  lsb_lpf2_x1 = lpf1_out;
  lsb_lpf2_y2 = lsb_lpf2_y1;  lsb_lpf2_y1 = lpf2_out;

  return lpf2_out;
}

// --- LSB専用AGC ---
// CWのAGCと独立したゲイン変数(lsbAgcGain)を使用する。
// CW→LSB切替時に高ゲイン状態が引き継がれて過大増幅になる問題を解消。
//
// CWとの違い:
//   ・maxGain 40→10: 強信号でのADC飽和ノイズを防ぐ
//   ・attackRate 0.01→0.08: 強信号到来時に素早くゲインを下げる
//   ・targetAmp 0.5→0.35: クリッピングマージンを確保
float applyLsbAGC(float input) {
  float error = LSB_AGC_TARGET - fabsf(input);
  lsbAgcGain += (error > 0) ? LSB_AGC_ATTACK * error : LSB_AGC_DECAY * error;
  lsbAgcGain = constrain(lsbAgcGain, LSB_AGC_MIN_GAIN, LSB_AGC_MAX_GAIN);
  return input * lsbAgcGain;
}

// --- AGC (自動利得制御) ---
// 目標振幅に対してゲインを追跡する。アタック速いディケイ遅いの非対称制御。
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
// [6] CWキーヤー処理
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

// CWキーヤー (エレキー/縦振り電鍵 両対応)
// CWモード専用: LSBモードではマイクで送信するため呼び出さない
void handleKeyer() {
  static unsigned long stateStart = 0;
  static int keyerState = 0;  // 0=待機, 1=送信中, 2=要素間スペース
  unsigned long now = millis();

  // 縦振り電鍵モード
  if (straightKeyMode) {
    bool keyDown = (digitalRead(PADDLE_DASH) == LOW);
    if (keyDown  && !transmitting) startTransmit();
    if (!keyDown &&  transmitting) stopTransmit();
    return;
  }

  // エレキーモード
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
    // 送信継続時間チェック
    if (now - stateStart >= (sendingDash ? dashDuration : dotDuration)) {
      stopTransmit();
      keyerState = 2;
      stateStart = now;
    }
  } else if (keyerState == 2) {
    // 要素間スペース待機
    if (now - stateStart >= elementSpace) keyerState = 0;
  }
}

// ============================================================
// [7] LSB送信 PTT処理
// ============================================================

// LSBモードのPTT: GPIO9 (PTT_BUTTON) を専用プッシュトークスイッチとして使用する。
// PADDLE_DASH とは独立した専用ピンなので CWパドルと干渉しない。
// 押している間だけ送信 (プッシュトーク動作)。
void handleLsbPTT() {
  bool pttPressed = (digitalRead(PTT_BUTTON) == LOW);
  if (pttPressed && !transmitting) startTransmit();
  if (!pttPressed &&  transmitting) stopTransmit();
}

// ============================================================
// [8] ユーザーインターフェース
// ============================================================

// 周波数を "7.000.000" 形式の文字列に変換する
static String fmtMHz(unsigned long f_hz) {
  char buf[16];
  unsigned long mhz = f_hz / 1000000UL;
  unsigned long khz = (f_hz % 1000000UL) / 1000UL;
  snprintf(buf, sizeof(buf), "%lu.%03lu", mhz, khz);
  return String(buf);
}

// バンドスコープ 振幅[dB]→ピクセル高さ変換
int barLength(double d) {
  float fy = SCOPE_SENSITIVITY * (log10(d) + SCOPE_OFFSET);
  return constrain((int)fy, 0, 20);
}

// ロータリーエンコーダー 割り込みハンドラ
void rotary_encoder() {
  unsigned char result = r.process();
  if (result) {
    if (result == DIR_CW) FREQ += STEP;
    else                  FREQ -= STEP;
  }
  FREQ     = constrain(FREQ, (long)LOW_FREQ, (long)HI_FREQ);
  FREQ_ULL = (unsigned long long)FREQ * 100ULL;
}

// --- 音量調節モード ---
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
    u8g2.drawBox(14, 20, (val - 10) * 5, 8);  // 10→0%, 30→100%
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

// --- WPM設定モード ---
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
      if (holdStart && (now - holdStart) >= 800) break;  // 長押し=キャンセル
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

// --- KEY_MODE_BUTTON ハンドラ (CWモード時: キーヤー切替 / 長押し: 音量) ---
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
      // CWモード時のみキーヤーモード切替 (LSBモードではマイク送信なので不要)
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

// --- STEP_BUTTON ハンドラ (周波数ステップ切替 / 長押し: WPM設定) ---
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

// --- ★MODE_BUTTON ハンドラ (CW/LSBモード切替) ---
// 短押しでCW⇔LSBを切り替え、EEPROMに保存する。
// 切替時にフィルタ状態をリセットし、VFO周波数を再設定する。
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
      // VFO周波数の補正 (モード切替前に実行)
      // ============================================================
      // CWとLSBでは BFOオフセットが異なるため、VFO表示周波数の「意味」がずれる。
      //
      //   CW  : LO = FREQ − 700Hz  → FREQ の局が 700Hz トーンとして聞こえる
      //   LSB : LO = FREQ          → FREQ + 700Hz の局が 700Hz として聞こえる
      //
      // 例: CW で FREQ=7.001MHz → 7.001MHzの局が700Hzトーン
      //   そのままLSBへ切替 → 同じ局は (7001000 − 7001000) = 0Hz(DC) → 聞こえない!
      //   LSBで聞こえる700Hzは FREQ + 700Hz = 7.001700MHzの局
      //
      // 補正: モード切替時にFREQを±700Hz動かして同じ局が常に700Hzで聞こえるようにする
      //   CW → LSB: FREQ -= 700Hz  (LO変化なし: FREQ_new = FREQ_old - 700)
      //   LSB→ CW : FREQ += 700Hz  (LO変化なし: FREQ_new - 700 = FREQ_old)
      //
      // 確認:
      //   CW FREQ=7.001MHz, 局=7.001MHz, 音=700Hz
      //   →LSB FREQ=7.000300MHz, LO=7.000300MHz, 局=7.001MHz → 音=700Hz ✓
      //
      //   LSB FREQ=7.000300MHz, 局=7.001MHz, 音=700Hz
      //   →CW FREQ=7.001MHz, LO=7.001MHz−700Hz=7.000300MHz, 局=7.001MHz → 音=700Hz ✓
      if (!lsbMode) {
        // 現在CW → LSBへ切替: FREQ を 700Hz 下げる
        FREQ = (unsigned long)constrain((long)FREQ - 700, (long)LOW_FREQ, (long)HI_FREQ);
      } else {
        // 現在LSB → CWへ切替: FREQ を 700Hz 上げる
        FREQ = (unsigned long)constrain((long)FREQ + 700, (long)LOW_FREQ, (long)HI_FREQ);
      }
      FREQ_ULL = (unsigned long long)FREQ * 100ULL;
      FREQ_OLD = FREQ;
      saveFrequencyToEEPROM(FREQ);

      // モード切替
      lsbMode = !lsbMode;
      saveModeToEEPROM(lsbMode);

      // フィルタ・AGC状態変数をリセット
      // CW→LSB切替でCWの高ゲインが引き継がれるとポップ音が発生するため両方リセット
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

      // VFO再設定 (補正済みのFREQで)
      muteCounter = 500;
      Freq_Set();

      // モード表示 (新モード名 + 補正後の周波数)
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
// [9] 画面描画
// ============================================================

// --- バンドスコープ描画 ---
// 上部スコープ (y=0〜26): IQ-FFT結果をスペクトラム表示する
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

  // --- 右半分 (正の周波数) ---
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

  // --- 左半分 (負の周波数) ---
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

  // --- 周波数目盛 (y=49) ---
  unsigned long leftHz  = (FREQ > 15000) ? FREQ - 15000 : 0;
  unsigned long rightHz = FREQ + 15000;
  u8g2.setFont(u8g2_font_micro_tr);
  u8g2.drawStr(0, 49, fmtMHz(leftHz).c_str());
  String midS = fmtMHz(FREQ);
  u8g2.drawStr(constrain(64 - (int)(midS.length() * 2), 34, 80), 49, midS.c_str());
  String rS = fmtMHz(rightHz);
  u8g2.drawStr(constrain(128 - (int)(rS.length() * 4), 98, 128), 49, rS.c_str());

  // --- 周波数・ステップ表示 (上部) ---
  u8g2.setFont(u8g2_font_8x13B_tr);
  String freqt = String(FREQ);
  u8g2.setCursor(2, 1);
  u8g2.print(freqt.substring(0, 1) + "." + freqt.substring(1, 4) + "." + freqt.substring(4));

  u8g2.setFont(u8g2_font_micro_tr);
  // TX状態表示 (CWモードのみ)
  if (transmitting && !lsbMode) {
    u8g2.setCursor(90, 6);
    u8g2.print("TX ");
    if      (sendingDot)  u8g2.print("DOT");
    else if (sendingDash) u8g2.print("DASH");
    else                  u8g2.print("KEY");
    u8g2.print(" "); u8g2.print(wpm); u8g2.print("W");
  }
  // TX状態表示 (LSBモード)
  if (transmitting && lsbMode) {
    u8g2.setCursor(90, 6);
    u8g2.print("LSB TX");
  }

  u8g2.setCursor(78, 0);  u8g2.print("STEP:");
  if      (STEP == 1000) u8g2.drawStr(102, 0, "1000");
  else if (STEP == 100)  u8g2.drawStr(102, 0, " 100");
  else                   u8g2.drawStr(106, 0, "  10");
}

// --- Sメーター描画 ---
// バンドスコープデータからピークパワーを算出しバー表示する
void showS_meter() {
  for (int xi = 1; xi < 64; xi++) {
    int d = (barLength(vReal[xi * 2]) + barLength(vImag[xi * 2 + 1])) * 2;
    u8g2.drawBox(86, 6, d, 6);
  }
}

// --- ウォーターフォール描画 (y=27〜47) ---
void displayWaterfall() {
  for (int y = 0; y < WATERFALL_HEIGHT; y++) {
    int histY = (waterfallIndex - y + WATERFALL_HEIGHT) % WATERFALL_HEIGHT;
    for (int x = 0; x < 128; x++) {
      if (waterfallHistory[histY][x] > THRESHOLD) u8g2.drawPixel(x, 27 + y);
    }
  }
  waterfallIndex = (waterfallIndex + 1) % WATERFALL_HEIGHT;
}

// --- 固定グラフィック枠・ラベル描画 ---
void showGraphics() {
  u8g2.drawHLine(0,   26, 128);   // スコープ下端
  u8g2.drawHLine(0,   48, 128);   // ウォーターフォール下端
  u8g2.drawFrame(86,  6,  42, 6); // Sメーター枠
  u8g2.drawBox(0,   26, 2, 2);
  u8g2.drawBox(126, 26, 2, 2);

  u8g2.setFont(u8g2_font_micro_tr);
  u8g2.drawStr(78, 6, "S:");
  u8g2.drawStr(78, 0, "STEP:");
  u8g2.drawStr(122, 0, "Hz");

  // ★モード表示: Sメーターバーのすぐ下 (y=13)、右端に最小フォントで "CW" または "LSB"
  // y=6がSメーター上端(高さ6px)、y=12がその直下から開始するのでy=13に描画
  u8g2.drawStr(lsbMode ? 117 : 121, 13, lsbMode ? "LSB" : "CW");

  // 周波数 (大フォント)
  String freqt = String(FREQ);
  u8g2.setFont(u8g2_font_8x13B_tr);
  u8g2.setCursor(2, 1);
  u8g2.print(freqt.substring(0, 1) + "." + freqt.substring(1, 4) + "." + freqt.substring(4));
}

// --- CW解読テキスト表示 (y=54〜63, 10px) ---
// 最新文字を右端に固定し、古い文字は左へスクロールアウトする
void displayCWText() {
  const int CHAR_W   = 5;
  const int CHAR_GAP = 2;
  const int STEP_W   = CHAR_W + CHAR_GAP;  // 7px/文字
  const int MAX_VIS  = 128 / STEP_W;       // 最大18文字
  const int TEXT_Y   = 55;                 // y=54〜63 内 上1px余白

  u8g2.setFont(u8g2_font_5x8_tr);
  int count  = (cwDecodedLen < MAX_VIS) ? cwDecodedLen : MAX_VIS;
  int start  = cwDecodedLen - count;
  int xStart = (MAX_VIS - count) * STEP_W;  // 右寄せ
  for (int i = 0; i < count; i++) {
    char buf[2] = { cwDecodedBuf[start + i], '\0' };
    u8g2.drawStr(xStart + i * STEP_W, TEXT_Y, buf);
  }
}

// --- ★LSB波形表示 (y=56〜63, 8px) ---
// 画面最下部8ピクセルに音声波形をオシロスコープ表示する。区切り線は描画しない。
// 送信時: マイク入力波形 (4kHz / 4 = 1kHz表示サンプルレート)
// 受信時: AGC後の復調音声波形 (40kHz / 32 = 1250Hz表示サンプルレート)
//
// 表示エリア: y=56(上端) 〜 y=63(下端), 中心 y=59
// バッファ: 128要素リングバッファ → 128ピクセルに1:1で対応
void displayLsbWaveform() {
  const int CENTER_Y = 59;   // 8px エリア(y=56-63)の中心

  // ================================================================
  // ★★ 波形感度の調整はここ ★★
  // WAVE_SCALE : 数値を大きくすると波形が大きく表示される
  //   推奨範囲: 3.0〜20.0
  //   3.0  → 大きな声/強い信号でのみ振れる（感度低）
  //   8.0  → 通常の会話音量で適度に振れる（推奨デフォルト）
  //   15.0 → 微弱な信号でも振れる（感度高、ノイズも見える）
  // WAVE_MAX_AMP: 波形の最大振れ幅 [px]（±この値で上下限クランプ）
  //   3 → 上下3pxずつ = 合計6px（8px枠内に収まる）
  // ================================================================
  const float WAVE_SCALE   = 8.0f;  // ← ここを変えて感度調整
  const int   WAVE_MAX_AMP = 3;     // ← 上下限クランプ幅 [px]

  // Core1が更新するwrIdxをスナップショット取得
  // wrIdxは「次に書き込む位置」なので古い順に読み出す
  uint8_t wrSnap = lsbWaveWrIdx;

  int prevX = -1, prevY = CENTER_Y;
  for (int x = 0; x < 128; x++) {
    uint8_t bufIdx = (uint8_t)((wrSnap + x) % LSB_WAVE_BUF_SIZE);
    float sample   = lsbWaveBuf[bufIdx];

    // 振幅→ピクセル変換 (±WAVE_MAX_AMP にクランプ)
    int dy = constrain((int)(sample * WAVE_SCALE), -WAVE_MAX_AMP, WAVE_MAX_AMP);
    int y  = CENTER_Y - dy;

    if (prevX >= 0) u8g2.drawLine(prevX, prevY, x, y);
    prevX = x;  prevY = y;
  }
}

// ============================================================
// [10] セットアップ・メインループ (Core0: UI・制御)
// ============================================================

void setup() {
  // ピン初期化
  pinMode(LED_INDICATOR,  OUTPUT);
  pinMode(RX_SW,          OUTPUT);
  pinMode(PADDLE_DOT,     INPUT_PULLUP);
  pinMode(PADDLE_DASH,    INPUT_PULLUP);
  pinMode(KEY_MODE_BUTTON,INPUT_PULLUP);
  pinMode(STEP_BUTTON,    INPUT_PULLUP);
  pinMode(MODE_BUTTON,    INPUT_PULLUP);  // CW/LSBモードボタン
  pinMode(PTT_BUTTON,     INPUT_PULLUP);  // LSB PTTボタン

  EEPROM.begin(512);
  Wire.begin();

  // Si5351 初期化
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 25001042, 0);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_8MA);
  si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_8MA);

  // ★PLLの明示的な初期化 (LSB TX修正)
  // etherkit Si5351ライブラリはCLK0/CLK1をPLLA、CLK2をPLLBに割り当てる。
  // set_freq_manual(CLK2)は「PLLBがpll_freqで動作している」前提で計算するが、
  // PLLBが未初期化だとCLK2の出力周波数が全くずれる → LSB TX無出力の原因。
  // 両PLLをpll_freqで明示的に初期化してからCLK2マルチシンスを設定する。
  si5351.set_pll(pll_freq, SI5351_PLLA);   // CLK0/CLK1用 PLLA を初期化
  si5351.set_pll(pll_freq, SI5351_PLLB);   // CLK2用 PLLB を初期化 ★重要
  // CLK2マルチシンスを初期化（set_freq_manualが正しく動作するための前処理）
  si5351.set_freq(FREQ_ULL, SI5351_CLK2);
  si5351.output_enable(SI5351_CLK2, 0);  // TX初期状態はOFF
  digitalWrite(RX_SW, LOW);

  // ロータリーエンコーダー 割り込み設定
  r.begin();
  attachInterrupt(0, rotary_encoder, CHANGE);
  attachInterrupt(1, rotary_encoder, CHANGE);

  // EEPROM から設定復元
  unsigned long savedFreq = readFrequencyFromEEPROM();
  FREQ = (savedFreq >= (unsigned long)LOW_FREQ && savedFreq <= (unsigned long)HI_FREQ) ? savedFreq : 7000000UL;
  wpm             = readWPMFromEEPROM();
  straightKeyMode = readKeyModeFromEEPROM();
  volumeMultiplier= readVolFromEEPROM() / 10.0f;
  lsbMode         = readModeFromEEPROM();  // ★CW/LSBモード復元

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

  // OLED 初期化
  u8g2.begin();
  u8g2.setFlipMode(0);
  u8g2.setFont(u8g2_font_micro_tr);
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();

  // 起動スプラッシュ
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

  // ボタン処理 (毎ループ)
  handleKeyModeButton();
  handleModeButton();   // ★CW/LSBモード切替

  if (lsbMode) {
    // === LSBモード ===
    handleLsbPTT();     // PTT_BUTTON(GPIO9)をPTTとして使用

    // 周波数変化検出
    if (FREQ != FREQ_OLD) {
      muteCounter = 500;
      Freq_Set_LSB();
      FREQ_OLD = FREQ;
      saveFrequencyToEEPROM(FREQ);
    }

    if (!transmitting) {
      if (digitalRead(STEP_BUTTON) == LOW) Fnc_Stp();

      // FFTデータが揃ったら画面を更新する
      if (sharedBufferReady) {
        static float avgI2 = 0.01f, avgQ2 = 0.01f;
        for (int i = 0; i < SAMPLES; i++) {
          float si = sharedBufferI[i], sq = sharedBufferQ[i];
          avgI2 = avgI2 * 0.9995f + si * si * 0.0005f;
          avgQ2 = avgQ2 * 0.9995f + sq * sq * 0.0005f;
        }
        float qGain = (avgQ2 > 0.0001f) ? constrain(sqrtf(avgI2 / avgQ2), 0.8f, 1.25f) : 1.0f;
        // ★IQ振幅補正係数をCore1のLSB復調処理へ共有する
        // Core1はlsbDemodulate()内でこの値をQチャンネルに乗じて振幅バランスを取る
        // これにより image rejection が改善し、対側波帯(USB)のビート音が減少する
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
          displayLsbWaveform();  // ★LSB受信波形を最下段に表示
          u8g2.sendBuffer();
          digitalWrite(LED_INDICATOR, LOW);
          lastUIUpdate = millis();
        }
      }
    } else {
      // =====================================================
      // LSB TX中: Core0 が Si5351 CLK2 を 4kHz で更新する
      // =====================================================
      // I2CはすべてCore0に集約する（Core1からはI2Cを呼ばない）:
      //   Core1: ADC → ヒルベルト変換 → PAバイアス(GPIO) + txFreqShared書き込み（I2Cなし）
      //   Core0: txFreqShared を読んで Si5351 CLK2 を 4kHz で更新（I2Cをここに集約）
      //   OLEDは更新しない（Core0のI2Cを Si5351専用にしTX変調を乱さないため）
      // ※ Wire はCore0で begin() しているためCore1からのI2C呼び出しは不安定で、
      //   stopTransmit()のI2Cと衝突するとCLK2が無効化されず送信後にノイズが残る。
      {
        static unsigned long lastTxUpdate = 0;
        unsigned long nowUs = micros();
        if (nowUs - lastTxUpdate >= (unsigned long)LSB_TX_INTERVAL_US) {
          lastTxUpdate = nowUs;
          uint32_t f = txFreqShared;  // アトミック読み取り（32bit）
          if (f > 0) {
            // Core0 が Si5351 CLK2 周波数を更新（I2C競合なし）
            si5351.set_freq_manual((unsigned long long)f, pll_freq, SI5351_CLK2);
          }
        }
      }

      // --- ★LSB TX中のマイク波形表示 ---
      // 全画面sendBuffer()はI2Cを長時間占有しTX変調を乱すため使えない。
      // updateDisplayArea()で最下段タイル1行(y=56-63, 128byte)のみ部分転送し、
      // I2C占有時間を最小化する。更新頻度は10fpsに抑えTX中断を最小限にする。
      {
        static unsigned long lastTxWaveDraw = 0;
        if (millis() - lastTxWaveDraw >= 100) {   // 10fps
          lastTxWaveDraw = millis();
          u8g2.setDrawColor(0);
          u8g2.drawBox(0, 56, 128, 8);            // 最下段8pxを消去
          u8g2.setDrawColor(1);
          displayLsbWaveform();                    // マイク波形を描画
          u8g2.updateDisplayArea(0, 7, 16, 1);     // タイル行7(y=56-63)のみ転送
        }
      }

      lastUIUpdate = millis();  // タイマーリセット: PTT解放後の一括更新を防ぐ
    }

  } else {
    // === CWモード ===
    handleKeyer();
    handleCWDecoder();  // CW自動解読

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
          displayCWText();   // CW解読テキストを最下段に表示
          u8g2.sendBuffer();
          digitalWrite(LED_INDICATOR, LOW);
          lastUIUpdate = millis();
        }
      }
    } else {
      // CW TX中
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
// [11] Core1 セットアップ・ループ (オーディオ・DSP処理)
// ============================================================

void setup1() {
  pinMode(speakerPin,  OUTPUT);
  pinMode(PA_BIAS_PIN, OUTPUT);  // ★PAバイアス制御 PWM出力
  pinMode(inputPinI,   INPUT);
  pinMode(inputPinQ,   INPUT);
  pinMode(MIC_IN,      INPUT);   // マイク入力

  analogReadResolution(12);
  analogWriteResolution(12);
  // speakerPin(GPIO16) と PA_BIAS_PIN(GPIO17) は同じPWMスライス(slice0)を共有するため
  // analogWriteFreq() で設定した44100Hzが両方に適用される。
  // 44100Hz は PA バイアス用としても十分に高く、PA側の RC フィルタで平滑化される。
  analogWriteFreq(pwmFrequency);  // 44100Hz → GPIO16/17 両方に適用

  analogWrite(PA_BIAS_PIN, 0);   // 起動時はPAバイアス0 (PA OFF)
}

void loop1() {
  unsigned long now = micros();

  // ★ LSB TX 処理: 4kHz レートで Si5351 CLK2 を位相変調する
  // 通常の 40kHz RX ループとは独立したタイミングで動作する
  if (lsbMode && transmitting) {
    static unsigned long lastLsbTxTime = 0;
    if (now - lastLsbTxTime < (unsigned long)LSB_TX_INTERVAL_US) return;
    lastLsbTxTime = now;
    handleLsbTxCore1();
    return;
  }

  // === 40kHz RX / CW TX 共通タイミング制御 ===
  static bool prevTxState  = false;
  static unsigned long lastSampleTime = 0;
  static float toneAngle   = 0.0f;   // CW モニター音の位相累積器
  static float lastRxPwm   = 2047.5f;// TX→RX クロスフェード用の最終RX出力値

  const float  ENV_STEP      = 0.002f; // ~12.5ms でフェード完了 (@40kHz)
  const float  FADE_WIN      = 0.05f;  // クロスフェード区間
  const unsigned long TARGET = 25;     // 40kHz = 25µs

  // TX開始時にタイムスタンプをリセット (RX中の積算ずれによるキャッチアップを防ぐ)
  if (transmitting && !prevTxState) lastSampleTime = now;
  prevTxState = transmitting;

  if (now - lastSampleTime < TARGET) return;
  lastSampleTime += TARGET;

  // --- CW TX フェードイン/アウト処理 ---
  // txEnvelope は transmitting フラグと独立して動作し、
  // stopTransmit() 後もエンベロープが 0 になるまでサイン波を出力し続ける。
  txEnvelope += transmitting ? ENV_STEP : -ENV_STEP;
  txEnvelope  = constrain(txEnvelope, 0.0f, 1.0f);

  if (!lsbMode && txEnvelope > 0.0f) {
    // 700Hz サイン波生成
    toneAngle += 2.0f * (float)PI * (float)CW_TONE / (float)sampleRate;
    if (toneAngle >= 2.0f * (float)PI) toneAngle -= 2.0f * (float)PI;
    float sine = sinf(toneAngle);

    // TX開始直後クロスフェード (最終RX出力値からサイン波へ滑らかに移行)
    float blend = (txEnvelope < FADE_WIN) ? (txEnvelope / FADE_WIN) : 1.0f;
    float txPwm = (sine * 0.1f * txEnvelope + 1.0f) * 2047.5f;
    float pwm   = lastRxPwm * (1.0f - blend) + txPwm * blend;

    analogWrite(speakerPin, (uint16_t)constrain((int)pwm, 0, 4095));
    return;  // RX処理はスキップ
  }

  // --- RX 処理 (CW/LSB 共通) ---
  // lastRxPwm を更新してもらうため参照渡し相当として引数で渡す
  handleRxCore1(lastRxPwm);
}

// --- RX 処理 (CW/LSBモード共通) ---
// I/Q サンプリング → モード別デジタルフィルタ → AGC → PWM出力
// 同時にFFT用の共有バッファへデータを蓄積する。
// lastRxPwm: loop1() の CW TX クロスフェード用に最終出力値を書き戻す
void handleRxCore1(float& lastRxPwm) {
  static bool wasTransmitting = false;
  static float smoothMute  = 1.0f;

  // TX→RX 遷移時のクリーンアップ: IIRフィルタ残留状態をリセット
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
    lsbAgcGain  = 1.0f;       // LSB AGCリセット: TX後の過大ゲインを防ぐ
    smoothMute  = 0.0f;       // RX再開時にフェードイン
    lastRxPwm   = 2047.5f;    // 次のTX開始クロスフェード用にリセット
    wasTransmitting = false;
  }
  if (transmitting) { wasTransmitting = true; return; }

  // 1. 4倍オーバーサンプリング (+3dB SNR)
  long sumI = 0, sumQ = 0;
  for (int i = 0; i < 4; i++) {
    sumI += analogRead(inputPinI);
    sumQ += analogRead(inputPinQ);
  }
  float rawI = ((float)sumI / 4.0f / 2047.5f) - 1.0f;
  float rawQ = ((float)sumQ / 4.0f / 2047.5f) - 1.0f;

  // 2. ミュート・DCオフセット補正
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

  // 3. FFT用共有バッファへ蓄積
  sharedBufferI[sharedIndex] = mutedI;
  sharedBufferQ[sharedIndex] = mutedQ;
  sharedIndex++;
  if (sharedIndex >= SAMPLES) {
    sharedIndex = 0;
    sharedBufferReady = true;
  }

  // 4. モード別フィルタで復調
  float demodulated;
  if (lsbMode) {
    // LSBモード: I−Q + 500-2400Hz BPF でLSBを選択
    demodulated = lsbDemodulate(mutedI, mutedQ);
  } else {
    // CWモード: 700Hz中心 ±100Hz 狭帯域 BPF
    demodulated = cwDemodulate(mutedI, mutedQ);
  }

  // 5. CWモード時のみ: SNR検出 + イベント生成 (自動解読用)
  if (!lsbMode) {
    updateCwDetector(demodulated);
  }

  // 6. AGC 適用 (CWとLSBで独立したゲイン変数・パラメータを使用)
  float agcOut;
  if (lsbMode) {
    // LSB専用AGC: 低maxGain(10) + 速い応答(0.08) でADC飽和歪みを防ぐ
    agcOut = (smoothMute < 0.9f) ? demodulated * lsbAgcGain : applyLsbAGC(demodulated);
  } else {
    // CW AGC: 高maxGain(40) + 微弱信号対応 (感度を損なわない)
    agcOut = (smoothMute < 0.9f) ? demodulated * agcGain : applyAGC(demodulated);
  }
  agcOut *= volumeMultiplier;

  // 7. LSBモード: AGC後の信号を波形表示バッファへデシメーション書き込み
  // 40kHz を 1/32 に間引き → 1250Hz の表示サンプルレート
  // 128ピクセルで約102ms分の波形を表示する。
  // AGC後なので振幅が正規化されており波形として視認しやすい。
  if (lsbMode) {
    static uint8_t waveDecim = 0;
    if (++waveDecim >= 32) {
      waveDecim = 0;
      lsbWaveBuf[lsbWaveWrIdx] = agcOut;  // AGC後の値を使用
      lsbWaveWrIdx = (lsbWaveWrIdx + 1) % LSB_WAVE_BUF_SIZE;
    }
  }

  // 8. PWM 出力 (lastRxPwm を更新して loop1() のクロスフェードに使えるようにする)
  uint16_t pwmOut = (uint16_t)constrain((int)((agcOut + 1.0f) * 2047.5f), 0, 4095);
  lastRxPwm = (float)pwmOut;  // TX開始時クロスフェード用に保持
  analogWrite(speakerPin, pwmOut);
}

// ============================================================
// [12] ★LSB TX 位相変調処理 (Core1)
// ============================================================
// uSDX/QCX-SSBと同じ原理をPico向けに実装。
// マイク音声にヒルベルト変換を施してI/Q取得し、位相差(dp)を瞬時周波数偏移に
// 変換して Si5351 CLK2 の周波数を更新することでLSBキャリアを生成する。
//
// 【更新レート】4000Hz (250µs間隔)
// 【Si5351更新時間】~180µs @400kHz I2C → 250µs 間隔に収まる
// 【音声帯域】0-2kHz (ナイキスト 2kHz)
//
// ヒルベルト変換 (15タップFIR): uSDXと同一係数
//   q = ((v[0]-v[14])*2 + (v[2]-v[12])*8 + (v[4]-v[10])*21 + (v[6]-v[8])*16) / 64
//       + (v[6]-v[8])
//   i = v[7]  (中央タップ = 7サンプル遅延)
void handleLsbTxCore1() {
  // マイク用FIRバッファ (15タップ)
  static float micBuf[LSB_TX_HILBERT_TAPS] = {0};
  static float prevPhase = 0.0f;

  // マイク (ADC2 = GPIO28) をサンプリング
  float micRaw = (float)analogRead(MIC_IN) / 2047.5f - 1.0f;

  // DC オフセット除去（uSDX の dc = (in + dc) / 2 に相当）
  // ★【重要】マイク入力の DC オフセット を正確に追跡することで、
  // 位相計算の安定性と振幅検出の精度が大幅に改善される。
  static float micDC = 0.0f;
  micDC = micDC * 0.99609375f + micRaw * 0.00390625f;  // LPF: fc ≈ 10Hz @ 4kHz
  float micAC = micRaw - micDC;  // DC 成分を除去

  // ゲイン適用（uSDX の * 2 に相当）
  // + PA_TX_DRIVE による増幅（uSDX の << drive に相当）
  float micGained = micAC * MIC_GAIN;
  micGained = micGained * (1 << PA_TX_DRIVE);  // PA_TX_DRIVE: 0-8 の左シフト（×1, ×2, ×4, ... ×256）

  // ソフトクリップ: ±1.0 で飽和（歪みを最小化しつつ、大幅な過入力で IIR 発振を防ぐ）
  // uSDX の lut[] テーブルマッピングに相当
  if (micGained >  1.0f) micGained =  1.0f;
  if (micGained < -1.0f) micGained = -1.0f;

  float micAC_scaled = micGained;  // ヒルベルト変換への入力

  // マイク波形表示バッファへ書き込み (4kHz TX サンプルを4回に1回間引き → 1kHz表示レート)
  // 128ピクセルで約128ms分のマイク波形を表示する。
  static uint8_t txWaveDecim = 0;
  if (++txWaveDecim >= 4) {
    txWaveDecim = 0;
    lsbWaveBuf[lsbWaveWrIdx] = micAC_scaled;  // ゲイン・シフト後の値をそのまま格納
    lsbWaveWrIdx = (lsbWaveWrIdx + 1) % LSB_WAVE_BUF_SIZE;
  }

  // FIRバッファをシフト (最新サンプルを末尾に追加)
  // ヒルベルト変換：uSDX と同一の 15 タップ係数を使用
  for (int j = 0; j < LSB_TX_HILBERT_TAPS - 1; j++) micBuf[j] = micBuf[j + 1];
  micBuf[LSB_TX_HILBERT_TAPS - 1] = micAC_scaled;

  // ヒルベルト変換でI/Q生成 (15タップFIR, uSDX係数)
  // I = 中央タップ (= v[7], 7サンプル遅延)
  float i_val = micBuf[7];

  // Q = ヒルベルト変換出力 (奇数対称FIR)
  // Q = ((v[0]-v[14])*2 + (v[2]-v[12])*8 + (v[4]-v[10])*21 + (v[6]-v[8])*16) / 64
  //     + (v[6]-v[8])
  float q_val = ((micBuf[0]  - micBuf[14]) * 2.0f
               + (micBuf[2]  - micBuf[12]) * 8.0f
               + (micBuf[4]  - micBuf[10]) * 21.0f
               + (micBuf[6]  - micBuf[8])  * 16.0f) / 64.0f
               + (micBuf[6]  - micBuf[8]);

  // =========================================================
  // PAバイアス制御 (GPIO17 PWM) — uSDX の amp（vibration amplitude） に相当
  // =========================================================
  // 【uSDX との同期実装】
  //   uSDX: _amp = magn(i/2, q/2)
  //         _amp = _amp << drive           (PA_TX_DRIVE による増幅)
  //         _amp = (_amp > 255) ? 255 : _amp  (クリップ)
  //         amp = (tx) ? lut[_amp] : 0
  //
  // Pico: I/Q ベクトルの大きさ = 音声の瞬時振幅
  //      alpha-max plus beta-min 近似 (sqrt不要, 誤差 < 12%):
  //      amplitude ≈ max(|I|, |Q|) + 0.5 × min(|I|, |Q|)
  //
  // マイクゲイン + PA_TX_DRIVE が既に micAC_scaled に反映されているため、
  // 振幅計算後は即座に PWM 値に変換できる。
  {
    float absI = fabsf(i_val);
    float absQ = fabsf(q_val);
    float amplitude = (absI > absQ) ? absI + 0.5f * absQ
                                    : absQ + 0.5f * absI;

    // 振幅を直接 PWM 値に変換する（エンベロープフォロワーは使わない）。
    // uSDXと同様に、振幅に素直に応答する PA ゲート制御を実現する。
    // これにより、音声の瞬時振幅の変化が RF 包絡線に正確に反映される。
    //
    // 振幅 0.0〜1.0（後述のクリップで正規化） → PWM 0〜4095 に変換
    // PA_BIAS_SCALE でスケーリング（PA回路のゲイン特性に合わせる）
    int amplitude_clipped = (int)constrain(amplitude * 1024.0f, 0.0f, 1024.0f);  // 0-1024 に正規化
    int paBias = (int)constrain((float)amplitude_clipped * PA_BIAS_SCALE, 0.0f, 4095.0f);

    // ★ゼロ振幅でも VOX（音声起動）のための微少なバイアスを残す（不要ならコメントアウト）
    // if (paBias < 10) paBias = 0;  // 10 未満はオフ

    analogWrite(PA_BIAS_PIN, paBias);
  }

  // =========================================================
  // 位相変調 — 瞬時位相差 → txFreqShared 書き込み（I2Cなし）
  // Core0 の loop() が txFreqShared を読んで Si5351 CLK2 を更新する
  // =========================================================
  // 瞬時位相を計算
  float phase = atan2f(q_val, i_val);

  // 位相差 dp を求め [-π, π] に正規化
  float dp = phase - prevPhase;
  prevPhase = phase;
  if (dp >  (float)PI) dp -= 2.0f * (float)PI;
  if (dp < -(float)PI) dp += 2.0f * (float)PI;

  // 位相差 → 瞬時周波数偏移 [Hz]
  // df = dp / (2π) × サンプルレート
  // LSBは搬送波の下側帯域 → 符号を反転して下方向に変調
  float df = dp / (2.0f * (float)PI) * (float)LSB_TX_SAMPLE_RATE;
  long long freqShift = (long long)(df * 100.0f);  // centihz 変換

  // 目標周波数を計算してCore0と共有（Core0がI2CでSi5351を更新する）
  // ★ si5351.set_freq_manual() はここで呼ばない ★ (I2Cをcore0に集約)
  long long txFreqLL = (long long)FREQ_ULL - freqShift;

  // 周波数範囲クランプ (±5kHz 以内)
  long long minF = (long long)FREQ_ULL - 500000LL;
  long long maxF = (long long)FREQ_ULL + 500000LL;
  txFreqLL = constrain(txFreqLL, minF, maxF);

  // uint32_t アトミック書き込み（Cortex-M0+ は 32bit ストアが1命令 = 安全）
  // 40m帯の centihz 値は ~700M < 2^30 なので uint32_t に収まる
  txFreqShared = (uint32_t)txFreqLL;
}

// ============================================================
// [13] CW信号検出・イベント生成 (Core1から呼ばれる)
// ============================================================

// CW信号のSNR比計算とイベントバッファへの書き込み。
// handleRxCore1() 内の CWモード時に呼ばれる。
void updateCwDetector(float demodulated) {
  // --- SNR比計算 ---
  static float cwEnvLP = 0.0f;
  static float cwNoise = 0.05f;
  float absD = fabsf(demodulated);

  // 非対称LPF: アタック ~0.5ms, ディケイ ~12ms
  cwEnvLP = (absD > cwEnvLP)
            ? cwEnvLP * 0.95f  + absD * 0.05f
            : cwEnvLP * 0.998f + absD * 0.002f;

  // ノイズフロア追跡 (信号中は更新しない)
  float ratio = cwEnvLP / max(cwNoise, 0.0001f);
  if (ratio < 1.5f)
    cwNoise = cwNoise * 0.9995f + cwEnvLP * 0.0005f;  // ノイズ収束 ~50ms
  else
    cwNoise = cwNoise * 0.99999f + cwEnvLP * 0.00001f; // 信号中は変化なし

  cwEnvelope = ratio;  // Core0 へ共有 (CW_DETECT_THRESHOLD 以上 = CW信号あり)

  // --- CWイベント生成 (デバウンス 80サンプル = 2ms) ---
  static bool     cwSt  = false;
  static uint32_t cwCnt = 0;
  static uint8_t  cwDB  = 0;
  const  uint8_t  DB_TH = 80;

  bool raw = (cwEnvelope > CW_DETECT_THRESHOLD);

  if (raw != cwSt) {
    if (++cwDB >= DB_TH) {
      // デバウンス確定 → リングバッファへイベント書き込み
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
// [14] CW自動解読 (Core0で実行)
// ============================================================

// モールス符号テーブル
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

// CWデコーダー本体 (イベント駆動方式)
// Core1 が計測したマーク/スペース長イベントを消費して文字を確定する。
void handleCWDecoder() {
  if (transmitting || lsbMode) { cwEvRd = cwEvWr; return; }

  static float dotEst      = 60.0f;   // ドット長推定値 [ms] (初期=20WPM)
  static char  morse[10]   = "";
  static int   morseLen    = 0;
  static bool  charDecoded = false;
  static bool  wordAdded   = false;
  static unsigned long keyUpMs = 0;
  static bool  keyIsUp = true;

  unsigned long nowMs = millis();

  // Core1 からのイベントを全消費
  while (cwEvRd != cwEvWr) {
    CWEvent ev;
    ev.type  = cwEvBuf[cwEvRd].type;
    ev.durMs = cwEvBuf[cwEvRd].durMs;
    cwEvRd   = (cwEvRd + 1) % CW_EVBUF_SIZE;

    if (ev.type == 1) {
      // マーク終了 → ドット/ダッシュ判定
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

  // 沈黙タイムアウトによる文字/単語確定
  if (keyIsUp) {
    unsigned long silMs = nowMs - keyUpMs;

    // 文字間スペース: ドット長×2.5 以上 → 1文字確定
    if (!charDecoded && morseLen > 0 && silMs >= (unsigned long)(dotEst * 2.5f)) {
      addCWDecodedChar(decodeMorse(morse));
      morseLen = 0;  morse[0] = '\0';
      charDecoded = true;
    }

    // 単語間スペース: ドット長×6 以上 → スペース追加
    if (charDecoded && !wordAdded && silMs >= (unsigned long)(dotEst * 6.0f)) {
      addCWDecodedChar(' ');
      wordAdded = true;
    }
  }
}
