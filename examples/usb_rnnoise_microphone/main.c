/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Reinhard Panhuber
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

/* plot_audio_samples.py requires following modules:
 * $ sudo apt install libportaudio
 * $ pip3 install sounddevice matplotlib
 *
 * Then run
 * $ python3 plot_audio_samples.py
 */

#include "bsp/board_api.h"
#include "pico/stdlib.h"
#include "tusb.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/pdm_microphone.h"

#include "rnnoise.h"
//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum {
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED = 1000,
  BLINK_SUSPENDED = 2500,
};

// Add microphone configuration
#define PDM_MIC_CLOCK_PIN 22
#define PDM_MIC_DATA_PIN 21
#define PDM_MIC_SAMPLE_RATE CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE

// Create microphone instance
const struct pdm_microphone_config pdm_config = {
    .gpio_data = PDM_MIC_DATA_PIN,
    .gpio_clk = PDM_MIC_CLOCK_PIN,
    .pio = pio0,
    .pio_sm = 0,
    .sample_rate = PDM_MIC_SAMPLE_RATE,
    .sample_buffer_size = (CFG_TUD_AUDIO_EP_SZ_IN - 2) / 2,
};

// Buffer to store microphone data
// Define buffer structure
typedef struct {
  int16_t samples[(CFG_TUD_AUDIO_EP_SZ_IN - 2) / 2]; // Sample data
  volatile bool ready; // Flag to indicate if buffer is ready
} pdm_buffer_t;

// Use double buffering system
#define BUFFER_COUNT 2
pdm_buffer_t pdm_buffers[BUFFER_COUNT];
volatile uint8_t current_read_buffer = 0;
volatile uint8_t current_write_buffer = 0;


// RNNoise processor related definitions
#define FRAME_SIZE 480 // RNNoise processing frame size

// RNNoise processor structure
typedef struct {
  DenoiseState *state;                 // RNNoise state
  float input_frame[FRAME_SIZE];       // Input frame buffer
  float output_frame[FRAME_SIZE];      // Output frame buffer
  int16_t sample_buffer[FRAME_SIZE];   // Only store one frame of data
  int16_t output_buffer[FRAME_SIZE];   // Store processed results
  int sample_pos;                      // Current sample position
  float vad;                           // Voice activity detection value
  bool frame_ready;                    // Flag to indicate if a complete frame is ready for processing
} rnnoise_processor_t;


static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

// Audio controls
// Current states
bool mute[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX + 1]; // +1 for master channel 0
uint16_t
    volume[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX + 1]; // +1 for master channel 0
uint32_t sampFreq;
uint8_t clkValid;

// Range states
audio_control_range_2_n_t(
    1) volumeRng[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX + 1]; // Volume range state
audio_control_range_4_n_t(1) sampleFreqRng; // Sample frequency range state

// Audio test data
uint16_t test_buffer_audio[(CFG_TUD_AUDIO_EP_SZ_IN - 2) / 2];
uint16_t startVal = 0;

void led_blinking_task(void);
void audio_task(rnnoise_processor_t *processor);

// just for test
#define SAMPLE_AMPLITUDE 15000 // Suitable amplitude value (close to half of int16_t range)
#define WAVE_FREQUENCY 160 // Frequency (Hz)
static float phase = 0.0f; // Variable to maintain phase continuity

void on_pdm_samples_ready() {
  // Read samples from the current buffer
  pdm_microphone_read(pdm_buffers[current_write_buffer].samples,
                      (CFG_TUD_AUDIO_EP_SZ_IN - 2) / 2);
  // use cosine wave to test
  //  float phase_increment = 2.0f * M_PI * WAVE_FREQUENCY /
  //  PDM_MIC_SAMPLE_RATE; for (size_t i = 0; i < (CFG_TUD_AUDIO_EP_SZ_IN - 2) /
  //  2; i++) {
  //    // Sine wave generation (using fixed amplitude)
  //    pdm_buffers[current_write_buffer].samples[i] =
  //    (int16_t)(SAMPLE_AMPLITUDE * sinf(phase));

  //   // Update phase to maintain continuity
  //   phase += phase_increment;
  //   // Keep phase within 0-2π range to avoid floating-point precision issues
  //   if (phase >= 2.0f * M_PI) {
  //     phase -= 2.0f * M_PI;
  //   }
  // }
  // Mark the current buffer as ready
  pdm_buffers[current_write_buffer].ready = true;

  // Switch to the next write buffer
  current_write_buffer = (current_write_buffer + 1) % BUFFER_COUNT;
}

/*------------- MAIN -------------*/
int main(void) {
  // log to console
  stdio_init_all();

  
  board_init();
  printf("USB RNNoise Microphone Example\n");

  // init device stack on configured roothub port
  tusb_rhport_init_t dev_init = {.role = TUSB_ROLE_DEVICE,
                                 .speed = TUSB_SPEED_AUTO};
  tusb_init(BOARD_TUD_RHPORT, &dev_init);

  if (board_init_after_tusb) {
    board_init_after_tusb();
  }

  // Init values
  sampFreq = CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE;
  clkValid = 1;

  sampleFreqRng.wNumSubRanges = 1;
  sampleFreqRng.subrange[0].bMin = CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE;
  sampleFreqRng.subrange[0].bMax = CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE;
  sampleFreqRng.subrange[0].bRes = 0;

  if (pdm_microphone_init(&pdm_config) < 0) {
    printf("PDM microphone initialization failed!\n");
    while (1) {
      tight_loop_contents();
    }
  }

  // Set callback that is called when all the samples in the library
  pdm_microphone_set_samples_ready_handler(on_pdm_samples_ready);
  pdm_microphone_set_filter_gain(16);
  // Start capturing data from the PDM microphone
  if (pdm_microphone_start() < 0) {
    printf("PDM microphone start failed!\n");
    while (1) {
      tight_loop_contents();
    }
  }
  memset(test_buffer_audio, 0, sizeof(test_buffer_audio));

  rnnoise_processor_t rnnoise_processor = {0};
  rnnoise_processor.state = rnnoise_create(NULL);
  if (!rnnoise_processor.state) {
    printf("RNNoise initialization failed!\n");
    while (1) {
      tight_loop_contents();
    }
  }

  for (int i = 0; i < BUFFER_COUNT; i++) {
    memset(pdm_buffers[i].samples, 0, sizeof(pdm_buffers[i].samples));
    pdm_buffers[i].ready = false;
  }
  current_read_buffer = 0;
  current_write_buffer = 0;

  printf("USB Microphone Example1\n");

  while (1) {
    tud_task(); // tinyusb device task
    led_blinking_task();
    audio_task(&rnnoise_processor);
  }
  // never reach here
  if (rnnoise_processor.state) {
    rnnoise_destroy(rnnoise_processor.state);
  }
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void) { blink_interval_ms = BLINK_MOUNTED; }

// Invoked when device is unmounted
void tud_umount_cb(void) { blink_interval_ms = BLINK_NOT_MOUNTED; }

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en) {
  (void)remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void) {
  blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
}

//--------------------------------------------------------------------+
// AUDIO Task
//--------------------------------------------------------------------+

void audio_task(rnnoise_processor_t *processor) {
  if (!processor || !processor->state) return;
  
  // Check if the current read buffer is ready
  if (!pdm_buffers[current_read_buffer].ready) return;

  // PDM data size for this acquisition
  int pdm_size = (CFG_TUD_AUDIO_EP_SZ_IN - 2) / 2;
  
  // Add new data to the accumulation buffer
  for (int i = 0; i < pdm_size && processor->sample_pos < FRAME_SIZE; i++) {
    processor->sample_buffer[processor->sample_pos++] = 
        pdm_buffers[current_read_buffer].samples[i];
  }
  
  // Mark buffer as processed
  pdm_buffers[current_read_buffer].ready = false;
  current_read_buffer = (current_read_buffer + 1) % BUFFER_COUNT;
  
  // If we've accumulated enough samples, process with RNNoise
  if (processor->sample_pos >= FRAME_SIZE) {
    // Convert int16 to float
    for (int i = 0; i < FRAME_SIZE; i++) {
      processor->input_frame[i] = processor->sample_buffer[i] / 32768.0f;
    }
    
    // Apply RNNoise
    processor->vad = rnnoise_process_frame(
        processor->state, processor->output_frame, processor->input_frame);
    
    // Convert back to int16 format and store in output buffer
    for (int i = 0; i < FRAME_SIZE; i++) {
      float sample = processor->output_frame[i] * 32768.0f;
      if (sample > 32767.0f) sample = 32767.0f;
      if (sample < -32768.0f) sample = -32768.0f;
      processor->output_buffer[i] = (int16_t)sample;
    }
    
    // Processing complete, reset sample position
    processor->sample_pos = 0;
    processor->frame_ready = true;
    
    // Adjust LED blinking based on VAD
    blink_interval_ms = (processor->vad > 0.7f) ? 100 : 
                       (tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED);
  }
  

  // Always send processed data to USB
  if (processor->frame_ready) {
    // Send processed data to USB
    for (int i = 0; i < pdm_size; i++) {
      test_buffer_audio[i] = (uint16_t)(processor->output_buffer[i]);
    } 
    // Mark frame as processed
    processor->frame_ready = false;
  }
  // TU_LOG1("VAD: %f\n", processor->vad);
}

//--------------------------------------------------------------------+
// Application Callback API Implementations
//--------------------------------------------------------------------+

// Invoked when audio class specific set request received for an EP
bool tud_audio_set_req_ep_cb(uint8_t rhport,
                             tusb_control_request_t const *p_request,
                             uint8_t *pBuff) {
  (void)rhport;
  (void)pBuff;

  // We do not support any set range requests here, only current value requests
  TU_VERIFY(p_request->bRequest == AUDIO_CS_REQ_CUR);

  // Page 91 in UAC2 specification
  uint8_t channelNum = TU_U16_LOW(p_request->wValue);
  uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
  uint8_t ep = TU_U16_LOW(p_request->wIndex);

  (void)channelNum;
  (void)ctrlSel;
  (void)ep;

  return false; // Yet not implemented
}

// Invoked when audio class specific set request received for an interface
bool tud_audio_set_req_itf_cb(uint8_t rhport,
                              tusb_control_request_t const *p_request,
                              uint8_t *pBuff) {
  (void)rhport;
  (void)pBuff;

  // We do not support any set range requests here, only current value requests
  TU_VERIFY(p_request->bRequest == AUDIO_CS_REQ_CUR);

  // Page 91 in UAC2 specification
  uint8_t channelNum = TU_U16_LOW(p_request->wValue);
  uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
  uint8_t itf = TU_U16_LOW(p_request->wIndex);

  (void)channelNum;
  (void)ctrlSel;
  (void)itf;

  return false; // Yet not implemented
}

// Invoked when audio class specific set request received for an entity
bool tud_audio_set_req_entity_cb(uint8_t rhport,
                                 tusb_control_request_t const *p_request,
                                 uint8_t *pBuff) {
  (void)rhport;

  // Page 91 in UAC2 specification
  uint8_t channelNum = TU_U16_LOW(p_request->wValue);
  uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
  uint8_t itf = TU_U16_LOW(p_request->wIndex);
  uint8_t entityID = TU_U16_HIGH(p_request->wIndex);

  (void)itf;

  // We do not support any set range requests here, only current value requests
  TU_VERIFY(p_request->bRequest == AUDIO_CS_REQ_CUR);

  // If request is for our feature unit
  if (entityID == 2) {
    switch (ctrlSel) {
    case AUDIO_FU_CTRL_MUTE:
      // Request uses format layout 1
      TU_VERIFY(p_request->wLength == sizeof(audio_control_cur_1_t));

      mute[channelNum] = ((audio_control_cur_1_t *)pBuff)->bCur;

      TU_LOG2("    Set Mute: %d of channel: %u\r\n", mute[channelNum],
              channelNum);
      return true;

    case AUDIO_FU_CTRL_VOLUME:
      // Request uses format layout 2
      TU_VERIFY(p_request->wLength == sizeof(audio_control_cur_2_t));

      volume[channelNum] = (uint16_t)((audio_control_cur_2_t *)pBuff)->bCur;

      TU_LOG2("    Set Volume: %d dB of channel: %u\r\n", volume[channelNum],
              channelNum);
      return true;

      // Unknown/Unsupported control
    default:
      TU_BREAKPOINT();
      return false;
    }
  }
  return false; // Yet not implemented
}

// Invoked when audio class specific get request received for an EP
bool tud_audio_get_req_ep_cb(uint8_t rhport,
                             tusb_control_request_t const *p_request) {
  (void)rhport;

  // Page 91 in UAC2 specification
  uint8_t channelNum = TU_U16_LOW(p_request->wValue);
  uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
  uint8_t ep = TU_U16_LOW(p_request->wIndex);

  (void)channelNum;
  (void)ctrlSel;
  (void)ep;

  //	return tud_control_xfer(rhport, p_request, &tmp, 1);

  return false; // Yet not implemented
}

// Invoked when audio class specific get request received for an interface
bool tud_audio_get_req_itf_cb(uint8_t rhport,
                              tusb_control_request_t const *p_request) {
  (void)rhport;

  // Page 91 in UAC2 specification
  uint8_t channelNum = TU_U16_LOW(p_request->wValue);
  uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
  uint8_t itf = TU_U16_LOW(p_request->wIndex);

  (void)channelNum;
  (void)ctrlSel;
  (void)itf;

  return false; // Yet not implemented
}

// Invoked when audio class specific get request received for an entity
bool tud_audio_get_req_entity_cb(uint8_t rhport,
                                 tusb_control_request_t const *p_request) {
  (void)rhport;

  // Page 91 in UAC2 specification
  uint8_t channelNum = TU_U16_LOW(p_request->wValue);
  uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
  // uint8_t itf = TU_U16_LOW(p_request->wIndex); 			// Since
  // we have only one audio function implemented, we do not need the itf value
  uint8_t entityID = TU_U16_HIGH(p_request->wIndex);

  // Input terminal (Microphone input)
  if (entityID == 1) {
    switch (ctrlSel) {
    case AUDIO_TE_CTRL_CONNECTOR: {
      // The terminal connector control only has a get request with only the CUR
      // attribute.
      audio_desc_channel_cluster_t ret;

      // Those are dummy values for now
      ret.bNrChannels = 1;
      ret.bmChannelConfig = (audio_channel_config_t)0;
      ret.iChannelNames = 0;

      TU_LOG2("    Get terminal connector\r\n");

      return tud_audio_buffer_and_schedule_control_xfer(
          rhport, p_request, (void *)&ret, sizeof(ret));
    } break;

      // Unknown/Unsupported control selector
    default:
      TU_BREAKPOINT();
      return false;
    }
  }

  // Feature unit
  if (entityID == 2) {
    switch (ctrlSel) {
    case AUDIO_FU_CTRL_MUTE:
      // Audio control mute cur parameter block consists of only one byte - we
      // thus can send it right away There does not exist a range parameter
      // block for mute
      TU_LOG2("    Get Mute of channel: %u\r\n", channelNum);
      return tud_control_xfer(rhport, p_request, &mute[channelNum], 1);

    case AUDIO_FU_CTRL_VOLUME:
      switch (p_request->bRequest) {
      case AUDIO_CS_REQ_CUR:
        TU_LOG2("    Get Volume of channel: %u\r\n", channelNum);
        return tud_control_xfer(rhport, p_request, &volume[channelNum],
                                sizeof(volume[channelNum]));

      case AUDIO_CS_REQ_RANGE:
        TU_LOG2("    Get Volume range of channel: %u\r\n", channelNum);

        // Copy values - only for testing - better is version below
        audio_control_range_2_n_t(1) ret;

        ret.wNumSubRanges = 1;
        ret.subrange[0].bMin = -90; // -90 dB
        ret.subrange[0].bMax = 90;  // +90 dB
        ret.subrange[0].bRes = 1;   // 1 dB steps

        return tud_audio_buffer_and_schedule_control_xfer(
            rhport, p_request, (void *)&ret, sizeof(ret));

        // Unknown/Unsupported control
      default:
        TU_BREAKPOINT();
        return false;
      }
      break;

      // Unknown/Unsupported control
    default:
      TU_BREAKPOINT();
      return false;
    }
  }

  // Clock Source unit
  if (entityID == 4) {
    switch (ctrlSel) {
    case AUDIO_CS_CTRL_SAM_FREQ:
      // channelNum is always zero in this case
      switch (p_request->bRequest) {
      case AUDIO_CS_REQ_CUR:
        TU_LOG2("    Get Sample Freq.\r\n");
        return tud_control_xfer(rhport, p_request, &sampFreq, sizeof(sampFreq));

      case AUDIO_CS_REQ_RANGE:
        TU_LOG2("    Get Sample Freq. range\r\n");
        return tud_control_xfer(rhport, p_request, &sampleFreqRng,
                                sizeof(sampleFreqRng));

        // Unknown/Unsupported control
      default:
        TU_BREAKPOINT();
        return false;
      }
      break;

    case AUDIO_CS_CTRL_CLK_VALID:
      // Only cur attribute exists for this request
      TU_LOG2("    Get Sample Freq. valid\r\n");
      return tud_control_xfer(rhport, p_request, &clkValid, sizeof(clkValid));

    // Unknown/Unsupported control
    default:
      TU_BREAKPOINT();
      return false;
    }
  }

  TU_LOG2("  Unsupported entity: %d\r\n", entityID);
  return false; // Yet not implemented
}

bool tud_audio_tx_done_pre_load_cb(uint8_t rhport, uint8_t itf, uint8_t ep_in,
                                   uint8_t cur_alt_setting) {
  (void)rhport;
  (void)itf;
  (void)ep_in;
  (void)cur_alt_setting;

  tud_audio_write((uint8_t *)test_buffer_audio, CFG_TUD_AUDIO_EP_SZ_IN - 2);

  return true;
}

bool tud_audio_tx_done_post_load_cb(uint8_t rhport, uint16_t n_bytes_copied,
                                    uint8_t itf, uint8_t ep_in,
                                    uint8_t cur_alt_setting) {
  (void)rhport;
  (void)n_bytes_copied;
  (void)itf;
  (void)ep_in;
  (void)cur_alt_setting;

  return true;
}

bool tud_audio_set_itf_close_EP_cb(uint8_t rhport,
                                   tusb_control_request_t const *p_request) {
  (void)rhport;
  (void)p_request;
  startVal = 0;

  return true;
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void) {
  static uint32_t start_ms = 0;
  static bool led_state = false;

  // Blink every interval ms
  if (board_millis() - start_ms < blink_interval_ms)
    return; // not enough time
  start_ms += blink_interval_ms;

  board_led_write(led_state);
  led_state = 1 - led_state; // toggle

  //print blink interval ms as status
  if (blink_interval_ms == BLINK_NOT_MOUNTED) {
    printf("Device not mounted\n");
  } else if (blink_interval_ms == BLINK_MOUNTED) {
    printf("Device mounted\n");
  } else if (blink_interval_ms == BLINK_SUSPENDED) {
    printf("Device suspended\n");
  }
}
