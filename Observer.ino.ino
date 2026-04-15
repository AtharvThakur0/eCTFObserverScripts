// #define I2S_IMPL
#define DEBUG
#define FAST_UART

#ifdef FAST_UART
#include "driver/uart.h"
#endif

#ifdef I2S_IMPL
#include <driver/i2s.h>
#else
#include "esp_adc/adc_continuous.h"
#endif

#include <esp32-hal-cpu.h>
#include "bootloader_random.h"
#include "driver/uart.h"


#define TXD2 17
#define RXD2 16

#define TRACE_STATUS_PIN 23

// #define RX1 18
// #define TX1 19

#define BUFFER_SIZE 64

#define I2S_SAMPLE_RATE 150000
#ifdef I2S_IMPL
#define ADC_BUFFER_SIZE 1024
#else
#define ADC_BUFFER_SIZE 49152
#endif

// expected to be a multiple of 128
#define CONV_FRAME_SIZE 1024
#define ADC_TIMEOUT_MS 1024
#define ADC_READING_MASK 0x0FFF
#define MAX_TRACE_LSHIFT 4

#define BOARD_BAUD 115200

#define HOST_BAUD 921600

#define BOARD_TIMEOUT 1000
#define HOST_TIMEOUT 250

#define HEADER_SIZE 4
#define HOST_PACKET_SIZE SOC_UART_FIFO_LEN

// HardwareSerial hostWatch(1);
HardwareSerial boardWatch(2);
hw_timer_t* timer = NULL;
adc_continuous_handle_t adc_handle;


char boardInBuffer[BUFFER_SIZE] = { 0 };
// char hostInBuffer[BUFFER_SIZE];
char hostDirectBuffer[BUFFER_SIZE] = { 0 };

uint16_t adcBuffer[ADC_BUFFER_SIZE >> 1] = { 0 };
// assume each trace is equal and fits perfectly, then double the size to be safe
uint16_t averages[ADC_BUFFER_SIZE >> MAX_TRACE_LSHIFT << 1] = { 0 };
uint16_t adc_reading_size_buffer[1 << MAX_TRACE_LSHIFT] = { 0 };

uint32_t state = 0;

const uint8_t listCommand[] = { (uint8_t)'%', (uint8_t)'L', 6, 0 };
const uint8_t readCommand[] = { (uint8_t)'%', (uint8_t)'R', 7, 0 };
const uint8_t ACK[] = { (uint8_t)'%', (uint8_t)'A', 0, 0 };
const uint8_t FAILED_READ[] = { 0, 0, 'F', '!' };

const char* pin = "5caa5b";

size_t bytesInQ = 0;

#ifdef FAST_UART
#define WHILE_WAITING_ON_BOARD(code) \
  uart_get_buffered_data_len(UART_NUM_2, &bytesInQ); \
  while (!bytesInQ) { \
    code \
      uart_get_buffered_data_len(UART_NUM_2, &bytesInQ); \
  }
#else
#define WHILE_WAITING_ON_BOARD(code) \
  while (!boardWatch.available()) { \
    code \
  }
#endif

#define printf_host(format, ...) Serial.printf(format, __VA_ARGS__)

#ifdef FAST_UART
#define write_board(buf, len) uart_tx_chars(UART_NUM_2, (const char*)buf, len)
#define write_host(buf, len) uart_tx_chars(UART_NUM_0, (const char*)buf, len)
#define read_board(buf, len) uart_read_bytes(UART_NUM_2, buf, len, BOARD_TIMEOUT)
#define read_host(buf, len) uart_read_bytes(UART_NUM_0, buf, len, HOST_TIMEOUT)
#define wait_for_tx() uart_wait_tx_done(UART_NUM_2, BOARD_TIMEOUT)
#define getBoardBytesInQ() uart_get_buffered_data_len(UART_NUM_2, &bytesInQ)
#define getHostBytesInQ() uart_get_buffered_data_len(UART_NUM_0, &bytesInQ)
/*
#define printf_host(format, ...) \
sprintf(hostDirectBuffer, format, __VA_ARGS__); \
uart_tx_chars(UART_NUM_0,hostDirectBuffer, BUFFER_SIZE);
*/
#else

#define write_board(buf, len) boardWatch.write(buf, len)
#define write_host(buf, len) Serial.write(buf, len)
#define read_board(buf, len) boardWatch.readBytes(buf, len)
#define read_host(buf, len) Serial.readBytes(buf, len)
#define wait_for_tx() boardWatch.flush()
#define getBoardBytesInQ() bytesInQ = boardWatch.available()
#define getHostBytesInQ() bytesInQ = Serial.available()
// #define printf_host(format, ...) Serial.printf(format, __VA_ARGS__)

#endif

void setup() {
  // For more accurate sampling because the ADC theoretically maxes out at 2 MSPS and some cycles are wasted during power monitoring
  setCpuFrequencyMhz(240);



  Serial.begin(HOST_BAUD);
  boardWatch.begin(BOARD_BAUD, SERIAL_8N1, RXD2, TXD2);
  // hostWatch.begin(115200, SERIAL_8N1, RX1, TX1);
  boardWatch.setTimeout(BOARD_TIMEOUT);

  /*
  #ifdef FAST_UART
  uart_disable_intr_mask(UART_NUM_2, ~0);
  uart_disable_intr_mask(UART_NUM_0, ~0);
  #endif
  */


#ifdef DEBUG
  timer = timerBegin(32000000);  // 32 MHz = MSPM0l CPU clock
  if (timer == NULL)
    Serial.println("Timer Configuration Failed");
  timerStop(timer);
#endif

  pinMode(TRACE_STATUS_PIN, OUTPUT);

#ifdef I2S_IMPL
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
    .sample_rate = I2S_SAMPLE_RATE,                // The format of the signal using ADC_BUILT_IN
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,  // is fixed at 12bit, stereo, MSB
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 2,  // no choice
    .dma_buf_len = ADC_BUFFER_SIZE,
    .use_apll = true,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11);
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_adc_mode(ADC_UNIT_1, ADC1_CHANNEL_4);
  i2s_adc_enable(I2S_NUM_0);
#else
  adc_continuous_handle_cfg_t adc_config = {
    .max_store_buf_size = ADC_BUFFER_SIZE,
    .conv_frame_size = CONV_FRAME_SIZE,
  };
  ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adc_handle));

  adc_continuous_config_t dig_cfg = {
    .sample_freq_hz = SOC_ADC_SAMPLE_FREQ_THRES_HIGH,
    .conv_mode = ADC_CONV_SINGLE_UNIT_1,
  };

  adc_digi_pattern_config_t adc_pattern;
  dig_cfg.pattern_num = 1;

  adc_pattern.atten = ADC_ATTEN_DB_12;
  adc_pattern.channel = ADC_CHANNEL_4;  // GPIO_32
  adc_pattern.unit = ADC_UNIT_1;
  adc_pattern.bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

  dig_cfg.adc_pattern = &adc_pattern;

  ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &dig_cfg));
#endif
}

// sizeof(buffer) must be a multiple of HOST_PACKET_SIZE
inline void sendToHostAsPackets(uint8_t* buffer, uint16_t len) {

  write_host((uint8_t*)&len, sizeof(len));
  for (int i = 0; i < len; i += HOST_PACKET_SIZE) {
    // necessary to wait in between for the host to consume sent bytes because
    // we avoid using the implicit ring-buffer-to-hardware-FIFO ISR in order to have consistent timing
    read_host(hostDirectBuffer + BUFFER_SIZE - 1, 1);  // A
    write_host(buffer + i, HOST_PACKET_SIZE);          // packet
  }
}

// HOST must send an acknowledgement byte after receiving 128 bytes of reading data
// maxSamples must be a multiple of (HOST_PACKET_SIZE>>1) samples (i.e. HOST_PACKET_SIZE bytes)
int powerTrace(const uint8_t* command, size_t c_len, uint8_t* plaintext, size_t p_len, uint8_t traceLShift, uint16_t maxSamples) {
#ifdef I2S_IMPL
  size_t bytesRead;
#else
  uint32_t bytesRead;
#endif

  uint8_t* adc_read_marker = (uint8_t*)adcBuffer;

  // cleanup from any past uses
  memset(adc_reading_size_buffer, 0, sizeof(adc_reading_size_buffer));
  memset(averages, 0, sizeof(averages));

  int traceCount = 1 << traceLShift;

  for (int trace = 0; trace < traceCount; trace++) {
    uint32_t traceBytesRead = 0;

    uart_flush(UART_NUM_2);
    write_board(command, c_len);
    wait_for_tx();

#ifdef DEBUG
    printf_host("\nOn trace %d\n", trace);
#endif

    bytesInQ = read_board(boardInBuffer, sizeof(ACK));
#ifdef DEBUG
    if (bytesInQ <= 0 || boardInBuffer[1] != 'A')  // equiv to checking that we read ACK
    {
      printf_host("Failed Reading ACK 1, (Buffer: %s)", boardInBuffer);
      hexdumpRX(bytesInQ);
      return -1;
    }
#else
    if (bytesInQ <= 0) {
      write_host(FAILED_READ, sizeof(FAILED_READ));
      write_host(boardInBuffer, sizeof(ACK));
      return -1;
    }
#endif

    write_board(plaintext, p_len);
    digitalWrite(TRACE_STATUS_PIN, HIGH);
    wait_for_tx();

    // We start reading at this point so that processing beyond the ACK on the target board
    // (aka our target operations) dont slip through the cracks while we process the ACK on our end


#ifdef I2S_IMPL
    i2s_read(I2S_NUM_0, &adc_read_marker, (size_t)(ADC_BUFFER_SIZE), &bytesRead, 0);
#else
    adc_continuous_flush_pool(adc_handle);
    adc_continuous_start(adc_handle);
#endif

    bytesInQ = read_board(boardInBuffer + sizeof(ACK), sizeof(ACK));
#ifdef DEBUG
    if (bytesInQ <= 0 || boardInBuffer[1 + sizeof(ACK)] != 'A')  // equiv to checking that we read ACK
    {
      // adc_continuous_stop(adc_handle);
      printf_host("Failed Reading ACK 2, (Buffer: %s)", boardInBuffer);
      hexdumpRX(bytesInQ);
      digitalWrite(TRACE_STATUS_PIN, LOW);
      return -1;
    }
#else
    if (bytesInQ <= 0)  // bne on reg
    {
      // adc_continuous_stop(adc_handle);
      write_host(FAILED_READ, sizeof(FAILED_READ));
      write_host(boardInBuffer, 2 * sizeof(ACK));
      digitalWrite(TRACE_STATUS_PIN, LOW);
      return -1;
    }
#endif
    WHILE_WAITING_ON_BOARD(
#ifdef I2S_IMPL
      i2s_read(I2S_NUM_0, adc_read_marker, (size_t)(CONV_FRAME_SIZE), &bytesRead, ADC_TIMEOUT_MS);
#else
      adc_continuous_read(adc_handle, adc_read_marker, CONV_FRAME_SIZE, &bytesRead, ADC_TIMEOUT_MS);
#endif

      // moving to next read location
      adc_read_marker += bytesRead;
      traceBytesRead += bytesRead;)

#ifndef I2S_IMPL
    adc_continuous_stop(adc_handle);
#endif
    // Consume all data in RX buffer
    read_board(boardInBuffer + 2 * sizeof(ACK), HEADER_SIZE);
    write_board(ACK, sizeof(ACK));
    wait_for_tx();
    // Interpert the length bytes of the header and use them to read the body (up to first 256 bytes, we only care insofar as the data looks right)
    read_board(boardInBuffer + 2 * sizeof(ACK) + HEADER_SIZE, (size_t)(*(uint16_t*)(boardInBuffer + 10)));
    write_board(ACK, sizeof(ACK));

#ifdef DEBUG
    printf_host("After Trace Response:\t%s\t(", boardInBuffer);
    hexdumpRX(BUFFER_SIZE);
    printf_host("Read %d bytes of trace\n", traceBytesRead);
#else
    delay(25);  // for stability across iterations
#endif

    adc_reading_size_buffer[trace] = (uint16_t)traceBytesRead;

    // if we want to cut off some samples, we expect to hit this often
    if (traceBytesRead >> 1 >= maxSamples)
      break;

    // if it seems like next sample cant fit and we arent on the last sample
    // could overflow if totalBytesRead is an underestimate. but that is unlikely and loop code is performance critical

    if ((adc_read_marker - (uint8_t*)adcBuffer) + traceBytesRead > sizeof(adcBuffer) && trace < traceCount - 1) {
#ifdef DEBUG
      printf_host("adcBuffer expected to overflow. Try increasing its size if you want to use all %d traces", traceCount);
#else
      write_host(FAILED_READ, sizeof(FAILED_READ));
#endif
      digitalWrite(TRACE_STATUS_PIN, LOW);
      return -1;
    }

    memset(boardInBuffer, 0, BUFFER_SIZE);
  }

  digitalWrite(TRACE_STATUS_PIN, LOW);

  // skip costly averaging
  if (traceCount == 1) {

    uint16_t frameLength = adc_reading_size_buffer[0] >> 1;
    uint16_t samplesToSend = frameLength > maxSamples ? maxSamples : frameLength;

    for (int i = 0; i < samplesToSend; i++) {
      adcBuffer[i] &= ADC_READING_MASK;
    }
#ifdef DEBUG
    printf_host("Skipping averaging. Writing %d bytes to host.", samplesToSend << 1);
#endif
    sendToHostAsPackets((uint8_t*)adcBuffer, (samplesToSend << 1));
    return 0;
  }

  // uart_flush(UART_NUM_2);

  uint16_t largestFrameSampleCount = 0;

  uint16_t* packetStart = adcBuffer;
  for (int trace = 0; trace < traceCount; trace++) {
    uint16_t packetLength = adc_reading_size_buffer[trace] >> 1;
    for (int j = 0; j < packetLength; j++) {
      averages[j] += (packetStart[j] & ADC_READING_MASK);
    }

    if (packetLength > largestFrameSampleCount)
      largestFrameSampleCount = packetLength;

    packetStart += packetLength;
    /*
    #ifdef DEBUG
      // printf_host("\nPacket Length: %d",packetLength);
      printf_host("Current averages[0]: %d",averages[0]);
    #endif
    */
  }

#ifdef DEBUG
  printf_host("largestFrameSampleCount=%d", largestFrameSampleCount);
#endif

#ifdef DEBUG
  if (largestFrameSampleCount >= sizeof(averages)) {
    printf_host("Too many bytes in the largest trace to fit into the averages buffer. Increase the buffer size and try again.", 0);
    return -1;
  }
#endif

  if (largestFrameSampleCount > maxSamples)
    largestFrameSampleCount = maxSamples;

  for (int i = 0; i < largestFrameSampleCount; i++) {
    averages[i] >>= traceLShift;
    /*
    #ifdef DEBUG
    printf_host("%d\t",averages[i]);
    if ((i & 0x3F) == 0)
      printf_host("\n",0);
    #endif
    */
  }
  // We now use the fact that  CONV_FRAME_SIZE is a multiple of 128 bytes and therefore so is the original maxSampleCount in bytes
  // int16_t remainder = maxSampleCount & 0x7F; // this is modulo 128, which = 0 with the above condition

  sendToHostAsPackets((uint8_t*)averages, largestFrameSampleCount << 1);

#ifdef DEBUG
  printf_host("\n", 0);
#endif

  return 0;
}

// m_IV<N
// where:
// I = byte index
// V = byte value
// < = left shift of 1 for trace count to average across
// N = number of random plaintexts to take
/* 
void powerMultiShot()
{
  uint8_t targetIndex = hostDirectBuffer[2];
  if (targetIndex > 5)
    targetIndex = 5;

  uint8_t value = hostDirectBuffer[3];

  uint8_t traceLShift = hostDirectBuffer[4];
  if (traceLShift > MAX_TRACE_LSHIFT)
    traceLShift = MAX_TRACE_LSHIFT; 

  uint8_t N = hostDirectBuffer[5];

  uint8_t plaintext[7];
  
  for (int i = 0; i < N; i++)
  {
    esp_fill_random(plaintext, sizeof(plaintext));
    plaintext[targetIndex] = value;
    if (powerTrace(readCommand, sizeof(readCommand),plaintext,sizeof(plaintext),traceLShift) < 0)
      return;
  } 
}
*/
// p_KEYVALS<MC
// where:
// S = slot byte
// < = left shift of 1 that gives trace count to average across
// MC = 2 bytes for max sample count
inline void powerSingleShot() {
  // Modify for arbitrary power
  uint8_t traceLShift = (uint8_t)hostDirectBuffer[9];
  if (traceLShift > MAX_TRACE_LSHIFT)
    traceLShift = MAX_TRACE_LSHIFT;

  uint16_t maxSamples = *((uint16_t*)&hostDirectBuffer[10]);
#ifdef DEBUG
  printf_host("Read %d as maxSamples", maxSamples);
#endif

  if (maxSamples == 0)
    maxSamples = sizeof(averages) >> 1;

  // Convert to a multiple of 64
  maxSamples &= ~0x3F;

  powerTrace(readCommand, sizeof(readCommand), (uint8_t*)(hostDirectBuffer + 2), 7, traceLShift, maxSamples);

  // Serial.printf("\t%d\t%d\n",sampleCount,powerSum);
}
#ifdef DEBUG
// Should be based on the LED in the future
void timingAnalysis() {
  // ensure empty buffer (formerly loop)
  uart_flush(UART_NUM_2);
  memset(boardInBuffer, 0, (size_t)BUFFER_SIZE);
  timerRestart(timer);

  printf_host("\n\nTiming command: %s\n", readCommand);
  write_board(readCommand, sizeof(readCommand));
  wait_for_tx();

  /*
  Serial.printf("\n\nTiming command: %s\n",readCommand);
  boardWatch.write(readCommand, sizeof(readCommand));
  boardWatch.flush();
  */
  // READ ACK

  bytesInQ = read_board(boardInBuffer, sizeof(ACK));
  if (!bytesInQ) {
    printf_host("Failed to read first ACK", 0);
    return;
  }


  write_board(pin, 6);
  write_board(pin, 1);
  wait_for_tx();

  bytesInQ = read_board(boardInBuffer + sizeof(ACK), sizeof(ACK));
  if (!bytesInQ) {
    printf_host("Failed to read second  ACK", 0);
    return;
  }

  timerStart(timer);

  // terminate upon reading a single char to minimize the time measured that went to UART
  bytesInQ = read_board(boardInBuffer + 2 * sizeof(ACK), 1);
  timerStop(timer);
  char c = *(boardInBuffer + 2 * sizeof(ACK));

  if (bytesInQ) {
    if (c != '%') {
      printf_host("Improper message detected (%c)\n", c);
      printf_host("Managed to read:\t%.*s\t(", 2 * sizeof(ACK) + 1, boardInBuffer);
      hexdumpRX(BUFFER_SIZE);
      return;
    }
    // finish reading the header
    read_board(boardInBuffer + 2 * sizeof(ACK) + 1, HEADER_SIZE - 1);
    write_board(ACK, sizeof(ACK));
    wait_for_tx();
    // Interpert the length bytes of the header and use them to read the body (up to first 256 bytes, we only care insofar as the data looks right)
    read_board(boardInBuffer + 2 * sizeof(ACK) + HEADER_SIZE, (size_t)(*(uint16_t*)(boardInBuffer + 10)));
    write_board(ACK, sizeof(ACK));


    printf_host("Time in Microseconds:\t%lld\n", timerReadMicros(timer));
    printf_host("Full Response:\t%.*s\t(", 2 * sizeof(ACK) + HEADER_SIZE, boardInBuffer);
    hexdumpRX(BUFFER_SIZE);
  } else {
    printf_host("No Response Detected\n", 0);
    printf_host("Managed to read:\t%s\t(", boardInBuffer);
    hexdumpRX(BUFFER_SIZE);
  }
}
#endif

#ifdef DEBUG
inline void hexdumpRX(int bytesInQ) {
  for (int i = 0; i < bytesInQ; i++)
    printf_host("%02x ", (uint8_t)boardInBuffer[i]);
  printf_host("\n", 0);
}
#endif

void makeListen() {
  uint8_t msg[4] = { '%', 'N', 0, 0 };
  write_board(msg, sizeof(msg));
  read_board(boardInBuffer, 4);
  printf_host("Received: %s", boardInBuffer);
}

void makeReceive() {
  uint8_t msg[4] = {'%', 'C', 8,0};
  write_board(msg, sizeof(msg));
  read_board(boardInBuffer,4); // ack
  printf_host("Received: %s", boardInBuffer);
  write_board(pin,6);
  write_board(&msg[3],1); // read from slot 0
  uint8_t one = 1;
  write_board(&one, 1); // write to slot 1
  read_board(boardInBuffer,4); // ack
  printf_host("Received: %s", boardInBuffer);
}

void interrogate() {
  memset(boardInBuffer, 0, sizeof(boardInBuffer));
  printf_host("\nInterrogating.", NULL);
  uint8_t msg[4] = { '%', 'I', 0, 0 };
  //write_board(msg, sizeof(msg));
  //read_board(boardInBuffer, sizeof(ACK));
  //write_board(pin,6);
  read_board(boardInBuffer, 4);
  printf_host("\nReceived header: \n%s", boardInBuffer);
  getBoardBytesInQ();
  if (bytesInQ) {
    read_board(boardInBuffer, *((uint16_t*)&boardInBuffer[2]));
    printf_host("\nReceived from interrogation:\n%s\nHexdump:\n", bytesInQ, boardInBuffer);
    hexdumpRX(bytesInQ);
  } else
    printf_host("Failed to get anything from interrogation.", NULL);
}

void loop() {

  getHostBytesInQ();

  if (bytesInQ) {
    read_host(hostDirectBuffer, (size_t)bytesInQ);

    /*
    if (state & 1)
      Serial.printf("Recieved Directly From Host:\n%s\n", hostDirectBuffer);
    */
    switch (hostDirectBuffer[0]) {
#ifdef DEBUG

      case 'e':
        state ^= 1;
        break;
      case 's':
        printf_host("Sending List Command\n", 0);
        write_board(listCommand, (size_t)4);
        write_board(pin, 6);
        break;
      // Modify for arbitrary timing
      case 't':
        timingAnalysis();
        break;
      case 'i':
        interrogate();
        break;
      case 'n':
        makeListen();
        break;
      case 'a':
        write_board(ACK, sizeof(ACK));
        break;
      case 'c':
        makeReceive();
        break;

#endif
      // syntax p_KEYVALS<MC
      case 'p':
        powerSingleShot();
        break;

        //syntax m_IV<N
        /*
      case 'm':
        powerMultiShot();
        break;
      */
    }


    memset(hostDirectBuffer, 0, BUFFER_SIZE);
    memset(boardInBuffer, 0, BUFFER_SIZE);
  }
#ifdef DEBUG
  getBoardBytesInQ();
  if (bytesInQ && state & 1) {
    // Read data and display it
    read_board(boardInBuffer, bytesInQ);
    printf_host("Eavesdropped From Board:\n%s\t(", boardInBuffer);
    hexdumpRX(bytesInQ);

    memset(boardInBuffer, 0, BUFFER_SIZE);
  }
#endif
  /*
  bytesInQ = hostWatch.available();
  if (bytesInQ && state & 1) {
    hostWatch.readBytes(hostInBuffer, (size_t)bytesInQ);
    Serial.printf("Eavedropped From Host:\n%s\t",hostInBuffer);
    for (int i = 0; i < bytesInQ; i++)
      Serial.printf("%02x ", (uint8_t)hostInBuffer[i]);
    Serial.print("\n",0);

    memset(hostInBuffer, 0, (size_t)BUFFER_SIZE);
  }
  */
}
