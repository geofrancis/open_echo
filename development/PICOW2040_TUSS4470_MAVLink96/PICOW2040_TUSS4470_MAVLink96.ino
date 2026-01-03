#include <SPI.h>
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "mavlink/common/mavlink.h"
#include "mavlink/ardupilotmega/mavlink_msg_data96.h"
#include "mavlink/common/mavlink_msg_rc_channels.h"
#include "mavlink/common/mavlink_msg_mission_current.h"
#include "mavlink/common/mavlink_msg_command_long.h"





// -------------------- PIN DEFINITIONS --------------------
const int SPI_CS = 17;    // Chip select for TUSS4470
const int IO1 = 3;        // Enable pin or control (set HIGH)
const int IO2 = 2;        // Burst output pin (transducer drive)
const int O4 = 20;        // TUSS4470 OUT4 threshold detect input
const int analogIn = 26;  // ADC0 (GPIO26)

// -------------------- SAMPLING SETTINGS --------------------
#define NUM_SAMPLES 5000
#define BLINDZONE_SAMPLE_END 450
#define THRESHOLD_VALUE 0x19

uint16_t samples[NUM_SAMPLES];
volatile bool detectedDepth = false;
volatile int depthDetectSample = 0;
volatile int sampleIndex = 0;

float temperature = 0.0f;
int vDrv = 0;



#define SYSTEM_ID 1
#define COMPONENT_ID 191
#define PARAM_COUNT 2  // Total parameters defined

// Global parameters with default values
float param1 = 1.23f;
float param2 = 4.56f;


#define Scaled_Column 96
uint16_t currentDepthIndex = 0;  // Sample index (0-5000)
float currentDepth = 0;          // Depth in cm (for display)

#define NUM_SAMPLES 5000      // Number of samples per measurement - must match RP2040
#define SPEED_OF_SOUND 330    // Speed of sound in m/s (330=air, ~1500=water)
#define SAMPLE_TIME 1.554e-6  // Time per sample in seconds - must match RP2040 setup

void downsampleToColumn(uint16_t *samples, uint16_t *column);

int previousMillis1;

// -------------------- SPI BUFFERS --------------------
byte misoBuf[2];
byte inByteArr[2];

// -------------------- BURST GENERATION --------------------
void generateBurst(uint pin, float frequency, uint cycles) {
  PIO pio = pio0;
  uint sm = pio_claim_unused_sm(pio, true);

  float clk_sys_hz = (float)clock_get_hz(clk_sys);
  float period = 1.0f / frequency;
  float half_period = period / 2.0f;
  float target_rate = 1.0f / half_period;
  float div = clk_sys_hz / target_rate;

  static const uint16_t program[] = {
    0xe081,  // set pins, 1
    0xe001   // set pins, 0
  };
  uint offset = pio_add_program_at_offset(pio, (const pio_program_t *)&program, 0);

  pio_gpio_init(pio, pin);
  pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);

  pio_sm_config c = pio_get_default_sm_config();
  sm_config_set_set_pins(&c, pin, 1);
  sm_config_set_clkdiv(&c, div);
  sm_config_set_wrap(&c, offset, offset + 1);
  pio_sm_init(pio, sm, offset, &c);
  pio_sm_set_enabled(pio, sm, true);

  for (uint i = 0; i < cycles; i++) {
    pio_sm_exec(pio, sm, pio_encode_set(pio_pins, 1));
    sleep_us(half_period * 1e6);
    pio_sm_exec(pio, sm, pio_encode_set(pio_pins, 0));
    sleep_us(half_period * 1e6);
  }

  pio_sm_set_enabled(pio, sm, false);
  pio_remove_program(pio, (const pio_program_t *)&program, offset);
  pio_sm_unclaim(pio, sm);
}

// -------------------- INTERRUPT HANDLER --------------------
void handleInterrupt() {
  if (!detectedDepth) {
    depthDetectSample = sampleIndex;
    detectedDepth = true;
  }
}

// -------------------- SPI UTILS --------------------
void spiTransfer(byte *mosi, byte sizeOfArr) {
  memset(misoBuf, 0x00, sizeof(misoBuf));

  digitalWrite(SPI_CS, LOW);
  for (int i = 0; i < sizeOfArr; i++) {
    misoBuf[i] = SPI.transfer(mosi[i]);
  }
  digitalWrite(SPI_CS, HIGH);
}

unsigned int BitShiftCombine(unsigned char x_high, unsigned char x_low) {
  return (x_high << 8) | x_low;
}

byte parity16(unsigned int val) {
  byte ones = 0;
  for (int i = 0; i < 16; i++) {
    if ((val >> i) & 1) {
      ones++;
    }
  }
  return (ones + 1) % 2;
}

byte tuss4470Parity(byte *spi16Val) {
  return parity16(BitShiftCombine(spi16Val[0], spi16Val[1]));
}

byte tuss4470Read(byte addr) {
  inByteArr[0] = 0x80 + ((addr & 0x3F) << 1);
  inByteArr[1] = 0x00;
  inByteArr[0] |= tuss4470Parity(inByteArr);
  spiTransfer(inByteArr, sizeof(inByteArr));
  return misoBuf[1];
}

void tuss4470Write(byte addr, byte data) {
  inByteArr[0] = (addr & 0x3F) << 1;
  inByteArr[1] = data;
  inByteArr[0] |= tuss4470Parity(inByteArr);
  spiTransfer(inByteArr, sizeof(inByteArr));
}



void command_heartbeat() {

  //< ID 1 for this system
  int sysid = 100;
  //< The component sending the message.
  int compid = MAV_COMP_ID_PATHPLANNER;

  // Define the system type, in this case ground control station
  uint8_t system_type = MAV_TYPE_GCS;
  uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;

  uint8_t system_mode = 0;
  uint32_t custom_mode = 0;
  uint8_t system_state = 0;

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_heartbeat_pack(sysid, compid, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  // Send the message
  // delay(1);
  Serial.write(buf, len);
}

void send_data96() {
  const uint8_t type = 42;  // arbitrary type identifier
  uint8_t data[96];         // payload buffer

  // fill the payload – here we just use a counter pattern
  for (uint8_t i = 0; i < sizeof(data); ++i) {
    data[i] = i;
  }

  mavlink_message_t msg;

  // Pack the message.
  //   sysid = 1, compid = 200, type, len (=96), and payload pointer
  mavlink_msg_data96_pack(
    1,    // system_id
    200,  // component_id
    &msg,
    type,          // type field
    sizeof(data),  // len field (number of valid bytes in data[])
    data);         // pointer to payload

  // Convert the message into a byte buffer ready for transmission.
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  // Send over Serial2 (MAVLink UART)
  Serial2.write(buf, len);
}


void Sonar_Scan() {
  tuss4470Write(0x1B, 0x01);         // Start time-of-flight
  generateBurst(IO2, 40000.0f, 16);  // 40kHz, 16 cycles
  // generateBurst(IO2, 1000000.0f, 16);  // 1000kHz, 16 cycles needs to be fixed!! #TODO

  unsigned long startTime = micros();

  for (sampleIndex = 0; sampleIndex < NUM_SAMPLES; sampleIndex++) {
    adc_select_input(0);
    adc_run(true);
    adc_hw->cs |= ADC_CS_START_ONCE_BITS;
    while (adc_hw->cs & ADC_CS_START_ONCE_BITS)
      ;
    samples[sampleIndex] = adc_hw->result;
    delayMicroseconds(5);  // 6 uS total sampling per sample
    //delayMicroseconds(11); //    delayMicroseconds(12); // 13 uS total sampling per sample

    if (sampleIndex == BLINDZONE_SAMPLE_END)
      detectedDepth = false;
  }

  tuss4470Write(0x1B, 0x00);  // Stop time-of-flight

  unsigned long elapsedTime = micros() - startTime;

  // Serial.println(elapsedTime);
  sendData();

  delay(10);
}


void sendData() {

  uint16_t depth;
  float temperature, driveVoltage;

  // Store depth values
  currentDepthIndex = depth;                                        // Sample index (0-5000)
  currentDepth = depth * (SPEED_OF_SOUND * SAMPLE_TIME * 100) / 2;  // cm

  // Downsample 5000 samples to waterfall height (240 pixels vertically)
  uint16_t downsampledColumn[Scaled_Column];
  downsampleToColumn(samples, downsampledColumn);
  send_data96();
}




// ==================== DOWNSAMPLING ====================
void downsampleToColumn(uint16_t *samples, uint16_t *column) {
  // Downsample 5000 samples to 200 pixels (vertical)
  // Each pixel represents ~25 samples (5000 / 200)
  const int samplesPerPixel = NUM_SAMPLES / Scaled_Column;

  for (int y = 0; y < Scaled_Column; y++) {
    uint32_t blockSum = 0;
    int startIdx = y * samplesPerPixel;

    for (int i = 0; i < samplesPerPixel; i++) {
      blockSum += samples[startIdx + i];
    }

    uint16_t average = static_cast<uint16_t>(blockSum / samplesPerPixel);

    // FLIP VERTICALLY: sample 0 (near) at bottom, sample 4999 (far) at top
    int displayY = Scaled_Column - 1 - y;
    column[displayY] = average;
  }
}











void FetchMavlinkSerial() {
  //Serial.println("*");
  mavlink_message_t msg;
  mavlink_status_t status;

  //telnet.print("fc modein ");
  //telnet.println(fcmodein);
  // Serial.print("-");
  while (Serial2.available()) {
    uint8_t c = Serial2.read();
    // Serial.println(c);
    //Serial.println("serial data");
    //Get new message
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      //  Serial.println("mav packet");
      //Handle new message from autopilot

      switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
          {
            // Serial.println(" ");
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&msg, &hb);

            //Serial.print("custom Mode: ");
            //Serial.println(hb.custom_mode);
            //Serial.print("Type: ");
            //Serial.println(hb.type);
            //Serial.print("Autopilot: ");
            // Serial.println(hb.autopilot);
            //Serial.print("Base Mode: ");
            // Serial.println(hb.base_mode);


            //  Serial.println("fcmodein ");
            // Serial.println(fcmodein);
            //Serial.print("System Status: ");
            //Serial.println(hb.system_status);
            //     Serial.print("Mavlink Version: ");
            //     Serial.println(hb.mavlink_version);
            //Serial.println();
          }
          break;
        case MAVLINK_MSG_ID_PARAM_SET:
          processParamSet(&msg);
          break;
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
          processParamRequestRead(&msg);
          Serial.println(" Parameters READ REQUEST");
          break;
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
          Serial.println(" Parameters READ REQUEST LIST");
          processParamRequestList();
          break;

        case MAVLINK_MSG_ID_GPS_RAW_INT:
          {
            mavlink_gps_raw_int_t packet;
            mavlink_msg_gps_raw_int_decode(&msg, &packet);

            mavlink_gps_raw_int_t datagps;
            mavlink_msg_gps_raw_int_decode(&msg, &datagps);
            // Serial.println("PX HB");
            // Serial.println("GPS Data ");
            //Serial.print("time usec: ");
            // Serial.println(datagps.time_usec);
            //  Serial.print("lat: ");
            //     Serial.println(datagps.lat);
            // GPSLAT = (datagps.lat);
            //Serial.print("lon: ");
            //Serial.println(datagps.lon);
            // GPSLON = (datagps.lon);
            //  Serial.print("alt: ");
            //   Serial.println(datagps.alt);
            //Serial.print("Sattelite visible: ");
            //  Serial.println(datagps.satellites_visible);
            //  satelites = (datagps.satellites_visible);
            //   Serial.println(datagps.eph);
            // HDOP = (datagps.eph);
            //Serial.println(datagps.epv);
          }
          break;

        case MAVLINK_MSG_ID_SYS_STATUS:  // #1: SYS_STATUS
          {
            //mavlink_message_t* msg;
            mavlink_sys_status_t sys_status;
            mavlink_msg_sys_status_decode(&msg, &sys_status);
            //Serial.print("PX SYS STATUS: ");
            // Serial.print("[Bat (V): ");
            // Serial.print(sys_status.voltage_battery);
            // VOLTS = (sys_status.voltage_battery);
            // Serial.print(sys_status.voltage_battery / 1000);
            //   Serial.print("], [Bat (A): ");
            //   Serial.print(sys_status.current_battery);
            // AMPS = (sys_status.current_battery);
            //  Serial.print("], [Comms loss (%): ");
            //  Serial.print(sys_status.drop_rate_comm);
            //comdroprate = (sys_status.drop_rate_comm);
            //      Serial.println("]");
          }


          break;

        case MAVLINK_MSG_ID_ATTITUDE:  // #30
          {

            mavlink_attitude_t attitude;
            mavlink_msg_attitude_decode(&msg, &attitude);
            // Serial.println("PX ATTITUDE");
            // Serial.println(attitude.roll);
  //          roll = (attitude.roll);
    //        pitch = (attitude.pitch);
    //        yaw = (attitude.yaw);
            //if (attitude.roll > 1) leds_modo = 0;
            //else if (attitude.roll < -1) leds_modo = 2;
            //else leds_modo = 1;
          }

          break;

        case MAVLINK_MSG_ID_RC_CHANNELS_RAW:  // #35
          {
            mavlink_rc_channels_raw_t chs;
            mavlink_msg_rc_channels_raw_decode(&msg, &chs);
            //  Serial.print("Chanel 1 raw ");
            //   Serial.println(chs.chan1_raw);
          }

          break;
          /*
        case MAVLINK_MSG_ID_RC_CHANNELS_SCALED:  // #35
          {
            mavlink_rc_channels_scaled_t RCCHANNEL;
            mavlink_msg_rc_channels_scaled_decode(&msg, &RCCHANNEL);
            //  Serial.print("Chanel 6 (3-Kanal Schalter): ");
            // int RAW_SERVO = RCCHANNEL.chan6_scaled;
            // Serial.println(RAW_SERVO);
            // Serial.print("Chanel 5 (Schub): ");
            //  Serial.println(RCCHANNEL.chan5_scaled);
            //  Serial.print("Drei Kanal: ");
            //  Serial.println(mavlink_msg_rc_channels_scaled_get_chan6_scaled(&msg));
            //  Serial.print("Schub: ");
            //  Serial.println(mavlink_msg_rc_channels_scaled_get_chan5_scaled(&msg));
          }

          break;

        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:  // #30
          {
            mavlink_global_position_int_t Position;
            //mavlink_msg_attitude_decode(&msg, &attitude);
            mavlink_msg_global_position_int_decode(&msg, &Position);
          }
          break;
*/



        case MAVLINK_MSG_ID_RC_CHANNELS:  // #35
          {
            mavlink_rc_channels_t chs;
            mavlink_msg_rc_channels_decode(&msg, &chs);
            //       Serial.print("Chanel 8: ");
            //  RCIN = (chs.chan8_raw);
            //Functions();
            //  Serial.println("Channel 1");
            //  Serial.println(chs.chan1_raw);
          }
          break;
          /* case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:  // #35
          {
            mavlink_servo_output_raw_t SERVOCHANNEL;
            mavlink_msg_servo_output_raw_decode(&msg, &SERVOCHANNEL);
            //  Serial.println(SERVOCHANNEL.servo1_raw);
            //  Serial.println(SERVOCHANNEL.servo2_raw);
            //  Serial.println(SERVOCHANNEL.servo3_raw);
            //  Serial.print("Chanel 1 (raw): ");
          }
          break;
*/
        case MAVLINK_MSG_ID_MISSION_CURRENT:
          {
            mavlink_mission_current_t RPNUM;
            mavlink_msg_mission_current_decode(&msg, &RPNUM);
            //  Serial.print("wp_number ");
            //  Serial.println(wp_number);
        //    wp_number = (RPNUM.seq);
          }
          break;

        case MAVLINK_MSG_ID_VFR_HUD:
          {
            mavlink_vfr_hud_t vfrhud;
            mavlink_msg_vfr_hud_decode(&msg, &vfrhud);
            //   Serial.print("Ground Speed: ");
            //   Serial.println(vfrhud.groundspeed);
            ////  Serial.print("Heading ");
            //   Serial.println(vfrhud.heading);
          //  gps_Vel = vfrhud.groundspeed;
            // gps_Head = vfrhud.heading;
          }

          break;
        case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
          {
            mavlink_nav_controller_output_t navout;
            mavlink_msg_nav_controller_output_decode(&msg, &navout);
            //  navbearing = navout.target_bearing;
            //   wpdist = navout.wp_dist;
            //  xtrackerror = navout.xtrack_error;
            //    Serial.print("navbearing: ");
            //    Serial.println(navout.nav_bearing);
            // Serial.print("wpdist ");
            //  Serial.println(navout.wp_dist);
            // Serial.print("xtrackerror ");
            // Serial.println(navout.xtrack_error);
          }

          break;

        case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT:
          mavlink_named_value_float_t valueFloat;
          mavlink_msg_named_value_float_decode(&msg, &valueFloat);
          // Serial.print(" float: ");
          // Serial.println(valueFloat.name);
          // Serial.print("  ");
          // Serial.println(valueFloat.value, 0);
          //  String stringname = (valueFloat.name);
          //  messagedata = (valueFloat.value);
          // Serial.print(" float: ");
          //Serial.print(valueFloat.name);
          // Serial.print("  ");
          // Serial.println(valueFloat.value);
          break;


        case MAVLINK_MSG_ID_STATUSTEXT:  //  #253  https://mavlink.io/en/messages/common.html#STATUSTEXT
          {
            mavlink_statustext_t packet;
            mavlink_msg_statustext_decode(&msg, &packet);
            //if (packet.severity > 0) {
            // Serial.print("MAVMSG---------------------------------------------------");
            //Serial.print(" severity: ");
            // Serial.print(packet.severity);
            //    Serial.print("     text: ");
            //  Serial.println(packet.text);
            // mavmessage = (packet.text);
          }
      }
      break;
    }
  }
}





void saveParameters() {
  Serial.println ("Save Parameters");
  //preferences.putFloat("param1", param1);
//  preferences.putFloat("param2", param2);
}

/*
  loadParameters()
  Loads parameter values from storage, using default values if none are stored.
*/
void loadParameters() {
  Serial.println ("Load Parameters");
//  param1 = preferences.getFloat("param1", 1.23f);
 // param2 = preferences.getFloat("param2", 4.56f);
}

/*
  sendParamValue()
  Packs and sends a PARAM_VALUE message with the given parameter ID and value.
  
  Parameters:
    - param_id: The string identifier of the parameter.
    - value: The floating point parameter value.
    - param_index: The index of the parameter in the global list.
*/
void sendParamValue(const char* param_id, float value, uint8_t param_index) {
  mavlink_message_t msg;
    Serial.println ("Send Parameter Value");
  mavlink_msg_param_value_pack(1, 191, &msg,
                               param_id, value, MAV_PARAM_TYPE_REAL32,
                               PARAM_COUNT, param_index);

  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial2.write(buf, len);
}

/*
  processParamSet()  Handles received PARAM_SET messages: updates the parameter, saves it, and replies.
*/
void processParamSet(const mavlink_message_t* msg) {
  mavlink_param_set_t param_set;
  mavlink_msg_param_set_decode(msg, &param_set);

  if (strncmp(param_set.param_id, "param1", sizeof(param_set.param_id)) == 0) {
    param1 = param_set.param_value;
    saveParameters();
    sendParamValue("param1", param1, 0);

  } else if (strncmp(param_set.param_id, "param2", sizeof(param_set.param_id)) == 0) {
    param2 = param_set.param_value;
    saveParameters();
    sendParamValue("param2", param2, 1);
  }
}

/*
  processParamRequestRead()
  Handles PARAM_REQUEST_READ messages by sending the requested parameter value.
*/
void processParamRequestRead(const mavlink_message_t* msg) {
  mavlink_param_request_read_t req;
  mavlink_msg_param_request_read_decode(msg, &req);

  // If a specific parameter ID is provided, return its value.
  if (strlen(req.param_id) > 0) {
    if (strncmp(req.param_id, "param1", sizeof(req.param_id)) == 0) {
      sendParamValue("param1", param1, 0);
      return;
    } else if (strncmp(req.param_id, "param2", sizeof(req.param_id)) == 0) {
      sendParamValue("param2", param2, 1);
      return;
    }
  } else {
    // Otherwise, use the param_index to determine which parameter to send.
    if (req.param_index == 0) {
      sendParamValue("param1", param1, 0);
      return;
    } else if (req.param_index == 1) {
      sendParamValue("param2", param2, 1);
      return;
    }
  }
}

/*
  processParamRequestList()
  Sends all parameters as a list.
*/
void processParamRequestList() {
  sendParamValue("param1", param1, 0);
  sendParamValue("param2", param2, 1);
}












void setup() {
  Serial.begin(115200);
  delay(100);

  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));

  pinMode(SPI_CS, OUTPUT);
  digitalWrite(SPI_CS, HIGH);

  pinMode(IO1, OUTPUT);
  digitalWrite(IO1, HIGH);
  pinMode(O4, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(O4), handleInterrupt, RISING);

  // --- Initialize TUSS4470 ---
  tuss4470Write(0x10, 0x00);             // BPF 40kHz
  tuss4470Write(0x16, 0x0F);             // Enable VDRV
  tuss4470Write(0x1A, 0x0F);             // 16 pulses
  tuss4470Write(0x17, THRESHOLD_VALUE);  // Threshold detect OUT4

  // --- ADC init ---
  adc_init();
  adc_gpio_init(analogIn);
  adc_select_input(0);
}






// -------------------- LOOP --------------------
void loop() {
  Sonar_Scan();
  delay(10);
}

void setup1() {}


void loop1() {
  FetchMavlinkSerial();
  unsigned long currentMillis1 = millis();
  if (currentMillis1 - previousMillis1 >= 1000) {
    previousMillis1 = currentMillis1;
    command_heartbeat();
  }
}
