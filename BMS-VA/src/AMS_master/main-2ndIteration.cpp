#include <FlexCAN_T4.h>
#include <Arduino.h>
#include <stdint.h>
#include <SPI.h>
#include "Linduino.h"
#include "LT_SPI.h"
#include "UserInterface.h"
#include "LTC681x.h"
#include "LTC6813.h"


FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
CAN_message_t msg;
CAN_message_t receivedMsg;
char buf[40];
String checks;
int8_t  dataTherm[8];
int value1, adcValue;
unsigned long value3;
uint32_t lastTransmissionTime = 0;
unsigned long highStartTime = 0;
bool isHigh = false; 
#define ENABLED 1
#define DISABLED 0
#define DATALOG_ENABLED 1
#define DATALOG_DISABLED 0
#define PWM 1
#define SCTL 2
int8_t select_s_pin(void);
char read_hex(void); 
char get_char(void);
const uint8_t TOTAL_IC = 4;//!< Number of ICs in the daisy chain
const uint8_t ADC_OPT = ADC_OPT_DISABLED; //!< ADC Mode option bit
const uint8_t ADC_CONVERSION_MODE =MD_7KHZ_3KHZ; //!< ADC Mode
const uint8_t ADC_DCP = DCP_DISABLED; //!< Discharge Permitted 
const uint8_t CELL_CH_TO_CONVERT =CELL_CH_ALL; //!< Channel Selection for ADC conversion
const uint8_t AUX_CH_TO_CONVERT = AUX_CH_ALL; //!< Channel Selection for ADC conversion
const uint8_t STAT_CH_TO_CONVERT = STAT_CH_ALL; //!< Channel Selection for ADC conversion
const uint8_t SEL_ALL_REG = REG_ALL; //!< Register Selection 
const uint8_t SEL_REG_A = REG_1; //!< Register Selection 
const uint8_t SEL_REG_B = REG_2; //!< Register Selection 
const uint16_t MEASUREMENT_LOOP_TIME = 500; //!< Loop Time in milliseconds(ms)

//Under Voltage and Over Voltage Thresholds
const uint16_t OV_THRESHOLD = 42000; //!< Over voltage threshold ADC Code. LSB = 0.0001 ---(4.1V)
const uint16_t UV_THRESHOLD = 25000; //!< Under voltage threshold ADC Code. LSB = 0.0001 ---(3V)

//Loop Measurement Setup. These Variables are ENABLED or DISABLED. Remember ALL CAPS
const uint8_t WRITE_CONFIG = DISABLED;  //!< This is to ENABLED or DISABLED writing into to configuration registers in a continuous loop
const uint8_t READ_CONFIG = DISABLED; //!< This is to ENABLED or DISABLED reading the configuration registers in a continuous loop
const uint8_t MEASURE_CELL = ENABLED; //!< This is to ENABLED or DISABLED measuring the cell voltages in a continuous loop
const uint8_t MEASURE_AUX = DISABLED; //!< This is to ENABLED or DISABLED reading the auxiliary registers in a continuous loop
const uint8_t MEASURE_STAT = DISABLED; //!< This is to ENABLED or DISABLED reading the status registers in a continuous loop
const uint8_t PRINT_PEC = DISABLED; //!< This is to ENABLED or DISABLED printing the PEC Error Count in a continuous loop
cell_asic BMS_IC[TOTAL_IC]; //!< Global Battery Variable
bool REFON = true; //!< Reference Powered Up Bit
bool ADCOPT = false; //!< ADC Mode option bit
bool GPIOBITS_A[5] = {false,false,true,true,true}; //!< GPIO Pin Control // Gpio 1,2,3,4,5
bool GPIOBITS_B[4] = {false,false,false,false}; //!< GPIO Pin Control // Gpio 6,7,8,9
uint16_t UV=UV_THRESHOLD; //!< Under voltage Comparison Voltage
uint16_t OV=OV_THRESHOLD; //!< Over voltage Comparison Voltage
bool DCCBITS_A[12] = {false,false,false,false,false,false,false,false,false,false,false,false}; //!< Discharge cell switch //Dcc 1,2,3,4,5,6,7,8,9,10,11,12
bool DCCBITS_B[7]= {false,false,false,false,false,false,false}; //!< Discharge cell switch //Dcc 0,13,14,15
bool DCTOBITS[4] = {true,false,true,false}; //!< Discharge time value //Dcto 0,1,2,3  // Programed for 4 min 
/*Ensure that Dcto bits are set according to the required discharge time. Refer to the data sheet */
bool FDRF = false; //!< Force Digital Redundancy Failure Bit
bool DTMEN = true; //!< Enable Discharge Timer Monitor
bool PSBITS[2]= {false,false}; //!< Digital Redundancy Path Selection//ps-0,1
const int LED_PIN = 13; // Change this to the desired LED pin on the Teensy board

const int OUTPUT_PIN = 17;


void setup() {
  can1.begin();
  can1.setBaudRate(500000);
  can1.setMaxMB(16);
  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  can1.onReceive(canSniff);
  can1.mailboxStatus();
  can2.begin();
  can2.setBaudRate(500000);
  Serial.begin(115200);
  quikeval_SPI_connect();
  spi_enable(SPI_CLOCK_DIV16); // This will set the Linduino to have a 1MHz Clock
  LTC6813_init_cfg(TOTAL_IC, BMS_IC);
  LTC6813_init_cfgb(TOTAL_IC,BMS_IC);
  for (uint8_t current_ic = 0; current_ic<TOTAL_IC;current_ic++) 
  {
    LTC6813_set_cfgr(current_ic,BMS_IC,REFON,ADCOPT,GPIOBITS_A,DCCBITS_A, DCTOBITS, UV, OV);
    LTC6813_set_cfgrb(current_ic,BMS_IC,FDRF,DTMEN,PSBITS,GPIOBITS_B,DCCBITS_B);   
  }   
  LTC6813_reset_crc_count(TOTAL_IC,BMS_IC);
  LTC6813_init_reg_limits(TOTAL_IC,BMS_IC);
  wakeup_sleep(TOTAL_IC);
}


void loop() {
  
  int8_t error = 0;
  can1.events();
  static uint32_t timeout = millis();
  wakeup_sleep(TOTAL_IC);
  error = LTC6813_rdcv(SEL_ALL_REG, TOTAL_IC, BMS_IC);
  LTC6813_wrcfg(TOTAL_IC,BMS_IC); // Write into Configuration Register
  LTC6813_wrcfgb(TOTAL_IC,BMS_IC); // Write into Configuration Register B
  LTC6813_adcv(ADC_CONVERSION_MODE,ADC_DCP,CELL_CH_TO_CONVERT);
  check_error(error); 
  print_cells(DATALOG_DISABLED);
  wakeup_idle(TOTAL_IC);
  LTC6813_adstat(ADC_CONVERSION_MODE, STAT_CH_TO_CONVERT);
  error = LTC6813_rdstat(SEL_REG_A,TOTAL_IC,BMS_IC); // Set to read back stat register A
  check_error(error);
  print_SUM_OF_CELLS(DATALOG_DISABLED);
  if (millis() - timeout > 2000) {
    sendtherm();
    timeout = millis();
  }
}

void canSniff(const CAN_message_t &msg)  {
  value3 = msg.id;
  if (msg.id == 0x1838F380 || msg.id == 0x1838F381) {
    Serial.print("CAN ID:  ");
    Serial.print(msg.id, HEX); 
    Serial.print(" ");
    for (int k = 0; k < 8; k++) {
      dataTherm[k] = (int8_t)msg.buf[k];
      Serial.print(dataTherm[k]);
      Serial.print(" ");
    }
    Serial.println();
    if (dataTherm[2] > 60) {
      if (!isHigh) {
        highStartTime = millis();
      } else {
        if (millis() - highStartTime > 1000) {
          digitalWrite(OUTPUT_PIN, HIGH);
        }
      }
  }
}
}

void sendtherm() {
  CAN_message_t msg;
  msg.id = 0x100;
  msg.len = 2;
  msg.buf[0] = dataTherm[4];
  msg.buf[1] = dataTherm[5];
  can2.write(msg);
}

void print_cells(uint8_t datalog_en) {
 for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    if (datalog_en == 0)
    {
      Serial.print(" IC ");
      Serial.print(current_ic+1, DEC);
      Serial.print(", ");
      for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++)
      {
        Serial.print(" C");
        Serial.print(i+1, DEC);
        Serial.print(":");
        Serial.print(BMS_IC[current_ic].cells.c_codes[i] * 0.0001, 4);
        Serial.print(",");
      }
      Serial.println();
    }
  }
  Serial.println("\n");
}


void print_SUM_OF_CELLS(uint8_t datalog_en) {
     float total_voltage = 0.0;
    
    for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
    {
      if (datalog_en == 0) {
        Serial.print(F(" IC "));
        Serial.print(current_ic + 1, DEC);
        Serial.print(F(" Total Voltage in Each Segment :"));
        float ic_summed_voltage = BMS_IC[current_ic].stat.stat_codes[0] * 0.0001 * 30;
        Serial.print(ic_summed_voltage, 2);
        Serial.print(F(","));
        
        total_voltage += ic_summed_voltage;
      }
    }
    Serial.println("\nTotal voltage of the TSAC: ");
    Serial.println(total_voltage, 3);
}

void latch_pcb(uint8_t datalog_en) {
  bool threscrossed = false;
  static bool flag = false;
  static unsigned long timeitcrossedthres = 0;

  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++) {
      uint16_t cell_voltage = BMS_IC[current_ic].cells.c_codes[i];
      
      if (cell_voltage < UV_THRESHOLD) {
        threscrossed = true;
        Serial.print("Under voltage detected in IC ");
        Serial.print(current_ic + 1);
        Serial.print(", Cell ");
        Serial.println(i + 1);
      }
      
      if (cell_voltage > OV_THRESHOLD) {
        threscrossed = true;
        Serial.print("Over voltage detected in IC ");
        Serial.print(current_ic + 1);
        Serial.print(", Cell ");
        Serial.println(i + 1);
      }
    }
  }
  
  if (threscrossed) {
    if (!flag) {
      timeitcrossedthres = millis();
      flag = true;
    }
    if (millis() - timeitcrossedthres >= 500) {
      digitalWrite(OUTPUT_PIN, HIGH);
    }
  } else {
    flag = false;
    digitalWrite(OUTPUT_PIN, LOW);
  }
}

void check_error(int error) {
   if (error == -1)
  {
    Serial.println(F("A PEC error was detected in the received data"));
  }
}

