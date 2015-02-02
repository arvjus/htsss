/**************************************************************************
Firmware for Arduino Uno compatible boards (ATMega328p)
This HW/SW is developed as a replacement for proprietary Hitec HTS-SS sensor station.

Controller measures altitude, current, voltage, temperature and sends to 
Hitec Optima 7/9 receiver.

Copyright © 2015 Arvid Juskaitis arvydas.juskaitis@gmail.com


Board - sensor wiring
=====================

Current ACS712ELC-30A - ADC
---------------------------
A0 - OUT

Voltage w/ divider 1/5.3917 - ADC
---------------------------------
A1 - OUT

Temperatrure LM335 - ADC
------------------------
A2 - OUT

Hitec Optima 7/9 receiver - I2C
-------------------------------
A4 - SDA
A5 - SCL

Barometer GY-63/MS561101BA - SPI
--------------------------------
IO11 - MOSI
IO12 - MISO
IO13 - SCK 

v0.1
Initial release. Supports I2C communication with Optima 7/9, can report 
voltage, current.
**************************************************************************/

#include <util/twi.h>
#include <util/delay.h>
#include <WProgram.h>

#define HITEC_I2C_ADDRESS   0x08    /* Optima 7/9 talks to this slave */
#define ADC_REFERENCE (1 << 6)      /* AVcc - REFS1=0, REFS0=1 */

#define ADC_I_IN    0  /* ADC channel for voltage */
#define ADC_U_IN    1  /* ADC channel for current */
#define ADC_T_IN    2  /* ADC channel for temperature */

#define MIN_CELL_VOLTAGE    32      /* Min voltage per cell * 10 */
#define CUT_OFF_VOLTAGE     35      /* Cut-off voltage per cell * 10 */
#define FULL_CHARGED_CELL   41      /* Max voltage per cell * 10 */

uint16_t zero_current_ref = 0;
uint16_t number_of_battery_cells = 1; /* Avoid div/0 */

/**************************************************************************
 * Message block, to send to Optima 7/9 by I2C
 **************************************************************************/
char data[8][7] = { 
  // 1 - Frametype, 4-5 - Internal SPC voltage
  { 0x11, 0xAF, 0x00, 0x2D, 0x00, 0x00, 0x11 }, 
  // 1-4 - Latitude, 5 - GPS sec
  { 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12 }, 
  // 1-4 - Longitude, 5 - TEMP2
  { 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13 }, 
  // 1-2 - Speed, 3-4 - Altitude, 5 - Temp1
  { 0x14, 0x01, 0x01, 0x01, 0x01, 0x01, 0x14 }, 
  // 1 - Fuelgauge, 2-3 - RPM1, 4-5 - RPM2
  { 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x15 }, 
  // GPS year, month, day, hour, min
  { 0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16 }, 
  // 3 - GPS signal, 4 - Temp3, 5 - Temp4
  { 0x17, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17 }, 
  // 1-2 - Voltage, 3-4 - Current
  { 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18 }
}; 

/**************************************************************************
 * Forward declarations
 **************************************************************************/
uint16_t read_adc(uint8_t ch);
void set_temp(uint8_t nr, uint8_t value);
void set_rpm(uint8_t nr, uint16_t value);
void set_speed(uint16_t value);
void set_altitude(uint16_t value);
void set_fuel_gauge(uint8_t value);
void set_voltage(uint16_t value);
void set_current(uint16_t value);


/**************************************************************************
 * Called once
 **************************************************************************/
void setup() {
  cli();

  /***
   * Initialise TWI (I2C bus) for SLA+R mode.
   */

  // Hitec i2c slave address, respond to general calls
  TWAR = (HITEC_I2C_ADDRESS << 1) | (1 << TWGCE);  

  // Enable TWI, ACK, interrupt; clear START, STOP condition bits
  TWCR = (1 << TWEN) | (1 << TWEA) | (1 << TWIE) | (0 << TWSTA) | (0 << TWSTO);
  TWDR = 0x00;


  /***
   * Init ADC
   */

  // Enable ADC, set prescale factor to 128, 16Mhz / 128 = 125Khz
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

  /***
   * Init SPI
   */


  sei();

  // Read reference current
  read_adc(ADC_I_IN);	// discard 1st result
  zero_current_ref = read_adc(ADC_I_IN);

  // Detect number of cells - 2/3/4s
  // see loop(), ADC conversion for ADC_U_IN
  uint8_t voltage = (uint8_t)(read_adc(ADC_U_IN) * 0.263266f);
  for (int i = 2; i < 5; i++) {
    if (voltage >= 2 * MIN_CELL_VOLTAGE) {
      number_of_battery_cells = i;
      break;
    }
  }

//  Serial.begin(115200);
}

/**************************************************************************
 * Perform slow operations here, anything else handled by ISRs
 **************************************************************************/
void loop() {
  uint16_t value;

  /***
   Measure Current. The ACS712ELC-30A gives 66mV/A. 
   At 0 (zero) amp this sensor gives Vcc/2. 
   Amp value is sent to Hitec in Amps * 10 format.
   (1/2^10)*3.3/0.066*10 = 0.48828f
   (1/2^10)*5.0/0.066*10 = 0.73982f
   */
  value = read_adc(ADC_I_IN);
  value = (value > zero_current_ref) ? value - zero_current_ref : 0;
  set_current((uint16_t)(value * 0.73982f));

  /***
   ADC conversion. ADC_U_IN input is divided by 5.3917
   Uin value is sent to Hitec in Uin * 10 format.
   A9 expects ADC_U_IN times 10. So
   (3.3*5.3917*10)/2^10 = 0.173755f
   (5.0*5.3917*10)/2^10 = 0.263266f
   */
  value = (uint8_t)(read_adc(ADC_U_IN) * 0.263266f);
  set_voltage(value);

  /*** 
   Calculate fuel gauge. 
   Set 0 if voltage is just about to drop to cut-off voltage 
   and 4 if battery is full charged.
   */
  uint8_t voltage_per_cell = value / number_of_battery_cells;
  set_fuel_gauge(3);



/*
  // Calculate temperature. LM335 is connected to PC3 by R/2R divider
  // LM335 gives 10mV/K so with R/2R divider and Uref = 3.3V: 
  // U(lm335) = 2 * (3.3/1024) * adc
  // Temp(kelvin) = U(lm335) / 10mV
  // Temp(Celcius) = Temp(kelvin) - 273.15
  // Temp(Celcium) = adc * 0.6445 - 271.15;
  value = read_adc(ADC_T_IN);
  set_temp(1, (uint8_t)((value * 0.6445f) - 273.15f));
  set_temp(2, (uint8_t)((value * 0.6445f) - 273.15f));
  set_temp(3, (uint8_t)((value * 0.6445f) - 273.15f));
  set_temp(4, (uint8_t)((value * 0.6445f) - 273.15f));
  
  set_speed(100);
  set_altitude(1000);
*/
  set_rpm(1, 500);
  set_rpm(2, 1500);

  delay(300);
}

/**************************************************************************
 * ISR for Slave Transmitter Mode
 **************************************************************************/
ISR(TWI_vect, ISR_BLOCK) {
  static unsigned char msg_row = 0;
  static unsigned char msg_col = 0;

  switch(TWSR & 0xF8)   // Mask prescaler bits
  {
  case 0xA8:	// SLA+R has been received, start transmitting message
  case 0xB0:    // Arbitration lost, start as slave
    msg_col = 0;
    TWDR = data[msg_row][msg_col++];
    TWCR |= (1 << TWINT) | (1 << TWEA);
    break;

  case 0xB8:    // Data byte has been transmitted, ACK received
    TWDR = data[msg_row][msg_col++];
    if (msg_col < 7) {
      TWCR |= (1 << TWINT) | (1 << TWEA);
    } 
    else {
      TWCR &=  ~(1 << TWEA);
      TWCR |= (1 << TWINT);
    }	
    break;

  case 0xC0:    // Data byte has been transmitted, NOT ACK received
  case 0xC8:    // Last byte has been transmitted, ACK received
    msg_row ++;
    msg_row %= 8;
    TWCR |= (1 << TWINT) | (1 << TWEA);
    break;

  case 0x00:    // Bus error (ilegal START/STOP condition)
    TWCR &= ~((1 << TWSTA) | (1 << TWEA));
    TWCR |= (1 << TWINT) | (1 << TWSTO);
    break;

  case 0xF8:    // No relevant state info available
  default:
    break;
  }
}

/**************************************************************************
 * Select channel, start ADC and return value
 **************************************************************************/
uint16_t read_adc(uint8_t ch) {
  uint16_t value;
  ADMUX = ADC_REFERENCE | (ch & 0x07);

  // start the conversion
  ADCSRA |= (1 << ADSC);

  // ADSC is cleared when the conversion finishes
  while ((ADCSRA & (1 << ADSC)))
    ;
  value = ADCL;
  value += (ADCH << 8);
  return value;
}

/**************************************************************************
 * Set temperature value in array
 **************************************************************************/
void set_temp(uint8_t nr, uint8_t value)
{
  uint8_t temp = value + 40;
  switch( nr )
  {
  case 1:
    data[3][5] = temp;
    break;
  case 2:
    data[2][5] = temp;
    break;
  case 3:
    data[6][4] = temp;
    break;
  case 4:
    data[6][5] = temp;
    break;
  default:
    break;
  }
}

/**************************************************************************
 * Set RPM value in array
 **************************************************************************/
void set_rpm(uint8_t nr, uint16_t value) {
  unsigned char rpml = (value/10) % 0x100;
  unsigned char rpmh = (value/10) / 0x100;
  switch( nr )
  {
  case 1:
    data[4][2] = rpml;
    data[4][3] = rpmh;
    break;
  case 2:
    data[4][4] = rpml;
    data[4][5] = rpmh;
    break;
  default:
    break;
  }
}

/**************************************************************************
 * Set speed value in array
 **************************************************************************/
void set_speed(uint16_t value) {
  data[3][1] = (uint8_t)(value & 0xFF);  // lsb	
  data[3][2] = (uint8_t)(value >> 8);    // msb
}

/**************************************************************************
 * Set altitude value in array
 **************************************************************************/
void set_altitude(uint16_t value) {
  data[3][3] = (uint8_t)(value & 0xFF);  // lsb	
  data[3][4] = (uint8_t)(value >> 8);    // msb
}

/**************************************************************************
 * Set fuel gauge value in array
 **************************************************************************/
void set_fuel_gauge(uint8_t value) {
  data[4][1] = value;
}

/**************************************************************************
 * Set voltage value in array
 **************************************************************************/
void set_voltage(uint16_t value)
{
  value -= 2;	// compensate .2v
  data[7][1] = (uint8_t)(value & 0xFF);  // lsb	
  data[7][2] = (uint8_t)(value >> 8);    // msb
}

/**************************************************************************
 * Set current value in array.
 **************************************************************************/
void set_current(uint16_t value)
{
  // calculate current in A9 units
  uint16_t val = ((value + 114.875) * 1.441);	
  data[7][3] = (uint8_t)(val & 0xFF);	// lsb
  data[7][4] = (uint8_t)(val >> 8);	// msb
}

/**************************************************************************
 * Substitute for Arduino main()
 **************************************************************************/
int main() {
  init_no_adc();
  setup();
  for(;;) loop();
  return 0;
}

