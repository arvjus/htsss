/**************************************************************************
htsss v0.1

Firmware for Arduino Mini Pro compatible boards (ATMega328p 8MHz)
This HW/SW is developed as a replacement for proprietary Hitec HTS-SS sensor station.

Controller measures altitude, current, voltage, temperature and sends to 
Hitec Optima 7/9 receiver.

Copyright © 2015 Arvid Juskaitis arvydas.juskaitis@gmail.com


Board - sensor wiring
=====================

Hitec Optima 7/9 receiver - I2C
-------------------------------
SDA - A4
SCL - A5

Current ACS712ELC-30A - ADC
---------------------------
OUT - A0      // PORTC 0bit

Voltage w/ divider 1/5.3917 - ADC
---------------------------------
OUT - A1      // PORTC 1bit

Temperatrure LM335 - ADC
------------------------
OUT - A2      // PORTC 2bit

Barometer GY-63/MS561101BA - SPI
--------------------------------
CSB/SS - D10  // PORTB 2bit
MOSI   - D11  // PORTB 3bit
MISO   - D12  // PORTB 4bit
SCK    - D13  // PORTB 5bit

v0.1 - 2015-02-03
Initial release. Supports I2C communication with Optima 7/9, can report 
voltage, current, fuel gauge.
**************************************************************************/

#include <util/twi.h>
#include <util/delay.h>
#include <WProgram.h>

#define HITEC_I2C_ADDRESS   0x08    /* Optima 7/9 talks to this slave */
#define ADC_REFERENCE (1 << 6)      /* AVcc - REFS1=0, REFS0=1 */

#define ADC_I_IN    0               /* ADC channel for voltage */
#define ADC_U_IN    1               /* ADC channel for current */
#define ADC_T_IN    2               /* ADC channel for temperature */

#define MIN_CELL_VOLTAGE    32      /* Min voltage per cell * 10 */
#define CUT_OFF_VOLTAGE     33      /* Cut-off voltage per cell * 10 */
#define FULL_CHARGED_CELL   41      /* Max voltage per cell * 10 */

/*
 ADC conversion, 10bit resolution. ACS712ELC-30A gives 66mV/A. 
 (1/(2^10))*3.3/0.066 = 0.048828f
 (1/(2^10))*5.0/0.066 = 0.073982f
 */
#define ADC_I_IN_FACTOR     ((1/(2^10))*5.0/0.066)

/*
 ADC conversion, 10bit resolution. ADC_U_IN input is divided by 5.3917
 (3.3*5.3917)/(2^10) = 0.0173755f
 (5.0*5.3917)/(2^10) = 0.0263266f
 */
#define ADC_U_IN_FACTOR     ((3.3*5.3917)/(2^10))

/*
 GY-63/MS561101BA definitions for SPI
 */
#define MS5611_RESET       0x1E // ADC reset command
#define MS5611_ADC_READ    0x00 // ADC read command
#define MS5611_ADC_CONV    0x40 // ADC conversion command
#define MS5611_ADC_D1      0x00 // ADC D1 conversion 
#define MS5611_ADC_D2      0x10 // ADC D2 conversion
#define MS5611_ADC_256     0x00 // ADC OSR=256
#define MS5611_ADC_512     0x02 // ADC OSR=512
#define MS5611_ADC_1024    0x04 // ADC OSR=1024 
#define MS5611_ADC_2048    0x06 // ADC OSR=2056
#define MS5611_ADC_4096    0x08 // ADC OSR=4096
#define MS5611_PROM_READ   0xA0 // Prom read command
  
#define ms5611_csb_lo() (_SFR_BYTE(PORTB) &= ~_BV(2))  // setting CSB low
#define ms5611_csb_hi() (_SFR_BYTE(PORTB) |=  _BV(2))  // setting CSB high


uint16_t zero_current_ref = 0;
uint16_t number_of_battery_cells = 1; /* Avoid div/0 */
uint16_t ground_altitude = 0;

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
 * MS561101BA calibration coefficients
 **************************************************************************/
uint16_t C[7];


/**************************************************************************
 * Forward declarations
 **************************************************************************/
uint16_t read_adc(uint8_t ch);
void set_current(uint16_t value);
void set_voltage(uint16_t value);
void set_fuel_gauge(uint8_t value);
void set_temp(uint8_t nr, uint8_t value);
void set_altitude(uint16_t value);
void set_speed(uint16_t value);
void set_rpm(uint8_t nr, uint16_t value);
void reset_ms5611();
uint16_t read_ms5611_prom(uint8_t coef_num);
uint32_t read_ms5611(uint8_t cmd);
uint32_t calculate_altitude();


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

  // Enable ADC, set prescale factor to 64, 8Mhz / 64 = 125Khz
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);


  /***
   * Init SPI
   */
   
  // SPI settings: master, mode 0, fosc/4
  SPCR = (1 << SPE) | (1 << MSTR);
  
  // Define CSK, MOSI and CSB pins as output, MISO as input
  DDRB = (1 << 5) | (1 << 3) | (1 << 2);
  
  // Pull SCK and MOSI low, SS high.
  PORTB = (1 << 2);
  
  sei();

  // Read reference current
  read_adc(ADC_I_IN);	// discard 1st result
  zero_current_ref = read_adc(ADC_I_IN);

  // Detect number of cells - 2/3/4s
  // see loop(), ADC conversion for ADC_U_IN
  uint8_t voltage = (uint8_t)(read_adc(ADC_U_IN) * ADC_U_IN_FACTOR * 10);
  for (int i = 5; i > 0; i--) {
    if (voltage >= i * MIN_CELL_VOLTAGE) {
      number_of_battery_cells = i;
      break;
    }
  }

  // Reset SPI, read calibration coeficients
  reset_ms5611();
  for (int i = 0; i < 8; i++)
    C[i] = read_ms5611_prom(i);
  
  // Read initial altitude
  ground_altitude = calculate_altitude();


  Serial.begin(115200);
}

/**************************************************************************
 * Perform slow operations here, anything else handled by ISRs
 **************************************************************************/
void loop() {
  uint16_t value; // Temprary values

  /***
   Measure Current. At 0 (zero) amp this sensor gives Vcc/2. 
   Amp value is sent to Hitec in Amps * 10 format.
   */
  value = read_adc(ADC_I_IN);
  value = (value > zero_current_ref) ? value - zero_current_ref : 0;
  set_current((uint16_t)(value * ADC_I_IN_FACTOR * 10));

  /***
   Measure voltage. 
   Uin value is sent to Hitec in Uin * 10 format.
   */
  value = (uint8_t)(read_adc(ADC_U_IN) * ADC_U_IN_FACTOR * 10);
  set_voltage(value);

  /*** 
   Calculate fuel gauge. There are 4 segmets to represent value.
   Set 0 if voltage is just about to drop to cut-off voltage 
   and 4 if battery is full charged.
   */
  float voltage_per_cell = value / number_of_battery_cells;
  int8_t fuel_gauge = ((voltage_per_cell - CUT_OFF_VOLTAGE) / 
      (FULL_CHARGED_CELL - CUT_OFF_VOLTAGE) * 4);
  if (fuel_gauge < 0) fuel_gauge = 0;
  if (fuel_gauge > 4) fuel_gauge = 4;
  set_fuel_gauge((uint8_t)fuel_gauge);

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

  /***
   Get altitude
   */
   value = calculate_altitude();
   value = (value > ground_altitude) ? value - ground_altitude : 0;
   set_altitude(value);


  set_rpm(1, 500);
  set_rpm(2, 1500);

  _delay_ms(300);
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
 * Send 8 bit using SPI
 **************************************************************************/
void spi_send(uint8_t cmd) {
  // put the byte in the SPI hardware buffer and start sending 
  SPDR = cmd;       
  
  // wait that the data is sent
  while (!(SPSR & (1 << SPIF)))
    ;
}

/**************************************************************************
 * Reset MS561101BA
 **************************************************************************/
void reset_ms5611() {
  ms5611_csb_lo();
  spi_send(MS5611_RESET);
  _delay_ms(3);
  ms5611_csb_hi();
}

/**************************************************************************
 * Read MS561101BA PROM
 **************************************************************************/
uint16_t read_ms5611_prom(uint8_t coef_num) {
  uint16_t ret;
  ms5611_csb_lo();
  spi_send(MS5611_PROM_READ - coef_num * 2);  // Send PROM READ command
  spi_send(0x00);                             // Send 0 to read MSB
  ret = (SPDR << 8);
  spi_send(0x00);                             // Send 0 to read LSB
  ret += SPDR;
  ms5611_csb_hi();
  return ret;
}

/**************************************************************************
 * Perform MS561101BA ADC conversion
 **************************************************************************/
uint32_t read_ms5611(uint8_t cmd) {
  uint32_t ret = 0;

  // send conversion command
  ms5611_csb_lo();
  spi_send(MS5611_ADC_CONV + cmd);    
  _delay_ms(10);
  ms5611_csb_hi();
  
  // start reading
  ms5611_csb_lo();   
  spi_send(MS5611_ADC_READ);
  spi_send(0x00);
  ret = (SPDR << 16);
  spi_send(0x00);
  ret += (SPDR << 8);
  spi_send(0x00);
  ret += SPDR;
  ms5611_csb_hi();   

  return ret;
}


/**************************************************************************
 * Read pressure, temperature, calculate altitude
 **************************************************************************/
uint32_t calculate_altitude() {
  const float sea_press = 1013.25;
  uint32_t D1;    // ADC value of the pressure conversion
  uint32_t D2;    // ADC value of the temperature conversion
  double P;       // difference between actual and measured temperature
  double T;       // compensated temperature value
  double dT;      // difference between actual and measured temperature
  double OFF;     // offset at actual temperature
  double SENS;    // sensitivity at actual temperature

  D1 = read_ms5611(MS5611_ADC_D1 + MS5611_ADC_4096);  // read uncompensated pressure
  D2 = read_ms5611(MS5611_ADC_D2 + MS5611_ADC_4096);  // read uncompensated temperature

  // Apply MS5611 1st order algorithm
  dT   = D2 - C[5] * pow(2,8);
  OFF  = C[2] * pow(2,17) + dT * C[4] / pow(2,6);
  SENS = C[1] * pow(2,16) + dT * C[3] / pow(2,7);
  T    = (2000 + (dT * C[6]) / pow(2,23)) / 100;
  P    = (((D1 * SENS) / pow(2,21) - OFF) / pow(2,15)) / 100;

  // Calculate altitude
  // return (1.0f - pow(P/101325.0f, 0.190295f)) * 4433000.0f;
  return ((pow((sea_press / P), 1/5.257) - 1.0) * (T + 273.15)) / 0.0065;
}


/**************************************************************************
 * Set voltage value in array
 **************************************************************************/
void set_voltage(uint16_t value)
{
Serial.print(value);
Serial.print("v, ");

  value -= 2;	// compensate .2v
  data[7][1] = (uint8_t)(value & 0xFF);  // lsb	
  data[7][2] = (uint8_t)(value >> 8);    // msb
}

/**************************************************************************
 * Set current value in array.
 **************************************************************************/
void set_current(uint16_t value)
{
Serial.print(value);
Serial.print("a, ");

  // calculate current in A9 units
  uint16_t val = ((value + 114.875) * 1.441);	
  data[7][3] = (uint8_t)(val & 0xFF);	  // lsb
  data[7][4] = (uint8_t)(val >> 8);     // msb
}

/**************************************************************************
 * Set fuel gauge value in array
 **************************************************************************/
void set_fuel_gauge(uint8_t value) {
Serial.print(number_of_battery_cells);
Serial.print("s, gauge:");
Serial.print((uint16_t)value);

  data[4][1] = value;
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
 * Set altitude value in array
 **************************************************************************/
void set_altitude(uint16_t value) {
  Serial.print(", altitude:");
  Serial.print(value);

  data[3][3] = (uint8_t)(value & 0xFF);  // lsb	
  data[3][4] = (uint8_t)(value >> 8);    // msb
}

/**************************************************************************
 * Set speed value in array
 **************************************************************************/
void set_speed(uint16_t value) {
  data[3][1] = (uint8_t)(value & 0xFF);  // lsb	
  data[3][2] = (uint8_t)(value >> 8);    // msb
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
 * Substitute for Arduino main()
 **************************************************************************/
int main() {
  init_no_adc();
  setup();
  for(;;) loop();
  return 0;
}
