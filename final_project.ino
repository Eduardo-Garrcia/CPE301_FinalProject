#include <LiquidCrystal.h>
#include <DHT.h>
#include <Stepper.h>
#include <RTClib.h>

#define RDA 0x80
#define TBE 0x20

#define RLED 44
#define GLED 45
#define BLED 46
#define YLED 47

#define DHT11_PIN 49 
#define DHTTYPE DHT11
#define waterSensor A0
#define fanPin 2
#define ventPin A1

#define strtButton 50 
#define RstButton 51
#define stpButton 52

    // LED State
enum States {
  DISABLED = 1,
  IDLE = 2,
  ERROR = 3,
  RUNNING = 4
};

States currentState = DISABLED;

bool fanRunning = false;
bool systemEnabled = false;

  // Water Humidity and Temperature/Steps
const int LOW_WATER = 300;   
const int LOW_TEMP = 25;    // low temperature threshold
const int HIGH_TEMP = 30;   // high temperature threshold
const int MAX_STEPS = 200; 

  //Buttons
  bool startButton = false;
  bool resetButton = false;
  bool stopButton = false;

  //global ticks counter
int currentTicks = 0;
int timer_running = 0;

  // Setup LCD
LiquidCrystal lcd(35, 34, 33, 32, 31, 30);

  // Setup DHT
DHT dht(DHT11_PIN, DHT11);

RTC_DS1307 rtc;

const long interval = 1000;  // interval at which to blink (milliseconds)

// Setup Stepper
Stepper stepperMot(2048, 9, 10, 11, 12);

  // Water Level
  int waterLevel = 0;
  // Fan
const int speedPin = 7;
const int dir1 = 6;
const int dir2 = 5;
const int motorSpeed = 255;

bool displayTemperatureHumidity = false;
bool stepperOn = false;
bool monitorWater = false;
bool fanOn = false;
int led1 = -1;

  // UART Pointers
volatile unsigned char *myUCSR0A  = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B  = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C  = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0   = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0    = (unsigned char *) 0x00C6;

  // GPIO Pointers
volatile unsigned char *portA     = (unsigned char *) 0x22; 
volatile unsigned char *portDDRA  = (unsigned char *) 0x21;
volatile unsigned char *pinA      = (unsigned char *) 0x20;

volatile unsigned char *portB     = (unsigned char *) 0x25;
volatile unsigned char *portDDRB  = (unsigned char *) 0x24;
volatile unsigned char *pinB      = (unsigned char *) 0x23;

volatile unsigned char *portC     = (unsigned char *) 0x28;
volatile unsigned char *portDDRC  = (unsigned char *) 0x27;
volatile unsigned char *pinC      = (unsigned char *) 0x26;

volatile unsigned char *portD     = (unsigned char *) 0x2B;
volatile unsigned char *portDDRD  = (unsigned char *) 0x2A;
volatile unsigned char *pinD      = (unsigned char *) 0x29;

volatile unsigned char *portE     = (unsigned char *) 0x2E;
volatile unsigned char *portDDRE  = (unsigned char *) 0x2D;
volatile unsigned char *pinE      = (unsigned char *) 0x2C;

  // ADC Pointers
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

  // ISR Pointers
volatile unsigned char *myTCCR1A = (unsigned char *) 0x80;
volatile unsigned char *myTCCR1B = (unsigned char *) 0x81;
volatile unsigned char *myTCCR1C = (unsigned char *) 0x82;
volatile unsigned char *myTIMSK1 = (unsigned char *) 0x6F;
volatile unsigned char *myTIFR1   = (unsigned char *) 0x36;
volatile unsigned int  *myTCNT1   = (unsigned  int *) 0x84; 

void setup() {
    // Start the UART
  U0Init(9600);
    // setup the ADC
  adc_init();
    // Initialize LCD
  lcd.begin(16, 2);
    // Initialize DHT sensor
  dht.begin();
    // Initialize RTC
  rtc.begin();
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // Stepper Motor
  stepperMot.setSpeed(50);

  currentState = IDLE;
}

void loop() {
  switch (currentState) {
    case DISABLED:
      fanOn = false;
      displayTemperatureHumidity = false;
      stepperOn = true;
      monitorWater = false;
      led1 = 3;
      
      break;

    case IDLE:
      fanOn = false;
      displayTemperatureHumidity = true;
      stepperOn = true;
      monitorWater = true;
      led1 = 2;

      break;

    case ERROR:
    lcd.clear();
    lcd.print("Error: Low Water Levels!");
  
      fanOn = false;
      displayTemperatureHumidity = true;
      stepperOn = false;
      monitorWater = true;
      led1 = 0;

      break;

    case RUNNING:
      fanOn = true;
      displayTemperatureHumidity = true;
      stepperOn = true;
      monitorWater = true;
      led1 = 1;
      break;
  }

    waterLevel = adc_read(0);

    // Temperature and Humidity
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();

    // ADC
    unsigned char adc_channel_num = 0;

    // Display Temperature and Humidity on LCD
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(temperature);
    lcd.print(" C");

    lcd.setCursor(0, 1);
    lcd.print("Humidity: ");
    lcd.print(humidity);
    lcd.print("%");

    Serial.print("Humidity: ");
    Serial.print(humidity);
}

/*
ADC Functions
*/
void adc_init()
{
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}

unsigned int adc_read(unsigned char adc_channel_num)
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}

/*
UART Functions
*/
void U0Init(int U0baud)
{
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 // Same as (FCPU / (16 * U0baud)) - 1;
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}

unsigned char kbhit()
{
  return *myUCSR0A & RDA;
}

unsigned char getChar()
{
  return *myUDR0;
}

void putChar(unsigned char U0pdata)
{
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}

/*
ISR Functions
*/
// Timer setup function
void setup_timer_regs()
{
  // setup the timer control registers
  *myTCCR1A= 0x00;
  *myTCCR1B= 0X00;
  *myTCCR1C= 0x00;
  
  // reset the TOV flag
  *myTIFR1 |= 0x01;
  
  // enable the TOV interrupt
  *myTIMSK1 |= 0x01;
}


// TIMER OVERFLOW ISR
ISR(TIMER1_OVF_vect)
{
  // Stop the Timer
  *myTCCR1B &= 0xF8;
  // Load the Count
  *myTCNT1 =  (unsigned int) (65535 -  (unsigned long) (currentTicks));
  // Start the Timer
  *myTCCR1B |=   0x05;
  // if it's not the STOP amount
  if(currentTicks != 65535)
  {
    // XOR to toggle PB6
    *portB ^= 0x40;
  }
}
