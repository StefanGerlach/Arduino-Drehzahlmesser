#include <PinChangeInterrupt.h>
#include <PinChangeInterruptPins.h>

#include <MsTimer2.h>

#include <Adafruit_GFX.h>
#include <Adafruit_TFTLCD.h>

#define  SPLASH


// The control pins for the LCD can be assigned to any digital or
// analog pins...but we'll use the analog pins as this allows us to
// double up the pins with the touch screen (see the TFT paint example).
#define LCD_CS A3 // Chip Select goes to  Analog 3
#define LCD_CD A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to    Analog 1
#define LCD_RD A0 // LCD Read goes to     Analog 0
#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin

// #define SIGNAL_INTERRUPT 0 // Define our new Interrupt (SW) Pin

// For the Arduino Uno, Duemilanove, Diecimila, etc.:
//   D0 connects to digital pin 8  (Notice these are
//   D1 connects to digital pin 9   NOT in order!)
//   D2 connects to digital pin 2
//   D3 connects to digital pin 3
//   D4 connects to digital pin 4
//   D5 connects to digital pin 5
//   D6 connects to digital pin 6
//   D7 connects to digital pin 7

// Assign human-readable names to some common 16-bit color values:
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

// Display Handle
Adafruit_TFTLCD lcd(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);


// Initialize global variables

// Timer Config for Time Measurements
// For 16MHz:
// Prescale  Time per counter tick  Max Period
// 1          0.0625 uS               8.192 mS
// 8          0.5 uS                  65.536 mS
// 64         4 uS                    524.288 mS
// 256        16 uS                   2097.152 mS
// 1024       64uS                    8388.608mS

const unsigned long _timer_period_microseconds = 524288; // this is calculated for 16 MHz and Prescale 64
const byte _timer_us_per_tick = 4;

// For this use case we have a Ignition Impulse Signal from Georges Quad - that indicates a iginition in the engine. 
// The RPM ranges from 1200 to 10000 and leads to a time gap between the ignitions (1 Cylinder, 4 Cycles)
// 100 ms to 12 ms (1200 to 10000 RPM)

// Simple Calculation: 
// if we drive with 1200 RPM, this means 20 rounds per second (20 Hz) - and 1 round in 50 ms,
// and 600 ignitions per minute - 10 ignitions per second (10 Hz)     - and 1 ignition in 100 ms

// if we have the time between the ignitions, we can simply calculate 
// RPM = ( 1 / ((t_in_µs / 1000) / 1000) * 60         ) * 2
//               measure   to ms   to s    to minute    ignition to rpm
//
// RPM = ( 1.000.000 / t_in_µs ) * 60 * 2 
// RPM = ( 60.000.000 / t_in_µs ) * 2 

// constant - will not be changed
const float _time_gap_divident = 60000000; 

// this should be variable to adjust other timings of the engine
volatile float _ignition_multiplier = 2.0; 

// this is the global volatile variable for the RPM
volatile unsigned short _current_RPM = 0;

volatile unsigned short _current_color = 0xFFFF;

const unsigned short _max_RPM = 7700;
const unsigned short _min_RPM = 1000;


// Configuration of Operational Modes
volatile bool _simulation_mode = 0;

volatile float _timer2_simulator_ms_delay = 57;
volatile bool _go_up = 1;
volatile int _wait = 10;



// The Error Status of the System
volatile byte error_code = 0x00;



// DEBUG VARIABLES
volatile unsigned long _current_ms = 0;


void setup() 
{
  // Initialize Display and show nice Screen
  lcd.reset();
  lcd.begin(0x9341); // SDFP5408
  lcd.setRotation(1); 

#ifdef SPLASH
  showNiceBootScreen_LetsMud();
#endif
  
  initScreenFonts();

  // Simulate Interrupt with Timer2
  if(_simulation_mode)
  {
    MsTimer2::set((unsigned long)_timer2_simulator_ms_delay, timer2_simulator); // 500ms period
    MsTimer2::start();
  }
  
  // start Timer1 for measurement
  setup_timer1_registers();

  // init interrupts on A5
  // setup_port_c_a5_interrupt();
  int SIGNAL_PIN = A5;
  
  pinMode(SIGNAL_PIN, LOW);
  attachPCINT(digitalPinToPCINT(SIGNAL_PIN), test, RISING);
 
  // clear errors
  clearErrors();
}

void test()
{  
  signal_interrupt_handler();
} 


void initScreenFonts()
{
  lcd.fillScreen(BLACK);
  lcd.setTextColor(WHITE);    
  lcd.setRotation(1);
}

void setup_port_c_a5_interrupt()
{
  pinMode(A5, INPUT);
  digitalWrite(A5, HIGH); // pull down resistor
  
  cli();
  // Setzen des PCIE1-Bit im Pin Change Interrupt Control Register (PCICR)
  PCICR  = (1 << PCIE1);
  PCMSK1 = (1 << PCINT13); // A5
  
  sei();
}

void setup_timer1_registers()
{
    noInterrupts();
  
    TCCR1A = 0;  // Timer/Counter1 Control Register A
    TCCR1B = 0;  // Timer/Counter1 Control Register B
    TCNT1  = 0;       // High and Low for 16 bit
    OCR1A  = 0;  // Compare Register Value (High and Low for 16 bit)
    
    TCCR1B |= (1 << WGM12);   // Timer/Counter1 Control Register A - CTC mode 
    TCCR1B |= (1 << CS11);    // Timer/Counter1 Control Register A - CS11 and CS10 for 64 Prescaler!
    TCCR1B |= (1 << CS10);    // Timer/Counter1 Control Register A - CS11 and CS10 for 64 Prescaler!

    TIMSK1 = 0;
    TIMSK1 |= (1 << TOIE1);   // Timer/Counter1 Interrupt Mask Register - Overflow Interrupt Enable

    interrupts();
}

ISR(TIMER1_OVF_vect) 
{
  // if we have an overflow, we can be sure that the current_rpm is zero
  _current_RPM = 0;
}


// Interrupt Service Routine for Port C
ISR (PCINT1_vect)
{  
  signal_interrupt_handler();
} 

void signal_interrupt_handler()
{
    unsigned long _current_timer_us = calc_ticks_to_microsecs(read_reset_timer1());

    _current_RPM = (unsigned long) calc_microsec_to_upm(_current_timer_us);
    _current_ms  = _current_timer_us / 1000;
}



void timer2_simulator()
{
  signal_interrupt_handler();   

  unsigned long _min = 12;
  unsigned long _max = 100;

  if(_go_up && (_timer2_simulator_ms_delay >= _max))
    _go_up = 0;

  if(!_go_up && (_timer2_simulator_ms_delay <= _min ))
    _go_up = 1;
    
  if(_wait <= 0)
  {
     _go_up ? _timer2_simulator_ms_delay++ : _timer2_simulator_ms_delay--;
    _wait = ((_max - _timer2_simulator_ms_delay) * (_max - _timer2_simulator_ms_delay)) / 2500;
  }
  else
  {
    _wait--;      
  }

  MsTimer2::stop();
  MsTimer2::set((unsigned long) _timer2_simulator_ms_delay, timer2_simulator);
  MsTimer2::start();

}


void loop() 
{
  visualize();
  
  if(error_code != 0x00)
    drawErrors();
}


unsigned long read_reset_timer1()
{
  unsigned short ticks_of_timer1 = TCNT1;
  
  TCNT1 = 0; 
  
  return (unsigned long) ticks_of_timer1;
}


unsigned long calc_ticks_to_microsecs(unsigned long ticks)
{
  return ticks * _timer_us_per_tick; 
}


unsigned short round_to_hundrets(unsigned short value)
{
  unsigned short add = 0;
  if(value/10 % 10 >= 5)
    add = 100;  
  return (round(value / 100) * 100) + add;
}


float calc_microsec_to_upm(unsigned long microseconds)
{
  // we dont want to devide by zero !!
  if(microseconds < 1)
    return 0;
  
  // RPM = ( 60.000.000 / t_in_µs ) * 2 
  return (_time_gap_divident / (float) microseconds) * _ignition_multiplier; 
}


void clearErrors()
{
  error_code = 0;
}



void visualize()
{
    show_color_bar();
  
    lcd.setTextColor(WHITE, BLACK); 
    lcd.setTextSize(10);
    lcd.setCursor(16, 64);

    String s_current_RPM = String(round_to_hundrets(_current_RPM));
    
    if(_current_RPM < 10000)
      s_current_RPM = " " + s_current_RPM;
      
    if(_current_RPM < 1000)
      s_current_RPM = " " + s_current_RPM;

    if(_current_RPM < _min_RPM)
      s_current_RPM = "  ---";
    
    lcd.println(s_current_RPM);

    lcd.setTextSize(6);
    lcd.setCursor(96, 144);
    lcd.println("U/Min");

    //DEBUG
    if(_simulation_mode && 0)
    {
      // DEBUG
      lcd.setTextSize(2);
      lcd.setCursor(0, 187);
      lcd.println("Simulated: " + String(_timer2_simulator_ms_delay) + " ms  ");
      lcd.println("Measured : " + String(_current_ms) + " ms  ");
      lcd.println("NonRound : " + String(_current_RPM) + " = " + String(round_to_hundrets(_current_RPM)));
    }
    return;
}

void show_color_bar()
{
  byte red   = 0x00;
  byte green = 0x00;
  byte blue  = 0x00;

  red = (byte) min(255, 255 * (_current_RPM / (float)_max_RPM));
  green = 255 - red;
  
  lcd.fillRect(0,0, 320, 42, make_color(red, green, blue));
  lcd.fillRect(0,198, 320, 42, make_color(red, green, blue));
}

unsigned short make_color(byte red, byte green, byte blue)
{
    byte scaled_red   = red   >> 3; // means "/ 8"
    byte scaled_green = green >> 2;
    byte scaled_blue  = blue  >> 3;

    unsigned short color = (scaled_red << 11) | (scaled_green << 5) | (scaled_blue);
    return color;
}


void drawErrors()
{
    lcd.setTextColor(RED, BLACK); 
    lcd.setTextSize(2);
    
    lcd.setCursor(15, 196);

    lcd.println(String("Error Code: ") + String(error_code));

    return;
}


void showNiceBootScreen_LetsMud()
{
  lcd.fillScreen(BLACK);
  lcd.setTextColor(WHITE, BLACK); 
  lcd.setTextSize(4);

  byte num_points = 4;
  unsigned int delay_msec = 1000;
  String start_msg = "Starting ";

  for(unsigned int i = 0; i < num_points; i++)
  {
     lcd.setCursor(16, 32); 
     lcd.println(start_msg); 
     start_msg = start_msg + ".";
     delay(delay_msec);
  }

  lcd.setCursor(16, 96); 
  
  lcd.setTextColor(RED, BLACK);
  lcd.setTextSize(5);
  lcd.println("-LETS MUD-"); 

  delay(2000);
  
  lcd.setTextColor(WHITE, BLACK);
  lcd.setTextSize(1);
  lcd.setCursor(176, 212); 
  lcd.println("powered by ZVSkutch"); 
  
  delay(5000);
}
