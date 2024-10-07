/******************************************************
 * binary game -- user enters binary numbers
 *				  by toggling buttons
 *
 * Author: Benjamin Rogers
 *
 * Purpose: CAB202 Assignment
 *
 * Usage:
 *
 *****************************************************/

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#define SET_BIT(reg, pin) (reg) |= (1 << (pin))
#define CLEAR_BIT(reg, pin) (reg) &= ~(1 << (pin))
#define WRITE_BIT(reg, pin, value) (reg) = (((reg) & ~(1 << (pin))) | ((value) << (pin)))
#define BIT_VALUE(reg, pin) (((reg) >> (pin)) & 1)
#define BIT_IS_SET(reg, pin) (BIT_VALUE((reg), (pin)) == 1)

// BAUD rate globals
#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR F_CPU / 16 / BAUD - 1

#define PRESCALE (256.0)
#define FREQ (16000000.0)

// LCD definitions

#define LCD_USING_4PIN_MODE (1)

// #define LCD_DATA0_DDR (DDRD)
// #define LCD_DATA1_DDR (DDRD)
// #define LCD_DATA2_DDR (DDRD)
// #define LCD_DATA3_DDR (DDRD)
#define LCD_DATA4_DDR (DDRD)
#define LCD_DATA5_DDR (DDRD)
#define LCD_DATA6_DDR (DDRD)
#define LCD_DATA7_DDR (DDRD)

// #define LCD_DATA0_PORT (PORTD)
// #define LCD_DATA1_PORT (PORTD)
// #define LCD_DATA2_PORT (PORTD)
// #define LCD_DATA3_PORT (PORTD)
#define LCD_DATA4_PORT (PORTD)
#define LCD_DATA5_PORT (PORTD)
#define LCD_DATA6_PORT (PORTD)
#define LCD_DATA7_PORT (PORTD)

// #define LCD_DATA0_PIN (0)
// #define LCD_DATA1_PIN (1)
// #define LCD_DATA2_PIN (2)
// #define LCD_DATA3_PIN (3)
#define LCD_DATA4_PIN (4)
#define LCD_DATA5_PIN (5)
#define LCD_DATA6_PIN (6)
#define LCD_DATA7_PIN (7)

#define LCD_RS_DDR (DDRB)
#define LCD_ENABLE_DDR (DDRB)

#define LCD_RS_PORT (PORTB)
#define LCD_ENABLE_PORT (PORTB)

#define LCD_RS_PIN (1)
#define LCD_ENABLE_PIN (0)

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

// Function declarations
void timer_setup(void);
void uart_setup(unsigned int ubrr);
void uart_transfer(unsigned char data);
unsigned char uart_receive(void);
void setup_leds(void);
void turn_on_leds(void);
void determine_state(int button_number, volatile unsigned char reg, int pin_number);
void adc_setup(void);
void process(void);
void reset_input(void);

// LCD libary functions
void lcd_init(void);
void lcd_write_string(uint8_t x, uint8_t y, char string[]);
void lcd_write_char(uint8_t x, uint8_t y, char val);
void lcd_clear(void);
void lcd_home(void);

void lcd_createChar(uint8_t, uint8_t[]);
void lcd_setCursor(uint8_t, uint8_t);

void lcd_noDisplay(void);
void lcd_display(void);
void lcd_noBlink(void);
void lcd_blink(void);
void lcd_noCursor(void);
void lcd_cursor(void);
void lcd_leftToRight(void);
void lcd_rightToLeft(void);
void lcd_autoscroll(void);
void lcd_noAutoscroll(void);
void scrollDisplayLeft(void);
void scrollDisplayRight(void);

size_t lcd_write(uint8_t);
void lcd_command(uint8_t);

void lcd_send(uint8_t, uint8_t);
void lcd_write4bits(uint8_t);
void lcd_write8bits(uint8_t);
void lcd_pulseEnable(void);

uint8_t _lcd_displayfunction;
uint8_t _lcd_displaycontrol;
uint8_t _lcd_displaymode;

// Binary states for screen (will change as buttons are pressed)
int button_confirm = 0;
unsigned char input_bitmask = 0b00000000;
unsigned char compare_bitmask = 0b00000001;

// Curent level
int guess_decimal = 1;

// 2d array for debouncing variables
// [x][0] is state_count for that button x
// [x][1] is pressed toggle for button x
// [x][2] is previous state for button x
volatile uint8_t button_states[9][3] = {{0}};

// Setup buttons
void setup_buttons(void)
{
  // butttons in order left to right
  CLEAR_BIT(DDRB, 5); // btn 1
  CLEAR_BIT(DDRB, 1); // btn 2
  CLEAR_BIT(DDRB, 0); // btn 3
  CLEAR_BIT(DDRD, 7); // btn 4
  CLEAR_BIT(DDRC, 4); // btn 5
  CLEAR_BIT(DDRC, 3); // btn 6
  CLEAR_BIT(DDRC, 2); // btn 7
  CLEAR_BIT(DDRC, 1); // btn 8
  CLEAR_BIT(DDRC, 5); // btn confirm
}

int main(void)
{

  timer_setup();
  setup_leds();
  setup_buttons();
  adc_setup();
  uart_setup(MYUBRR);

  // Display initial value for user to enter once
  // Once entered correct, process() function will display next number to ener

  char string[] = "Enter 1 in binary";
  uart_putstring(string);

  while (1)
  {
    process();

    _delay_ms(100);
  }
}

void setup_leds(void)
{

  SET_BIT(DDRB, 4); // LED 1 (GREEN)
  SET_BIT(DDRB, 3); // LED 2 (RED)
}

void process()
{

  char string[50];

  char current_bitmask[50];

  // ================== 1st button (DEBOUNCED) ================

  if (button_states[0][1] != button_states[0][2])
  {

    if ((!button_states[0][2]) && (button_states[0][1]))
    {

      // Store user input in bitmask

      if (BIT_VALUE(input_bitmask, 7) == 0)
      {
        SET_BIT(input_bitmask, 7);
      }
      else if (BIT_VALUE(input_bitmask, 7) == 1)
      {
        CLEAR_BIT(input_bitmask, 7);
      }

      sprintf(string, "Bit 7: set to '%d'", BIT_VALUE(input_bitmask, 7));
      uart_putstring(string);

      sprintf(current_bitmask, "Current bitmask: 0b%d%d%d%d%d%d%d%d",
              BIT_VALUE(input_bitmask, 7), BIT_VALUE(input_bitmask, 6),
              BIT_VALUE(input_bitmask, 5), BIT_VALUE(input_bitmask, 4),
              BIT_VALUE(input_bitmask, 3), BIT_VALUE(input_bitmask, 2),
              BIT_VALUE(input_bitmask, 1), BIT_VALUE(input_bitmask, 0));

      uart_putstring(current_bitmask);
    }

    button_states[0][2] = button_states[0][1];
  }

  // ================== 2nd button (DEBOUNCED) ================

  if (button_states[1][1] != button_states[1][2])
  {

    if ((!button_states[1][2]) && (button_states[1][1]))
    {

      // Store user input in bitmask

      if (BIT_VALUE(input_bitmask, 6) == 0)
      {
        SET_BIT(input_bitmask, 6);
      }
      else if (BIT_VALUE(input_bitmask, 6) == 1)
      {
        CLEAR_BIT(input_bitmask, 6);
      }

      sprintf(string, "Bit 6: set to '%d'", BIT_VALUE(input_bitmask, 6));
      uart_putstring(string);

      sprintf(current_bitmask, "Current bitmask: 0b%d%d%d%d%d%d%d%d",
              BIT_VALUE(input_bitmask, 7), BIT_VALUE(input_bitmask, 6),
              BIT_VALUE(input_bitmask, 5), BIT_VALUE(input_bitmask, 4),
              BIT_VALUE(input_bitmask, 3), BIT_VALUE(input_bitmask, 2),
              BIT_VALUE(input_bitmask, 1), BIT_VALUE(input_bitmask, 0));

      uart_putstring(current_bitmask);
    }

    button_states[1][2] = button_states[1][1];
  }

  // ================== 3rd button (DEBOUNCED) ================

  if (button_states[2][1] != button_states[2][2])
  {

    if ((!button_states[2][2]) && (button_states[2][1]))
    {

      // Store user input in bitmask

      if (BIT_VALUE(input_bitmask, 5) == 0)
      {
        SET_BIT(input_bitmask, 5);
      }
      else if (BIT_VALUE(input_bitmask, 5) == 1)
      {
        CLEAR_BIT(input_bitmask, 5);
      }

      sprintf(string, "Bit 5: set to '%d'", BIT_VALUE(input_bitmask, 5));
      uart_putstring(string);

      sprintf(current_bitmask, "Current bitmask: 0b%d%d%d%d%d%d%d%d",
              BIT_VALUE(input_bitmask, 7), BIT_VALUE(input_bitmask, 6),
              BIT_VALUE(input_bitmask, 5), BIT_VALUE(input_bitmask, 4),
              BIT_VALUE(input_bitmask, 3), BIT_VALUE(input_bitmask, 2),
              BIT_VALUE(input_bitmask, 1), BIT_VALUE(input_bitmask, 0));

      uart_putstring(current_bitmask);
    }

    button_states[2][2] = button_states[2][1];
  }

  // ================== 4th button (DEBOUNCED) ================

  if (button_states[3][1] != button_states[3][2])
  {

    if ((!button_states[3][2]) && (button_states[3][1]))
    {

      // Store user input in bitmask

      if (BIT_VALUE(input_bitmask, 4) == 0)
      {
        SET_BIT(input_bitmask, 4);
      }
      else if (BIT_VALUE(input_bitmask, 4) == 1)
      {
        CLEAR_BIT(input_bitmask, 4);
      }

      sprintf(string, "Bit 4: set to '%d'", BIT_VALUE(input_bitmask, 4));
      uart_putstring(string);

      sprintf(current_bitmask, "Current bitmask: 0b%d%d%d%d%d%d%d%d",
              BIT_VALUE(input_bitmask, 7), BIT_VALUE(input_bitmask, 6),
              BIT_VALUE(input_bitmask, 5), BIT_VALUE(input_bitmask, 4),
              BIT_VALUE(input_bitmask, 3), BIT_VALUE(input_bitmask, 2),
              BIT_VALUE(input_bitmask, 1), BIT_VALUE(input_bitmask, 0));

      uart_putstring(current_bitmask);
    }

    button_states[3][2] = button_states[3][1];
  }

  // ================== 5th button (DEBOUNCED) ================

  if (button_states[4][1] != button_states[4][2])
  {

    if ((!button_states[4][2]) && (button_states[4][1]))
    {

      // Store user input in bitmask

      if (BIT_VALUE(input_bitmask, 3) == 0)
      {
        SET_BIT(input_bitmask, 3);
      }
      else if (BIT_VALUE(input_bitmask, 3) == 1)
      {
        CLEAR_BIT(input_bitmask, 3);
      }

      sprintf(string, "Bit 3: set to '%d'", BIT_VALUE(input_bitmask, 3));
      uart_putstring(string);

      sprintf(current_bitmask, "Current bitmask: 0b%d%d%d%d%d%d%d%d",
              BIT_VALUE(input_bitmask, 7), BIT_VALUE(input_bitmask, 6),
              BIT_VALUE(input_bitmask, 5), BIT_VALUE(input_bitmask, 4),
              BIT_VALUE(input_bitmask, 3), BIT_VALUE(input_bitmask, 2),
              BIT_VALUE(input_bitmask, 1), BIT_VALUE(input_bitmask, 0));

      uart_putstring(current_bitmask);
    }

    button_states[4][2] = button_states[4][1];
  }

  // ================== 6th button (DEBOUNCED) ================

  if (button_states[5][1] != button_states[5][2])
  {

    if ((!button_states[5][2]) && (button_states[5][1]))
    {

      // Store user input in bitmask

      if (BIT_VALUE(input_bitmask, 2) == 0)
      {
        SET_BIT(input_bitmask, 2);
      }
      else if (BIT_VALUE(input_bitmask, 2) == 1)
      {
        CLEAR_BIT(input_bitmask, 2);
      }

      sprintf(string, "Bit 2: set to '%d'", BIT_VALUE(input_bitmask, 2));
      uart_putstring(string);

      sprintf(current_bitmask, "Current bitmask: 0b%d%d%d%d%d%d%d%d",
              BIT_VALUE(input_bitmask, 7), BIT_VALUE(input_bitmask, 6),
              BIT_VALUE(input_bitmask, 5), BIT_VALUE(input_bitmask, 4),
              BIT_VALUE(input_bitmask, 3), BIT_VALUE(input_bitmask, 2),
              BIT_VALUE(input_bitmask, 1), BIT_VALUE(input_bitmask, 0));

      uart_putstring(current_bitmask);
    }

    button_states[5][2] = button_states[5][1];
  }

  // ================== 7th button (DEBOUNCED) ================

  if (button_states[6][1] != button_states[6][2])
  {

    if ((!button_states[6][2]) && (button_states[6][1]))
    {

      // Store user input in bitmask

      if (BIT_VALUE(input_bitmask, 1) == 0)
      {
        SET_BIT(input_bitmask, 1);
      }
      else if (BIT_VALUE(input_bitmask, 1) == 1)
      {
        CLEAR_BIT(input_bitmask, 1);
      }

      sprintf(string, "Bit 1: set to '%d'", BIT_VALUE(input_bitmask, 1));
      uart_putstring(string);

      sprintf(current_bitmask, "Current bitmask: 0b%d%d%d%d%d%d%d%d",
              BIT_VALUE(input_bitmask, 7), BIT_VALUE(input_bitmask, 6),
              BIT_VALUE(input_bitmask, 5), BIT_VALUE(input_bitmask, 4),
              BIT_VALUE(input_bitmask, 3), BIT_VALUE(input_bitmask, 2),
              BIT_VALUE(input_bitmask, 1), BIT_VALUE(input_bitmask, 0));

      uart_putstring(current_bitmask);
    }

    button_states[6][2] = button_states[6][1];
  }

  // ================== 8th button (DEBOUNCED) ================

  if (button_states[7][1] != button_states[7][2])
  {

    if ((!button_states[7][2]) && (button_states[7][1]))
    {

      // Store user input in bitmask

      if (BIT_VALUE(input_bitmask, 0) == 0)
      {
        SET_BIT(input_bitmask, 0);
      }
      else if (BIT_VALUE(input_bitmask, 0) == 1)
      {
        CLEAR_BIT(input_bitmask, 0);
      }

      sprintf(string, "Bit 0: set to '%d'", BIT_VALUE(input_bitmask, 0));
      uart_putstring(string);

      sprintf(current_bitmask, "Current bitmask: 0b%d%d%d%d%d%d%d%d",
              BIT_VALUE(input_bitmask, 7), BIT_VALUE(input_bitmask, 6),
              BIT_VALUE(input_bitmask, 5), BIT_VALUE(input_bitmask, 4),
              BIT_VALUE(input_bitmask, 3), BIT_VALUE(input_bitmask, 2),
              BIT_VALUE(input_bitmask, 1), BIT_VALUE(input_bitmask, 0));

      uart_putstring(current_bitmask);
    }

    button_states[7][2] = button_states[7][1];
  }

  // ================== Confirm button (DEBOUNCED) ================

  if (button_states[8][1] != button_states[8][2])
  {

    if ((!button_states[8][2]) && (button_states[8][1]))
    {

      // set button value for lcd
      if (button_confirm == 0)
        button_confirm = 1;
      else
        button_confirm = 0;

      // ================== CHECK IF USER ENTERED CORRECT BINARY ================
      bool equal;

      if (input_bitmask == compare_bitmask)
        equal = true;
      else
        equal = false;

      if (equal)
      {

        sprintf(string, "You got it right!");
        uart_putstring(string);

        // TURN ON GREEN
        SET_BIT(PORTB, 4);

        _delay_ms(1500);

        CLEAR_BIT(PORTB, 3);

        // Setup Next Level

        guess_decimal++; // next number to guess
        sprintf(string, "\nEnter %d in binary\n", guess_decimal);
        uart_putstring(string);

        input_bitmask = 0;                      // reset buttons
        compare_bitmask = 0;                    // reset bitpattern
        compare_bitmask = (guess_decimal << 0); // set new bitpattern
      }
      else
      {
        input_bitmask = 0; // reset button

        sprintf(string, "Try again...");
        uart_putstring(string);

        // TURN ON RED
        SET_BIT(PORTB, 3);

        _delay_ms(1500);

        CLEAR_BIT(PORTB, 4);
      }
    }
    else
    {
      CLEAR_BIT(PORTB, 4);
    }

    button_states[8][2] = button_states[8][1];
  }

  // ================== RESET DIAL ============================

  // Start single conversion by setting ADSC bit in ADCSRA
  ADCSRA |= (1 << ADSC);

  // Wait for ADSC bit to clear, signalling conversion complete.
  while (ADCSRA & (1 << ADSC))
  {
  }

  // Result now available.
  uint16_t pot = ADC;

  // Turn on led if dial greater greater than 512
  if (pot > 512)
  {

    // Reset buttons
    input_bitmask = 0;

    // Turn on LEDs to indicate reset
    SET_BIT(PORTB, 4);
    SET_BIT(PORTB, 3);
    // SET_BIT(PORTD, 2);
    // SET_BIT(PORTD, 4);
  }
  else if (pot < 512)
  {

    CLEAR_BIT(PORTB, 4);
    CLEAR_BIT(PORTB, 3);
    // CLEAR_BIT(PORTD, 2);
    // CLEAR_BIT(PORTD, 4);
  }

  // Compare user input to binary
}

// ================== SETUP FUNCTIONS ================

void timer_setup(void)
{

  TCCR0A = 0; // Normal mode

  TCCR0B = 4; // Set to use period of approx 0.004 seconds

  TIMSK0 = 1; // Enable timer overflow interrupt for Timer 0.

  sei(); // Turn on interupts
}

void uart_setup(unsigned int ubrr)
{
  // Set BAUD rate
  UBRR0H = (unsigned char)(ubrr >> 8);
  UBRR0L = (unsigned char)(ubrr);

  // Enable receiver and transmitter
  UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0); // Enable receiver and transmitter
  UCSR0C = (3 << UCSZ00);
}

void adc_setup(void)
{

  // Initialise ADC
  //  ADEN - Enables ADC
  //  ADP[2:0] are the prescaler bits
  //  0b111 is 128
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

  // Select channel and ref voltage
  // Leave REFS0 to 1, and REFS1 to 0
  // Dial connected to ADC0 which is MUX3:0 == 0b0000
  ADMUX = (1 << REFS0);
}

void ldc_setup(void)
{
  lcd_init();

  lcd_write_string(0, 0, "Enter decimal");
  _delay_ms(1000);

  lcd_clear();
}

// ================== SERIAL FUNCTIONS ================

void uart_putstring(char *string)
{
  for (int i = 0; i < strlen(string); i++)
  {
    uart_transfer(string[i]);
  }
  uart_transfer('\n');
}

void uart_transfer(unsigned char data)
{
  while (!(UCSR0A & (1 << UDRE0)))
    ; // Loop while 5th bit equals 0

  UDR0 = data; // UDR0 is I/0 register
}

unsigned char uart_receive(void)
{
  // Wait for data to be received

  while (!(UCSR0A & (1 << RXC0)))
    ; // Loop while 7th bit equals 0

  return UDR0;
}

ISR(TIMER0_OVF_vect)
{

  // Debounce buttons
  determine_state(0, PINB, 5); // Btn 1
  determine_state(1, PIND, 2); // Btn 2
  determine_state(2, PIND, 3); // Btn 3
  determine_state(3, PINB, 2); // Btn 4
  determine_state(4, PINC, 4); // Btn 5
  determine_state(5, PINC, 3); // Btn 6
  determine_state(6, PINC, 2); // Btn 7
  determine_state(7, PINC, 1); // Btn 8
  determine_state(8, PINC, 5); // Btn confirm
}

// Determines pressed state (0 or 1) for buttons and stores in 2D array

void determine_state(int button_number, volatile unsigned char reg, int pin_number)
{
  // Bitmask used for all
  uint8_t bitmask = 0b01111111;

  //[x][0] is state_count
  //[x][1] is pressed variable
  //[x][2] is previous state

  button_states[button_number][0] = (button_states[button_number][0] << 1);
  button_states[button_number][0] = (button_states[button_number][0] & bitmask);
  button_states[button_number][0] |= BIT_VALUE(reg, pin_number);

  if (button_states[button_number][0] == bitmask)
  {
    button_states[button_number][1] = 1;
  }
  else if (button_states[button_number][0] == 0)
  {
    button_states[button_number][1] = 0;
  }
}

// ================== LCD FUNCTIONS ================

void lcd_init(void)
{
  // dotsize
  if (LCD_USING_4PIN_MODE)
  {
    _lcd_displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
  }
  else
  {
    _lcd_displayfunction = LCD_8BITMODE | LCD_1LINE | LCD_5x8DOTS;
  }

  _lcd_displayfunction |= LCD_2LINE;

  // RS Pin
  LCD_RS_DDR |= (1 << LCD_RS_PIN);
  // Enable Pin
  LCD_ENABLE_DDR |= (1 << LCD_ENABLE_PIN);

#if LCD_USING_4PIN_MODE
  // Set DDR for all the data pins
  LCD_DATA4_DDR |= (1 << 4);
  LCD_DATA5_DDR |= (1 << 5);
  LCD_DATA6_DDR |= (1 << 6);
  LCD_DATA7_DDR |= (1 << 7);

#else
  // Set DDR for all the data pins
  LCD_DATA0_DDR |= (1 << LCD_DATA0_PIN);
  LCD_DATA1_DDR |= (1 << LCD_DATA1_PIN);
  LCD_DATA2_DDR |= (1 << LCD_DATA2_PIN);
  LCD_DATA3_DDR |= (1 << LCD_DATA3_PIN);
  LCD_DATA4_DDR |= (1 << LCD_DATA4_PIN);
  LCD_DATA5_DDR |= (1 << LCD_DATA5_PIN);
  LCD_DATA6_DDR |= (1 << LCD_DATA6_PIN);
  LCD_DATA7_DDR |= (1 << LCD_DATA7_PIN);
#endif

  // SEE PAGE 45/46 OF Hitachi HD44780 DATASHEET FOR INITIALIZATION SPECIFICATION!

  // according to datasheet, we need at least 40ms after power rises above 2.7V
  // before sending commands. Arduino can turn on way before 4.5V so we'll wait 50
  _delay_us(50000);
  // Now we pull both RS and Enable low to begin commands (R/W is wired to ground)
  LCD_RS_PORT &= ~(1 << LCD_RS_PIN);
  LCD_ENABLE_PORT &= ~(1 << LCD_ENABLE_PIN);

  // put the LCD into 4 bit or 8 bit mode
  if (LCD_USING_4PIN_MODE)
  {
    // this is according to the hitachi HD44780 datasheet
    // figure 24, pg 46

    // we start in 8bit mode, try to set 4 bit mode
    lcd_write4bits(0b0111);
    _delay_us(4500); // wait min 4.1ms

    // second try
    lcd_write4bits(0b0111);
    _delay_us(4500); // wait min 4.1ms

    // third go!
    lcd_write4bits(0b0111);
    _delay_us(150);

    // finally, set to 4-bit interface
    lcd_write4bits(0b0010);
  }
  else
  {
    // this is according to the hitachi HD44780 datasheet
    // page 45 figure 23

    // Send function set command sequence
    lcd_command(LCD_FUNCTIONSET | _lcd_displayfunction);
    _delay_us(4500); // wait more than 4.1ms

    // second try
    lcd_command(LCD_FUNCTIONSET | _lcd_displayfunction);
    _delay_us(150);

    // third go
    lcd_command(LCD_FUNCTIONSET | _lcd_displayfunction);
  }

  // finally, set # lines, font size, etc.
  lcd_command(LCD_FUNCTIONSET | _lcd_displayfunction);

  // turn the display on with no cursor or blinking default
  _lcd_displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
  lcd_display();

  // clear it off
  lcd_clear();

  // Initialize to default text direction (for romance languages)
  _lcd_displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
  // set the entry mode
  lcd_command(LCD_ENTRYMODESET | _lcd_displaymode);
}

/********** high level commands, for the user! */
void lcd_write_string(uint8_t x, uint8_t y, char string[])
{
  lcd_setCursor(x, y);
  for (int i = 0; string[i] != '\0'; ++i)
  {
    lcd_write(string[i]);
  }
}

void lcd_write_char(uint8_t x, uint8_t y, char val)
{
  lcd_setCursor(x, y);
  lcd_write(val);
}

void lcd_clear(void)
{
  lcd_command(LCD_CLEARDISPLAY); // clear display, set cursor position to zero
  _delay_us(2000);               // this command takes a long time!
}

void lcd_home(void)
{
  lcd_command(LCD_RETURNHOME); // set cursor position to zero
  _delay_us(2000);             // this command takes a long time!
}

// Allows us to fill the first 8 CGRAM locations
// with custom characters
void lcd_createChar(uint8_t location, uint8_t charmap[])
{
  location &= 0x7; // we only have 8 locations 0-7
  lcd_command(LCD_SETCGRAMADDR | (location << 3));
  for (int i = 0; i < 8; i++)
  {
    lcd_write(charmap[i]);
  }
}

void lcd_setCursor(uint8_t col, uint8_t row)
{
  if (row >= 2)
  {
    row = 1;
  }

  lcd_command(LCD_SETDDRAMADDR | (col + row * 0x40));
}

// Turn the display on/off (quickly)
void lcd_noDisplay(void)
{
  _lcd_displaycontrol &= ~LCD_DISPLAYON;
  lcd_command(LCD_DISPLAYCONTROL | _lcd_displaycontrol);
}
void lcd_display(void)
{
  _lcd_displaycontrol |= LCD_DISPLAYON;
  lcd_command(LCD_DISPLAYCONTROL | _lcd_displaycontrol);
}

// Turns the underline cursor on/off
void lcd_noCursor(void)
{
  _lcd_displaycontrol &= ~LCD_CURSORON;
  lcd_command(LCD_DISPLAYCONTROL | _lcd_displaycontrol);
}
void lcd_cursor(void)
{
  _lcd_displaycontrol |= LCD_CURSORON;
  lcd_command(LCD_DISPLAYCONTROL | _lcd_displaycontrol);
}

// Turn on and off the blinking cursor
void lcd_noBlink(void)
{
  _lcd_displaycontrol &= ~LCD_BLINKON;
  lcd_command(LCD_DISPLAYCONTROL | _lcd_displaycontrol);
}
void lcd_blink(void)
{
  _lcd_displaycontrol |= LCD_BLINKON;
  lcd_command(LCD_DISPLAYCONTROL | _lcd_displaycontrol);
}

// These commands scroll the display without changing the RAM
void scrollDisplayLeft(void)
{
  lcd_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}
void scrollDisplayRight(void)
{
  lcd_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

// This is for text that flows Left to Right
void lcd_leftToRight(void)
{
  _lcd_displaymode |= LCD_ENTRYLEFT;
  lcd_command(LCD_ENTRYMODESET | _lcd_displaymode);
}

// This is for text that flows Right to Left
void lcd_rightToLeft(void)
{
  _lcd_displaymode &= ~LCD_ENTRYLEFT;
  lcd_command(LCD_ENTRYMODESET | _lcd_displaymode);
}

// This will 'right justify' text from the cursor
void lcd_autoscroll(void)
{
  _lcd_displaymode |= LCD_ENTRYSHIFTINCREMENT;
  lcd_command(LCD_ENTRYMODESET | _lcd_displaymode);
}

// This will 'left justify' text from the cursor
void lcd_noAutoscroll(void)
{
  _lcd_displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
  lcd_command(LCD_ENTRYMODESET | _lcd_displaymode);
}

/*********** mid level commands, for sending data/cmds */

inline void lcd_command(uint8_t value)
{
  //
  lcd_send(value, 0);
}

inline size_t lcd_write(uint8_t value)
{
  lcd_send(value, 1);
  return 1; // assume sucess
}

/************ low level data pushing commands **********/

// write either command or data, with automatic 4/8-bit selection
void lcd_send(uint8_t value, uint8_t mode)
{
  // RS Pin
  LCD_RS_PORT &= ~(1 << LCD_RS_PIN);
  LCD_RS_PORT |= (!!mode << LCD_RS_PIN);

  if (LCD_USING_4PIN_MODE)
  {
    lcd_write4bits(value >> 4);
    lcd_write4bits(value);
  }
  else
  {
    lcd_write8bits(value);
  }
}

void lcd_pulseEnable(void)
{
  // Enable Pin
  LCD_ENABLE_PORT &= ~(1 << LCD_ENABLE_PIN);
  _delay_us(1);
  LCD_ENABLE_PORT |= (1 << LCD_ENABLE_PIN);
  _delay_us(1); // enable pulse must be >450ns
  LCD_ENABLE_PORT &= ~(1 << LCD_ENABLE_PIN);
  _delay_us(100); // commands need > 37us to settle
}

void lcd_write4bits(uint8_t value)
{
  // Set each wire one at a time

  LCD_DATA4_PORT &= ~(1 << LCD_DATA4_PIN);
  LCD_DATA4_PORT |= ((value & 1) << LCD_DATA4_PIN);
  value >>= 1;

  LCD_DATA5_PORT &= ~(1 << LCD_DATA5_PIN);
  LCD_DATA5_PORT |= ((value & 1) << LCD_DATA5_PIN);
  value >>= 1;

  LCD_DATA6_PORT &= ~(1 << LCD_DATA6_PIN);
  LCD_DATA6_PORT |= ((value & 1) << LCD_DATA6_PIN);
  value >>= 1;

  LCD_DATA7_PORT &= ~(1 << LCD_DATA7_PIN);
  LCD_DATA7_PORT |= ((value & 1) << LCD_DATA7_PIN);

  lcd_pulseEnable();
}

void lcd_write8bits(uint8_t value)
{
  // Set each wire one at a time

#if !LCD_USING_4PIN_MODE
  LCD_DATA0_PORT &= ~(1 << LCD_DATA0_PIN);
  LCD_DATA0_PORT |= ((value & 1) << LCD_DATA0_PIN);
  value >>= 1;

  LCD_DATA1_PORT &= ~(1 << LCD_DATA1_PIN);
  LCD_DATA1_PORT |= ((value & 1) << LCD_DATA1_PIN);
  value >>= 1;

  LCD_DATA2_PORT &= ~(1 << LCD_DATA2_PIN);
  LCD_DATA2_PORT |= ((value & 1) << LCD_DATA2_PIN);
  value >>= 1;

  LCD_DATA3_PORT &= ~(1 << LCD_DATA3_PIN);
  LCD_DATA3_PORT |= ((value & 1) << LCD_DATA3_PIN);
  value >>= 1;

  LCD_DATA4_PORT &= ~(1 << LCD_DATA4_PIN);
  LCD_DATA4_PORT |= ((value & 1) << LCD_DATA4_PIN);
  value >>= 1;

  LCD_DATA5_PORT &= ~(1 << LCD_DATA5_PIN);
  LCD_DATA5_PORT |= ((value & 1) << LCD_DATA5_PIN);
  value >>= 1;

  LCD_DATA6_PORT &= ~(1 << LCD_DATA6_PIN);
  LCD_DATA6_PORT |= ((value & 1) << LCD_DATA6_PIN);
  value >>= 1;

  LCD_DATA7_PORT &= ~(1 << LCD_DATA7_PIN);
  LCD_DATA7_PORT |= ((value & 1) << LCD_DATA7_PIN);

  lcd_pulseEnable();
#endif
}
