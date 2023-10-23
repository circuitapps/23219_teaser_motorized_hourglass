
/*
  Motorized hourglass 
  
  INCOMING SIGNALS TO Attiny85:
  -----------------------------
  CLOCK_IN: Incoming clock signal from the hourglass is connected to a pin change interrupt on teh ATtiny85.
  U2D10_SIGNAL: The highest bit of the hourglass shift register is connected as input to ATtiny85.
  
  OUTGOING SIGNALS FROM Attiny85:
  -----------------------------
  SERVO_OUT: Signal that rotates the servo 180 or 1 degrees when needed.
  TEST_LED: Connected to an LED for debug/trobleshooting purposes. (Not required for the core operation)

  OPERATION: The servo is connected to the hourglass such that it can be rotated by 0 or 180 degrees everytime it times out. The time out condition is checked by reading
  U2D10_SIGNAL input. Due to intermittent instabilities of this signal (especially at power up), CLOCK_IN signal is monitored for TARGET_CLOCK_COUNT_B4_ROTATE cycles before
  triggering the servo to rotate. The number of cycles has been specified empirically by observing the operation of the hourglass using an oscilloscope.

  by circuitapps
  October 2023
*/

#include <avr/io.h>
#include "Servo8Bit.h"

// PORT B DEFINITIONS
#define PB5_PIN1 PB5
#define PB4_PIN3 PB4
#define PB3_PIN2 PB3
#define PB2_PIN7 PB2
#define PB1_PIN6 PB1
#define PB0_PIN5 PB0

#define CLOCK_IN PB3_PIN2  // Incoming clock signal from hourglass
#define U2D10_SIGNAL PB4_PIN3  // Hourglass U2 D10 output signal indicating when to rotate
#define SERVO_OUT PB1_PIN6  // This is the signal going to servo motor
#define TEST_LED PB0_PIN5  // Used for visual testing with LED (for debug purposes only)

#define PC_INT_VECTOR PCINT0_vect  // On ATtiny85, there is ONE interrupt vector (i.e., one interrupt service routine) that gets called when ANY of the active interrupt pins are triggered !

#define CLOCKWISE 1
#define ANTICLOCKWISE 0

#define TRUE 1
#define FALSE 0

//#define HIGH 1  already defined in Arduino header
//#define LOW 0   already defined in Arduino header

#define TARGET_CLOCK_COUNT_B4_ROTATE 10  // Before rotating the hourglass, we wait for this many clock cycles for D10 to stabilize (based on hourglass operation)

Servo8Bit myServo;  // create a global servo object. 

bool current_direction = CLOCKWISE;  // rotation direction that will occur next
bool last_D10_level = HIGH;
bool current_D10_level = LOW;

bool rotate_flag = FALSE;  // if TRUE, hourglass will be rotated
bool pin_D10_read = FALSE;

int current_clock_count = 0;

void pin_interrupt_config()
{
  cli();  // disable GLOBAL interrupts during set up (USE WITH CAUTION as timing functions such as millis(), micros(), delay() get disrupted)
  // Three pin interrupts are enabled below
  PCMSK |= (1 << CLOCK_IN);  // Only CLOCK_IN input pin drives an interrupt
  GIFR  |= (1 << PCIF);  // clear any outstanding interrupts
  GIMSK &= ~(1 << PCIE);  // Pin Change Interrupts are DISABLED
  pinMode(TEST_LED, OUTPUT); // LED pin is set as output
  pinMode(CLOCK_IN, INPUT);  // Hourglass clock output accepted as input
  pinMode(U2D10_SIGNAL, INPUT);  // Hourglass D10 signal accepted as input
  sei();  // enable GLOBAL interrupts after set up (last line of set up) (USE WITH CAUTION as timing functions such as millis(), micros(), delay() get enabled)
}

void enable_PC_interrupts()
{// Only enables Pin Change interrupt
  GIMSK |= (1 << PCIE);
}

void disable_PC_interrupts()
{// Only disables Pin Change interrupt
  GIMSK &= ~(1 << PCIE);
}

void servo_rotate(bool direction)
{
  if(direction == CLOCKWISE)
  {// Clockwise turn means moving to 179 degrees on the servo (maximum is 180)
    myServo.write(179);  // 179 degree movement
  }
  else
  {// Anti-clockwise turn means moving to 1 degree on the servo
    myServo.write(1);  // 1 degree movement (minimum is 0 degrees)
  }
}


ISR(PC_INT_VECTOR)
{// Pin change interrupt routine

  if(digitalRead(CLOCK_IN) == LOW)
  {
    if(current_clock_count >= TARGET_CLOCK_COUNT_B4_ROTATE)
    {// waited long enough before triggering the servo rotation.
      rotate_flag = TRUE;  // enable servo rotation
      digitalWrite(TEST_LED, HIGH);  // for troubleshooting if necessary
    }
    else
    {
      ++current_clock_count;  // one more clock pulse arrived
    }
  }
  // take no action on clock high levels

}

// the setup function runs once when you press reset or power the board
void setup()
{
  myServo.attach(SERVO_OUT);  //attach the servo to pin PB1
  servo_rotate(current_direction);  // servo position reset (one CLOCKWISE rotation executed)
  current_direction = ANTICLOCKWISE;  // next rotation direction set for the rest of the code
  delay(150);  // servo positioning delay
  pin_interrupt_config();  // set up interrupts
  last_D10_level = digitalRead(U2D10_SIGNAL);
}


// the loop function runs over and over again forever
void loop()
{

  bool current_D10_level;

  if(rotate_flag == TRUE)
  {
    disable_PC_interrupts();  // disable pin change interrupts
    servo_rotate(current_direction);  // next rotation will be in the opposite direction
    
    delay(150);  // This is to give time to the servo to activate and complete its rotation. Critically important to include here !

    if(current_direction == CLOCKWISE)
      current_direction = ANTICLOCKWISE;
    else
      current_direction = CLOCKWISE;

    rotate_flag = FALSE;  // one rotation executed. Flag inverted.
    last_D10_level = digitalRead(U2D10_SIGNAL);  // D10 level after servo rotation
    digitalWrite(TEST_LED, LOW);
  }
  else
  {
    current_D10_level = digitalRead(U2D10_SIGNAL);
    if( last_D10_level !=  current_D10_level )
    {// D10 level changed. Every triangular group of lit LEDs mean the D10 pin on U2 has toggled.
      last_D10_level = current_D10_level;  // Update D10 level to avoid revisiting this part often.
      current_clock_count = 0; // reset
      enable_PC_interrupts();  // enable interrupt to wait for stabilization before the rotation
    }
  }

}
