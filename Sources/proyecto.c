#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include "../address_map_arm.h"
#include "../interrupt_ID.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sara Pastor & Inigo Samaniego");
MODULE_DESCRIPTION("DE10SoC Interrupt Handler");

// DEFINES
// ===========================================================================================
#define TIMER2_LOAD 2500000
#define TIMER3_LOAD 250
#define FPGA_TIMER_LOAD 1000
#define SERVO_ANGLE_JUMP 50
#define SERVO_MAX_ANGLE 2400
#define SERVO_MIN_ANGLE 600
#define SUBSTRACT 0
#define ADD 1
#define PWM_CYCLE_SIZE 2000
#define MASK_OR_1_SERVO_X 0x20                  // 0b00000000000000000000000000100000
#define MASK_OR_1_SERVO_Y 0x80                  // 0b00000000000000000000000010000000
#define MASK_AND_0_SERVO_X 0x1FFFFFFDF          // 0b11111111111111111111111111011111
#define MASK_AND_0_SERVO_Y 0x1FFFFFF7F          // 0b11111111111111111111111101111111
#define MASK_OR_1_ACCELEROMETER_X 0x20000       // 0b00000000000000100000000000000000
#define MASK_OR_1_ACCELEROMETER_Y 0x80000       // 0b00000000000010000000000000000000
#define MASK_OR_SERVOS_OUTPUT 0xA0              // 0b00000000000000000000000010100000
#define MASK_AND_ACCELEROMETER_INPUT 0xFFF5FFFF // 0b11111111111101011111111111111111

// GLOBAL VARIABLES
// ===========================================================================================

// Virtual addresses
void *LW_virtual;                 // LIGHTWEIGHT BRIDGE virtual address
void *HPS_Timer2_virtual;         // HPS_Timer2 virtual address
void *HPS_Timer3_virtual;         // HPS_Timer3 virtual address
volatile uint32_t *JP1_ptr;       // JP1 port virtual address
volatile uint32_t *JP1_Direction; // JP1 Direction Register virtual address
volatile uint32_t *LEDR_ptr;      // LEDR port virtual address
volatile uint32_t *HEX3_HEX0_ptr; // HEX3 - HEX0 ports virtual address
volatile uint32_t *HEX5_HEX4_ptr; // HEX5 - HEX4 ports virtual address
volatile uint32_t *SW_ptr;        // SW port virtual address

// KEY port virtual addresses
volatile uint32_t *KEY_ptr;
volatile uint32_t *KEY_Interrupt_Mask;
volatile uint32_t *KEY_Edgecapture;

// FPGA_Timer virtual addresses
volatile uint32_t *FPGA_Timer_ptr;
volatile uint32_t *FPGA_Timer_Control;
volatile uint32_t *FPGA_Timer_Load_Low;
volatile uint32_t *FPGA_Timer_Load_High;

// HPS_Timer2 virtual addresses
volatile uint32_t *HPS_Timer2_ptr;
volatile uint32_t *HPS_Timer2_Control;
volatile uint32_t *HPS_Timer2_Load;
volatile uint32_t *HPS_Timer2_EOI;

// HPS_Timer3 virtual addresses
volatile uint32_t *HPS_Timer3_ptr;
volatile uint32_t *HPS_Timer3_Control;
volatile uint32_t *HPS_Timer3_Load;
volatile uint32_t *HPS_Timer3_EOI;

// Variables for accelerometer PWM control
uint32_t data_X;                   // Value read on pin 17 of JP1 at each FPGA_Timer interrupt
uint32_t data_Y;                   // Value read on pin 19 of JP1 at each FPGA_Timer interrupt
uint8_t rising_edge_X;             // Determines if there has been a rising edge on pin 17 of JP1
uint8_t rising_edge_Y;             // Determines if there has been a rising edge on pin 19 of JP1
uint32_t high_level_perthousand_X; // Accelerometer X-axis duty cycle in percent per thousand
uint32_t high_level_perthousand_Y; // Accelerometer Y-axis duty cycle in percent per thousand
uint32_t high_level_counter_X;     // Number of logic 1's read on pin 17 of JP1
uint32_t high_level_counter_Y;     // Number of logic 1's read on pin 19 of JP1

// Variables for the control of servomotors
uint32_t ticks;                            // Number of times HPS_Timer3 interrupted
uint32_t angle_servo_X_us;                 // Angle at which the servo X is to be positioned, measured in us
uint32_t angle_servo_Y_us;                 // Angle at which the servo Y is to be positioned, measured in us
uint32_t control_servo_key;                // Push-button control of servos activated
uint32_t control_servo_accelerometer;      // Accelerometer control of servos activated
uint32_t amount_angle_servos_travelled_us; // Amount of servo angle travelled, measured in us

// HPS_Timer2 variables for the timer
uint32_t ds; // Number of tenths of a second
uint32_t s;  // Number of seconds
uint32_t m;  // Number of minutes

// FUNCIONES
// ===========================================================================================
void config_FPGA_Timer(void)
{
   /*
    * Configure the FPGA timer. Stop the counter by setting the STOP bit to 1 and the START bit to 0,
    * enable continuous run mode by setting the CONT bit to 1, and disable interrupts by ITO bit set to 0.
    * These 4 bits are in the control register.
    *
    * In addition, it loads the timer to count 10us. Given an operating frequency of 100MHz, for 10us
    * the timer has to count 1000 ticks.
    */

   // Stop timer, enable continuous mode and disable interrupts
   *FPGA_Timer_Control = 0b1010; // STOP->1, START->0, CONT->1, ITO->0

   // Timer frequency = 100MHz
   // Load timer to count 10 us
   *FPGA_Timer_Load_Low = FPGA_TIMER_LOAD;
   *FPGA_Timer_Load_High = 0;
}

void config_Timer2(void)
{
   /*
    * Set timer 2 of the HPS. Stop the timer by setting the E bit to 0 in the control register and 
    * load it with 2500000 ticks to count 100ms, since it runs at 25MHz.
    */

   // Stop Timer2
   *HPS_Timer2_Control = 0b0;

   *HPS_Timer2_Load = TIMER2_LOAD; // 25 MHz -> 100 ms 2500000 ticks
}

void config_Timer3(void)
{
   /*
    * Configure timer 3 of the HPS. Stop the timer by setting the E bit to 0 in the control register 
    * and load it with 250 ticks to count 10us, since it operates at 25MHz.
    */

   // Stop Timer3
   *HPS_Timer3_Control = 0b0;

   *HPS_Timer3_Load = TIMER3_LOAD; // 25 MHz -> 10 us 250 tics
}

uint32_t seven_segment_digit(uint32_t num)
{
   /*
    * Generates the mask so that any number between 0 and 9 can be shown on a 7-segment display. 
    * If a number is received that is not between 0 and 9, an F is returned.
    *
    * PARAMETERS
    *    num -> number to encode.
    *
    * RETURN
    *    mask -> encoding of the number received in 7-segment display format or an F in its absence.
    */

   uint32_t mask;

   switch (num)
   {
   case 0:
      mask = 0b0111111;
      break;

   case 1:
      mask = 0b0000110;
      break;

   case 2:
      mask = 0b1011011;
      break;

   case 3:
      mask = 0b1001111;
      break;

   case 4:
      mask = 0b1100110;
      break;

   case 5:
      mask = 0b1101101;
      break;

   case 6:
      mask = 0b1111101;
      break;

   case 7:
      mask = 0b0000111;
      break;

   case 8:
      mask = 0b1111111;
      break;

   case 9:
      mask = 0b1101111;
      break;

   default:
      mask = 0b1110001; // F
      break;
   }

   return mask;
}

uint32_t table_PWM_accelerometer_us_servo(uint32_t high_level_perthousand)
{
   /*
    * Table of correspondence between the values of the duty cycle of the accelerometer and the angular position of the servomotor.
    *
    * PARAMETERES
    *    high_level_perthousand -> duty cycle value in per thousand
    *
    * RETURN
    *    angle_servo_us -> angular position of the servomotor measured in us.
    */

   uint32_t angle_servo_us;

   if (high_level_perthousand <= 420)
   {
      angle_servo_us = SERVO_MIN_ANGLE;
   }
   else if (high_level_perthousand > 420 && high_level_perthousand <= 440)
   {
      angle_servo_us = 850;
   }
   else if (high_level_perthousand > 440 && high_level_perthousand <= 460)
   {
      angle_servo_us = 1100;
   }
   else if (high_level_perthousand > 460 && high_level_perthousand <= 480)
   {
      angle_servo_us = 1350;
   }
   else if (high_level_perthousand > 480 && high_level_perthousand <= 500)
   {
      angle_servo_us = 1475;
   }
   else if (high_level_perthousand > 500 && high_level_perthousand <= 520)
   {
      angle_servo_us = 1600;
   }
   else if (high_level_perthousand > 520 && high_level_perthousand <= 540)
   {
      angle_servo_us = 1850;
   }
   else if (high_level_perthousand > 540 && high_level_perthousand <= 560)
   {
      angle_servo_us = 2100;
   }
   else if (high_level_perthousand > 560 && high_level_perthousand <= 580)
   {
      angle_servo_us = 2350;
   }
   else if (high_level_perthousand >= 580)
   {
      angle_servo_us = SERVO_MAX_ANGLE;
   }

   return angle_servo_us;
}

uint32_t calc_angle_servo(uint32_t angle_servo_us, uint8_t ADD_SUBSTRACT)
{
   /*
    * Calculation of the increase or decrease of the angular position of the servomotor measured in us. 
    * This increase or decrease is always done in steps of 50 us, corresponding to angular displacements of 5º. 
    * The position will not be increased or decreased if the servomotor is at one of the extremes.
    *
    * PARAMETROS
    *    angle_servo_us -> angular position of the servomotor to be increased or decreased. Measured in us.
    *    ADD_SUBSTRACT -> variable that determines whether to increment or decrement the position.
    *
    * RETURN
    *    angle_servo_us -> servomotor angular position updated. Measured in us.
    */

   switch (ADD_SUBSTRACT)
   {
   case ADD:
      if (angle_servo_us <= SERVO_MAX_ANGLE)
      {
         angle_servo_us = angle_servo_us + SERVO_ANGLE_JUMP;
      }
      break;

   case SUBSTRACT:
      if (angle_servo_us >= SERVO_MIN_ANGLE)
      {
         angle_servo_us = angle_servo_us - SERVO_ANGLE_JUMP;
      }
      break;
   }

   return angle_servo_us;
}

irq_handler_t KEY_irq_handler(int irq, void *dev_id, struct pt_regs *regs)
{
   /*
    * Routine of service of the interruptions generated by the KEYs. 
    * It reads from the Edgecapture register the KEY that has generated the interrupt. 
    * Each KEY is serviced in a different and independent way.
    *
    * KEY 8
    * Changes the control mode of the servomotor according to the value indicated in binary on the SWITCHes. 
    * The control mode change is only effective when this KEY is pressed. Changing the position of the SWITCHes 
    * without pressing this KEY does not cause any change. 
    * There are 3 possible cases.
    *       Case 1 -> the servo motor is controlled by the KEYs and its position will be incremented or decremented 
    *                 in steps of size 'SERVO_ANGLE_JUMP'. Corresponds to the case SW0 = 1.
    *       Case 2 -> the servomotor is controlled by the accelerometer and its position will change smoothly.
    *                 Corresponds to the case SW0 = 1.
    *       Default -> the servomotor is not controlled in any way, and the servomotor does not change its angular position. 
    *                  LEDR9 lights up to let the user know that the servomotors are not under control.
    *
    * KEY 2
    * Decrements the angular position of the servo motors by calling the function 'calc_angle_servo' with the 'SUBSTRACT' mode. 
    * This functionality is only enabled if the user has previously selected the KEYs control mode (case 1).
    *
    * KEY 1
    * Increments the angular position of the servomotors by calling the 'calc_angle_servo' function with the 'ADD' mode. 
    * This functionality is only enabled if the user has previously selected the KEYs control mode (case 1).
    */

   int pressed_key = *KEY_Edgecapture;

   if (pressed_key == 8)
   {
      int sw_value = *SW_ptr;

      switch (sw_value)
      {
      case 1: // Servo control with push-buttons
         *LEDR_ptr = sw_value;
         control_servo_accelerometer = 0;
         control_servo_key = 1;
         break;

      case 2: // Servo control with accelerometer
         *LEDR_ptr = sw_value;
         control_servo_accelerometer = 1;
         control_servo_key = 0;
         break;

      default: // Servomotor control deactivated
         *LEDR_ptr = 0x200;
         control_servo_accelerometer = 0;
         control_servo_key = 0;
         break;
      }
   }

   if (control_servo_key) // Servo control with push-buttons
   {
      if (pressed_key == 1) // Increase value of the angle of the serevomotors
      {
         angle_servo_X_us = calc_angle_servo(angle_servo_X_us, ADD);
         angle_servo_Y_us = calc_angle_servo(angle_servo_Y_us, ADD);
      }
      else if (pressed_key == 2) // Decrease value of the angle of the serevomotors
      {
         angle_servo_X_us = calc_angle_servo(angle_servo_X_us, SUBSTRACT);
         angle_servo_Y_us = calc_angle_servo(angle_servo_Y_us, SUBSTRACT);
      }
   }

   // Clear the current interrupt by writing to the Edgecapture register 
   *KEY_Edgecapture = 0xF;

   return (irq_handler_t)IRQ_HANDLED;
}

irq_handler_t fpga_timer_irq_handler(int irq, void *dev_id, struct pt_regs *regs)
{
   /*
    * Service routine of the interrupts generated by the FPGA timer. The PWM signals generated by the accelerometer are sampled. 
    * Given that these signals have a frequency of 100Hz, and that this routine is executed every 10us, 1000 samples are obtained
    * for each PWM cycle. We work with both signals (X and Y) at the same time and sequentially.
    * 
    * First, the signals generated by the accelerometer are read at the corresponding pins of JP1. To do this, an OR mask is applied
    * to be able to read a single JP1 pin each time, and at the same time, this bit is moved to the position with the lowest weight
    * in order to be able to make comparisons later on.
    *
    * Secondly, the value read is compared with a 1 and with a 0. If a 1 has been read, it means that the PWM signal cycle has started
    * (because there has been a rising edge), and therefore, the number of times a 1 is read begins to be counted. At the moment when a 
    * 0 is reached (which means that there has been a falling edge), the duty cycle rate is saved and the count is restarted.
    * 
    * Thirdly, the correspondence between the duty cycle percent per mille and the angular position of the servomotors is obtained using 
    * the function 'table_PWM_accelerometer_us_servo'.
    */

   data_X = ((*JP1_ptr) & MASK_OR_1_ACCELEROMETER_X) >> 17; // Transversal axis
   data_Y = ((*JP1_ptr) & MASK_OR_1_ACCELEROMETER_Y) >> 19; // Longitudinal axis

   if (data_X == 1)
   {
      rising_edge_X = 1;
      high_level_counter_X++;
   }
   else if (data_X == 0 && rising_edge_X)
   {
      high_level_perthousand_X = high_level_counter_X;
      rising_edge_X = 0;
      high_level_counter_X = 0;
   }

   if (data_Y == 1)
   {
      rising_edge_Y = 1;
      high_level_counter_Y++;
   }
   else if (data_Y == 0 && rising_edge_Y)
   {
      high_level_perthousand_Y = high_level_counter_Y;
      rising_edge_Y = 0;
      high_level_counter_Y = 0;
   }

   if (control_servo_accelerometer)
   {
      angle_servo_X_us = table_PWM_accelerometer_us_servo(high_level_perthousand_X);
      angle_servo_Y_us = table_PWM_accelerometer_us_servo(high_level_perthousand_Y);
   }

   // Clear current interrupt by writing to the STATUS register
   *(FPGA_Timer_ptr) = 0b0;

   return (irq_handler_t)IRQ_HANDLED;
}

irq_handler_t hps_timer2_irq_handler(int irq, void *dev_id, struct pt_regs *regs)
{
   /*
    * Service routine for the interruptions generated by timer 2 of the HPS. It implements a chronometer that measures 
    * the advance of time and the display shows the minutes, seconds and tenths of a second elapsed since the start-up. 
    * For this purpose, timer 2 has been configured to interrupt every 100 ms (1 ds), so that when it interrupts 10 times, 
    * 1 s will have elapsed; and after 60 seconds, 1 minute will have elapsed.

    * In order to write in the 7-segment displays, a mask is used in which the encodings of the different digits are written 
    * in 7-segment format. The elapsed seconds are written first and then the decimal places. In order to write the decimal 
    * places in the mask, the seconds must be shifted 8 positions to the left, because the corresponding displays are in the 
    * same register.
    * 
    * To write in the mask, the tens of the number are first encoded, then the tens are added and shifted 8 positions to the left. 
    * Finally, the units are encoded and added.
    * 
    * Finally, the minutes are encoded and written using the same technique described above, but writing in the register corresponding 
    * to HEX5 and HEX4, which is not the same as that of HEX3, HEX2, HEX1 and HEX0.
    */

   int mask = 0;

   ds++;

   if (ds >= 10)
   {
      s++;
      ds = 0;
   }

   if (s >= 60)
   {
      m++;
      s = 0;
   }

   // Add seconds to the mask. First the tens are added and then the units.
   // Add a space after the seconds to add the tenths of a second later.
   mask = seven_segment_digit(s / 10);
   mask = mask << 8;
   mask = mask | seven_segment_digit(s % 10);
   mask = mask << 8;

   // Add tenths of a second to the mask. First the tens are added and then the units.
   mask = mask | seven_segment_digit(ds / 10);
   mask = mask << 8;
   mask = mask | seven_segment_digit(ds % 10);

   // Write mask on the 7-segment display
   *HEX3_HEX0_ptr = mask;

   // Add minutes to the mask. First the tens are added, then the units.
   mask = seven_segment_digit(m / 10);
   mask = mask << 8;
   mask = mask | seven_segment_digit(m % 10);

   // Write mask on the 7-segment display
   *HEX5_HEX4_ptr = mask;

   // Clear the current interrupt by reading the EOI (End Of Interruption) register.
   *(HPS_Timer2_EOI);

   return (irq_handler_t)IRQ_HANDLED;
}

irq_handler_t hps_timer3_irq_handler(int irq, void *dev_id, struct pt_regs *regs)
{
   /*
    * Service routine of the interruptions generated by timer 3 of the HPS. 
    * Generates the PWM signals to be sent to the servomotors. These signals must 
    * have a frequency of 50 Hz, so the generation of the signal is limited to 0.02 seconds, 
    * after this time the process is restarted. This is controlled by comparing the number 
    * of times the attention routine has been executed (ticks) with the number of clock cycles 
    * equivalent to 0.02 s given the frequency of execution of the routine.
    * 
    * To generate the PWM signals, a counter ('amount_angle_servos_travelled_us') is used, 
    * whereby steps of 10 us are taken. This time corresponds to the interrupt time of the timer. 
    * In the variables 'angle_servo_X_us' and 'angle_servo_Y_us' the duty cycle of the PWM signals 
    * is registered in us.
    * 
    * In each run of the service routine, the value of 'amount_angle_servos_travelled_us' is compared 
    * with the value of 'angle_servo_X_us' and 'angle_servo_Y_us'. If the counter is less, a logic 1 
    * is sent, via masks, through the JP1 pins corresponding to the servos. Otherwise, a logic 0 is sent. 
    * 
    * Since the X and Y signals of the accelerometer can have a different duty cycle, the comparison with 
    * 'amount_angle_servos_travelled_us' must be done twice (once for each signal).
    */

   if (control_servo_accelerometer || control_servo_key)
   {
      // Reset count every 0.02 s period
      if (ticks > PWM_CYCLE_SIZE)
      {
         ticks = 0;
         amount_angle_servos_travelled_us = 0;
      }

      // PWM cycle change control
      if (amount_angle_servos_travelled_us < angle_servo_X_us)
      {
         *JP1_ptr = *JP1_ptr | MASK_OR_1_SERVO_X;
      }
      else
      {
         *JP1_ptr = *JP1_ptr & MASK_AND_0_SERVO_X;
      }

      // PWM cycle change control
      if (amount_angle_servos_travelled_us < angle_servo_Y_us)
      {
         *JP1_ptr = *JP1_ptr | MASK_OR_1_SERVO_Y;
      }
      else
      {
         *JP1_ptr = *JP1_ptr & MASK_AND_0_SERVO_Y;
      }

      ticks = ticks + 1;
      amount_angle_servos_travelled_us = amount_angle_servos_travelled_us + 10;
   }

   // Clear the current interrupt by reading the EOI (End Of Interruption) register.
   *(HPS_Timer3_EOI);

   return (irq_handler_t)IRQ_HANDLED;
}

static int __init initialize_handler(void)
{
   // Local variales
   uint32_t value;

   // Initialise global variables
   data_X = 0;
   data_Y = 0;
   high_level_counter_X = 0;
   high_level_counter_Y = 0;
   high_level_perthousand_X = 0;
   high_level_perthousand_Y = 0;
   angle_servo_X_us = 1500;
   angle_servo_Y_us = 1500;
   rising_edge_X = 0;
   rising_edge_Y = 0;
   ticks = 0;
   ds = 0;
   s = 0;
   m = 0;

   // Create virtual address space for FPGA peripherals.
   // LIGHTWEIGHT BRIDGE is used.
   LW_virtual = ioremap_nocache(LW_BRIDGE_BASE, LW_BRIDGE_SPAN);

   // Create virtual address space for HPS timers 2 and 3.
   HPS_Timer2_virtual = ioremap_nocache(HPS_TIMER2, HPS_TIMER2_SPAN);
   HPS_Timer3_virtual = ioremap_nocache(HPS_TIMER3, HPS_TIMER3_SPAN);

   // Initialise LEDR -> save virtual register address
   LEDR_ptr = LW_virtual + LEDR_BASE;
   *LEDR_ptr = 0x200; // Encender LEDR9

   // Initialise 7-segment displays -> save virtual addresses of registers
   HEX3_HEX0_ptr = LW_virtual + HEX3_HEX0_BASE;
   HEX5_HEX4_ptr = LW_virtual + HEX5_HEX4_BASE;

   // Initialise SWITCHes -> save registry virtual address
   SW_ptr = LW_virtual + SW_BASE;

   // Initialise KEYs -> store virtual addresses of the registers
   KEY_ptr = LW_virtual + KEY_BASE;
   KEY_Interrupt_Mask = KEY_ptr + 2;
   KEY_Edgecapture = KEY_ptr + 3;

   // Initialize JP1 -> save virtual register address
   // Set the pins corresponding to the accelerometer as input and those corresponding to the servo motors as output.
   JP1_ptr = LW_virtual + JP1_BASE;
   JP1_Direction = JP1_ptr + 1;
   *JP1_Direction = (*JP1_Direction & MASK_AND_ACCELEROMETER_INPUT) | MASK_OR_SERVOS_OUTPUT;

   // Initialise FPGA_Timer -> save virtual addresses of registers
   FPGA_Timer_ptr = LW_virtual + TIMER_BASE;
   FPGA_Timer_Control = FPGA_Timer_ptr + 1;
   FPGA_Timer_Load_Low = FPGA_Timer_ptr + 2;
   FPGA_Timer_Load_High = FPGA_Timer_ptr + 3;

   // Configure FPGA_Timer
   config_FPGA_Timer();

   // Initialise HPS_Timer2 -> save virtual addresses of registers
   HPS_Timer2_ptr = (uint32_t *)HPS_Timer2_virtual;
   HPS_Timer2_Load = HPS_Timer2_ptr + 0;
   HPS_Timer2_Control = HPS_Timer2_ptr + 2;
   HPS_Timer2_EOI = HPS_Timer2_ptr + 3;

   // Configure Timer2
   config_Timer2();

   // Initialise HPS_Timer3 -> save virtual addresses of registers
   HPS_Timer3_ptr = (uint32_t *)HPS_Timer3_virtual;
   HPS_Timer3_Load = HPS_Timer3_ptr + 0;
   HPS_Timer3_Control = HPS_Timer3_ptr + 2;
   HPS_Timer3_EOI = HPS_Timer3_ptr + 3;

   // Configure Timer3
   config_Timer3();

   // Recording service routines
   value = request_irq(INTERVAL_TIMER_IRQ, (irq_handler_t)fpga_timer_irq_handler, IRQF_SHARED,
                       "fpga_timer_irq_handler", (void *)(fpga_timer_irq_handler));

   value = request_irq(HPS_TIMER2_IRQ, (irq_handler_t)hps_timer2_irq_handler, IRQF_SHARED,
                       "hps_timer2_irq_handler", (void *)(hps_timer2_irq_handler));

   value = request_irq(HPS_TIMER3_IRQ, (irq_handler_t)hps_timer3_irq_handler, IRQF_SHARED,
                       "hps_timer3_irq_handler", (void *)(hps_timer3_irq_handler));

   value = request_irq(KEYS_IRQ, (irq_handler_t)KEY_irq_handler, IRQF_SHARED,
                       "KEY_irq_handler", (void *)(KEY_irq_handler));

   // Enable all interrupts
   // FPGA_Timer interrupts and start-up
   *FPGA_Timer_Control = 0b0111; // STOP->0, START->1, CONT->1, ITO->1

   // HPS_Timer2 interruptions
   *HPS_Timer2_Control = 0b011; // Interrupt mask->0, Mode->1, Enable->1
                                // Mask == 0 Enable interruptions
                                // Mask == 1 Disable interrutions

   // HPS_Timer3 interruptions
   *HPS_Timer3_Control = 0b011; // Interrupt mask->0,Mode->1,Enable->1
                                // Mask == 0 Enable interruptions
                                // Mask == 1 Disable interrutions

   // KEY interruptions
   *KEY_Edgecapture = 0xF;
   *KEY_Interrupt_Mask = 0xF;

   return value;
}

static void __exit cleanup_handler(void)
{
   /*
    * Disable interrupts, turn off LEDRs and 7-segment displays and release service routines.
    */

   // Stop FPGA_Timer and disable interrupts
   *FPGA_Timer_Control = 0b1010; // STOP->1, START->0, CONT->1, ITO->0

   // Disable interrupts of HPS_Timer2 and HPS_Timer3
   *HPS_Timer2_Control = 0b000; // Interrupt mask->0,Mode->0,Enable->0
   *HPS_Timer3_Control = 0b000; // Interrupt mask->0,Mode->0,Enable->0

   // Disable interrupts from KEYs
   *KEY_Interrupt_Mask = 0;

   // Switching off LEDR
   *LEDR_ptr = 0;
   // Switching off 7-segment displays
   *HEX3_HEX0_ptr = 0;
   *HEX5_HEX4_ptr = 0;

   // Releasing service routines
   free_irq(INTERVAL_TIMER_IRQ, (void *)fpga_timer_irq_handler);
   free_irq(HPS_TIMER2_IRQ, (void *)hps_timer2_irq_handler);
   free_irq(HPS_TIMER3_IRQ, (void *)hps_timer3_irq_handler);
   free_irq(KEYS_IRQ, (void *)KEY_irq_handler);
}

module_init(initialize_handler);
module_exit(cleanup_handler);