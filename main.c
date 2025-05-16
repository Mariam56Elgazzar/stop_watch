/*
 * stopwatch2.c
 *
 * Created on: Sep 18, 2024
 * Author: Maryem
 */

#define CLOCK_FREQ 16000000UL // Clock frequency set to 16 MHz

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// Button debounce states
unsigned char prev_state_PB1 = 1; // Previous state of PB1 (Increment Hours)
unsigned char prev_state_PB0 = 1; // Previous state of PB0 (Decrement Hours)
unsigned char prev_state_PB4 = 1; // Previous state of PB4 (Increment Minutes)
unsigned char prev_state_PB3 = 1; // Previous state of PB3 (Decrement Minutes)
unsigned char prev_state_PB6 = 1; // Previous state of PB6 (Increment Seconds)
unsigned char prev_state_PB5 = 1; // Previous state of PB5 (Decrement Seconds)
unsigned char prev_state_PB7 = 1; // Previous state of PB7 (Toggle Button)

// Global variables
unsigned char seven_segment[6] = {0, 0, 0, 0, 0, 0}; // Array for storing the values of each digit on the 7-segment display
unsigned char increment_mode = 0; // Mode of the stopwatch (0: decrement, 1: increment)
unsigned char pause = 0; // Pause state (0: running, 1: paused)
unsigned int tick = 0; // Tick counter for timer

// Timer initialization for a 1-second interval
void timer1_CTCinit(void) {
    TCNT1 = 0; // Reset Timer1 counter
    TCCR1A = (1 << FOC1A) | (1 << FOC1B); // Non-PWM mode
    TCCR1B = (1 << WGM12) | (1 << CS12) | (1 << CS10); // CTC mode, prescaler = 1024
    OCR1A = 15624; // 1-second interval with 16 MHz clock
    TIMSK |= (1 << OCIE1A); // Enable Output Compare A Match Interrupt
}

// Timer1 Output Compare A Match Interrupt Service Routine
ISR(TIMER1_COMPA_vect) {
    if (!pause) {
        tick++;
        if (tick == 1) {  // 1-second tick
            toggle(); // Call toggle function to handle mode switching
            tick = 0;
        }
    }
}

// Toggle between increment and decrement modes after resume long press to toggle
void toggle(void) {
    if (!(PINB & (1 << PB7)) && prev_state_PB7) {  // Check if PB7 is pressed
        _delay_ms(10);  // Debounce delay
        if (!(PINB & (1 << PB7))) {  // Check if still pressed
            increment_mode = !increment_mode; // Toggle mode
            if (increment_mode) {
                PORTD |= (1 << PD4);  // Red LED for increment mode
                PORTD &= ~(1 << PD5); // Turn off yellow LED
            } else {
                PORTD |= (1 << PD5);  // Yellow LED for decrement mode
                PORTD &= ~(1 << PD4); // Turn off red LED
            }
        }
    }
    prev_state_PB7 = (PINB & (1 << PB7));  // Update the previous state

    // Call the appropriate function based on the mode
    if (increment_mode) {
        increment(); // Increment mode
    } else {
        decrement(); // Decrement mode
    }
}

// Increment function for the stopwatch
// Increment function for the stopwatch
void increment(void) {
    increment_mode = 1;
    PORTA = 0xFF; // Select all digits

    // Increment seconds (units)
    if (seven_segment[5] < 9) {
        seven_segment[5]++;
    } else {
        seven_segment[5] = 0;
        // Increment seconds (tens)
        if (seven_segment[4] < 5) {
            seven_segment[4]++;
        } else {
            seven_segment[4] = 0;
            // Increment minutes (units)
            if (seven_segment[3] < 9) {
                seven_segment[3]++;
            } else {
                seven_segment[3] = 0;
                // Increment minutes (tens)
                if (seven_segment[2] < 5) {
                    seven_segment[2]++;
                } else {
                    seven_segment[2] = 0;
                    // Increment hours (units)
                    if (seven_segment[1] < 9) {
                        seven_segment[1]++;
                    } else {
                        seven_segment[1] = 0;
                        // Increment hours (tens)
                        if (seven_segment[0] < 2 || (seven_segment[0] == 2 && seven_segment[1] < 4)) {
                            seven_segment[0]++;
                        } else {
                            seven_segment[0] = 0; // Reset to 00 after 23:59:59
                            seven_segment[1] = 0;
                        }
                    }
                }
            }
        }
    }

    display_all();
}



// Decrement function for the stopwatch
// Decrement function for the stopwatch
// Decrement function for the stopwatch
// Decrement function for the stopwatch
void decrement(void) {
    increment_mode = 0;
    PORTA = 0xFF; // Select all digits

    // Check if time has reached 00:00:00
    if (seven_segment[0] == 0 && seven_segment[1] == 0 &&
        seven_segment[2] == 0 && seven_segment[3] == 0 &&
        seven_segment[4] == 0 && seven_segment[5] == 0) {

        PORTD |= (1 << PD0);  // Turn on buzzer when time reaches 00:00:00
        return;  // Stop decrementing further
    }

    // Decrement seconds (units)
    if (seven_segment[5] > 0) {
        seven_segment[5]--;
    } else {
        seven_segment[5] = 9;
        // Decrement seconds (tens)
        if (seven_segment[4] > 0) {
            seven_segment[4]--;
        } else {
            seven_segment[4] = 5;
            // Decrement minutes (units)
            if (seven_segment[3] > 0) {
                seven_segment[3]--;
            } else {
                seven_segment[3] = 9;
                // Decrement minutes (tens)
                if (seven_segment[2] > 0) {
                    seven_segment[2]--;
                } else {
                    seven_segment[2] = 5;
                    // Decrement hours (units)
                    if (seven_segment[1] > 0) {
                        seven_segment[1]--;
                    } else {
                        seven_segment[1] = 9;
                        // Decrement hours (tens)
                        if (seven_segment[0] > 0 || (seven_segment[0] == 0 && seven_segment[1] > 0)) {
                            seven_segment[0]--;
                        } else {
                            seven_segment[0] = 0; // Keep at 00 if it reaches 00:00:00
                            seven_segment[1] = 0;
                            seven_segment[2] = 0;
                            seven_segment[3] = 0;
                            seven_segment[4] = 0;
                            seven_segment[5] = 0;
                            PORTD |= (1 << PD0);  // Turn on buzzer
                        }
                    }
                }
            }
        }
    }

    display_all();
}


// Display a single digit on the 7-segment display
void display_digit(unsigned char display, unsigned char value) {
    PORTA = (1 << display);  // Select the display digit
    PORTC = (PORTC & 0xF0) | (value & 0x0F);  // Send value to 7-segment
    _delay_ms(1); // Delay for display stability
}

// Update all digits on the display
void display_all(void) {
    display_digit(0, seven_segment[0]);
    display_digit(1, seven_segment[1]);
    display_digit(2, seven_segment[2]);
    display_digit(3, seven_segment[3]);
    display_digit(4, seven_segment[4]);
    display_digit(5, seven_segment[5]);
}

// Interrupt Service Routine for Reset button (connected to PD2)
ISR(INT0_vect) {
	if (!(PIND & (1 << PD2))) {
    _delay_ms(15); // Debounce delay
    if (!(PIND & (1 << PD2))) {  // Reset if still pressed
        for (int i = 0; i < 6; i++) seven_segment[i] = 0; // Reset all time values
        tick = 0; // Reset tick counter
    }

	}
    display_all(); // Update display

}

// Interrupt Service Routine for Pause button (connected to PD3)
ISR(INT1_vect) {
	if (!(PIND & (1 << PD3))){
    _delay_ms(15); // Debounce delay
    if (!(PIND & (1 << PD3))) {pause = !pause; } // Toggle pause state
}}

// Interrupt Service Routine for Resume button (connected to PB2)
ISR(INT2_vect) {
	if (!(PINB & (1 << PB2))){
    _delay_ms(15); // Debounce delay
    if (!(PINB & (1 << PB2))) {pause = 0;}}  // Resume if pressed
}

// Interrupt setup function
void setup_interrupts() {
    MCUCR |= (1 << ISC01);  // Falling edge for INT0 (reset)
    GICR |= (1 << INT0);    // Enable INT0
    MCUCR |= (1 << ISC11) ;  // Falling edge for INT1 (pause/resume)
    GICR |= (1 << INT1);    // Enable INT1
    MCUCSR &= ~(1 << ISC2);  // Falling edge for INT2 (resume)
    GICR |= (1 << INT2);    // Enable INT2
    sei();  // Enable global interrupts
}

// Adjust time function (for adjusting time while paused) it start from 0 it becomes 1 after 2 press
void adjust_time(void) {
    if (pause) {  // Only allow adjustments when paused
        // Increment hours (PB1)
        if (!(PINB & (1 << PB1)) && prev_state_PB1) {
            _delay_ms(15);  // Debounce
            if (!(PINB & (1 << PB1))) {
                if (seven_segment[1] < 9) {
                    seven_segment[1]++;
                } else {
                    seven_segment[1] = 0;
                    if (seven_segment[0] < 2) {
                        seven_segment[0]++;
                    }
                }
            }
            prev_state_PB1 = 0;  // Mark as pressed
        } else if (PINB & (1 << PB1)) {
            prev_state_PB1 = 1;  // Mark as released
        }

        // Decrement hours (PB0)
        if (!(PINB & (1 << PB0)) && prev_state_PB0) {
            _delay_ms(15);  // Debounce
            if (!(PINB & (1 << PB0))) {
                if (seven_segment[1] > 0) {
                    seven_segment[1]--;
                } else {
                    seven_segment[1] = 9;
                    if (seven_segment[0] > 0) {
                        seven_segment[0]--;
                    }
                }
            }
            prev_state_PB0 = 0;  // Mark as pressed
        } else if (PINB & (1 << PB0)) {
            prev_state_PB0 = 1;  // Mark as released
        }

        // Increment minutes (PB4)
        if (!(PINB & (1 << PB4)) && prev_state_PB4) {
            _delay_ms(15);  // Debounce
            if (!(PINB & (1 << PB4))) {
                if (seven_segment[3] < 9) {
                    seven_segment[3]++;
                } else {
                    seven_segment[3] = 0;
                    if (seven_segment[2] < 5) {
                        seven_segment[2]++;
                    }
                }
            }
            prev_state_PB4 = 0;  // Mark as pressed
        } else if (PINB & (1 << PB4)) {
            prev_state_PB4 = 1;  // Mark as released
        }

        // Decrement minutes (PB3)
        if (!(PINB & (1 << PB3)) && prev_state_PB3) {
            _delay_ms(15);  // Debounce
            if (!(PINB & (1 << PB3))) {
                if (seven_segment[3] > 0) {
                    seven_segment[3]--;
                } else {
                    seven_segment[3] = 9;
                    if (seven_segment[2] > 0) {
                        seven_segment[2]--;
                    }
                }
            }
            prev_state_PB3 = 0;  // Mark as pressed
        } else if (PINB & (1 << PB3)) {
            prev_state_PB3 = 1;  // Mark as released
        }

        // Increment seconds (PB6)
        if (!(PINB & (1 << PB6)) && prev_state_PB6) {
            _delay_ms(15);  // Debounce
            if (!(PINB & (1 << PB6))) {
                if (seven_segment[5] < 9) {
                    seven_segment[5]++;
                } else {
                    seven_segment[5] = 0;
                    if (seven_segment[4] < 5) {
                        seven_segment[4]++;
                    }
                }
            }
            prev_state_PB6 = 0;  // Mark as pressed
        } else if (PINB & (1 << PB6)) {
            prev_state_PB6 = 1;  // Mark as released
        }

        // Decrement seconds (PB5)
        if (!(PINB & (1 << PB5)) && prev_state_PB5) {
            _delay_ms(15);  // Debounce
            if (!(PINB & (1 << PB5))) {
                if (seven_segment[5] > 0) {
                    seven_segment[5]--;
                } else {
                    seven_segment[5] = 9;
                    if (seven_segment[4] > 0) {
                        seven_segment[4]--;
                    }
                }
            }
            prev_state_PB5 = 0;  // Mark as pressed
        } else if (PINB & (1 << PB5)) {
            prev_state_PB5 = 1;  // Mark as released
        }

         // Update the display after each adjustment
    }
    display_all();
}

// Main function
int main() {
    // Seven-segment display setup
    DDRC |= 0x0F; // Set lower 4 bits of PORTC as output for 7-segment display
    DDRA |= 0x3F; // Set lower 6 bits of PORTA as output for digit selection

    // Setup buttons as inputs
    DDRD &= ~(1 << PD2);  // Reset button input
    DDRD &= ~(1 << PD3);  // Pause/Resume button input
    DDRB &= ~(1 << PB2);  // Resume button input
    DDRB &= ~(1 << PB7);  // Toggle button input
    DDRB &= ~(1 << PB1);  // Increment Hours button input
    DDRB &= ~(1 << PB0);  // Decrement Hours button input
    DDRB &= ~(1 << PB4);  // Increment Minutes button input
    DDRB &= ~(1 << PB3);  // Decrement Minutes button input
    DDRB &= ~(1 << PB6);  // Increment Seconds button input
    DDRB &= ~(1 << PB5);  // Decrement Seconds button input

    // Output for buzzer and LEDs
    DDRD |= (1 << PD0);  // Buzzer output
    DDRD |= (1 << PD4);  // Red LED output (increment mode)
    DDRD |= (1 << PD5);  // Yellow LED output (decrement mode)

    // Initialize interrupts and timer
    setup_interrupts();
    timer1_CTCinit();

    // Main loop
    while (1) {
        display_all();  // Continuously update the display
        adjust_time();  // Allow time adjustment while paused
    }

    return 0;
}
