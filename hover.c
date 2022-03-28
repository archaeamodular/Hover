//----------------------------------------------------------------------------------------//
//
//  Hover Hand Distance Sensor CV and Gate Controller firmware
//
//    Copyright 2022 Arran Derbyshire
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <https://www.gnu.org/licenses/>.
//
//    arran@archaea.co.uk
//
//----------------------------------------------------------------------------------------//

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdbool.h>

#include "dbg_putchar.h"
#include "dbg_putstring.h"

//----------------------------------------------------------------------------------------//
// Build configuration
//----------------------------------------------------------------------------------------//
// build configuration definitions in the Makefile:
// F_CPU: 1000000/8000000, the system clock frequency in Hz (and need to change fuse bits to change CPU clock)
// DEBUG_LOG: 0/1, debug logging via the software serial port

#define EEPROM_SIZE 512                 // number of EEPROM locations used to save state (for wear levelling)
#define MANAGE_STATE 1                  // 0/1, load and save state
#define DEBUG_LOG_IO 0                  // 0/1, log the device IO, outputs a lot of logging, showing sensor, CV data etc.
#define DEBUG_NUM_INTERRUPTS_LOG_IO 10  // number of interrupts interval to log IO
#define CALIBRATE_OSCILLOSCOPE 0        // 0/1, generate a ramping waveform for CV calibration using an oscilloscope
#define CALIBRATE_MULTIMETER 1          // 0/1, generate high output on run and low output on stop for CV calibration using a multimeter

//----------------------------------------------------------------------------------------//
// Macro definitions
//----------------------------------------------------------------------------------------//
#if DEBUG_LOG
#pragma message("DEBUG LOGGING ENABLED")
#define DBG_LOG(s) dbg_putstring1(PSTR(s),0)   // store logging strings in program memory
#define DBG_LOG1(s,a) dbg_putstring1(PSTR(s),a)
#else
#define DBG_LOG(s)
#define DBG_LOG1(s,a)
#endif
    
#define SET_PIN(pt, p) pt|=(1<<p)
#define CLR_PIN(pt, p) pt&=~(1<<p)

//----------------------------------------------------------------------------------------//
// IO definitions
//----------------------------------------------------------------------------------------//
#define LED_SHIFT_PORT          PORTA
#define LED_SHIFT_DDR           DDRA
#define ACTION_REDLED_PORT      PORTA
#define ACTION_REDLED_DDR       DDRA
#define ACTION_GREENLED_PORT    PORTA
#define ACTION_GREENLED_DDR     DDRA

#define ACTION_REDLED_PIN       PINA0
#define ACTION_GREENLED_PIN     PINA1
#define AREF_PIN                PINA3
#define LED_DATA_PIN            PINA5
#define LED_SHIFT_CLK_PIN       PINA6
#define LED_STORE_CLK_PIN       PINA7

#define GATE_PORT               PORTB
#define GATE_DDR                DDRB
#define CVPWM_DDR               DDRB
#define MODE_BUTTON_PORT        PORTB
#define MODE_BUTTON_DDR         DDRB
#define ACTION_BUTTON_PORT      PORTB
#define ACTION_BUTTON_DDR       DDRB
#define THRESHOLD_DDR           DDRB
#define IRSENSOR_DDR            DDRB

#define GATE_PIN                PINB0
#define CVPWM_PIN               PINB1
#define MODE_BUTTON_PIN         PINB2
#define ACTION_BUTTON_PIN       PINB3
#define THRESHOLD_PIN           PINB4
#define IRSENSOR_PIN            PINB5
#define DEBUGLOG_PIN            PINB6

#define THRESHOLD_ADC_CHAN      7
#define IRSENSOR_ADC_CHAN       8

//----------------------------------------------------------------------------------------//
// Constants
//----------------------------------------------------------------------------------------//
#define DEBOUNCE_TIME                   1000    // debounce time in us
#define INVERT_HOLD_TIME                83      // 83*12ms = 1s
#define INVERT_CYCLE_TIME               41      // 41*12ms = 0.5s
#define MODE_SAVE_AFTER_RELEASE_TIME    250     // 250*12ms = 3s
#define MIN_IR                          130     // minimum IR sample value before sensor noise floor, determines the maximum distance the sensor will sense a hand (130 -> 35cm)
#define THRESHOLD_HYSTERESIS            10      // hysteresis size at the gate threshold between gate on and gate off

//----------------------------------------------------------------------------------------//
// States
//----------------------------------------------------------------------------------------//
#define MODE_TOGGLE                 0
#define MODE_HOLD                   1
#define MODE_ZERO                   2
#define MODE_SAMPLE                 3
#define MODE_GATE                   4
#define MODE_CALIBRATE              5

#define MODE_BUTTON_STANDBY         0
#define MODE_BUTTON_PRESSED         1
#define MODE_BUTTON_HOLD_INVERT     2
#define MODE_BUTTON_CYCLE_INVERT    3
#define MODE_BUTTON_ACTION_PRESSED  4

#define GATE_OUTPUT                 0
#define GATE_INPUT                  1

//----------------------------------------------------------------------------------------//
// Global variable defintions
//----------------------------------------------------------------------------------------//
uint8_t *stateAddress;
volatile bool signalSaveState;
uint8_t mode;
bool modeState[6];
uint8_t modeButtonState;
bool actionButtonState;
bool actionState;
bool invertCV;
bool invertGate;
uint8_t gateDirection;
bool run;
volatile uint16_t cv;
volatile uint16_t ir;
volatile uint16_t irLastRun;
volatile uint16_t threshold;
volatile bool gate;
volatile uint16_t modeHoldTime;
uint16_t modeReleaseTime;
bool redLED;
bool greenLED;
volatile uint8_t redLEDTime;
uint8_t redLEDPeriod;
volatile uint8_t greenLEDTime;
uint8_t greenLEDPeriod;
bool LEDTimerRunning;
#if DEBUG_LOG_IO
volatile uint16_t debugIOTime;
#endif // DEBUG_LOG_IO
bool calibration;
volatile uint16_t irCalibrate;
uint8_t bargraphState;

//----------------------------------------------------------------------------------------//
// Action LEDs definitions for each of the modes and for gate direction.
//
// redLED: light the red action LED (starting state for toggling)
// redLEDPeriod: period (x12ms) between toggling red action LED
// greenLED: light the green action LED (starting state for toggling)
// greenLEDPeriod: period (x12ms) between toggling green action LED
struct actionLEDColour{bool redLED; uint8_t redLEDPeriod;
                       bool greenLED; uint8_t greenLEDPeriod;};
struct actionLEDColour actionLEDColourMode[2][6][2] =
    {
        // gate output
        {   //       off                    on                     mode
            { {true,0,false,0},     {false,0,true,42}   },      // toggle
            { {true,42,false,0},    {false,0,true,0}    },      // hold
            { {true,42,false,0},    {false,0,true,0}    },      // zero
            { {true,0,false,0},     {false,10,true,10}  },      // sample, flashes once only
            { {true,0,false,0},     {false,0,true,0}    },      // gate
            { {true,0,false,0},     {false,0,true,0}    }       // calibrate
        },
        // gate input
        {   //       off                    on                     mode
            { {true,0,true,0},      {false,0,true,42}   },      // toggle
            { {true,42,true,42},    {false,0,true,0}    },      // hold
            { {true,42,true,42},    {false,0,true,0}    },      // zero
            { {true,0,true,0},      {false,10,true,0}   },      // sample, flashes once only
            { {true,0,true,0},      {false,0,true,0}    },      // gate
            { {true,0,false,0},     {false,0,true,0}    }       // calibrate
        }
    };

//----------------------------------------------------------------------------------------//
// Function declarations
//----------------------------------------------------------------------------------------//
void setCV(uint16_t _ir);

//----------------------------------------------------------------------------------------//
// saveState:
// Save the main mode state variables to eeprom so the operating mode can be restored
// when the power is turned on.
//----------------------------------------------------------------------------------------//
void saveState(void)
{
#if MANAGE_STATE
    uint8_t state = 0;

    eeprom_update_byte(stateAddress++, 0xff);
    if(stateAddress>=(uint8_t *)EEPROM_SIZE)
        stateAddress=0;

    state = (gateDirection<<6) | (invertCV<<5) | (invertGate<<4) | mode;
    eeprom_update_byte(stateAddress, state);

    DBG_LOG1("Saved state=%02x ", state);
    DBG_LOG1("addr=%d\n", (uint16_t)stateAddress);
#endif // MANAGE_STATE
}

//----------------------------------------------------------------------------------------//
// checkSaveState:
// Check if the timer interrupt has signalled that the state should be saved.
//----------------------------------------------------------------------------------------//
void checkSaveState(void)
{
    if(signalSaveState)
    {
        saveState();
        signalSaveState = false;
    }
}

//----------------------------------------------------------------------------------------//
// loadState:
// Load the main mode state variables from eeprom.
//----------------------------------------------------------------------------------------//
void loadState(void)
{
#if MANAGE_STATE
    uint8_t state = 0;
    uint16_t i=0;

    stateAddress = (uint8_t *)0;
    
    for(i=0;i<EEPROM_SIZE;i++)
    {
        state = eeprom_read_byte(stateAddress);
        if(state!=0xff)
        {
            gateDirection = (state & 0x40)>>6;
            invertCV = (state & 0x20)>>5;
            invertGate = (state & 0x10)>>4;
            mode = state & 0x07;

            DBG_LOG1("Recalled state=%02x ", state);
            DBG_LOG1("addr=%d\n", (uint16_t)stateAddress);
            DBG_LOG1("mode=%d ", mode);
            DBG_LOG1("invertCV=%d ", invertCV);
            DBG_LOG1("invertGate=%d ", invertGate);
            DBG_LOG1("gateDirection=%d\n", gateDirection);
            return;
        }
        stateAddress++;
    }
#endif // MANAGE_STATE
    DBG_LOG("No saved state, starting with default.\n");
}

//----------------------------------------------------------------------------------------//
// switchPressed:
// Check if a switch pin has gone low to indicate a connected switch has been pressed.
// The pin is read twice with a delay to filter out switch bounce.
// Returns true if a switch has been pressed, otherwise false.
//----------------------------------------------------------------------------------------//
bool switchPressed(uint8_t switchPin)
{
    if(bit_is_clear(PINB, switchPin))
    {
        _delay_us(DEBOUNCE_TIME);
        if(bit_is_clear(PINB, switchPin))
            return true;
    }
    return false;
}

//----------------------------------------------------------------------------------------//
// switchPressedReleased:
// Check if a switch has been pressed or released by reading the pin connected to the
// switch and comparing with the previous switch state. Switch bounce is filtered by
// reading the pin twice with a delay.
// Updates the button state and returns true if the switch has changed state,
// otherwise false.
//----------------------------------------------------------------------------------------//
bool switchPressedReleased(uint8_t switchPin, bool *buttonState)
{
    if(!*buttonState)
    {
        // has button been pressed?
        if(bit_is_clear(PINB, switchPin))
        {
            _delay_us(DEBOUNCE_TIME);
            if(bit_is_clear(PINB, switchPin))
            {
                *buttonState=true;  // button has been pressed
                return true;
            }
        }
    }
    else
    {
        // has button been released?
        if(bit_is_set(PINB, switchPin))
        {
            _delay_us(DEBOUNCE_TIME);
            if(bit_is_set(PINB, switchPin))
            {
                *buttonState=false;  // button has been released
                return true;
            }
        }
    }
    return false;
}

//----------------------------------------------------------------------------------------//
// pulseShiftClock:
// Send a pulse to the LED shift register (HC595) shift clock pin to shift one bit into
// the register.
//----------------------------------------------------------------------------------------//
void pulseShiftClock(void)
{
    SET_PIN(LED_SHIFT_PORT, LED_SHIFT_CLK_PIN);
    CLR_PIN(LED_SHIFT_PORT, LED_SHIFT_CLK_PIN);
}

//----------------------------------------------------------------------------------------//
// pulseStoreClock:
// Send a pulse to the LED shift register (HC595) store clock pin to clock the shift
// register data to the parallel output pins.
//----------------------------------------------------------------------------------------//
void pulseStoreClock(void)
{
    SET_PIN(LED_SHIFT_PORT, LED_STORE_CLK_PIN);
    _delay_loop_1(1);
    CLR_PIN(LED_SHIFT_PORT, LED_STORE_CLK_PIN);
    _delay_loop_1(1);
}

//----------------------------------------------------------------------------------------//
// writeLEDShiftRegister:
// Write 8-bit data value to the LED shift register (HC595) to light the mode LEDs.
//----------------------------------------------------------------------------------------//
void writeLEDShiftRegister(uint8_t LEDData)
{
    for(uint8_t i=0;i<8;i++)
    {
        if(LEDData&0x80)
            SET_PIN(LED_SHIFT_PORT, LED_DATA_PIN);
        else
            CLR_PIN(LED_SHIFT_PORT, LED_DATA_PIN);
        
        pulseShiftClock();
        LEDData<<=1;
    }
    
    pulseStoreClock();
}

//----------------------------------------------------------------------------------------//
// setModeInvertLEDs:
// Turn the mode LEDs on or off according to the current mode, inverted CV and inverted Gate
// states.
//----------------------------------------------------------------------------------------//
void setModeInvertLEDs(void)
{   
    uint8_t LEDData = 0;

    LEDData = 0x01 << mode;
    LEDData |= invertCV << 5;
    LEDData |= invertGate << 6;
    
    writeLEDShiftRegister(LEDData);
    
}

//----------------------------------------------------------------------------------------//
// setActionLEDState:
// Set the action button LED state, whether on/off and the period between toggling to flash
// the LEDs. If the period is 0, the LEDs will be constant on/off, otherwise will flash.
//----------------------------------------------------------------------------------------//
void setActionLEDState(bool _redLED, uint8_t _redLEDPeriod, \
                       bool _greenLED, uint8_t _greenLEDPeriod)
{
    if(_redLED)
        SET_PIN(ACTION_REDLED_PORT, ACTION_REDLED_PIN);
    else
        CLR_PIN(ACTION_REDLED_PORT, ACTION_REDLED_PIN);

    if(_greenLED)
        SET_PIN(ACTION_GREENLED_PORT, ACTION_GREENLED_PIN);
    else
        CLR_PIN(ACTION_GREENLED_PORT, ACTION_GREENLED_PIN);

    redLED = _redLED;
    redLEDPeriod = _redLEDPeriod;
    
    greenLED = _greenLED;
    greenLEDPeriod = _greenLEDPeriod;

}

//----------------------------------------------------------------------------------------//
// setActionLEDs:
// Set the action switch LED state according to the mode, mode state and gate direction.
//----------------------------------------------------------------------------------------//
void setActionLEDs(void)
{   
    redLEDTime = 0;
    greenLEDTime = 0;
    LEDTimerRunning = true;

    struct actionLEDColour _actionLEDColour;

    _actionLEDColour = actionLEDColourMode[gateDirection][mode][modeState[mode]];

    setActionLEDState(_actionLEDColour.redLED, _actionLEDColour.redLEDPeriod,
                      _actionLEDColour.greenLED, _actionLEDColour.greenLEDPeriod);
    
}

//----------------------------------------------------------------------------------------//
// setGatePin:
// Output the gate on the gate pin. Used only when the gate direction is output.
// The gate output is inverted when the gate inversion mode is selected.
//----------------------------------------------------------------------------------------//
void setGatePin(void)
{
    bool gateOutput=gate^invertGate;

    if(gateOutput)
        SET_PIN(GATE_PORT, GATE_PIN);
    else
        CLR_PIN(GATE_PORT, GATE_PIN);
}
    
//----------------------------------------------------------------------------------------//
// setGatePinDirection:
// Set the gate pin to either output the gate or input the gate signal.
//----------------------------------------------------------------------------------------//
void setGatePinDirection(void)
{
    if(gateDirection==GATE_OUTPUT)
    {
        DBG_LOG("Gate output\n");
        GATE_DDR |= 1 << GATE_PIN;              // set gate pin to output
        setGatePin();
    }
    else
    {
        DBG_LOG("Gate input\n");
        GATE_DDR &= ~(1 << GATE_PIN);           // set gate pin to input
        GATE_PORT |= (1 << GATE_PIN);           // enable mode switch pin pull-up
    }
    modeState[mode] = false;                    // clear the mode state

}

//----------------------------------------------------------------------------------------//
// setInvertPWM:
// Set the timer 1 registers to configure the PWM output to be inverted.
// This flips the analog CV output upside down so hand movements are either
// tracked or mirrored by the CV.
//----------------------------------------------------------------------------------------//
static inline void setInvertPWM(void)
{
    if(invertCV)
    {
        TCCR1A |= (1 << COM1A0);    // output on OC1A, set on compare match
        TCCR1A |= (1 << COM1A1);    // output on OC1A, set on compare match
    }
    else
    {
        TCCR1A &= ~(1 << COM1A0);   // output on OC1A, clear on compare match
        TCCR1A |= (1 << COM1A1);    // output on OC1A, clear on compare match
    }

}

//----------------------------------------------------------------------------------------//
// updateInvertState:
// Update the invert state by moving to the next combination of inverted CV and inverted gate.
//----------------------------------------------------------------------------------------//
void updateInvertState(void)
{
    uint8_t invertMode = (invertGate<<1) | invertCV;

    invertMode++;
    if(invertMode>3)
        invertMode=0;
    
    invertCV=invertMode&0x1;
    invertGate=invertMode>>1;

    setModeInvertLEDs();
    setInvertPWM();
    if(!run)
        setCV(irLastRun);

    DBG_LOG1("Last IR=%d ",irLastRun);
    DBG_LOG1("Invert CV=%d ", invertCV);
    DBG_LOG1("Gate=%d\n", invertGate);
}

//----------------------------------------------------------------------------------------//
// setModeRunState:
// Set the run state according to the current mode state.
//----------------------------------------------------------------------------------------//
void setModeRunState(void)
{
    switch (mode)
    {
        case MODE_TOGGLE:
            run = modeState[mode];
            break;
        case MODE_HOLD:
            run = actionState;
            break;
        case MODE_ZERO:
            if(!actionState)
            {
                run = false;
                setCV(0);
            }
            else
            {
                run = true;
            }
            break;
        case MODE_SAMPLE:
            run = false;
            break;
        case MODE_GATE:
            run = true;
            break;
        case MODE_CALIBRATE:
            run = modeState[mode];
            break;
        default:
            run = false;
            break;
    }
            
}

//----------------------------------------------------------------------------------------//
// checkModeSwitch:
// Check if the mode switch has been pressed or held down. A short press (<1sec) will change
// the mode, and a long press (>1sec) will change the invert mode. Holding the mode button
// (>2sec) will cycle through the invert modes. The time between mode button releases is
// counted and if the counter goes higher than 3sec then the state is saved.
//----------------------------------------------------------------------------------------//
void checkModeSwitch(void)
{
    if(switchPressed(MODE_BUTTON_PIN))
    {
        if(modeButtonState==MODE_BUTTON_STANDBY)
        {
            // button has been pressed
            modeButtonState = MODE_BUTTON_PRESSED;
            modeHoldTime = 0;
        }
        else
        {
            if(modeHoldTime>=INVERT_HOLD_TIME && modeButtonState==MODE_BUTTON_PRESSED)
            {
                updateInvertState();
                modeButtonState = MODE_BUTTON_HOLD_INVERT;
                modeHoldTime = 0;
            }
        }
    }
    else
    {
        if(modeButtonState==MODE_BUTTON_PRESSED)
        {
            if(modeHoldTime<INVERT_HOLD_TIME)
            {
                // button has been released after short click

                // clear the current mode state in case the action button is being held
                // before changing mode
                modeState[mode] = false;

                // change mode
                mode++;
                if(mode>4)
                    mode = 0;
                
                DBG_LOG1("Mode %d\n", mode);

                setModeInvertLEDs();
                setActionLEDs();
                setModeRunState();

                // reset the count down to save the state to eeprom
                modeReleaseTime = MODE_SAVE_AFTER_RELEASE_TIME;
            }
        }
        if(modeButtonState==MODE_BUTTON_HOLD_INVERT || modeButtonState==MODE_BUTTON_CYCLE_INVERT)
        {
            // reset the count down to save the state to eeprom
            modeReleaseTime = MODE_SAVE_AFTER_RELEASE_TIME;
        }
        modeButtonState = MODE_BUTTON_STANDBY;

    }

}

//----------------------------------------------------------------------------------------//
// handleActionEdge:
// Handle an edge in the action state, either the action button has been pressed/released,
// or the gate input has gone high/low. The response depends on the current mode.
//----------------------------------------------------------------------------------------//
void handleActionEdge(bool action)
{
    if(modeButtonState==MODE_BUTTON_STANDBY)
    {
        switch (mode)
        {
            case MODE_TOGGLE:
                // toggle run/hold
                if(action)
                {
                    modeState[mode] = !modeState[mode]; // toggle the mode state
                    DBG_LOG1("....Mode Toggle: state=%d\n", modeState[mode]);

                    if(!modeState[mode])
                        run = false;            // stop, hold sample
                    else
                        run = true;             // run, pass samples

                    setActionLEDs();
                }
                break;
            case MODE_HOLD:
                // momentary run/hold
                if(action)
                {
                    run = true;
                    modeState[mode] = true;

                    DBG_LOG("....Mode Hold: state=1 run\n");
                }
                else
                {
                    run = false;
                    modeState[mode] = false;

                    DBG_LOG("....Mode Hold: state=0 hold\n");
                }
                setActionLEDs();
                break;
            case MODE_ZERO:
                // momentary run/zero
                if(action)
                {
                    run = true;
                    modeState[mode] = true;
                    DBG_LOG("....Mode Zero: state=1 run\n");
                }
                else
                {
                    run = false;
                    setCV(0);
                    modeState[mode] = false;
                    DBG_LOG("....Mode Zero: state=0 zero\n");
                }
                setActionLEDs();
                break;
            case MODE_SAMPLE:
                // sample and hold
                if(action)
                {
                    irLastRun = ir;
                    setCV(irLastRun);           // sample IR sensor on button press
                    modeState[mode] = true;
                    setActionLEDs();

                    DBG_LOG("....Mode Sample: sample\n");
                }
                else
                {
                    modeState[mode] = false;
                }
                break;
            case MODE_GATE:
                // run, action switch controls gate when gate direction is output
                if(action)
                {
                    modeState[mode] = true;
                    if(gateDirection == GATE_OUTPUT)
                    {
                        gate = true;
                        setGatePin();
                    }
                }
                else
                {
                    modeState[mode] = false;
                    if(gateDirection == GATE_OUTPUT)
                    {
                        gate = false;
                        setGatePin();
                    }
                }
                setActionLEDs();
                DBG_LOG1("....Mode Gate: gate=%d\n", gate);
                break;
            case MODE_CALIBRATE:
                // toggle high/low
                if(action)
                {
                    modeState[mode] = !modeState[mode]; // toggle the mode state
                    DBG_LOG1("....Mode Calibrate: state=%d\n", modeState[mode]);

                    if(!modeState[mode])
                        run = false;            // stop, hold sample
                    else
                        run = true;             // run, pass samples

                    setActionLEDs();
                }
                break;
            default:
                // stop, hold sample
                run = false;
                break;
        }
    }

}

//----------------------------------------------------------------------------------------//
// checkActionSwitchAndGateInput:
// Check if either the action button has been pressed/released or the gate input has gone
// high/low. The action state is the action button state OR the gate input state.
// If the mode button was being held while the action button is pressed, then change the
// gate direction.
//----------------------------------------------------------------------------------------//
void checkActionSwitchAndGateInput(void)
{

    bool action = false;
    bool actionButtonEdge = false;
    bool prevActionButtonState = actionButtonState;
    
    // control the action with the action switch press
    actionButtonEdge = switchPressedReleased(ACTION_BUTTON_PIN, &actionButtonState);

    if(modeButtonState==MODE_BUTTON_PRESSED && actionButtonEdge && !actionButtonState)
    {
        // action button was pressed before mode button released
        modeButtonState = MODE_BUTTON_ACTION_PRESSED;

        // toggle gate direction
        gateDirection ^= 1;
        setGatePinDirection();
        setActionLEDs();
        // clear action state for new gate mode
        actionState = false;
        // reset the count down to save the state to eeprom
        modeReleaseTime=MODE_SAVE_AFTER_RELEASE_TIME;
    }

    action = actionButtonState;

    // if gate direction is input, then also control
    // the action with the gate input
    if(gateDirection == GATE_INPUT)
        action |= bit_is_set(PINB, GATE_PIN);

    // on an action state edge, or action button press, handle the action
    // this allows button presses to still be registered when the gate is high
    if(action!=actionState || (!prevActionButtonState && actionButtonState))
    {
        actionState=action;
        handleActionEdge(action);
    }
    
}

//----------------------------------------------------------------------------------------//
// readADC:
// Read an analog-to-digital convertor channel.
//----------------------------------------------------------------------------------------//
uint16_t readADC(uint8_t channel)
{
    ADMUX = (0xf0 & ADMUX) | channel;       // select ADC channel from mux
    ADCSRA |= (1 << ADSC);                  // start conversion
    loop_until_bit_is_clear(ADCSRA, ADSC);  // wait for conversion to complete
    return ADC;
}

//----------------------------------------------------------------------------------------//
// calibrate:
// Run in calibration mode to calibrate the CV PWM output to give 0..10V at the CV
// analog output while adjusting the width and offset presets on the board.
// Define CALIBRATE_MULTIMETER to measure CV to be 0V when not running and 10V when
// running using a multimeter (most accurate).
// Define CALIBRATE_OSCILLOSCOPE to get the CV ramp waveform within the 0..10V
// limits (quickest).
// Lights the mode LEDs to show when the sensor has reached its maximum output value
// allowing the sensor preset on the board to be calibrated.
//----------------------------------------------------------------------------------------//
void calibrate(void)
{

    // set the infrared value to its highest/lowest values
    // to in turn set the CV output to its limits
#if CALIBRATE_MULTIMETER
    if(run)
        ir=0;
    else
        ir=1023;
#endif // CALIBRATE_MULTIMETER

#if CALIBRATE_OSCILLOSCOPE
    ir = 16*irCalibrate;
    irCalibrate++;
    if(irCalibrate>72)
        irCalibrate=0;
    if(ir>1023)
        ir=1023;
#endif // CALIBRATE_OSCILLOSCOPE
    
    setCV(ir);
    irLastRun = ir;

    // read the sensor analog input
    ir = readADC(IRSENSOR_ADC_CHAN);

    // update the mode LEDs to display the sensor input as a bar graph
    uint8_t leds = 0b01100000;
    // light the LEDs one at a time as IR increases,
    // with IR max. value showing all as 5 LEDs lit
    uint16_t currentThreshold = 1022-((4-bargraphState)*200);
    
    if(ir<(currentThreshold-200-THRESHOLD_HYSTERESIS) && bargraphState>0)
    {
        bargraphState--;
    }
    else
    {
        if(ir>currentThreshold && bargraphState<5)
        {
            bargraphState++;
        }
    }
    
    leds |= ((0b11111)>>(5-bargraphState));

    writeLEDShiftRegister(leds);

    
}

//----------------------------------------------------------------------------------------//
// setCV:
// Set the CV value allowing for the minimum IR sensor noise floor, set by MIN_IR.
// The CV PWM signal range MIN_IR..1023 is mapped to 0-10V when the CV is not inverted.
// Sensor data below MIN_IR is clamped to MIN_IR.
// When inverted, the CV PWM is flipped and the range 0..1023-MIN_IR is mapped to 0..10V.
//----------------------------------------------------------------------------------------//
void setCV(uint16_t _ir)
{
    if(invertCV)
        if(_ir>MIN_IR)
            cv = _ir-MIN_IR;
        else
            cv = 0;
    else
        if(_ir>MIN_IR)
            cv = _ir;
        else
            cv = MIN_IR;
}

//----------------------------------------------------------------------------------------//
// ISR: timer 0
// The interrupt service routine for timer 0. This is called every ~12ms to
// sample the IR sensor, sample the gate threshold knob and update the CV and gate
// outputs. It also handles all timing by updating counters. The frequency
// matches approximately the sample frequency of the Sharp GP2Y0A41SK0F IR sensor.
//----------------------------------------------------------------------------------------//
ISR(TIMER0_COMPA_vect)
{
    uint16_t irLimited = 0;

    if(calibration)
    {
        calibrate();
    }
    else
    {
        // read the sensor analog input
        ir = readADC(IRSENSOR_ADC_CHAN);
        
        if(run)
        {
            // if running, then update the CV PWM output
            setCV(ir);
            // keep the ir value in case the CV must be updated when no
            // longer in run mode, e.g. when inverting the CV
            irLastRun = ir;
        }
    }

    // set the CV output by writing the PWM value
    TC1H = (uint8_t)(cv >> 8);      // set 2 MSBs of 10-bit value
    OCR1A = (uint8_t)(0xff & cv);   // set 8 LSBs of 10-bit value

    // read and invert the gate knob analog input to get the threshold
    // for the gate output
    threshold = 1023 - readADC(THRESHOLD_ADC_CHAN);

    // calculate the upper threshold for the maximum knob setting (turned to right)
    if(threshold<MIN_IR-THRESHOLD_HYSTERESIS)
        threshold = MIN_IR-THRESHOLD_HYSTERESIS;

    // calculate the lower threshold for the minimum knob setting (turned to left)
    // gate never on at minimum setting on knob
    if(threshold>1023-THRESHOLD_HYSTERESIS)
        threshold = 1023-THRESHOLD_HYSTERESIS;

    // limit the IR sensor output to above the sensor noise floor
    // (setting the effective maximum sensor range), allowing for the hysteresis
    if(ir<MIN_IR-THRESHOLD_HYSTERESIS)
        irLimited = MIN_IR-THRESHOLD_HYSTERESIS;
    else
        irLimited = ir;

    // detect if the IR sensor is above the threshold with hysteresis and
    // then set the gate output
    if(gateDirection == GATE_OUTPUT)
    {
        if(mode!=MODE_GATE)
        {
            if(gate)
            {
                if(irLimited<=threshold)
                    gate = false;
            }
            else
            {
                if(irLimited>threshold+THRESHOLD_HYSTERESIS)
                    gate = true;
            }
            if(threshold==MIN_IR-THRESHOLD_HYSTERESIS)
            {
                gate = true;    // force gate always on at maximum setting on knob
            }
            setGatePin();
        }
    }

    // count the time the mode button is pressed
    modeHoldTime++;
    if(modeButtonState==MODE_BUTTON_HOLD_INVERT)
    {
        // mode button is being held down long enough to have moved to the next
        // invert mode
        if(modeHoldTime>INVERT_HOLD_TIME)
        {
            // still being held, so move to next invert mode again
            updateInvertState();
            modeHoldTime = 0;
            // now start to cycle continuously through the invert modes
            modeButtonState = MODE_BUTTON_CYCLE_INVERT;
        }
    }
    // cycle through the invert modes
    if(modeButtonState==MODE_BUTTON_CYCLE_INVERT)
    {
        if(modeHoldTime>INVERT_CYCLE_TIME)
        {
            updateInvertState();
            modeHoldTime = 0;
        }
    }
    
    // count when to save the state a time after the mode button is released
    if(modeReleaseTime>0)
    {
        modeReleaseTime--;
        if(modeReleaseTime==0)
            // signal the main run loop to do the state saving process
            signalSaveState = true;
    }

    // count when to toggle the action button LEDs to make them flash
    if(LEDTimerRunning)
    {
        redLEDTime++;
        greenLEDTime++;
    }

    // flash red action button LED
    if(redLEDTime>=redLEDPeriod && redLEDPeriod!=0)
    {
        redLEDTime = 0;
        redLED = !redLED;
        if(redLED)
            ACTION_REDLED_PORT |=  (1 << ACTION_REDLED_PIN);     // red LED on
        else
            ACTION_REDLED_PORT &= ~(1 << ACTION_REDLED_PIN);     // red LED off

        if(mode==MODE_SAMPLE)
            LEDTimerRunning=false;
    }

    // flash green action button LED
    if(greenLEDTime>=greenLEDPeriod && greenLEDPeriod!=0)
    {
        greenLEDTime = 0;
        greenLED = !greenLED;
        if(greenLED)
            ACTION_GREENLED_PORT |=  (1 << ACTION_GREENLED_PIN);   // green LED on
        else
            ACTION_GREENLED_PORT &= ~(1 << ACTION_GREENLED_PIN);   // green LED off

        if(mode==MODE_SAMPLE)
            LEDTimerRunning=false;
    }
        
#if DEBUG_LOG_IO
    // count when to log the IO signals
    if(debugIOTime<DEBUG_NUM_INTERRUPTS_LOG_IO)
        debugIOTime++;
#endif // DEBUG_LOG_IO
    
}

//----------------------------------------------------------------------------------------//
// checkLogIO:
// Check if it time to log the input and output data values. The IO is
// logged once every DEBUG_NUM_INTERRUPTS_LOG_IO interrupts to limit
// the logging rate.
//----------------------------------------------------------------------------------------//
void checkLogIO(void)
{
#if DEBUG_LOG_IO
    if(debugIOTime==DEBUG_NUM_INTERRUPTS_LOG_IO)
    {
        debugIOTime = 0;

        DBG_LOG1("ir=%d ",ir);
        DBG_LOG1("cv=%d ",cv);
        DBG_LOG1("th=%d ",threshold);
        if(gateDirection == GATE_INPUT)
        {
            DBG_LOG1("gi=%d\n", bit_is_set(PINB, GATE_PIN));
        }
        else
        {
            DBG_LOG1("go=%d\n",gate);
        }
    }
#endif // DEBUG_LOG_IO
}

//----------------------------------------------------------------------------------------//
// strobeLEDs:
// Strobe all LEDs in sequence at startup to warn that a reboot has occurred.
//----------------------------------------------------------------------------------------//
void strobeLEDs(void)
{
    uint8_t LEDData[] = {   0b00000001,     \
                            0b00000010,     \
                            0b00000100,     \
                            0b00001000,     \
                            0b00010000,     \
                            0b01000000,     \
                            0b00100000  };
    
    uint8_t i=0;

    // light mode LEDs in sequence
    for(i=0; i<7; i++)
    {
        writeLEDShiftRegister(LEDData[i]);
        _delay_ms(70);
        writeLEDShiftRegister(0);
        _delay_ms(20);
    }

    // light action LEDs in sequence
    SET_PIN(ACTION_REDLED_PORT, ACTION_REDLED_PIN);
    _delay_ms(70);
    CLR_PIN(ACTION_REDLED_PORT, ACTION_REDLED_PIN);
    _delay_ms(20);

    SET_PIN(ACTION_GREENLED_PORT, ACTION_GREENLED_PIN);
    _delay_ms(70);
    CLR_PIN(ACTION_GREENLED_PORT, ACTION_GREENLED_PIN);
    _delay_ms(20);

    // light all LEDs
    writeLEDShiftRegister(0b01111111);
    SET_PIN(ACTION_REDLED_PORT, ACTION_REDLED_PIN);
    SET_PIN(ACTION_GREENLED_PORT, ACTION_GREENLED_PIN);
    _delay_ms(200);

    // clear all LEDs
    writeLEDShiftRegister(0);
    CLR_PIN(ACTION_REDLED_PORT, ACTION_REDLED_PIN);
    CLR_PIN(ACTION_GREENLED_PORT, ACTION_GREENLED_PIN);
    _delay_ms(200);

}

//----------------------------------------------------------------------------------------//
// initPorts:
// Initialise the used pins of ports A and B. Sets the direction and enables internal
// pull-up resistors for switch inputs.
//----------------------------------------------------------------------------------------//
void initPorts(void)
{
    // port A
    LED_SHIFT_DDR |= 1 << LED_DATA_PIN;             // set LED serial data pin to output
    LED_SHIFT_DDR |= 1 << LED_SHIFT_CLK_PIN;        // set LED serial shift clock pin to output
    LED_SHIFT_DDR |= 1 << LED_STORE_CLK_PIN;        // set LED serial store clock pin to output
    
    ACTION_REDLED_DDR |= 1 << ACTION_REDLED_PIN;    // set action switch red LED pin to output
    ACTION_GREENLED_DDR |= 1 << ACTION_GREENLED_PIN;// set action switch green LED pin to output
    
    // port B
    GATE_DDR |= 1 << GATE_PIN;                      // set gate pin to output
    CVPWM_DDR |= 1 << CVPWM_PIN;                    // set cv pwm pin to output
    
    MODE_BUTTON_DDR &= ~(1 << MODE_BUTTON_PIN);     // set mode switch pin to input
    MODE_BUTTON_PORT |= (1 << MODE_BUTTON_PIN);     // enable mode switch pin pull-up resistor

    ACTION_BUTTON_DDR &= ~(1 << ACTION_BUTTON_PIN); // set action switch pin to input
    ACTION_BUTTON_PORT |= (1 << ACTION_BUTTON_PIN); // enable mode switch pin pull-up resistor

    THRESHOLD_DDR &= ~(1 << THRESHOLD_PIN);         // set threshold pin to input
    IRSENSOR_DDR &= ~(1 << IRSENSOR_PIN);           // set IR sensor pin to input
    
}

//----------------------------------------------------------------------------------------//
// initTimers:
// Initialise timer 0 and timer 1. Timer 0 triggers an interrupt every ~12ms. Timer 1
// runs in fast PWM mode, using the internal 64MHz PLL clock.
//----------------------------------------------------------------------------------------//
void initTimers(void)
{
    // Timer 0, ADC sampling, LED flashing and button press timing
    TCCR0A |= (1<<WGM00);       // clear on timer compare (OCR0A) (WGM00 defined in iotn861a.h but is CTC0 in datasheet!)
#if F_CPU==1000000
    TCCR0B |= (1<<CS00);        // F_CPU/64 CS=011
    TCCR0B |= (1<<CS01);        // F_CPU/64 CS=011
    OCR0A = 188;                // interrupt about every 12.032ms, min. IR sample period
#elif F_CPU==8000000
    TCCR0B |= (1<<CS02);        // F_CPU/1024 CS=101
    TCCR0B |= (1<<CS00);        // F_CPU/1024 CS=101
    OCR0A = 94;                 // interrupt about every 12.032ms, min. IR sample period
#else
#error Wrong CPU frequency specified by F_CPU in the Makefile. Must be 1000000 or 8000000
#endif // F_CPU
    TIMSK |= (1<<OCIE0A);       // enable timer 0 compare match A interrupt

    // Timer 1, CV output PWM
    TCCR1D |= (1 << WGM10);     // fast PWM mode
    TCCR1B |= (1 << CS10);      // PWM Freq = F_CPU/1/1024
    setInvertPWM();             // set the output CV direction by inverted/non-inverted PWM
    TCCR1A |= (1 << PWM1A);     // enable PWM on OCR1A
    TC1H = 0x3;                 // OCR1C = PWM TOP, set 2 MSBs of 1023
    OCR1C = 0xff;               // OCR1C = PWM TOP, set 8 LSBs of 1023
    PLLCSR |= (1 << PLLE);      // enable PLL, 64MHz clock source
    _delay_us(100);             // wait for PLL to stabilise
    loop_until_bit_is_set(PLLCSR, PLOCK);   // poll PLOCK to check when PLL locked
    PLLCSR |= (1 << PCKE);      // enable PLL for timer 1
    
}

//----------------------------------------------------------------------------------------//
// initADC:
// Initialise the analog to digital convertor. Use the external analog reference voltage
// on the board which is set to the just under the max. value coming from the sensor,
// usually about 2.5-2.7V. Uses a clock prescaler of /8.
//----------------------------------------------------------------------------------------//
void initADC(void)
{
    ADMUX |= (1<<REFS0);        // refence voltage on AREF pin
    ADCSRA |= (1<<ADPS0);       // ADC clock prescaler /8 ADPS = 11
    ADCSRA |= (1<<ADPS1);       // ADC clock prescaler /8 ADPS = 11
    ADCSRA |= (1<<ADEN);        // enable ADC
    
}

//----------------------------------------------------------------------------------------//
// initGlobals:
// Initialise all global variables.
//----------------------------------------------------------------------------------------//
void initGlobals(void)
{
    stateAddress = (uint8_t *)0;
    signalSaveState = false;
    mode = MODE_TOGGLE;
    for(uint8_t i=0; i<6; i++)
        modeState[i] = false;
    modeButtonState = MODE_BUTTON_STANDBY;
    actionButtonState = false;
    actionState = false;
    invertCV = false;
    invertGate = false;
    gateDirection = GATE_OUTPUT;
    run = false;
    cv = 0;
    ir = 0;
    irLastRun = 0;
    threshold = 0;
    gate = false;
    modeHoldTime = 0;
    modeReleaseTime = 0;
    redLED = false;
    greenLED = false;
    redLEDTime = 0;
    redLEDPeriod = 0;
    greenLEDTime = 0;
    greenLEDPeriod = 0;
    LEDTimerRunning = false;
#if DEBUG_LOG_IO
    debugIOTime = 0;
#endif // DEBUG_LOG_IO
    calibration = false;
    irCalibrate = 0;
    bargraphState = 0;

}

//----------------------------------------------------------------------------------------//
// main:
// Initialise, load the state from eeprom for the last time powered up, set up IO, start
// the interrupts and timers, and start the main run loop to poll the UI.
//----------------------------------------------------------------------------------------//
int main(void)
{

    initGlobals();
    
    dbg_tx_init();

    DBG_LOG("\nHover initialising...\n");
    
    initPorts();
    initTimers();
    initADC();

    if(switchPressed(MODE_BUTTON_PIN))
    {
        calibration = true;
        mode = MODE_CALIBRATE;
        DBG_LOG("Starting in calibration mode...\n");
    }
    else
    {
        loadState();
        strobeLEDs();

        setModeInvertLEDs();
    }
    
    setActionLEDs();
    setModeRunState();

    setGatePinDirection();
    setCV(0);
        
    DBG_LOG("Running.\n\n");
    
    sei();

    while (1)
    {
        if(!calibration)
            checkModeSwitch();
        
        checkActionSwitchAndGateInput();
        
        if(!calibration)
            checkSaveState();

        checkLogIO();

    }
        
    return 0;
    
}





