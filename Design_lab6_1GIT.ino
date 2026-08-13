#include <Arduino.h>
#include "DHT_Async.h"
#define TEMP_OFFSET -4
#define DHT_SENSOR_TYPE DHT_TYPE_11
static const int DHT_SENSOR_PIN = A4;
DHT_Async dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);

int button_pressed = 0;
int sensorValue = 0;
bool current_units = 1;
bool display_mode = 0; //Controls whether we display Celsius or Fahrenheit as a measurement unit
unsigned long last_button1_press = 0;
unsigned long last_button2_press = 0;
unsigned long last_button3_press = 0;


//Common cathode
#define DIGIT_PORT PORTC
#define DIGIT_DDR DDRC

#define DGT_1 (1 << 0) // D8
#define DGT_2 (1 << 1) // D9
#define DGT_3 (1 << 2) // D10
#define DGT_4 (1 << 3) // D11

const uint8_t digit_masks[] = {DGT_1, DGT_2, DGT_3, DGT_4};

//15624
#define LED_PIN 13
//Timers - initial configuration
const int PRESCALER = 1024; 
const int OCR1A_VALUE = 31248;//default15624//scroll2500

const long F_CPU_INTERNAL = 16000000; 
const int PRESCALER_T2 = 1024;
const byte OCR2A_VALUE = 60;

// D0-D7 pins register
#define SEGMENT_PORT PORTD 
#define SEGMENT_DDR DDRD

#define SEG_A (1 << 7)
#define SEG_B (1 << 6)
#define SEG_C (1 << 5)
#define SEG_D (1 << 4)
#define SEG_E (1 << 3)
#define SEG_F (1 << 2)
#define SEG_G (1 << 1)
#define SEG_DP (1 << 0)

//Mask for bitwise negation
#define SEG_MASK 0xFF

//LOW STATE means segment is turned on. We accomplish that by using negation
#define DIGIT_0_CA (~(SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F) & SEG_MASK)
#define DIGIT_1_CA (~(SEG_B | SEG_C) & SEG_MASK)
#define DIGIT_2_CA (~(SEG_A | SEG_B | SEG_D | SEG_E | SEG_G) & SEG_MASK)
#define DIGIT_3_CA (~(SEG_A | SEG_B | SEG_C | SEG_D | SEG_G) & SEG_MASK)
#define DIGIT_4_CA (~(SEG_B | SEG_C | SEG_F | SEG_G) & SEG_MASK)
#define DIGIT_5_CA (~(SEG_A | SEG_C | SEG_D | SEG_F | SEG_G) & SEG_MASK)
#define DIGIT_6_CA (~(SEG_A | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G) & SEG_MASK)
#define DIGIT_7_CA (~(SEG_A | SEG_B | SEG_C) & SEG_MASK)
#define DIGIT_8_CA (~(SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G) & SEG_MASK)
#define DIGIT_9_CA (~(SEG_A | SEG_B | SEG_C | SEG_D | SEG_F | SEG_G) & SEG_MASK)

#define CHAR_BLANK_CA 0xFF // Turn off all segments
#define CHAR_DEGREE (~(SEG_A | SEG_B | SEG_G | SEG_F)&SEG_MASK)
#define CHAR_C_CA (~(SEG_A | SEG_D | SEG_E | SEG_F) & SEG_MASK)
#define CHAR_L_CA (~(SEG_D | SEG_E | SEG_F) & SEG_MASK)
#define CHAR_H_CA (~(SEG_B | SEG_C | SEG_E | SEG_F | SEG_G) & SEG_MASK)
#define CHAR_r_CA (~(SEG_G | SEG_E) & SEG_MASK)

uint8_t get_segment_pattern(char c) {
    switch (c) {
        // Digits
        case '0': return DIGIT_0_CA;
        case '1': return DIGIT_1_CA;
        case '2': return DIGIT_2_CA;
        case '3': return DIGIT_3_CA;
        case '4': return DIGIT_4_CA;
        case '5': return DIGIT_5_CA;
        case '6': return DIGIT_6_CA;
        case '7': return DIGIT_7_CA;
        case '8': return DIGIT_8_CA;
        case '9': return DIGIT_9_CA;

        // Chars (some of them)
        case 'H': return CHAR_H_CA;
        case '*': return CHAR_DEGREE;
        case ' ': return CHAR_BLANK_CA;
        case '~': return CHAR_BLANK_CA;
        case 'C':
        case 'c': return CHAR_C_CA;
        case 'L': return CHAR_L_CA;
        case 'r': return CHAR_r_CA;

        default: return CHAR_BLANK_CA; // Unknown char
    }
}

static bool measure_environment(float *temperature, float *humidity) {
    static unsigned long measurement_timestamp = millis();

    //Take measurement once every 4 seconds
    if (millis() - measurement_timestamp > 4000ul) {
        if (dht_sensor.measure(temperature, humidity)) {
            measurement_timestamp = millis();
            return (true);
        }
    }

    return (false);
}

char char_buffer[5] = {' ',' ',' ',' ',' '};

uint8_t digitIndex  = 0;
uint8_t seg = 0;
volatile  float temperature;
volatile  float humidity;

void setup() {
    //Set all  port D pins as outputs
    SEGMENT_DDR = 0xFF;
    pinMode(LED_PIN, OUTPUT);
    //Timer 1 setup
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;
    OCR1A = OCR1A_VALUE;
    TCCR1B |= (1 << WGM12);
    TCCR1B |= (1 << CS12) | (1 << CS10);
    TIMSK1 |= (1 << OCIE1A);

    //Timer 2 setup
    TCCR2A = 0;
    TCCR2B = 0;
    TCNT2  = 0;
    OCR2A = OCR2A_VALUE;
    TCCR2A |= (1 << WGM21);
    TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);
    TIMSK2 |= (1 << OCIE2A);
    sei();

  //Define D0-D12 pins as output
  for(int i = 0; i<=12;i++)
  {
    pinMode(i,OUTPUT);
  }

    // Set port D as an output
    SEGMENT_DDR = 0xFF; 

    //Set PB0-PB3 as outputs
    DIGIT_DDR |= (DGT_1 | DGT_2 | DGT_3 | DGT_4);

    //Turn off all digits
    DIGIT_PORT |= (DGT_1 | DGT_2 | DGT_3 | DGT_4);
  sprintf(char_buffer, "%d", 8888);
  delay(3000);
}

volatile bool data_update_flag = false;

void loop() {

    float local_temperature;
    float local_humidity;
    button_pressed = read_keys();

    if(button_pressed == 1){
        current_units = !current_units;
    }

    
    //Get sensor measurement if possible
    if (measure_environment(&local_temperature, &local_humidity)) {
        //Copy values to volatile variables
        temperature = local_temperature;
        humidity = local_humidity;
    }
    
    //Update char buffor on Interrupt1
    if (data_update_flag) {
        if(display_mode == 0)
        {
            display_data ();
        }
    }

    // delay(1); 
}

//Display refresh interrupt
ISR(TIMER2_COMPA_vect) {

    // Light up current digit. digitIndex point to a digit that was previously turned on
    DIGIT_PORT |= digit_masks[digitIndex % 4];

    // Cycle to next digit
    digitIndex++;
    uint8_t current_digit = digitIndex % 4;

    // Set the pattern of a char u want to display
    SEGMENT_PORT = get_segment_pattern(char_buffer[current_digit]);

    // Turn on new digit (Only 1 digit at a time is visible)
    DIGIT_PORT &= ~digit_masks[current_digit]; 
}


//Interrupt every 1 sec
ISR(TIMER1_COMPA_vect) {
    seg++;
    data_update_flag = true;
}
    //FOR_SCROLLING
    /*char last_char = char_buffer[0];
    memmove(&char_buffer[0], &char_buffer[1], 3 * sizeof(uint8_t));
    char_buffer[3] = last_char;*/

void display_data () {
    int value_to_display;
    if(current_units == 0){
        value_to_display = (int)round(temperature+TEMP_OFFSET); 
        sprintf(char_buffer, "%2d*C", value_to_display%100);
    } else if (current_units == 1) {
        value_to_display = (int)round(humidity); 
        sprintf(char_buffer, "%2drH", value_to_display%100);
    }

    // Do not allow negative values
    if (value_to_display < 0 || value_to_display > 99) {
        sprintf(char_buffer, "Err "); 
    }
    data_update_flag = false; // Flag reset
    //current_units = !current_units;
}

//We have 3 buttons on analog pin A5. Currently they are not used. The method below allows inteaction with these buttons,
//It can also detect when multiple buttons are pressed at the same time.
int read_keys () {
    sensorValue = analogRead(A5);
    if(sensorValue > 150 && sensorValue < 300){
        //A button
        last_button2_press = -1;
        last_button3_press = -1;
        if(millis()-last_button1_press > 20)
        //    button_pressed = 1;
        last_button1_press = millis();
        return 1;

    } else if (sensorValue > 300 && sensorValue < 400) {
        //B button
        last_button1_press = 4294967295;
        last_button3_press = -1;
        last_button2_press = millis();
        return 0;

    } else if (sensorValue > 400 && sensorValue < 490) {
        //A&B buttons
        last_button1_press = 4294967295;
        last_button2_press = -1;
        last_button3_press = -1;
        return 0;

    } else if (sensorValue > 490 && sensorValue < 535) {
        //C button
        last_button1_press = 4294967295;
        last_button2_press = -1;
        last_button3_press = millis();
        return 0;

    } else if (sensorValue > 535 && sensorValue < 600) {
        //A&C buttons
        last_button1_press = 4294967295;
        last_button2_press = -1;
        last_button3_press = -1;
        return 0;

    } else if (sensorValue > 590 && sensorValue < 630) {
        //B&C buttons
        last_button1_press = 4294967295;
        last_button2_press = -1;
        last_button3_press = -1;
        return 0;

    } else if (sensorValue > 630) {
        //A&B&C buttons
        last_button1_press = 4294967295;
        last_button2_press = -1;
        last_button3_press = -1;
        return 0;
    }
}