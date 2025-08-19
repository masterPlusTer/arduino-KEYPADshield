#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdint.h>

// ==================== Pines del LCD ====================
#define LCD_RS PB0
#define LCD_EN PB1
#define LCD_D4 PD4
#define LCD_D5 PD5
#define LCD_D6 PD6
#define LCD_D7 PD7
#define LCD_BL PB2

#define BUTTON_PIN 0 // ADC0

typedef enum {
    ButtonNone=0, ButtonRight, ButtonUp, ButtonDown, ButtonLeft, ButtonSelect
} LCDButton;

// ==================== LCD ====================
void lcdPulseEnable() {
    PORTB &= ~(1<<LCD_EN);
    _delay_us(1);
    PORTB |= (1<<LCD_EN);
    _delay_us(1);
    PORTB &= ~(1<<LCD_EN);
    _delay_us(100);
}

void lcdWrite4bits(uint8_t nibble, uint8_t isData) {
    if(isData) PORTB |= (1<<LCD_RS); else PORTB &= ~(1<<LCD_RS);
    PORTD = (PORTD & 0x0F) | ((nibble & 0x0F)<<4);
    lcdPulseEnable();
}

void lcdCommand(uint8_t cmd) {
    lcdWrite4bits(cmd>>4,0);
    lcdWrite4bits(cmd & 0x0F,0);
    _delay_ms(2);
}

void lcdData(uint8_t data) {
    lcdWrite4bits(data>>4,1);
    lcdWrite4bits(data & 0x0F,1);
    _delay_ms(2);
}

void lcdInit() {
    DDRB |= (1<<LCD_RS)|(1<<LCD_EN)|(1<<LCD_BL);
    DDRD |= (1<<LCD_D4)|(1<<LCD_D5)|(1<<LCD_D6)|(1<<LCD_D7);
    PORTB |= (1<<LCD_BL);

    _delay_ms(50);
    lcdWrite4bits(0x03,0);
    _delay_ms(5);
    lcdWrite4bits(0x03,0);
    _delay_us(150);
    lcdWrite4bits(0x03,0);
    lcdWrite4bits(0x02,0);

    lcdCommand(0x28);
    lcdCommand(0x0C);
    lcdCommand(0x06);
    lcdCommand(0x01);
    _delay_ms(2);
}

void lcdSetCursor(uint8_t col,uint8_t row){
    uint8_t row_offsets[]={0x00,0x40};
    lcdCommand(0x80|(col+row_offsets[row]));
}

void lcdPrint(const char* str){
    while(*str) lcdData(*str++);
}

void lcdBacklightOn(){ PORTB |= (1<<LCD_BL); }
void lcdBacklightOff(){ PORTB &= ~(1<<LCD_BL); }

// ==================== Timer / millis ====================
volatile uint32_t sysMillis=0;
ISR(TIMER0_COMPA_vect){ sysMillis++; }
uint32_t millis(){ return sysMillis; }

void timer0_init(){
    TCCR0A = (1<<WGM01);
    TCCR0B = (1<<CS01)|(1<<CS00);
    OCR0A = 249;
    TIMSK0 = (1<<OCIE0A);
    sei();
}

// ==================== ADC ====================
void adcInit(){ ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); }

uint16_t readADC(uint8_t ch){
    ADMUX = (1<<REFS0) | (ch & 0x07);
    ADCSRA |= (1<<ADSC);
    while(ADCSRA & (1<<ADSC));
    return ADC;
}

// ==================== Botones ====================
LCDButton getButtons(){
    static LCDButton lastKey = ButtonNone;
    static uint32_t lastTime = 0;
    uint32_t now = millis();
    uint16_t val = readADC(BUTTON_PIN);
    LCDButton key;

    if(val < 50) key = ButtonRight;
    else if(val < 200) key = ButtonUp;
    else if(val < 300) key = ButtonDown;
    else if(val < 500) key = ButtonLeft;
    else if(val < 700) key = ButtonSelect;
    else key = ButtonNone;

    // debounce simple
    if(key != lastKey){
        if(now - lastTime > 50){
            lastTime = now;
            lastKey = key;
            return key;
        }else{
            return ButtonNone;
        }
    }
    return lastKey;
}

// ==================== Setup / Loop ====================
void setup(){
    lcdInit();
    lcdBacklightOn();
    lcdSetCursor(0,0);
    lcdPrint("Push the buttons");
    adcInit();
    timer0_init();
}

void loop(){
    static uint32_t lastSec=0;
    uint32_t now = millis();

    // contador de segundos
    if(now - lastSec >= 1000){
        lastSec = now;
        lcdSetCursor(9,1);
        uint32_t sec = now/1000;
        char buf[5];
        buf[0]='0'+(sec/1000)%10;
        buf[1]='0'+(sec/100)%10;
        buf[2]='0'+(sec/10)%10;
        buf[3]='0'+(sec%10);
        buf[4]='\0';
        lcdPrint(buf);
    }

    // leer botones
    LCDButton b = getButtons();
    lcdSetCursor(0,1);
    switch(b){
        case ButtonRight: lcdPrint("RIGHT "); break;
        case ButtonLeft:  lcdPrint("LEFT  "); break;
        case ButtonUp:    lcdPrint("UP    "); break;
        case ButtonDown:  lcdPrint("DOWN  "); break;
        case ButtonSelect:lcdPrint("SELECT"); break;
        case ButtonNone:  lcdPrint("NONE  "); break;
    }
}

int main(){ setup(); while(1) loop(); }
