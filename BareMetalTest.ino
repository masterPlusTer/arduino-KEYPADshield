#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

// ==================== Pines del LCD ====================
#define LCD_RS PB0
#define LCD_EN PB1
#define LCD_D4 PD4
#define LCD_D5 PD5
#define LCD_D6 PD6
#define LCD_D7 PD7
#define LCD_BL PB2

// Pin del teclado (ADC0)
#define BUTTON_PIN 0  

// ==================== Botones ====================
typedef enum {
    ButtonNone = 0,
    ButtonRight,
    ButtonUp,
    ButtonDown,
    ButtonLeft,
    ButtonSelect
} LCDButton;

// ==================== Variables globales ====================
volatile unsigned long millisCounter = 0;

// ==================== Timer0 para millis() ====================
ISR(TIMER0_COMPA_vect) {
    millisCounter++;
}

void timer0Init() {
    // CTC mode, prescaler 64, OCR0A = 249 → 1 ms
    TCCR0A = (1<<WGM01);
    TCCR0B = (1<<CS01)|(1<<CS00);
    OCR0A = 249;
    TIMSK0 = (1<<OCIE0A);
    sei(); // Habilitar interrupciones
}

unsigned long millis() {
    unsigned long m;
    cli();
    m = millisCounter;
    sei();
    return m;
}

// ==================== Funciones LCD ====================
void lcdPulseEnable() {
    PORTB &= ~(1<<LCD_EN);
    _delay_us(1);
    PORTB |= (1<<LCD_EN);
    _delay_us(1);
    PORTB &= ~(1<<LCD_EN);
    _delay_us(100);
}

void lcdWrite4bits(uint8_t nibble, uint8_t isData) {
    if(isData) PORTB |= (1<<LCD_RS);
    else PORTB &= ~(1<<LCD_RS);

    if(nibble & 0x01) PORTD |= (1<<LCD_D4); else PORTD &= ~(1<<LCD_D4);
    if(nibble & 0x02) PORTD |= (1<<LCD_D5); else PORTD &= ~(1<<LCD_D5);
    if(nibble & 0x04) PORTD |= (1<<LCD_D6); else PORTD &= ~(1<<LCD_D6);
    if(nibble & 0x08) PORTD |= (1<<LCD_D7); else PORTD &= ~(1<<LCD_D7);

    lcdPulseEnable();
}

void lcdCommand(uint8_t cmd) {
    lcdWrite4bits(cmd >> 4, 0);
    lcdWrite4bits(cmd & 0x0F, 0);
    _delay_ms(2);
}

void lcdData(uint8_t data) {
    lcdWrite4bits(data >> 4, 1);
    lcdWrite4bits(data & 0x0F, 1);
    _delay_ms(2);
}

void lcdInit() {
    DDRB |= (1<<LCD_RS)|(1<<LCD_EN)|(1<<LCD_BL);
    DDRD |= (1<<LCD_D4)|(1<<LCD_D5)|(1<<LCD_D6)|(1<<LCD_D7);

    PORTB |= (1<<LCD_BL); // Backlight ON
    _delay_ms(50);

    lcdWrite4bits(0x03, 0);
    _delay_ms(5);
    lcdWrite4bits(0x03, 0);
    _delay_us(150);
    lcdWrite4bits(0x03, 0);
    lcdWrite4bits(0x02, 0); // 4-bit mode

    lcdCommand(0x28); // 4-bit, 2 líneas, 5x8
    lcdCommand(0x0C); // Display ON, cursor OFF
    lcdCommand(0x06); // Incrementar cursor
    lcdCommand(0x01); // Clear display
    _delay_ms(2);
}

void lcdSetCursor(uint8_t col, uint8_t row) {
    uint8_t row_offsets[] = {0x00, 0x40};
    lcdCommand(0x80 | (col + row_offsets[row]));
}

void lcdPrint(const char* str) {
    while(*str) lcdData(*str++);
}

void lcdBacklightOn() { PORTB |= (1<<LCD_BL); }
void lcdBacklightOff() { PORTB &= ~(1<<LCD_BL); }

// ==================== Función de botones ====================
LCDButton getButtons() {
    static LCDButton keyLast = ButtonNone;
    static unsigned long keyTimeLast = 0;

    int analogKey;

    // Leer ADC0
    ADMUX = (1<<REFS0) | (BUTTON_PIN & 0x07);
    ADCSRA |= (1<<ADSC);
    while(ADCSRA & (1<<ADSC));
    analogKey = ADC;

    LCDButton key;
    if ((millis() - keyTimeLast) < 50) return keyLast; // debounce 50 ms
    keyTimeLast = millis();

    if (analogKey < 50) key = ButtonRight;
    else if (analogKey < 200) key = ButtonUp;
    else if (analogKey < 300) key = ButtonDown;
    else if (analogKey < 500) key = ButtonLeft;
    else if (analogKey < 700) key = ButtonSelect;
    else key = ButtonNone;

    keyLast = key;
    return key;
}

// ==================== Setup ====================
void setup() {
    lcdInit();
    lcdBacklightOn();
    lcdSetCursor(0,0);
    lcdPrint("Push the buttons");

    timer0Init();

    // Parpadeo del backlight 3 veces
    for(uint8_t i=0;i<3;i++) {
        lcdBacklightOff();
        _delay_ms(500);
        lcdBacklightOn();
        _delay_ms(500);
    }

    // Inicializar ADC
    ADCSRA = (1<<ADEN) | (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}

// ==================== Loop ====================
void loop() {
    static unsigned long lastSec = 0;

    // Leer botón y mostrarlo
    lcdSetCursor(0,1);
    switch(getButtons()) {
        case ButtonRight: lcdPrint("RIGHT "); break;
        case ButtonLeft:  lcdPrint("LEFT  "); break;
        case ButtonUp:    lcdPrint("UP    "); break;
        case ButtonDown:  lcdPrint("DOWN  "); break;
        case ButtonSelect:lcdPrint("SELECT"); break;
        case ButtonNone:  lcdPrint("NONE  "); break;
    }

    // Actualizar segundos sin bloquear
    if(millis() - lastSec >= 1000) {
        lastSec += 1000;
        lcdSetCursor(9,1);
        char buffer[6];
        sprintf(buffer, "%4lu", lastSec/1000);
        lcdPrint(buffer);
    }
}

// ==================== Main ====================
int main(void) {
    setup();
    while(1) loop();
}
