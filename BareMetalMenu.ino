#include <avr/io.h>
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

// Pin del teclado
#define BUTTON_PIN 0  // ADC0

// ==================== Botones ====================
enum LCDButton {
    ButtonNone = 0,
    ButtonRight = 1,
    ButtonUp = 2,
    ButtonDown = 3,
    ButtonLeft = 4,
    ButtonSelect = 5
};

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

    ADMUX = (1<<REFS0) | (BUTTON_PIN & 0x07);
    ADCSRA |= (1<<ADSC);
    while(ADCSRA & (1<<ADSC));
    analogKey = ADC;

    LCDButton key;
    if ((millis() - keyTimeLast) < 50) return ButtonNone; // más sensible
    keyTimeLast = millis();

    if (analogKey < 50) key = ButtonRight;
    else if (analogKey < 200) key = ButtonUp;
    else if (analogKey < 300) key = ButtonDown;
    else if (analogKey < 500) key = ButtonLeft;
    else if (analogKey < 700) key = ButtonSelect;
    else key = ButtonNone;

    if (key == keyLast) return ButtonNone; // evitar rebote
    keyLast = key;
    return key;
}

// ==================== Menú ====================
const char* menuItems[] = {"Item 1", "Item 2", "Item 3", "Item 4"};
uint8_t menuIndex = 0;

void showMenu() {
    lcdSetCursor(0,0);
    lcdPrint(menuItems[menuIndex]);

    lcdSetCursor(0,1);
    lcdPrint("Press SELECT");
}

// ==================== Setup y Loop ====================
void setup() {
    lcdInit();
    lcdBacklightOn();
    showMenu();
}

void loop() {
    LCDButton btn = getButtons();
    if(btn == ButtonUp) {
        if(menuIndex > 0) menuIndex--;
        showMenu();
    } else if(btn == ButtonDown) {
        if(menuIndex < 3) menuIndex++;
        showMenu();
    } else if(btn == ButtonSelect) {
        lcdSetCursor(0,1);
        lcdPrint("Selected!     ");
        _delay_ms(500);
        showMenu();
    }
}
