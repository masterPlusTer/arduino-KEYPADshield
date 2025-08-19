// Pines del LCD
#define LCD_RS 8
#define LCD_EN 9
#define LCD_D4 4
#define LCD_D5 5
#define LCD_D6 6
#define LCD_D7 7
#define LCD_BL 10

// Pin del teclado
#define BUTTON_PIN A0

// Botones
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
  digitalWrite(LCD_EN, LOW);
  delayMicroseconds(1);
  digitalWrite(LCD_EN, HIGH);
  delayMicroseconds(1);
  digitalWrite(LCD_EN, LOW);
  delayMicroseconds(100);
}

void lcdWrite4bits(uint8_t nibble, bool isData) {
  digitalWrite(LCD_RS, isData ? HIGH : LOW);
  digitalWrite(LCD_D4, (nibble >> 0) & 1);
  digitalWrite(LCD_D5, (nibble >> 1) & 1);
  digitalWrite(LCD_D6, (nibble >> 2) & 1);
  digitalWrite(LCD_D7, (nibble >> 3) & 1);
  lcdPulseEnable();
}

void lcdCommand(uint8_t cmd) {
  lcdWrite4bits(cmd >> 4, false);
  lcdWrite4bits(cmd & 0x0F, false);
  delay(2);
}

void lcdData(uint8_t data) {
  lcdWrite4bits(data >> 4, true);
  lcdWrite4bits(data & 0x0F, true);
  delay(2);
}

void lcdInit() {
  pinMode(LCD_RS, OUTPUT);
  pinMode(LCD_EN, OUTPUT);
  pinMode(LCD_D4, OUTPUT);
  pinMode(LCD_D5, OUTPUT);
  pinMode(LCD_D6, OUTPUT);
  pinMode(LCD_D7, OUTPUT);
  pinMode(LCD_BL, OUTPUT);

  digitalWrite(LCD_BL, HIGH); // Backlight ON
  delay(50);

  // Secuencia de inicialización
  lcdWrite4bits(0x03, false);
  delay(5);
  lcdWrite4bits(0x03, false);
  delayMicroseconds(150);
  lcdWrite4bits(0x03, false);
  lcdWrite4bits(0x02, false); // 4-bit mode

  lcdCommand(0x28); // 4-bit, 2 líneas, 5x8
  lcdCommand(0x0C); // Display ON, cursor OFF
  lcdCommand(0x06); // Incrementar cursor
  lcdCommand(0x01); // Clear display
  delay(2);
}

void lcdSetCursor(uint8_t col, uint8_t row) {
  uint8_t row_offsets[] = {0x00, 0x40};
  lcdCommand(0x80 | (col + row_offsets[row]));
}

void lcdPrint(const char* str) {
  while (*str) {
    lcdData(*str++);
  }
}

void lcdBacklightOn() { digitalWrite(LCD_BL, HIGH); }
void lcdBacklightOff() { digitalWrite(LCD_BL, LOW); }

// ==================== Función de botones ====================
LCDButton getButtons() {
  static LCDButton keyLast = ButtonNone;
  static unsigned long keyTimeLast = 0;

  int analogKey = analogRead(BUTTON_PIN);
  LCDButton key;

  if ((millis() - keyTimeLast) < 100) return keyLast;
  keyTimeLast = millis();

  if (analogKey < 50) key = ButtonRight;
  else if (analogKey < 200) key = ButtonUp;
  else if (analogKey < 300) key = ButtonDown;
  else if (analogKey < 500) key = ButtonLeft;
  else if (analogKey < 700) key = ButtonSelect;
  else key = ButtonNone;

  if (key == keyLast) return key;
  keyLast = key;
  return ButtonNone;
}

// ==================== Setup y Loop ====================
void setup() {
  lcdInit();
  lcdBacklightOn();
  lcdSetCursor(0, 0);
  lcdPrint("Push the buttons");

  // Parpadeo del backlight 3 veces
  for (uint8_t i = 0; i < 3; i++) {
    lcdBacklightOff();
    delay(500);
    lcdBacklightOn();
    delay(500);
  }
}

void loop() {
  // Mostrar segundos desde encendido
  lcdSetCursor(9, 1);
  char buffer[6];
  sprintf(buffer, "%4lu", millis() / 1000);
  lcdPrint(buffer);

  // Leer botón y mostrarlo
  lcdSetCursor(0, 1);
  switch (getButtons()) {
    case ButtonRight: lcdPrint("RIGHT "); break;
    case ButtonLeft:  lcdPrint("LEFT  "); break;
    case ButtonUp:    lcdPrint("UP    "); break;
    case ButtonDown:  lcdPrint("DOWN  "); break;
    case ButtonSelect:lcdPrint("SELECT"); break;
    case ButtonNone:  lcdPrint("NONE  "); break;
  }
}
