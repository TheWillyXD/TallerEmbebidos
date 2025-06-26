#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdio.h>


#include <avr/interrupt.h>

// Pines LCD
#define LCD_RS PC5
#define LCD_E  PC4
#define LCD_D4 PC1
#define LCD_D5 PC2
#define LCD_D6 PC3
#define LCD_D7 PD7

// Control
#define LED_PIN          PB0
#define BOTON_MEASURE    PD2
#define BOTON_METERS     PD3
#define BOTON_FEET       PD4
#define BOTON_INCHES     PD5
#define BOTON_POWER      PD6

volatile bool medirSolicitado = false;

// Estados
typedef enum { STATE_OFF, STATE_MEASURING, STATE_FROZEN } State;
typedef enum { UNIT_METERS, UNIT_FEET, UNIT_INCHES } Unit;

State currentState = STATE_OFF;
Unit currentUnit = UNIT_METERS;
float frozenDistance = 0.0;
bool unitChanged = false;

// Funciones LCD (idénticas)
void lcdPulseEnable() { PORTC |= (1 << LCD_E); _delay_us(1); PORTC &= ~(1 << LCD_E); _delay_us(100); }
void lcdSendNibble(uint8_t nibble) {
    if (nibble & 1) PORTC |= (1 << LCD_D4); else PORTC &= ~(1 << LCD_D4);
    if (nibble & 2) PORTC |= (1 << LCD_D5); else PORTC &= ~(1 << LCD_D5);
    if (nibble & 4) PORTC |= (1 << LCD_D6); else PORTC &= ~(1 << LCD_D6);
    if (nibble & 8) PORTD |= (1 << LCD_D7); else PORTD &= ~(1 << LCD_D7);
    lcdPulseEnable();
}
void lcdCommand(uint8_t cmd) { PORTC &= ~(1 << LCD_RS); lcdSendNibble(cmd >> 4); lcdSendNibble(cmd & 0x0F); _delay_ms(2); }
void lcdData(uint8_t data) { PORTC |= (1 << LCD_RS); lcdSendNibble(data >> 4); lcdSendNibble(data & 0x0F); _delay_ms(2); }
void lcdInit() {
    DDRC |= (1 << LCD_RS) | (1 << LCD_E) | (1 << LCD_D4) | (1 << LCD_D5) | (1 << LCD_D6);
    DDRD |= (1 << LCD_D7); _delay_ms(20);
    lcdSendNibble(0x03); _delay_ms(5); lcdSendNibble(0x03); _delay_us(200); lcdSendNibble(0x03); _delay_us(200); lcdSendNibble(0x02);
    lcdCommand(0x28); lcdCommand(0x0C); lcdCommand(0x06); lcdCommand(0x01); _delay_ms(2);
}
void lcdSetCursor(uint8_t row, uint8_t col) { uint8_t addr = (row == 1) ? 0x40 + col : col; lcdCommand(0x80 | addr); }
void lcdPrint(const char* str) { while (*str) lcdData(*str++); }
void lcdClear() { lcdCommand(0x01); _delay_ms(2); }

// ADC y entradas
void initPorts() {
    DDRB |= (1 << LED_PIN);
    DDRD &= ~((1 << BOTON_MEASURE) | (1 << BOTON_METERS) | (1 << BOTON_FEET) | (1 << BOTON_INCHES) | (1 << BOTON_POWER));
    PORTD |= (1 << BOTON_MEASURE) | (1 << BOTON_METERS) | (1 << BOTON_FEET) | (1 << BOTON_INCHES) | (1 << BOTON_POWER);
    DDRC &= ~(1 << PC0); PORTC &= ~(1 << PC0);
    ADMUX = (1 << REFS0);  // AVcc y ADC0
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

    // INT0 en flanco de bajada (PD2)
    EIMSK |= (1 << INT0);
    EICRA |= (1 << ISC01);
    EICRA &= ~(1 << ISC00);
}

uint16_t readADC() { ADMUX = (ADMUX & 0xF0); _delay_us(10); ADCSRA |= (1 << ADSC); while (ADCSRA & (1 << ADSC)); return ADC; }
float readDistanceCM() { return readADC() * 80.0 / 1023.0; }
float convertDistance(float cm) {
    switch (currentUnit) {
        case UNIT_METERS: return cm / 100.0;
        case UNIT_FEET: return cm * 0.0328084;
        case UNIT_INCHES: return cm * 0.393701;
        default: return cm;
    }
}
void lcdPrintFloat(float num, uint8_t decimals) {
    char buffer[16];
    int int_part = (int)num;
    int dec_part = (int)((num - int_part) * 100);
    if (dec_part < 0) dec_part = -dec_part;
    snprintf(buffer, sizeof(buffer), "%d.%02d", int_part, dec_part);
    lcdPrint(buffer);
}

// Botones excepto MEASURE
void checkButtons() {
    // POWER
    if (!(PIND & (1 << BOTON_POWER))) {
        _delay_ms(50);
        currentState = (currentState == STATE_OFF) ? STATE_MEASURING : STATE_OFF;
        lcdClear();
        while (!(PIND & (1 << BOTON_POWER)));
    }

    if (currentState == STATE_OFF) return;

    // Unidades (solo si congelado)
    if (currentState == STATE_FROZEN) {
        if (!(PIND & (1 << BOTON_METERS))) { currentUnit = UNIT_METERS; unitChanged = true; _delay_ms(150); }
        if (!(PIND & (1 << BOTON_FEET)))   { currentUnit = UNIT_FEET;   unitChanged = true; _delay_ms(150); }
        if (!(PIND & (1 << BOTON_INCHES))) { currentUnit = UNIT_INCHES; unitChanged = true; _delay_ms(150); }
    }
}

// Máquina de estados
void runStateMachine() {
    if (currentState == STATE_OFF) {
        lcdClear(); lcdSetCursor(0, 0); lcdPrint("Sistema Apagado");
        PORTB &= ~(1 << LED_PIN);
    }
    else if (currentState == STATE_MEASURING) {
        float cm = readDistanceCM();
        float val = convertDistance(cm);
        lcdClear(); lcdSetCursor(0, 0); lcdPrint("Dist: "); lcdPrintFloat(val, 2);
        lcdSetCursor(1, 0);
        switch (currentUnit) {
            case UNIT_METERS: lcdPrint("Unidad: Metros"); break;
            case UNIT_FEET:   lcdPrint("Unidad: Pies"); break;
            case UNIT_INCHES: lcdPrint("Unidad: Pulg"); break;
        }
        unitChanged = false;
    }
    else if (currentState == STATE_FROZEN) {
        if (unitChanged) {
            lcdClear(); lcdSetCursor(0, 0); lcdPrint("Dist: ");
            lcdPrintFloat(convertDistance(frozenDistance), 2);
            lcdSetCursor(1, 0);
            switch (currentUnit) {
                case UNIT_METERS: lcdPrint("Unidad: Metros"); break;
                case UNIT_FEET:   lcdPrint("Unidad: Pies"); break;
                case UNIT_INCHES: lcdPrint("Unidad: Pulg"); break;
            }
            unitChanged = false;
        }
        PORTB |= (1 << LED_PIN);
        _delay_ms(10);
    }
}

// Timer1 para parpadeo LED
void setupTimer1() {
    TCCR1B |= (1 << WGM12);
    TCCR1B |= (1 << CS11) | (1 << CS10); // prescaler 64
    OCR1A = 10000; // 40 ms
    TIMSK1 |= (1 << OCIE1A);
}

// INT0 - botón medir
ISR(INT0_vect) {
    medirSolicitado = true;
}

// Timer - parpadeo
ISR(TIMER1_COMPA_vect) {
    if (currentState == STATE_MEASURING) {
        PORTB ^= (1 << LED_PIN);
    }
}

// MAIN
int main() {
    initPorts();
    lcdInit();
    setupTimer1();
    sei();

    while (1) {
        if (medirSolicitado) {
            medirSolicitado = false;
            if (currentState == STATE_MEASURING) {
                frozenDistance = readDistanceCM();
                currentState = STATE_FROZEN;
            } else if (currentState == STATE_FROZEN) {
                currentState = STATE_MEASURING;
            }
        }
        checkButtons();
        runStateMachine();
    }
}
