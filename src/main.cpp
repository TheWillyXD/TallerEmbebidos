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

// Estados
typedef enum { STATE_OFF, STATE_MEASURING, STATE_FROZEN } State;
typedef enum { UNIT_METERS, UNIT_FEET, UNIT_INCHES } Unit;

State currentState = STATE_OFF;  // INICIA APAGADO
Unit currentUnit = UNIT_METERS;
float frozenDistance = 0.0;

bool unitChanged = false;
// Funciones LCD (igual que antes)
void lcdPulseEnable() {
    PORTC |= (1 << LCD_E);
    _delay_us(1);
    PORTC &= ~(1 << LCD_E);
    _delay_us(100);
}

void lcdSendNibble(uint8_t nibble) {
    if (nibble & 1) PORTC |= (1 << LCD_D4); else PORTC &= ~(1 << LCD_D4);
    if (nibble & 2) PORTC |= (1 << LCD_D5); else PORTC &= ~(1 << LCD_D5);
    if (nibble & 4) PORTC |= (1 << LCD_D6); else PORTC &= ~(1 << LCD_D6);
    if (nibble & 8) PORTD |= (1 << LCD_D7); else PORTD &= ~(1 << LCD_D7);
    lcdPulseEnable();
}

void lcdCommand(uint8_t cmd) {
    PORTC &= ~(1 << LCD_RS);
    lcdSendNibble(cmd >> 4);
    lcdSendNibble(cmd & 0x0F);
    _delay_ms(2);
}

void lcdData(uint8_t data) {
    PORTC |= (1 << LCD_RS);
    lcdSendNibble(data >> 4);
    lcdSendNibble(data & 0x0F);
    _delay_ms(2);
}

void lcdInit() {
    DDRC |= (1 << LCD_RS) | (1 << LCD_E) | (1 << LCD_D4) |
            (1 << LCD_D5) | (1 << LCD_D6);
    DDRD |= (1 << LCD_D7);
    _delay_ms(20);

    lcdSendNibble(0x03); _delay_ms(5);
    lcdSendNibble(0x03); _delay_us(200);
    lcdSendNibble(0x03); _delay_us(200);
    lcdSendNibble(0x02); // modo 4 bits

    lcdCommand(0x28);  // 2 líneas, 4 bits
    lcdCommand(0x0C);  // Display ON
    lcdCommand(0x06);  // Incrementar cursor
    lcdCommand(0x01);  // Limpiar pantalla
    _delay_ms(2);
}

void lcdSetCursor(uint8_t row, uint8_t col) {
    uint8_t addr = (row == 1) ? 0x40 + col : col;
    lcdCommand(0x80 | addr);
}

void lcdPrint(const char* str) {
    while (*str) lcdData(*str++);
}

void lcdClear() {
    lcdCommand(0x01);
    _delay_ms(2);
}

// Inicialización de puertos y ADC
void initPorts() {
    DDRB |= (1 << LED_PIN);

    // Botones como entradas con pull-ups
    DDRD &= ~((1 << BOTON_MEASURE) | (1 << BOTON_METERS) | (1 << BOTON_FEET) |
              (1 << BOTON_INCHES) | (1 << BOTON_POWER));
    PORTD |= (1 << BOTON_MEASURE) | (1 << BOTON_METERS) | (1 << BOTON_FEET) |
             (1 << BOTON_INCHES) | (1 << BOTON_POWER);

    // ADC0 en PC0
    DDRC &= ~(1 << PC0);    // Entrada analógica PC0
    PORTC &= ~(1 << PC0);   // Sin pull-up en PC0

    ADMUX = (1 << REFS0) | 0x00;   // AVcc ref + ADC0 (PC0)
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

// Leer ADC0 (PC0)
uint16_t readADC() {
    ADMUX = (ADMUX & 0xF0) | 0x00; // Canal 0
    _delay_us(10);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

// Simular distancia (potenciómetro en PC0)
float readDistanceCM() {
    return readADC() * 80.0 / 1023.0; // distancia fija para test
}

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
    int dec_part = (int)((num - int_part) * 100); // 2 decimales

    if (dec_part < 0) dec_part = -dec_part;

    snprintf(buffer, sizeof(buffer), "%d.%02d", int_part, dec_part);
    lcdPrint(buffer);
}

// Detecta botones y cambia estado
void checkButtons() {
    // POWER: enciende / apaga
    if (!(PIND & (1 << BOTON_POWER))) {
        _delay_ms(50);
        if (currentState == STATE_OFF) {
            currentState = STATE_MEASURING;
            lcdClear();
        } else {
            currentState = STATE_OFF;
            lcdClear();
        }
        while (!(PIND & (1 << BOTON_POWER)));
    }

    if (currentState == STATE_OFF) return;  // si está apagado, no sigue

    // Cambio unidad SOLO si está congelado
    if (currentState == STATE_FROZEN) {
        if (!(PIND & (1 << BOTON_METERS))) {
            currentUnit = UNIT_METERS;
            unitChanged = true;
            _delay_ms(150);
        }
        if (!(PIND & (1 << BOTON_FEET))) {
            currentUnit = UNIT_FEET;
            unitChanged = true;
            _delay_ms(150);
        }
        if (!(PIND & (1 << BOTON_INCHES))) {
            currentUnit = UNIT_INCHES;
            unitChanged = true;
            _delay_ms(150);
        }
    }

    // Medir o congelar
    if (!(PIND & (1 << BOTON_MEASURE))) {
        _delay_ms(50);
        if (currentState == STATE_MEASURING) {
            frozenDistance = readDistanceCM();
            currentState = STATE_FROZEN;
        } else if (currentState == STATE_FROZEN) {
            currentState = STATE_MEASURING;
        }
        while (!(PIND & (1 << BOTON_MEASURE)));
    }
}

// Lógica principal según estado
void runStateMachine() {
    if (currentState == STATE_OFF) {
        lcdClear();
        lcdSetCursor(0, 0);
        lcdPrint("Sistema Apagado");
        PORTB &= ~(1 << LED_PIN);
    }
    else if (currentState == STATE_MEASURING) {
        float cm = readDistanceCM();
        float val = convertDistance(cm);

        lcdClear();
        lcdSetCursor(0, 0);
        lcdPrint("Dist: ");
        lcdPrintFloat(val, 2);  // Imprime con 2 decimales

        lcdSetCursor(1, 0);
        switch (currentUnit) {
            case UNIT_METERS: lcdPrint("Unidad: Metros"); break;
            case UNIT_FEET:   lcdPrint("Unidad: Pies"); break;
            case UNIT_INCHES: lcdPrint("Unidad: Pulg"); break;
        }

    
        
        unitChanged = false; // No interesa aquí
    }
    else if (currentState == STATE_FROZEN) {
        if (unitChanged) {
            // Solo actualiza si hubo cambio de unidad
            lcdClear();
            lcdSetCursor(0, 0);
            lcdPrint("Dist: ");
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
        _delay_ms(10);  // corto delay para dar tiempo sin bloquear mucho
    }
}

void setupTimer1() {
    // CTC mode: Clear Timer on Compare Match
    TCCR1B |= (1 << WGM12);

    // Prescaler 64 → 16MHz / 64 = 250kHz → 25000 ticks = 100ms
    TCCR1B |= (1 << CS11) | (1 << CS10);  // Prescaler 64
    OCR1A = 10000;  // 40ms     (25000-10ms)

    // Habilita interrupción por comparación
    TIMSK1 |= (1 << OCIE1A);
}


ISR(TIMER1_COMPA_vect) {
    if (currentState == STATE_MEASURING) {
        PORTB ^= (1 << LED_PIN);  // Parpadea solo en modo medición
    }
}

int main() {
    initPorts();
    lcdInit();
    setupTimer1();
    sei();  // Habilita interrupciones globales
    while (1) {
        checkButtons();
        runStateMachine();
    }
}

