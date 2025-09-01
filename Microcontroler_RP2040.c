#include <stdio.h>
#include <unistd.h>
#include "pico/stdlib.h"
#include <hardware/i2c.h>
#include <hardware/pwm.h>


// Deklarationen
void echo_callback(uint gpio, uint32_t events);
float calculate_progress(float distance);
float filter_distance(float raw_distance);
void update_progress(uint8_t progress);
void setup_pwm();
void setup_gpio_interrupts();
void loop();

// I2C-Adresse des HT16K33A
#define I2C_ADDRESS 0x70

// Anzahl der LEDs in der Matrix
#define SDA_PIN 0
#define SCL_PIN 1
#define NUM_LEDS 16
#define NUM_SAMPLES 20 // Anzahl der Messungen für den Mittelwertfilter

volatile uint32_t pulse_start = 0;
volatile uint32_t pulse_end = 0;
volatile bool echo_received = false;

uint init_gpio = 28; // GPIO-Pin für Trigger
uint echo_gpio = 27; // GPIO-Pin für Echo
volatile float distance = 0.0;

void init_matrix() {
    // Initialisierung der I2C-Verbindung mit 100 kHz
    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    // Sende Initialisierungsbefehle an die LED-Matrix
    uint8_t cmds[] = {0x21, 0xA0, 0xEF, 0x81};
    for (int i = 0; i < 4; i++) {
        i2c_write_blocking(i2c0, I2C_ADDRESS, &cmds[i], 1, false);
    }

    uint8_t Led_off[] = {0,[1 ... 16] = 0x00}; // Alle LEDs ausschalten
    i2c_write_blocking(i2c0, I2C_ADDRESS, Led_off, sizeof(Led_off), false);
    printf("Matrix initialized\n");
}

// Funktion zur Berechnung des Fortschritts
float calculate_progress(float distance) {
    if (distance <= 0) {
        return 0; // Kein Fortschritt bei ungültiger Entfernung
    }
    const float TARGET_DISTANCE = 10.0; // Zielentfernung in cm
    float progress = (distance / TARGET_DISTANCE) * 100.0;
    if (progress > 200.0) {
        progress = 200.0; // Begrenzen auf 200%
    }
    return progress;
}

// Mittelwertfilter für Messungen
float filter_distance(float raw_distance) {
    static float samples[NUM_SAMPLES] = {0};
    static uint8_t index = 0;
    float avg_distance = 0;

    samples[index] = raw_distance;
    index = (index + 1) % NUM_SAMPLES;

    for (int i = 0; i < NUM_SAMPLES; i++) {
        avg_distance += samples[i];
    }
    return avg_distance / NUM_SAMPLES;
}

// Fortschrittsanzeige auf LED-Matrix
void update_progress(uint8_t progress) {
    uint8_t display_data[NUM_LEDS+1] = {[0 ... 16] = 0};
    
    uint8_t num_leds_on = (progress * NUM_LEDS) / 200;

    for (int i = 0; i < num_leds_on; i++) {
        display_data[i+1] = 0xFF;        
    }
    i2c_write_blocking(i2c0, I2C_ADDRESS, display_data, sizeof(display_data), false);
}

// PWM-Konfiguration für Trigger-Pin
void setup_pwm() {

    gpio_set_function(init_gpio, GPIO_FUNC_PWM);

    // Bestimme den PWM-Slice für den GPIO-Pin 34
    uint slice_num = pwm_gpio_to_slice_num(init_gpio);

    // Setze den PWM-Taktteiler, um die PWM-Frequenz zu steuern in diesem Fall wird der Taktteiler auf 4.0 festgelegt
    pwm_set_clkdiv(slice_num, 125);

    // Setze den PWM-Zählerumbruchwert; maximale Anzahl von Zählungen(1000), bevor der Zähler zurückgesetzt wird
    pwm_set_wrap(slice_num, 60000); // 60ms = 60000µs Periodendauer

    // Setze den PWM-Kanalpegel für Kanal A, um zu bestimmen, wann der Ausgang ein-(Channel A) oder ausgeschaltet (Channel B) wird;
    // 500 weil die Hälfte von 1000 ist
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 10); // 10 microsekunden High-Zeit

    // Aktiviere den PWM-Baustein
    pwm_set_enabled(slice_num, true);
}

//Hardwareseitige Fortschrittsanzeige (Aufgabe 4)
void setup_hardware_pwm() {

    gpio_init(echo_gpio);
    gpio_set_dir(echo_gpio, GPIO_IN);
    // Konfiguriere den GPIO-Pin 34 als PWM-Ausgang
    gpio_set_function(echo_gpio, GPIO_FUNC_PWM);

    // Bestimme den PWM-Slice für den GPIO-Pin 34
    uint slice_num = pwm_gpio_to_slice_num(echo_gpio);

    // Setze den PWM-Taktteiler, um die PWM-Frequenz zu steuern
    pwm_set_clkdiv(slice_num, 125);

    // Setze den PWM-Zählerumbruchwert
    pwm_set_wrap(slice_num, 60000);

    // stelle den PWM-Modus ein, PWM_DIV_FREE_RUNNING bedeutet, dass der Zähler bei jedem Überlauf zurückgesetzt wird
    //pwm_set_clkdiv_mode(slice_num, PWM_DIV_FREE_RUNNING);
    pwm_set_clkdiv_mode(slice_num, PWM_DIV_B_HIGH);

    // Aktiviere den PWM-Baustein
    pwm_set_enabled(slice_num, true);
}

// GPIO-Interrupts für Echo-Pin
void setup_gpio_interrupts() {
    gpio_set_irq_enabled_with_callback(
        echo_gpio,                               // Der Pin, der überwacht wird
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, // Ereignisse: Ansteigende und fallende Flanken
        true,                                    // Aktivieren des Interrupts
        &echo_callback                           // Die Callback-Funktion, die ausgeführt wird
    );
}


// Callback für Echo-Signal
void echo_callback(uint gpio, uint32_t events) {
    uint32_t slice_num = pwm_gpio_to_slice_num(echo_gpio); // PWM-Slice für den Pin holen
    //pwm_set_counter(slice_num, 0); // PWM-Zähler zurücksetzen

    if (events & GPIO_IRQ_EDGE_RISE) {
        pulse_start = time_us_32();
        //pulse_start = pwm_get_counter(slice_num); // Zeitstempel für die ansteigende Flanke
    }
    else if (events & GPIO_IRQ_EDGE_FALL) {
        if (pulse_start != 0) { // Sicherstellen, dass eine RISING EDGE vorliegt
            pulse_end = time_us_32();
            //pulse_end = pwm_get_counter(slice_num); // Zeitstempel für die fallende Flanke
            echo_received = true; // Signalisiert, dass ein vollständiges Echo empfangen wurde
            //pwm_set_counter(slice_num, 0); // Zähler zurücksetzen
        }
    }
}


int main() {
    timer_hw->dbgpause = 0x0; // Debugging pausieren

    // GPIO-Konfiguration

    // gpio_pull_down(echo_gpio);
    stdio_uart_init_full(uart0, 115200, 12, 13);

    // Initialisierung der LED-Matrix
    init_matrix();
    
    
    // Setup
    setup_pwm();
    setup_hardware_pwm();
    setup_gpio_interrupts();

    while (1)
    {
        // Ultraschalltrigger auslösen
        // gpio_put(init_gpio, 1);
        // sleep_us(10);
        // gpio_put(init_gpio, 0);

        // ECHO-Signal verarbeiten
        if (echo_received) {
            uint32_t duration = pulse_end - pulse_start;
            float raw_distance = (duration * 0.0343f) / 2; // cm

            if(raw_distance <= 5){
                raw_distance =0;
            }

            //distance = filter_distance(raw_distance);
            echo_received = false;

            uint progress = calculate_progress(raw_distance);
            update_progress(progress);

            // Debug-Ausgabe
            printf("Distance: %.2f cm, Progress: %u%%\n", distance, progress);
        }

        sleep_ms(50);
    }
    return 0;
}
