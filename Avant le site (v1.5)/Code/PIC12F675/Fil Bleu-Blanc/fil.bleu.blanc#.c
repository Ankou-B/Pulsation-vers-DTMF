#include "pic12f675.h"
#include <xc.h>
#define _XTAL_FREQ 4000000UL

// ================== CONFIG ==================
#pragma config FOSC = INTRCIO  // Osc interne, GP4/GP5 en I/O
#pragma config WDTE = OFF
#pragma config PWRTE = OFF
#pragma config MCLRE = OFF
#pragma config BOREN = OFF
#pragma config CP = OFF
#pragma config CPD = OFF

// ================== CONSTANTES ==================
#define T_LONG_HOLD_MS       1000  // 1,5 s : appui long avant relâche
#define END_OF_TRAIN_MS       220  // fin de train d'impulsions
#define DEBOUNCE_MS             8  // anti-rebond
#define MIN_GAP_MS             25  // intervalle min entre fronts montants

// Nibbles TP5088 (D3..D0) d'après la table du datasheet
#define NIB_0  0b1010
#define NIB_1  0b0001
#define NIB_2  0b0010
#define NIB_3  0b0011
#define NIB_4  0b0100
#define NIB_5  0b0101
#define NIB_6  0b0110
#define NIB_7  0b0111
#define NIB_8  0b1000
#define NIB_9  0b1001
#define NIB_STAR 0b1011      // '*'
#define NIB_HASH 0b1100      // '#'

// ================== VARIABLES ==================
volatile unsigned long ms_tick = 0; // horloge libre (~1.024 ms / tick)
volatile unsigned long millis  = 0; // chrono réutilisé pour inter-impulsions

volatile unsigned char pulses = 0;     // nombre d'impulsions comptées
volatile unsigned char note   = 0;     // a-t-on reçu au moins une impulsion ?
volatile unsigned char last_pin = 0;   // mémorisation niveau GP3 (0/1)

volatile unsigned char state = 0;
#define ST_IDLE         0
#define ST_ARMED        1   // butée: bas maintenu avant 1er front montant
#define ST_COUNT        2   // train d'impulsions en cours
#define ST_WAIT_RELEASE 3   // on attend retour repos stable

// Mesures de temps (en ms_tick)
volatile unsigned long hold_start_ms = 0; // début palier bas (butée)
volatile unsigned long last_rise_ms  = 0; // dernier front montant
volatile unsigned char long_hold     = 0; // flag appui long détecté

// ================== PROTOTYPES ==================
void emit_dtmf_nibble(unsigned char nib, unsigned int on_ms);
void emit_digit_from_pulses(unsigned char p, unsigned char long_hold_flag);

// ================== ISR TMR0 -> 1 ms approx ==================
void __interrupt() ISR(void)
{
    if (INTCONbits.TMR0IE && INTCONbits.TMR0IF) {
        // TMR0 ~ 1.024 ms selon prescaler
        ms_tick++;
        millis++;
        INTCONbits.TMR0IF = 0;
    }
}

// ================== OUTILS TP5088 ==================
static inline void set_nibble(unsigned char nib) {
    // D3..D0 = GP5,GP4,GP0,GP1
    GPIObits.GP5 = (nib >> 3) & 1;
    GPIObits.GP4 = (nib >> 2) & 1;
    GPIObits.GP0 = (nib >> 1) & 1;
    GPIObits.GP1 = (nib >> 0) & 1;
}

void emit_dtmf_nibble(unsigned char nib, unsigned int on_ms)
{
    // Prépare les 4 bits
    set_nibble(nib);

    // Front montant sur TONE ENABLE pour "latcher" les données (datasheet)
    GPIObits.GP2 = 0;           // assure niveau bas
    __delay_us(10);
    GPIObits.GP2 = 1;           // latch + génération
    // Durée du tone
    while (on_ms--) __delay_ms(1);

    // Stoppe la génération
    GPIObits.GP2 = 0;
    // Remet les bits à 0 (propre)
    GPIObits.GP5 = 0;
    GPIObits.GP4 = 0;
    GPIObits.GP0 = 0;
    GPIObits.GP1 = 0;
}

void emit_digit_from_pulses(unsigned char p, unsigned char long_hold_flag)
{
    unsigned char nib;

    // 10 pulses = '0'
    if (p >= 10) p = 0;

    // Appui long: remapping spécial
    if (long_hold_flag) {
        if (p == 2) { nib = NIB_HASH; emit_dtmf_nibble(nib, 300); return; } // 2 ? '#'
        if (p == 1) { nib = NIB_STAR; emit_dtmf_nibble(nib, 300); return; } // 1 ? '*'
        // autres chiffres: comportement normal
    }

    switch (p) {
        case 0: nib = NIB_0; break;
        case 1: nib = NIB_1; break;
        case 2: nib = NIB_2; break;
        case 3: nib = NIB_3; break;
        case 4: nib = NIB_4; break;
        case 5: nib = NIB_5; break;
        case 6: nib = NIB_6; break;
        case 7: nib = NIB_7; break;
        case 8: nib = NIB_8; break;
        case 9: nib = NIB_9; break;
        default: nib = NIB_0; break; // sécurité
    }
    emit_dtmf_nibble(nib, 300);
}

// ================== MAIN ==================
void main(void)
{
    // ------- Init périphériques -------
    ANSEL = 0x00;      // tout en numérique
    CMCON = 0x07;      // désactive comparateurs
    TRISIO = 0b00001000;  // GP3 en entrée, le reste en sortie
    GPIO   = 0x00;

    // OPTION_REG:
    // b7=1 (pull-ups off), b6=0 (INTEDG), b5=0 (TMR0 clock=fosc/4),
    // b4=0 (T0SE), b3=0 (prescaler assigné à TMR0), b2..b0=001 (1:4)
    OPTION_REG = 0b10000001;

    // TMR0
    INTCONbits.TMR0IF = 0;
    INTCONbits.TMR0IE = 1;
    INTCONbits.GIE    = 1;

    // État initial entrée
    last_pin = GPIObits.GP3;     // repos attendu = 1 (5V)
    state = ST_IDLE;

    // ------- Boucle principale -------
    for(;;)
    {
        unsigned char pin = GPIObits.GP3;

        // ==== Détection fronts avec anti-rebond simple ====
        if (pin != last_pin) {
            // petit anti-rebond
            __delay_ms(DEBOUNCE_MS);
            pin = GPIObits.GP3;
        }

        // FRONT DESCENDANT : 5V -> 0V  : doigt sur butée
        if ((last_pin == 1) && (pin == 0)) {
            last_pin = 0;

            if (state == ST_IDLE) {
                state = ST_ARMED;
                hold_start_ms = ms_tick;   // début du palier bas
                long_hold = 0;
                pulses = 0;
                note = 0;
                // on laisse "millis" tel quel (il servira pour la fin de train)
            }
        }

        // FRONT MONTANT : 0V -> 5V  : début/continuité des impulsions
        if ((last_pin == 0) && (pin == 1)) {
            last_pin = 1;

            unsigned long now = ms_tick;

            if (state == ST_ARMED) {
                // 1er front montant => début du train
                unsigned long hold_ms = now - hold_start_ms;
                long_hold = (hold_ms >= T_LONG_HOLD_MS) ? 1 : 0;

                pulses = 1;
                note   = 1;
                millis = 0;                // chronomètre fin de train
                last_rise_ms = now;
                state  = ST_COUNT;
            }
            else if (state == ST_COUNT) {
                // Fronts montants suivants (compter en filtrant les rebonds)
                if ((now - last_rise_ms) >= MIN_GAP_MS) {
                    pulses++;
                    last_rise_ms = now;
                    millis = 0;            // reset fin de train
                }
            }
        }

        // ==== Fin de train : plus de fronts depuis un moment ====
        if ((state == ST_COUNT) && (note == 1) && (millis > END_OF_TRAIN_MS)) {
            // Train terminé : convertir et émettre
            emit_digit_from_pulses(pulses, long_hold);

            // Remise à zéro pour le prochain chiffre
            pulses = 0;
            note   = 0;
            long_hold = 0;
            state  = ST_WAIT_RELEASE;
        }

        // ==== Retour repos: s'assurer que le cadran est bien relâché ====
        if ((state == ST_WAIT_RELEASE) && (pin == 1)) {
            // pin=1 (5V) maintenu -> on peut repartir
            state = ST_IDLE;
        }
    }
}
