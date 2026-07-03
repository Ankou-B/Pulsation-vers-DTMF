/*
 * pms150c_rotary_tp5088_v15_faster_end.c
 *
 * Version v15 : v14 fonctionnelle avec detection de fin de train plus rapide :
 *   chiffre 1 = haut -> bas armement -> haut -> bas impulsion utile -> haut
 *
 * Par defaut ROTARY_IGNORE_FIRST_LOW=1 : le premier bas est un armement
 * non compte. Les bas suivants sont les impulsions utiles.
 * Pour un telephone sans bas d'armement, compiler avec :
 *   -DROTARY_IGNORE_FIRST_LOW=0
 *
 * Securite : en cas de doute, silence.
 */

#include <stdint.h>
#include "easypdk/pdk.h"

#ifndef CAL_VDD_MV
#define CAL_VDD_MV 5000
#endif

#ifndef ROTARY_IGNORE_FIRST_LOW
#define ROTARY_IGNORE_FIRST_LOW 1
#endif

#ifndef ENABLE_LONG_HOLD_SPECIAL
/* Active par defaut : appui long a la butee pour *, # et +.
 * Pour desactiver : -DENABLE_LONG_HOLD_SPECIAL=0
 */
#define ENABLE_LONG_HOLD_SPECIAL 1
#endif

#define START_HIGH_STABLE_MS        200u
#define DEBOUNCE_TICKS               5u
#define MIN_ARM_LOW_MS              20u
#define MAX_ARM_LOW_MS            5000u
#define ARM_NO_PULSE_ABORT_MS       900u
#define MIN_LOW_MS                   8u
#define MAX_PULSE_LOW_MS           260u
#define MIN_HIGH_BEFORE_PULSE_MS     8u
#ifndef END_OF_TRAIN_MS
#define END_OF_TRAIN_MS            150u
#endif
#define ABORT_HIGH_STABLE_MS       500u
#ifndef T_LONG_HOLD_MS
#define T_LONG_HOLD_MS            1000u //1s est plus adapté, laissez 2s dans le manuel pour la protection anti-con
#endif
#define TONE_ON_MS                 300u
#define TONE_OFF_MS                 70u
#define MAX_PULSES                  10u

/* Le DTMF ne possede pas de symbole + standard.
 * Par defaut, un + est transforme en prefixe international 00, pratique en France/Europe.
 * Si tu veux envoyer le ton DTMF A a la place, compile avec -DPLUS_MODE=1.
 */
#define PLUS_MODE_PREFIX_00 0u
#define PLUS_MODE_DTMF_A    1u
#ifndef PLUS_MODE
#define PLUS_MODE PLUS_MODE_PREFIX_00
#endif

#define PLUS_INTER_TONE_MS          90u

#define NIB_0     0x0Au
#define NIB_1     0x01u
#define NIB_2     0x02u
#define NIB_3     0x03u
#define NIB_4     0x04u
#define NIB_5     0x05u
#define NIB_6     0x06u
#define NIB_7     0x07u
#define NIB_8     0x08u
#define NIB_9     0x09u
#define NIB_STAR  0x0Bu
#define NIB_HASH  0x0Cu
#define NIB_A     0x0Du

#define PIN_D1    0x01u  /* PA0 */
#define PIN_TE    0x08u  /* PA3 */
#define PIN_D0    0x10u  /* PA4 */
#define PIN_ROT   0x20u  /* PA5 / PRSTB */
#define PIN_D2    0x40u  /* PA6 */
#define PIN_D3    0x80u  /* PA7 */

#define PIN_DATA  (PIN_D3 | PIN_D2 | PIN_D1 | PIN_D0)
#define PIN_OUT   (PIN_DATA | PIN_TE)

#define ST_IDLE       0u
#define ST_ARM_LOW    1u
#define ST_BETWEEN    2u
#define ST_PULSE_LOW  3u
#define ST_ABORT      4u

static volatile uint16_t g_ms;
static uint8_t pa_shadow;
static uint8_t stable_level;
static uint8_t debounce_count;
static uint8_t state;
static uint8_t pulse_count;
static uint8_t long_hold_flag;
static uint16_t arm_low_ms;
static uint16_t low_ms;
static uint16_t high_ms;

void interrupt(void) __interrupt(0)
{
    if (INTRQ & INTRQ_T16) {
        g_ms++;
        INTRQ &= (uint8_t)(~INTRQ_T16);
    }
}

unsigned char __sdcc_external_startup(void)
{
    EASY_PDK_INIT_SYSCLOCK_1MHZ();
    EASY_PDK_CALIBRATE_IHRC(1000000, CAL_VDD_MV);
    return 0;
}

static uint16_t get_ms(void)
{
    uint16_t t;
    __asm
        disgint
    __endasm;
    t = g_ms;
    __asm
        engint
    __endasm;
    return t;
}

static void wait_ms(uint16_t delay)
{
    uint16_t start;
    start = get_ms();
    while ((uint16_t)(get_ms() - start) < delay) {
        /* wait */
    }
}

static void write_outputs(uint8_t out)
{
    /* PA5/PRSTB reste entree ; son latch reste a 1. */
    pa_shadow = (uint8_t)(out | PIN_ROT);
    PA = pa_shadow;
}

static uint8_t read_rot_raw(void)
{
    return (PA & PIN_ROT) ? 1u : 0u;
}

static uint8_t read_rot_debounced(void)
{
    uint8_t raw;
    raw = read_rot_raw();

    if (raw == stable_level) {
        debounce_count = 0u;
    } else {
        if (debounce_count < DEBOUNCE_TICKS) {
            debounce_count++;
        }
        if (debounce_count >= DEBOUNCE_TICKS) {
            stable_level = raw;
            debounce_count = 0u;
        }
    }
    return stable_level;
}

static void tp5088_idle(void)
{
    write_outputs(0u);
}

static void timer16_init(void)
{
    g_ms = 0u;
    INTEGS = INTEGS_T16_RISING;
    T16M = T16_CLK_SYSCLK | T16_CLK_DIV1 | T16_INTSRC_9BIT;
    T16C = 0u;
    INTRQ = 0u;
    INTEN = INTEN_T16;
    __asm
        engint
    __endasm;
}

static void io_init(void)
{
    PA = PIN_ROT;
    PAC = PIN_OUT;       /* PA5 reste entree */
    PADIER = PIN_ROT;    /* seule PA5 est lue */
    PAPH = PIN_ROT;      /* pull-up interne + pull-up externe */
    GPCC = 0u;
    GPCS = 0u;
    TM2C = 0u;
    TM2S = 0u;
    TM2B = 0u;
    tp5088_idle();
    stable_level = read_rot_raw();
    debounce_count = 0u;
}

static void tp5088_set_nibble(uint8_t nib)
{
    uint8_t out;
    out = 0u;
    if (nib & 0x08u) out |= PIN_D3;
    if (nib & 0x04u) out |= PIN_D2;
    if (nib & 0x02u) out |= PIN_D1;
    if (nib & 0x01u) out |= PIN_D0;
    write_outputs(out);
}

static void emit_dtmf_nibble(uint8_t nib)
{
    tp5088_set_nibble(nib);
    wait_ms(3u);
    pa_shadow |= PIN_TE;
    PA = pa_shadow;
    wait_ms(TONE_ON_MS);
    tp5088_idle();
    wait_ms(TONE_OFF_MS);
}

static void emit_plus(void)
{
#if PLUS_MODE == PLUS_MODE_DTMF_A
    /* Ton A du clavier DTMF 4x4. Attention : rarement compris par les box/ATA. */
    emit_dtmf_nibble(NIB_A);
#else
    /* Equivalent pratique du + sur une ligne telephonique : prefixe international 00. */
    emit_dtmf_nibble(NIB_0);
    wait_ms(PLUS_INTER_TONE_MS);
    emit_dtmf_nibble(NIB_0);
#endif
}

static void emit_digit(uint8_t digit)
{
    uint8_t nib;

#if ENABLE_LONG_HOLD_SPECIAL
    if (long_hold_flag != 0u) {
        if (digit == 1u) {
            emit_dtmf_nibble(NIB_STAR);
            return;
        }
        if (digit == 2u) {
            emit_dtmf_nibble(NIB_HASH);
            return;
        }
        if (digit == 3u) {
            emit_plus();
            return;
        }
    }
#endif

    if (digit == 0u) nib = NIB_0;
    else if (digit == 1u) nib = NIB_1;
    else if (digit == 2u) nib = NIB_2;
    else if (digit == 3u) nib = NIB_3;
    else if (digit == 4u) nib = NIB_4;
    else if (digit == 5u) nib = NIB_5;
    else if (digit == 6u) nib = NIB_6;
    else if (digit == 7u) nib = NIB_7;
    else if (digit == 8u) nib = NIB_8;
    else if (digit == 9u) nib = NIB_9;
    else return;

    emit_dtmf_nibble(nib);
}

static void reset_decoder(void)
{
    state = ST_IDLE;
    pulse_count = 0u;
    long_hold_flag = 0u;
    arm_low_ms = 0u;
    low_ms = 0u;
    high_ms = 0u;
}

static void abort_decoder(void)
{
    state = ST_ABORT;
    pulse_count = 0u;
    long_hold_flag = 0u;
    arm_low_ms = 0u;
    low_ms = 0u;
    high_ms = 0u;
}

static void startup_wait_high(void)
{
    uint16_t high_stable;
    high_stable = 0u;
    while (high_stable < START_HIGH_STABLE_MS) {
        wait_ms(1u);
        if (read_rot_debounced()) high_stable++;
        else high_stable = 0u;
    }
}

static void finish_if_train_done(void)
{
    if (pulse_count == 10u) {
        emit_digit(0u);
    } else if ((pulse_count >= 1u) && (pulse_count <= 9u)) {
        emit_digit(pulse_count);
    }
    reset_decoder();
}

static void process_level(uint8_t level)
{
    if (state == ST_IDLE) {
        if (level == 0u) {
#if ROTARY_IGNORE_FIRST_LOW
            state = ST_ARM_LOW;
            arm_low_ms = 0u;
#else
            state = ST_PULSE_LOW;
            low_ms = 0u;
#endif
            high_ms = 0u;
            pulse_count = 0u;
            long_hold_flag = 0u;
        }
    }

#if ROTARY_IGNORE_FIRST_LOW
    else if (state == ST_ARM_LOW) {
        if (level == 0u) {
            if (arm_low_ms < 60000u) arm_low_ms++;
            if (arm_low_ms > MAX_ARM_LOW_MS) abort_decoder();
        } else {
            if (arm_low_ms < MIN_ARM_LOW_MS) {
                reset_decoder();
            } else {
                long_hold_flag = (arm_low_ms >= T_LONG_HOLD_MS) ? 1u : 0u;
                state = ST_BETWEEN;
                high_ms = 0u;
                low_ms = 0u;
            }
        }
    }
#endif

    else if (state == ST_BETWEEN) {
        if (level != 0u) {
            if (high_ms < 60000u) high_ms++;
#if ROTARY_IGNORE_FIRST_LOW
            if (pulse_count == 0u) {
                if (high_ms >= ARM_NO_PULSE_ABORT_MS) reset_decoder();
            } else
#endif
            if (pulse_count != 0u) {
                if (high_ms >= END_OF_TRAIN_MS) finish_if_train_done();
            }
        } else {
            if (high_ms < MIN_HIGH_BEFORE_PULSE_MS) abort_decoder();
            else {
                state = ST_PULSE_LOW;
                low_ms = 0u;
            }
        }
    }

    else if (state == ST_PULSE_LOW) {
        if (level == 0u) {
            if (low_ms < 60000u) low_ms++;
            if (low_ms > MAX_PULSE_LOW_MS) abort_decoder();
        } else {
            if (low_ms >= MIN_LOW_MS) {
                if (pulse_count < MAX_PULSES) pulse_count++;
                else abort_decoder();
            }
            state = ST_BETWEEN;
            high_ms = 0u;
            low_ms = 0u;
        }
    }

    else if (state == ST_ABORT) {
        if (level != 0u) {
            if (high_ms < 60000u) high_ms++;
            if (high_ms >= ABORT_HIGH_STABLE_MS) reset_decoder();
        } else {
            high_ms = 0u;
        }
    }

    else {
        reset_decoder();
    }
}

void main(void)
{
    uint8_t level;
    io_init();
    timer16_init();
    startup_wait_high();
    reset_decoder();

    for (;;) {
        wait_ms(1u);
        level = read_rot_debounced();
        process_level(level);
    }
}
