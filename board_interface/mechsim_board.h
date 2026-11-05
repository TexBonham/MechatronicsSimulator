/* MechSim board interface: the only coupling between the simulator and firmware (invariant 8; FW-003, FW-009, FW-010).
 *
 * Native mode is polled (C3; CTL-004): the runtime fills one mechsim_board_inputs before each tick, calls
 * mechsim_app_tick, then reads mechsim_board_outputs. No interrupts. Levels are latched at the start of the tick;
 * counters carry the counts accumulated during the tick; byte buffers carry what arrived or must leave during one tick.
 * Sizes are fixed contract constants (docs/spec/fw.md, Open parameters), mirrored by the C++ shim.
 */
#ifndef MECHSIM_BOARD_H
#define MECHSIM_BOARD_H

#include <stdint.h>

#define MECHSIM_BOARD_INTERFACE_VERSION_MAJOR 2u
#define MECHSIM_BOARD_INTERFACE_VERSION_MINOR 2u
#define MECHSIM_BOARD_INTERFACE_VERSION \
    ((MECHSIM_BOARD_INTERFACE_VERSION_MAJOR * 65536u) + MECHSIM_BOARD_INTERFACE_VERSION_MINOR)

#define MECHSIM_DIGITAL_IN_COUNT 32u
#define MECHSIM_COUNTER_COUNT 8u
#define MECHSIM_ADC_COUNT 16u
#define MECHSIM_UART_RX_BYTES 64u
#define MECHSIM_UART_TX_BYTES 64u
#define MECHSIM_AXIS_COUNT 8u
#define MECHSIM_DIGITAL_OUT_COUNT 32u
#define MECHSIM_EVENT_LOG_COUNT 16u

#define MECHSIM_LEVEL_LOW 0u
#define MECHSIM_LEVEL_HIGH 1u

/* The build defines this from the design's control rate (C4); the simulator errors when it differs from the design. */
#ifndef MECHSIM_APP_TICK_HZ
#define MECHSIM_APP_TICK_HZ 1000u
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct mechsim_board_inputs {
    /* Level of each digital input pin, MECHSIM_LEVEL_LOW or MECHSIM_LEVEL_HIGH, latched at the start of the tick. */
    uint8_t digital_in[MECHSIM_DIGITAL_IN_COUNT];
    /* Signed counts accumulated on each counter channel during the tick just ended (CTL-004); 0 when idle. */
    int32_t counter_delta[MECHSIM_COUNTER_COUNT];
    /* Raw ADC code per channel in LSB, the latest conversion latched at the start of the tick. */
    uint16_t adc_sample[MECHSIM_ADC_COUNT];
    /* UART bytes received during the tick just ended, oldest first; only the first uart_rx_count are valid. */
    uint8_t uart_rx[MECHSIM_UART_RX_BYTES];
    /* Number of valid bytes in uart_rx, 0 to MECHSIM_UART_RX_BYTES. */
    uint16_t uart_rx_count;
} mechsim_board_inputs;

typedef struct mechsim_board_event {
    /* Firmware-defined event code; meaningful only for entries below event_count. */
    uint8_t code;
    /* Firmware-defined argument of the event. */
    uint16_t arg;
} mechsim_board_event;

typedef struct mechsim_board_outputs {
    /* Step pulses per axis to emit during the next tick; 0 for no motion. */
    uint32_t step_count[MECHSIM_AXIS_COUNT];
    /* Direction level per axis, held from the end of this tick until the next tick writes it. */
    uint8_t direction[MECHSIM_AXIS_COUNT];
    /* Driver enable level per axis, held from the end of this tick until the next tick writes it. */
    uint8_t enable[MECHSIM_AXIS_COUNT];
    /* Level of each digital output pin, held from the end of this tick until the next tick writes it. */
    uint8_t digital_out[MECHSIM_DIGITAL_OUT_COUNT];
    /* UART bytes to send after this tick, oldest first; only the first uart_tx_count are sent. */
    uint8_t uart_tx[MECHSIM_UART_TX_BYTES];
    /* Number of valid bytes in uart_tx, 0 to MECHSIM_UART_TX_BYTES. */
    uint16_t uart_tx_count;
    /* Firmware-defined controller state at the end of this tick, published each tick (CTL-018). */
    uint8_t state_code;
    /* Firmware-defined fault at the end of this tick, 0 for none, published each tick (CTL-018). */
    uint8_t fault_code;
    /* Events raised during this tick, oldest first (CTL-018). */
    mechsim_board_event event_log[MECHSIM_EVENT_LOG_COUNT];
    /* Number of valid entries in event_log, 0 to MECHSIM_EVENT_LOG_COUNT; starts at 0 every tick. */
    uint8_t event_count;
} mechsim_board_outputs;

/* Called once after each reset, before the first tick. */
void mechsim_app_init(void);

/* Called once per control tick at mechsim_app_tick_hz(). Both pointers are non-null; outputs hold no defined values on
 * entry, so the firmware writes every field. */
void mechsim_app_tick(const mechsim_board_inputs *inputs, mechsim_board_outputs *outputs);

/* MECHSIM_APP_TICK_HZ as built, in hertz; the simulator reads it once at load and checks it against the design. */
uint32_t mechsim_app_tick_hz(void);

/* MECHSIM_BOARD_INTERFACE_VERSION of the header the application was compiled against; the simulator reads it once at
 * load and rejects a library built against another version. */
uint32_t mechsim_app_board_interface_version(void);

#ifdef __cplusplus
}
#endif

#endif
