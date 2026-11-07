#include "key.h"
#include "main.h"
#include "pidtrack.h"

#define KEY_COUNT 3U
#define KEY_DEBOUNCE_TICKS 6U

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} keypin;

static const keypin key_pins[KEY_COUNT] = {
    {KEY1_GPIO_Port, KEY1_Pin},
    {KEY2_GPIO_Port, KEY2_Pin},
    {KEY3_GPIO_Port, KEY3_Pin}
};

typedef enum
{ 
    state_idle,
    state_press_debounce, 
    state_press, 
    state_release 
}keystate;

typedef struct 
{ 
    keystate state; 
    uint8_t ticks; 
}keycontrol;

static keycontrol keys[KEY_COUNT];
static volatile keymode mode = key4_idle;
static volatile uint32_t key4_confirm_count;

void key_init(void)
{
    for (int i=0;i<KEY_COUNT;i++){
        keys[i].state=state_idle;
        keys[i].ticks=0;
    }
    mode=key4_idle;
    key4_confirm_count=0;
}

int Key_ReadPress(keyid id)
{
    if ((unsigned int)id >= KEY_COUNT) return 1;
    return HAL_GPIO_ReadPin(key_pins[id].port, key_pins[id].pin) == GPIO_PIN_RESET ? 0 : 1;
}

void key_selectmode(keyid id)
{
    switch (id) {
    case id1: mode = key1_putong; break;
    case id2: mode = key2_teshu; break;
    case id3: mode = key3_baidong; break;
    case id4:
        ++key4_confirm_count;
        mode = key4_idle;
        set_target(0, 0);
        break;
    default: break;
    }
}

void key_tick5ms(void)
{
    unsigned int i;
    for (i = 0; i < KEY_COUNT; ++i) {
        int pressed = Key_ReadPress((keyid)i) == 0;
        switch (keys[i].state) {
        case state_idle:
            if (pressed) { keys[i].state = state_press_debounce; keys[i].ticks = 1; }
            break;
        case state_press_debounce:
            if (!pressed) { keys[i].state = state_idle; keys[i].ticks = 0; }
            else if (++keys[i].ticks >= KEY_DEBOUNCE_TICKS) {
                keys[i].state = state_press;
                keys[i].ticks = 0;
                key_selectmode((keyid)i);
            }
            break;
        case state_press:
            if (!pressed) { keys[i].state = state_release; keys[i].ticks = 0; }
            break;
        case state_release:
            if (pressed) { keys[i].state = state_press; keys[i].ticks = 0; }
            else if (++keys[i].ticks >= KEY_DEBOUNCE_TICKS) {
                keys[i].state = state_idle;
                keys[i].ticks = 0;
            }
            break;
        default: keys[i].state = state_idle; keys[i].ticks = 0; break;
        }
    }
}

keymode key_getmode(void) { return mode; }
uint32_t key_get_key4_confirm_count(void) { return key4_confirm_count; }
