#ifndef KEY_H_
#define KEY_H_

#include <stdint.h>

typedef enum { key1_putong, key2_teshu, key3_baidong, key4_idle } keymode;
typedef enum { id1, id2, id3, id4 } keyid;

int Key_ReadPress(keyid id); 
void key_selectmode(keyid id);
void key_tick5ms(void);
void key_init(void);
keymode key_getmode(void);
uint32_t key_get_key4_confirm_count(void);
uint32_t key_get_key3_confirm_count(void);

#endif
