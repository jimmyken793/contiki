#ifndef DENNAO_INTERRUPT_H
#define DENNAO_INTERRUPT_H

void attachInterrupt(uint8_t, void (*)(void), int mode);
void detachInterrupt(uint8_t);

#endif