#ifndef _ext_irq_h
#define _ext_irq_h

#define EXT_NUM_EXTI        23

typedef void extCallback_t(void);

typedef struct {
    extCallback_t *callbacks[EXT_NUM_EXTI];
} extStruct_t;

extern int extRegisterCallback(GPIO_TypeDef *port, uint16_t pin, EXTITrigger_TypeDef polarity, uint8_t priority, extCallback_t *callback);

#endif