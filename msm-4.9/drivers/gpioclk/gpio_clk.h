#ifndef _GPIO_CLK_H_
#define _GPIO_CLK_H_

int gpio_clock_duty_set(unsigned int value);
int gpio_clk_prepare_enable(void);
void gpio_clk_disable_unprepare(void);
int gpio_clock_freq_set(unsigned long value);

#endif
