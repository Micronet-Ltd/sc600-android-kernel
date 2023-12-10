obj-$(CONFIG_GPIO_CLK)	+= gpio_clk.o

ifeq ($(TARGET_BUILD_VARIANT),user)
$(obj)/gpio_clk.o: $(srctree)/$(obj)/gpio_clk.user FORCE
	cp $(srctree)/$(obj)/gpio_clk.user $(obj)/gpio_clk.o
else
ifeq ($(CONFIG_FUNCTION_TRACER),y)
$(obj)/gpio_clk.o: $(srctree)/$(obj)/gpio_clk FORCE
	cp $(srctree)/$(obj)/gpio_clk $(obj)/gpio_clk.o
else
$(obj)/gpio_clk.o: $(srctree)/$(obj)/gpio_clk.user FORCE
	cp $(srctree)/$(obj)/gpio_clk.user $(obj)/gpio_clk.o
endif
endif
