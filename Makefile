TARGET = main

CROSS_TOOL=
CC_C = $(CROSS_TOOL)arm-linux-gnueabihf-gcc

CFLAGS = -Wall -g  -Werror -pthread -ldl -static -I./COMMON/ \
							BCM/bcm2835.c \
							nRF_Drivers/nrf24l01.c \
							main.c			
all: clean $(TARGET) 

$(TARGET): 
	$(CC_C) -o $(TARGET) $(CFLAGS)
	
clean:
	rm -f $(TARGET)

.PHONY: clean











