MCU    = atmega328p
F_CPU  = 16000000UL
PORT   = /dev/ttyACM0
BAUD   = 115200
PATH_DIRECTORY = $(CURDIR)
FOLDER_NAME = $(notdir $(PATH_DIRECTORY))

C_FILE = $(wildcard *.c)

all: $(FOLDER_NAME).hex

$(FOLDER_NAME).elf: $(C_FILE)
	avr-gcc -mmcu=$(MCU) -DF_CPU=$(F_CPU) -O2 $(C_FILE) -o $(FOLDER_NAME).elf

$(FOLDER_NAME).hex: $(FOLDER_NAME).elf
	avr-objcopy -O ihex $(FOLDER_NAME).elf $(FOLDER_NAME).hex

flash: $(FOLDER_NAME).hex
	avrdude -c arduino -p $(MCU) -P $(PORT) -b $(BAUD) -U flash:w:$(FOLDER_NAME).hex:i

clean:
	rm -f $(FOLDER_NAME).elf $(FOLDER_NAME).hex
