CC = gcc

CFLAGS = -L/usr/local/lib -lwiringPi -lm -pthread

TARGET = main

SUPPORT = $(wildcard *.c)

all: $(TARGET) $(SUPPORT)

$(TARGET): $(TARGET).c
	$(CC) -o $(TARGET) $(CFLAGS) $(SUPPORT)
	
clean:
	$(RM) $(TARGET)
	
run: $(TARGET)
	./$(TARGET)
