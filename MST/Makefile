CC      := gcc
AR      := ar

CFLAGS  := -std=c99 -Wall -Wextra -pedantic
DEBUG   := -g -O0
RELEASE := -O2

TARGET  := app
LIBRARY := libprim.a

SRC_LIB := prim.c
OBJ_LIB := prim.o

SRC_APP := test.c
OBJ_APP := test.o

.PHONY: all debug release clean

all: release

release: CFLAGS += $(RELEASE)
release: $(LIBRARY) $(TARGET)

debug: CFLAGS += $(DEBUG)
debug: $(LIBRARY) $(TARGET)

$(LIBRARY): $(OBJ_LIB)
	$(AR) rcs $@ $^

$(TARGET): $(OBJ_APP) $(LIBRARY)
	$(CC) $(CFLAGS) -o $@ $(OBJ_APP) -L. -lprim

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f *.o
	rm -f $(LIBRARY)
	rm -f $(TARGET)