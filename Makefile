UNAME_S := $(shell uname -s)

SRC     = boids.c
OUT     = boids
CFLAGS  = -Wall -Wextra

ifeq ($(UNAME_S),Linux)
    CC      = gcc
    LDFLAGS = -lraylib -lGL -lm -lpthread -ldl -lrt -lX11 -g
endif

ifeq ($(OS),Windows_NT)
    CC      = gcc
    LDFLAGS = -lraylib -lopengl32 -lgdi32 -lwinmm
endif

all:
	$(CC) $(SRC) -o $(OUT) $(CFLAGS) $(LDFLAGS)

clean:
	rm -f $(OUT)

