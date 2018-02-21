CC = g++

#pkg_packages = libavformat libavcodec libswscale libavutil
#PKG_CFLAGS = $(shell pkg-config --cflags $(pkg_packages))
#PKG_LDFLAGS = $(shell pkg-config --libs $(pkg_packages))

CFLAGS = -std=c++11 -fPIC $(PKG_CFLAGS) -Ibuild_system/target/include/
LDFLAGS += -Lbuild_system/target/lib/ -lavformat -lavcodec -lavutil -lpthread -lm

SRC = motion_watch.cpp

TARGET = motion_detect

all: $(TARGET)
all: CFLAGS += -O3

debug: $(TARGET)
debug: CFLAGS += -O0 -g

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) $(SRC) $(LDFLAGS) -lz -o $(TARGET)

clean:
	rm -f $(TARGET)
	rm -f *.o