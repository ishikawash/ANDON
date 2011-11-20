
CXX := g++
CFLAGS := -Wall -Wextra -O3 -I/opt/local/include -I$(HOME)/local/include 
LDFLAGS := -L/opt/local/lib -L$(HOME)/local/lib -lopenctm -ltbb
OBJECTS := $(patsubst %.cpp,%.o,$(wildcard *.cpp))

ifndef TARGET
  TARGET := main
endif


all: $(TARGET) 

$(TARGET): $(OBJECTS)
	$(CXX) $(CFLAGS) $(LDFLAGS) $(OBJECTS) -o $@

%.o: %.cpp
	$(CXX) $(CFLAGS) $< -c

run: $(TARGET)
	time $(PWD)/$(TARGET) happy-budda.ctm && ppm2tiff out.ppm out.tiff 

clean:
	rm -f *.o $(TARGET)
