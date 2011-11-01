
CXX := g++
# CFLAGS := -Wall -g -I/opt/local/include -I$(HOME)/local/include 
CFLAGS := -Wall -Wextra -O3 -I/opt/local/include -I$(HOME)/local/include 
LDFLAGS := -L$(HOME)/local/lib -lopenctm
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
	time $(PWD)/$(TARGET) happy-budda.ctm && ppm2tiff out.ppm out.tiff && open out.tiff

clean:
	rm -f *.o $(TARGET)
