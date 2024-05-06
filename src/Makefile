# MS-MPI + MinGW-w64 MakeFile used by Kevin Fang (kevinfan)
# Running From The Night: Calculating The Lunar Magellan Route in Parallel

CXX = g++

CXXFLAGS = -O3 -I"C:/Program Files (x86)/Microsoft SDKs/MPI/Include"

LDFLAGS = -L"C:/Program Files (x86)/Microsoft SDKs/MPI/Lib/x64" -lmsmpi

TARGET = main.exe

SOURCES = main.cpp

all: $(TARGET)

$(TARGET): $(SOURCES)
	$(CXX) -o $(TARGET) $(SOURCES) $(CXXFLAGS) $(LDFLAGS)

clean:
	del $(TARGET)
