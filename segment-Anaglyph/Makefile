INCDIR = -I.
DBG    = -g
OPT    = -O3
CPP    = g++
CFLAGS = $(DBG) $(OPT) $(INCDIR)
LINK   = -lm `pkg-config --libs --cflags opencv`

.cpp.o:
	$(CPP) $(CFLAGS) -c $< -o $@

all: segment

segment: segment_mat.cpp segment-mat.h segment-graph.h disjoint-set.h
	$(CPP) $(CFLAGS) -o segment segment_mat.cpp $(LINK)

clean:
	/bin/rm -f segment *.o

clean-all: clean
	/bin/rm -f *~ 



