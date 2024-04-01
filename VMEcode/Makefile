########################################################################

EXE	=	proANUBIS

CC	=	g++

COPTS   =       -fPIC -DLINUX -Wall -std=c++11 `root-config --cflags`

FLAGS	=	-Wall -s `root-config --cflags`

DEPLIBS	=       -l CAENVME -lc -lm `root-config --glibs`

INCLUDEDIR =	-I.
#INCLUDEDIR =	-I../include #if we want to include files locally, and this is an old code
#INCLUDES =	../include/CAENVMElib.h ../include/CAENVMEtypes.h ../include/CAENVMEoslib.h

OBJS	=	proANUBIS.o

#########################################################################

all	:	$(EXE)

clean	:
		/bin/rm -f $(OBJS) $(EXE) run_summary*

$(EXE)	:	$(OBJS)
		/bin/rm -f $(EXE)
		$(CC) $(FLAGS) -o $(EXE) $(OBJS) $(DEPLIBS)

$(OBJS)	:	$(INCLUDES) Makefile

%.o	:	%.c
		$(CC) $(COPTS) $(INCLUDEDIR) -c -o $@ $<

