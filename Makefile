CXXFLAGS+= -fPIC -g -Wall -D_REENTRANT -fno-exceptions

ROOT_INCLUDE = /usr/include
ROOT_LIB = /usr/lib
LOCALLIB=/usr/local/BumbleBee2/examples-libdc-2/pgrlibdcstreo

CPPFLAGS= -g
CPPFLAGS+= -DLINUX

# OPENCV
CPPFLAGS+=`pkg-config --cflags opencv`
LDFLAGS=`pkg-config --libs opencv`

# ARIA
CPPFLAGS+=-I/usr/local/Aria/include
LDFLAGS+=-L/usr/local/Aria/lib -lAria -lArNetworking -lpthread -ldl -lrt

# Eigen
CPPFLAGS+=`pkg-config --cflags eigen3`

# PGR
LDFLAGS+=-L/usr/local/BumbleBee2/examples-libdc-2/pgrlibdcstereo
CPPFLAGS+=-I/usr/local/BumbleBee2/examples-libdc-2/pgrlibdcstereo #PN 08/28
#change to local version of library to debug extract color images 10/18
#CPPFLAGS+=-I/usr/local/BumbleBee2/newPGR/pgrlibdcstereo #PN 08/28
#LDFLAGS+=-L/usr/local/BumbleBee2/newPGR/pgrlibdcstereo #PN 08/28

# Triclops
CPPFLAGS+=-I/usr/local/triclops/include
LDFLAGS+=-L/usr/local/triclops/lib

# DC1394
CPPFLAGS+=-I$(ROOT_INCLUDE)/dc1394
LDFLAGS+= -ldc1394 -lraw1394 -pthread

LDFLAGS+=-lpgrlibdcstereo -ltriclops -lpnmutils

CPPFLAGS+=-I$(LOCALLIB)
LDFLAGS+=-L$(LOCALLIB)
LDFLAGS+=-L$(ROOT_LIB)
LDFLAG+=-L/usr/lib

# --------------------- Code modules ----------------------------

# Object files
OBJ = util.c stereoCamera.cpp vh.cpp home.cpp

# Definitions
DEFS = defs.h vh.h stereoCamera.h

# ------------------------ Rules --------------------------------

home: ${OBJ}
	g++ ${CPPFLAGS} ${OBJ} ${LDFLAGS} -o $@

clean: 
	rm -f *~ stereo-data/*


# Implicit rule used by Gnu Make: $(CC) -c $(CPPFLAGS) $(CFLAGS)
#${OBJ}: ${DEFS}
