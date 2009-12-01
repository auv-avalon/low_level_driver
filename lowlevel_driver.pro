TEMPLATE = app
QT = gui core network
CONFIG+=debug
DEPENDPATH += 
include ("../../programs.pri")


DEFINES += EKF_ESTIMATE_POSITION
DEFINES += CALIBRATION_MODE
DEFINES += EIGEN_DONT_VECTORIZE NSTDLIB CALIBRATION_DATA=\"<filter/calibration_data.cpp>\"
HEADERS += ../../../libs/imu/filter/StaticConfiguration.h types.h body.h

LIBS += -lsensors

# Input
HEADERS += 	lowlevel_processor.h \
		 ../../../../carlton-imu-code/protocol/protocol.h \
		 ../../../firmware/crumb-front/avalon_protocol.h \
		 avalon_protocol_functions.h  \
		 avalon_protocol_callbacks.h \
		 XsensThread.h
#		serial_protocol.h \
		

SOURCES += 	main.cpp \
		lowlevel_processor.cpp  \
		 ../../../../carlton-imu-code/protocol/protocol.cpp \
		 ../../../firmware/crumb-front/avalon_protocol.c \
		 avalon_protocol_functions.c \
		 avalon_protocol_callbacks.cpp \
		 XsensThread.cpp
#		serial_protocol.cpp \

DEFINES += PVDECL="" DLLEXPORT="" _x64 LINUX

