#-------------------------------------------------
#
# Project created by QtCreator 2017-07-24T09:15:35
#
#-------------------------------------------------

QMAKE_CXXFLAGS += -std=c++0x -pthread
QMAKE_CXXFLAGS +=  " -fsanitize=address   -fno-omit-frame-pointer -g"
QMAKE_CFLAGS+="-fsanitize=address  -fno-omit-frame-pointer -g"
QMAKE_LFLAGS+="-fsanitize=address -g"

QT       += core gui
QT       += network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets
LIBS += -Wl,--no-as-needed -lpthread
INCLUDEPATH += /usr/local/include/
LIBS +=-L/usr/local/lib
LIBS +=-lopencv_core -lopencv_highgui  -lopencv_calib3d -lopencv_features2d -lopencv_imgcodecs -lopencv_videoio -lopencv_imgproc

TARGET = Laser_GUI
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += main.cpp\
        mainwindow.cpp \
    contour_analysing.cpp \
    my_tcpsocket.cpp

HEADERS  += mainwindow.h \
    contour_analysing.h \
    my_tcpsocket.h

FORMS    += mainwindow.ui
