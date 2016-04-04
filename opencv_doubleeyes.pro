#-------------------------------------------------
#
# Project created by QtCreator 2015-10-26T15:50:59
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = opencv_test
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += main.cpp

HEADERS += Okapi64.h


INCLUDEPATH += D:\PosNo2\doubleeyes\opencv3.0\opencv\build\include
INCLUDEPATH += D:\PosNo2\doubleeyes\OkDemo\SDK_APIs

win32:CONFIG(debug, debug|release): {
LIBS += -LD:\PosNo2\doubleeyes\opencv3.0\opencv\build\x64\vc12\lib \
#LIBS += -LD:\PosNo2\doubleeyes\opencv3.0\opencv\build\x64\vc12\staticlib \
-lopencv_world300d \
-lopencv_ts300d \

} else:win32:CONFIG(release, debug|release): {
LIBS += -LD:\PosNo2\doubleeyes\opencv3.0\opencv\build\x64\vc12\lib \
-lopencv_core300 \
-lopencv_imgproc300 \
-lopencv_highgui300 \
-lopencv_ml300 \
-lopencv_video300 \
-lopencv_features2d300 \
-lopencv_calib3d300 \
-lopencv_objdetect300 \
-lopencv_contrib300 \
-lopencv_legacy300 \
-lopencv_flann300
}
#LIBS += -lws2_32 -lKernel32 -lIphlpapi -lWinmm
LIBS += -lUser32

win32:CONFIG(debug, debug|release): {
LIBS += -LD:\PosNo2\doubleeyes\OkDemo\SDK_APIs \
-lokapi64 \
}



