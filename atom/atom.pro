TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp

INCLUDEPATH += $$PWD/../chai3d-3.0.0/src
INCLUDEPATH += $$PWD/../chai3d-3.0.0/external/Eigen
INCLUDEPATH += $$PWD/../chai3d-3.0.0/external/glew/include

win32{
    CONFIG(release, debug|release): LIBS += -L$$PWD/../chai3d-3.0.0/lib/Release/x64/ -lchai3d
    else:CONFIG(debug, debug|release): LIBS += -L$$PWD/../chai3d-3.0.0/lib/Debug/x64/ -lchai3d
    INCLUDEPATH += $$PWD/../chai3d-3.0.0/extras/freeglut/include
    LIBS += -L$$PWD/../chai3d-3.0.0/extras/freeglut/lib/Release/x64/ -lfreeglut
}

unix {
    DEFINES += LINUX
    QMAKE_CXXFLAGS += -std=c++0x
    LIBS += -L$$PWD/../chai3d-3.0.0/lib/release/lin-x86_64-cc/ -lchai3d
    LIBS += -L$$PWD/../chai3d-3.0.0/external/DHD/lib/lin-x86_64/ -ldrd
    LIBS += -lusb-1.0
    LIBS += -lpthread -lrt -ldl -lGL -lGLU -lglut
}


