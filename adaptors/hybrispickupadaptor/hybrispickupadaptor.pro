TARGET       = hybrispickupadaptor

HEADERS += hybrispickupadaptor.h \
           hybrispickupadaptorplugin.h

SOURCES += hybrispickupadaptor.cpp \
           hybrispickupadaptorplugin.cpp

LIBS+= -L../../core -lhybrissensorfw-qt5

include( ../adaptor-config.pri )
config_hybris {
    PKGCONFIG += android-headers
}
