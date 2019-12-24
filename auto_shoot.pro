TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    main/main.cpp \
    cam_driver/open_camera.cpp \
    armor_detect/armor_detect.cpp \
    usb_serial/serial_usb.cpp \
    Info_solver/angle_solver.cpp \
    main/visual_proc.cpp \

HEADERS += \
    /usr/local/include \
    /usr/local/include/opencv \
    /usr/local/include/opencv2 \
    cam_driver/DVPCamera.h \
    cam_driver/open_camera.h \
    armor_detect/armor_detect.h \
    usb_serial/serial_usb.h \
    Info_solver/angle_solver.h \
    main/visual_proc.h \
    Info_solver/mnist.h

LIBS += /usr/local/lib/libopencv*.so.* \
        /usr/local/lib/libD3tKit.so \
        /usr/local/lib/libdvp.so \
        /usr/local/lib/libhzd.so \
        /usr/local/lib/usb3_m_all.dscam.so \
        /usr/local/lib/usb3_v_all.dscam.so
LIBS +=-lpthread
