#ifndef SERIAL_USB_H
#define SERIAL_USB_H

#include <iostream>
#include <stdio.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions 	   */
#include <errno.h>   /* ERROR Number Definitions           */

using namespace std;


class UsbSerial  //包含一些串口的函数
{
public:
    UsbSerial(){;}
    ~UsbSerial(){;}
    bool SerialInit();
    bool SerialSendData(uint8_t* serial_data);
    void SerialRecData(uint8_t * serial_data);
private:
    int fd;
};
#endif // SERIAL_USB_H

//    close(fd);/* Close the Serial port */

//}
