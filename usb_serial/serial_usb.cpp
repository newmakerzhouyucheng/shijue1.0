#include "serial_usb.h"

bool UsbSerial::SerialInit() 
{
    cout<<"+--------------------------------------"<<endl;
    cout<<"|            Usb Serial Send Data     |"<<endl;
    cout<<"+--------------------------------------"<<endl;
    //fd = open("/dev/ttyUSB0",O_RDWR | O_NOCTTY | O_NDELAY);
    fd = open("/dev/ttyUSB0",O_RDWR);
    if(-1 == fd)
    {
        cout<<"Error open Serial"<<endl;
        return false;
    }
    else
    {
        cout<<"Open Serial Successfully"<<endl;
    }
    /*---------- Setting the Attributes of the serial port using termios structure --------- */

    struct termios SerialPortSettings;	/* Create the structure                          */

    tcgetattr(fd, &SerialPortSettings);	/* Get the current attributes of the Serial port */

    cfsetispeed(&SerialPortSettings,B115200); /* Set Read  Speed as 9600                       */
    cfsetospeed(&SerialPortSettings,B115200); /* Set Write Speed as 9600                       */

    SerialPortSettings.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB),So No Parity   */
    SerialPortSettings.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
    SerialPortSettings.c_cflag &= ~CSIZE;	 /* Clears the mask for setting the data size             */
    SerialPortSettings.c_cflag |= CS8;       /* Set the data bits = 8                                 */

    SerialPortSettings.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
    SerialPortSettings.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */


    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          /* Disable XON/XOFF flow control both i/p and o/p */
    SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode                            */

    SerialPortSettings.c_oflag &= ~OPOST;/*No Output Processing*/

    SerialPortSettings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0) /* Set the attributes to the termios structure*/
    {
        cout<<"\n  ERROR ! in Setting attributes"<<endl;
    }
    else
    {
        cout<<"\n  BaudRate = 115200 \n  StopBits = 1 \n  Parity = none"<<endl;
        cout<<"Serial Start Work "<<endl;
    }
    tcflush(fd,TCIFLUSH);
    cout<<"+--------------------------------------"<<endl;
    /*------------------------------- Write data to serial port -----------------------------*/
    return true;
}


bool UsbSerial::SerialSendData(uint8_t* serial_data)
{
    int  bytes_written  = 0;
    bytes_written = write(fd,serial_data,10);
    std::cout << "OK" << std::endl;
}

void UsbSerial::SerialRecData(uint8_t * serial_data)
{
    uint8_t read_buffer[10];
    int bytes_read = 0;
    bytes_read = read(fd,&read_buffer,10);

    if(bytes_read >0)
    {
        for(int i = 0;i<bytes_read;i++)
        {
            serial_data[i] = read_buffer[i];
            //cout<<serial_data[i]<<endl;
        }
    }
}

