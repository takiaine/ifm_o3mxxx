#ifndef UDPRECEIVER_H
#define UDPRECEIVER_H

#define UNIX
#ifdef UNIX
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#endif

#ifdef WIN32
#include<stdio.h>
#include<winsock2.h>
#pragma comment(lib,"ws2_32.lib")
#endif

#include <iostream>

class UDPReceiver
{
public:
    UDPReceiver();
    ~UDPReceiver();

    /* Initialize the class with port and ip what to use on local interface
     * port = port where to receive data
     * interface = ip of local interface where to receive data(255.255.255.255 for all)
     */
    bool init(int port, std::string localInterface);

    /* Receives data from the port. Waits for data maximum of timeoutMicroSec
     * data = pointer to buffer where to save data
     * dataSize = size of the buffer
     */
    int receive(void *data, unsigned int dataSize, int timeoutMicroSec);

private:
    // My address
    struct sockaddr_in siMe_;
    // The other computers address
    struct sockaddr_in siOther_;
    int sock_;
    int slen_;

    bool initOK;

#ifdef WIN32
    WSADATA wsa;
#endif
};

#endif // UDPRECEIVER_H
