/*
 * Done by Antti Kolu, antti.kolu@tut.fi, anttikolu@gmail.com
*/

#include "ifm_o3mxxx/udpreceiver.h"
#include <iostream>
#include <stdio.h>
#include <string.h>

#include <unistd.h>

using std::cout;
using std::endl;
using std::string;

UDPReceiver::UDPReceiver()
{
    slen_=sizeof(siOther_);
    initOK = false;
}

UDPReceiver::~UDPReceiver()
{
#ifdef UNIX
    close(sock_);
#endif
#ifdef WIN32
    closesocket(sock_);
    WSACleanup();
#endif
}

bool UDPReceiver::init(int port, std::string localInterface) {

#ifdef WIN32
    if (WSAStartup(MAKEWORD(2,2),&wsa) != 0)
    {
        std::cout << "ERROR: WSAStartup error code " << WSAGetLastError() << std::endl;
        return false;
    }
    // create UDP socket that will be used to receive data
    if ((sock_=socket(AF_INET, SOCK_DGRAM, 0))==-1){
        std::cout << "ERROR: Could not create socket!" << std::endl;
        return false;
    }

    //char broadcast = 1;
    //setsockopt(sock_, SOL_SOCKET, SO_BROADCAST,
    //            &broadcast, sizeof broadcast);

    // Initialize socket address. Receive data from PORT and from user
    // specified local address(or any address is 255.255.255.255).
    memset((char *) &siMe_, 0, sizeof(siMe_));
    siMe_.sin_family = AF_INET;
    siMe_.sin_port = htons((short)port);
    if(0 == localInterface.compare("255.255.255.255"))
       siMe_.sin_addr.s_addr = htonl(INADDR_ANY);
    else
        siMe_.sin_addr.s_addr = inet_addr(localInterface.c_str());

    // Bind socket into socket address
    if (bind(sock_, (struct sockaddr *)&siMe_, sizeof(siMe_))==-1){
        std::cout << "ERROR: Could not bind socket, error code " << WSAGetLastError() << std::endl;
        return false;
    }
#endif

#ifdef UNIX
    // create UDP socket that will be used to receive data
    if ((sock_=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1){
        std::cout << "ERROR: Could not create socket!" << std::endl;
        return false;
    }

    int one = 1;
    setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    // Initialize socket address. Receive data from PORT and from any address.
    memset((char *) &siMe_, 0, sizeof(siMe_));
    siMe_.sin_family = AF_INET;
    siMe_.sin_port = htons((short)port);
    if(0 == localInterface.compare("255.255.255.255"))
       siMe_.sin_addr.s_addr = htonl(INADDR_ANY);
    else
        siMe_.sin_addr.s_addr = inet_addr(localInterface.c_str());

    // Bind socket into socket address
    if (bind(sock_, (struct sockaddr *)&siMe_, sizeof(siMe_))==-1){
        std::cout << "ERROR: Could not bind socket!" << std::endl;
        return false;
    }
#endif

    initOK = true;
    return true;
}

int UDPReceiver::receive(void *data, unsigned int dataSize, int timeoutMicroSec)
{
    if(!initOK){
        cout << "ERROR: UDP socket has not been initialized!" << endl;
        return false;
    }

    fd_set udpWaitSet;
    FD_ZERO(&udpWaitSet);
    FD_SET(sock_, &udpWaitSet);

    // initialize timeout value that select will wait for UDP packet. 500ms
    struct timeval timeout={0,timeoutMicroSec};

    // wait if there is any data to receive. If not, then continue to next loop.
    int status = select(sock_+1, &udpWaitSet, NULL, NULL, &timeout);
    if(status<0){
        //cout << "Select returned unexpected value: " << status << " (Ctrl+c called?)" << endl;
        return -2;
    }else if(status==0){
        //Timeout reached
        //cout << "Timeout on UDP receive." << endl;
        return -1;
    }

    // if there is something to receive, check if it is from the right socket.
    if(FD_ISSET(sock_, &udpWaitSet)){

        // receive the message into buf.
#ifdef UNIX
        int recvLength = recvfrom(sock_, data, dataSize, 0, (struct sockaddr *)&siOther_, (socklen_t *)&slen_);
#endif
#ifdef WIN32
        int recvLength = recvfrom(sock_, (char*)data, dataSize, 0, (struct sockaddr *)&siOther_, &slen_);
#endif
        //cout << "received data from" << siOther_.sin_addr.s_addr << endl;
        return recvLength;
    }
    return -2;
}
