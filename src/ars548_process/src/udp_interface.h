#ifndef UDP_INTERFACE_H
#define UDP_INTERFACE_H

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <string.h>
#include <netdb.h>
#include <errno.h>
#include <iostream>

using namespace std;

class UdpInterface
{

public:
    UdpInterface();
    ~UdpInterface();

    int initUdpMulticastServer(string ip, int port);
    int initUdpUnicastClient(string dest_ip, int dest_port, int local_port);
    void receiveFromRadar(char* data, int &len);
    int sendToRadar(char* data, int len);

private:
    int socket_server_fd;
    int socket_client_fd;
    struct sockaddr_in addr_serv; 
    struct sockaddr_in group_addr;  //group address
    socklen_t addr_len;





    
};



#endif 