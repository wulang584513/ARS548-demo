#include "udp_interface.h"

UdpInterface::UdpInterface()
{

}

int UdpInterface::initUdpMulticastServer(string ip, int port)
{
    char group_ip[20];
    int group_port = port;
    //int local_port = port;
    strcpy(group_ip, ip.c_str());

    struct sockaddr_in local_addr;//local address
    struct ip_mreq mreq;
    addr_len = sizeof(group_addr);

    socket_server_fd=socket(AF_INET,SOCK_DGRAM,0);
    if(socket_server_fd < 0)
    {
        perror("socket multicast!");
        return(socket_server_fd);
    }

     /*set up the destination address*/
    memset(&group_addr,0,sizeof(struct sockaddr_in));
    group_addr.sin_family = AF_INET;
    group_addr.sin_port = htons(group_port);
    group_addr.sin_addr.s_addr = inet_addr(ip.c_str());   

    /*set up the local address*/
    memset(&local_addr,0,sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    local_addr.sin_port = htons(group_port);              //this port must be the group port

    /*bind local address*/
    if(bind(socket_server_fd,(struct sockaddr *)&local_addr,sizeof(local_addr)) == -1)
    {
        perror("Binding the multicast!");
        return(-1);
    }    

    mreq.imr_multiaddr.s_addr=inet_addr(group_ip);
    mreq.imr_interface.s_addr=htonl(INADDR_ANY);
    if (setsockopt(socket_server_fd,IPPROTO_IP,IP_ADD_MEMBERSHIP,&mreq,sizeof(mreq)) < 0)
    {
        perror("setsockopt multicast!");
        return(-1);
    }

    return(0);
}

int UdpInterface::initUdpUnicastClient(string dest_ip, int dest_port, int local_port)
{
    struct sockaddr_in addr;
  
    /* set up udp socket */  
    socket_client_fd = socket(AF_INET, SOCK_DGRAM, 0);  
    if(socket_client_fd < 0)  
    {  
        perror("socket");  
        return(socket_client_fd);
    } 

	bzero(&addr, sizeof(struct sockaddr_in));
	addr.sin_family=AF_INET;
	addr.sin_addr.s_addr=htonl(INADDR_ANY);
	addr.sin_port=htons(local_port);

	if(bind(socket_client_fd,(struct sockaddr *)(&addr),sizeof(struct sockaddr_in))<0)
	{
		perror("Binding the socket!");
		return(-1);
	}	    

    memset(&addr_serv, 0, sizeof(addr_serv));  
    addr_serv.sin_family = AF_INET;  
    addr_serv.sin_addr.s_addr = inet_addr(dest_ip.c_str());  
    addr_serv.sin_port = htons(dest_port);  

    return(0);

}

void UdpInterface::receiveFromRadar(char* data, int &len)
{
    len = recvfrom(socket_server_fd, data, 40000, 0, (struct sockaddr *) &group_addr, &addr_len);
}


int UdpInterface::sendToRadar(char* data, int len)
{
    int n;
    n = sendto(socket_client_fd, data, len, 0, (struct sockaddr *)&addr_serv, sizeof(addr_serv));    
    return(n);  
}



UdpInterface::~UdpInterface()
{


}




