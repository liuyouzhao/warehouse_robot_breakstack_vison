/*
 * sys_linux_net_tcp.cpp
 *
 *  Created on: 2017-3-28
 *      Author: hujia
 *  Description:
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include "sys/sys_linux_net_tcp.h"


#define BUFSIZE 1024

namespace cainiao_robot
{

sys_linux_net_tcp::sys_linux_net_tcp() :
        m_sockfd(-1)
{
}

sys_linux_net_tcp::~sys_linux_net_tcp()
{
}

/*
 * error - wrapper for perror
 */
static void error(char *msg) {
    perror(msg);
    exit(0);
}

int sys_linux_net_tcp::net_connect(char *address, const int port)
{
    int portno, n;
    struct sockaddr_in serveraddr;
    struct hostent *server;
    char *hostname;
    char buf[BUFSIZE];

    hostname = address;
    portno = port;

    /* socket: create the socket */
    m_sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (m_sockfd < 0)
    {
        error((char*)"ERROR opening socket");
        return -1;
    }

    /* gethostbyname: get the server's DNS entry */
    server = gethostbyname(hostname);
    if (server == NULL) {
        fprintf(stderr,"ERROR, no such host as %s\n", hostname);
        return -1;
    }

    /* build the server's Internet address */
    bzero((char *) &serveraddr, sizeof(serveraddr));
    serveraddr.sin_family = AF_INET;
    bcopy((char *)server->h_addr,
      (char *)&serveraddr.sin_addr.s_addr, server->h_length);
    serveraddr.sin_port = htons(portno);

    /* connect: create a connection with the server */
    if (connect(m_sockfd, (const sockaddr*)&serveraddr, sizeof(serveraddr)) < 0)
    {
        error((char*)"ERROR connecting");
        return -1;
    }

    return 0;
}

int sys_linux_net_tcp::net_send(char *data, int len)
{
    /* send the message line to the server */
    if(m_sockfd == -1)
    {
        return -1;
    }
    int n = write(m_sockfd, data, len);
    return n;
}

int sys_linux_net_tcp::net_recv(char *data, int *len)
{
    if(m_sockfd == -1 || data == 0)
    {
        return -1;
    }
    int n = read(m_sockfd, data, BUFSIZE);
    *len = n;
    return n;
}

int sys_linux_net_tcp::net_close()
{
    close(m_sockfd);
    m_sockfd = -1;
}

} /* namespace cainiao_robot */
