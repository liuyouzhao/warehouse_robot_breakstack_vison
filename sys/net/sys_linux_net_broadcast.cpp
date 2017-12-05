/*
 * sys_linux_net_broadcast.cpp
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
#include <arpa/inet.h>
#include <netdb.h>
#include "sys/sys_linux_net_broadcast.h"


#define BUFSIZE 1024
#define BROADCAST_ADDR "255.255.255.255"

namespace cainiao_robot
{

sys_linux_net_broadcast::sys_linux_net_broadcast() :
        m_sockfd(-1),
        m_broad_intied(0)
{
    if((m_sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
    {
        printf("net_bind socket fail\n");
    }
}

sys_linux_net_broadcast::~sys_linux_net_broadcast()
{
}

/*
 * error - wrapper for perror
 */
static void error(char *msg) {
    perror(msg);
    exit(0);
}

int sys_linux_net_broadcast::net_bind(const int port)
{
    int set;
    setsockopt(m_sockfd, SOL_SOCKET, SO_REUSEADDR, &set, sizeof(int));
    memset(&m_addr, 0, sizeof(struct sockaddr_in));

    m_addr.sin_family = AF_INET;
    m_addr.sin_port = htons(port);
    m_addr.sin_addr.s_addr = INADDR_ANY;

    if(bind(m_sockfd, (struct sockaddr *)&m_addr, sizeof(struct sockaddr)) == -1)
    {
        printf("bind fail\n");
        return -1;
    }
}

int sys_linux_net_broadcast::net_broadcast(char *data, int len, int port)
{
    int optval = 1;
    int n = 0;

    if(m_sockfd == -1)
    {
        return -1;
    }

    if(!m_broad_intied)
    {
        setsockopt(m_sockfd, SOL_SOCKET, SO_BROADCAST | SO_REUSEADDR, &optval, sizeof(int));
        memset(&m_addr, 0, sizeof(struct sockaddr_in));
        m_addr.sin_family = AF_INET;
        m_addr.sin_addr.s_addr = inet_addr(BROADCAST_ADDR);
        m_addr.sin_port = htons(port);

        m_broad_intied = 1;
    }

    n = sendto(m_sockfd, data, len, 0, (struct sockaddr*)&m_addr, sizeof(struct sockaddr));
    return n;
}

int sys_linux_net_broadcast::net_recv(char *data, char *addr, int *len)
{
    struct sockaddr_in addr_from;
    int addr_len = sizeof(struct sockaddr_in);
    char *addr_get;
    int n = -1;
    if(m_sockfd == -1 || data == 0)
    {
        return -1;
    }

    n = recvfrom(m_sockfd, data, BUFSIZE, 0, (struct sockaddr *)&addr_from, (socklen_t*)&addr_len);
    *len = n;

    addr_get = inet_ntoa(addr_from.sin_addr);
    memcpy(addr, addr_get, strlen(addr_get));

    return n;
}

int sys_linux_net_broadcast::net_close()
{
    close(m_sockfd);
    m_sockfd = -1;
}

} /* namespace cainiao_robot */
