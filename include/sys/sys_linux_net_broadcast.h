/*
 * sys_linux_net_udp.h
 *
 *  Created on: 2017-3-28
 *      Author: hujia
 *  Description:
 */

#ifndef SYS_LINUX_NET_BROADCAST_H_
#define SYS_LINUX_NET_BROADCAST_H_

#include <netinet/in.h>

namespace cainiao_robot
{

/*
 *
 */
class sys_linux_net_broadcast
{
public:
    sys_linux_net_broadcast();
    virtual
    ~sys_linux_net_broadcast();

    int net_bind(const int port);
    int net_broadcast(char *data, const int len, int port);
    int net_recv(char *data, char *addr, int *len);
    int net_close();
private:
    int m_sockfd;
    struct sockaddr_in m_addr;
    int m_broad_intied;
};

} /* namespace cainiao_robot */
#endif /* SYS_LINUX_NET_BROADCAST_H_ */
