/*
 * sys_linux_net_tcp.h
 *
 *  Created on: 2017-3-28
 *      Author: hujia
 *  Description:
 */

#ifndef SYS_LINUX_NET_TCP_H_
#define SYS_LINUX_NET_TCP_H_

namespace cainiao_robot
{

/*
 *
 */
class sys_linux_net_tcp
{
public:
    sys_linux_net_tcp();
    virtual ~sys_linux_net_tcp();

    int net_connect(char *address, const int port);
    int net_send(char *data, const int len);
    int net_recv(char *data, int *len);
    int net_close();
private:
    int m_sockfd;
};

} /* namespace cainiao_robot */
#endif /* SYS_LINUX_NET_TCP_H_ */
