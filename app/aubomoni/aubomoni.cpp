#include "sys/sys_linux_net_tcp.h"
#include "sys/sys_linux_net_broadcast.h"
#include "cjson.h"
#include <string.h>
#include <stdio.h>

static int get_ip_addr_from_broadcast()
{
    cainiao_robot::sys_linux_net_broadcast *p_broad = new cainiao_robot::sys_linux_net_broadcast();
    p_broad->net_bind(9988);

    while(1)
    {
        char buf[64] = {0};
        char addr[16] = {0};
        int len;
        int n = p_broad->net_recv(buf, addr, &len);
        if(n > 0)
        {
            printf("received: %s %d from [%s]\n", buf, n, addr);
        }
        else
        {
            printf("received error\n");
        }
    }
    return 0;
}


int main(int argc, char **argv)
{
    cJSON *send_json = NULL;
    cJSON *data_json = NULL;

    get_ip_addr_from_broadcast();


    cainiao_robot::sys_linux_net_tcp *p_net = new cainiao_robot::sys_linux_net_tcp();

    int rc = p_net->net_connect((char*)"30.11.53.10", 8899);
    if(rc)
    {
        printf("cannot connect to 30.11.53.10:8899\n");
        return -1;
    }

    send_json = cJSON_CreateObject();
    data_json = cJSON_CreateObject();

    cJSON_AddItemToObject(data_json, "io_index", cJSON_CreateNumber(0));
    cJSON_AddItemToObject(data_json, "io_value", cJSON_CreateNumber(0));
    cJSON_AddItemToObject(send_json, "command", cJSON_CreateString("setPlcDOStatus"));
    cJSON_AddItemToObject(send_json, "data", data_json);

    char *buf = cJSON_Print(send_json);

    printf("I will send \n%s\n", buf);

    p_net->net_send(buf, strlen(buf));

    char buf_recv[256] = {0};
    int len = 0;
    p_net->net_recv(buf_recv, &len);

    printf("===> recvd: %s\n", buf_recv);

    cJSON_Delete(send_json);

    p_net->net_close();

    delete p_net;
    return 0;
}
