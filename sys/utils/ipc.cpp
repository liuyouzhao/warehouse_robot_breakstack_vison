#include "ipc.h"

ipc::ipc()
{

}

int ipc::send(const char *buf, int len, int k)
{
    int msg_id;
    int msg_flg = IPC_CREAT | 0666;
    key_t key;
    message_buf_t sbuf;
    int ret = -1;

    if(len > __MB_MSG_SIZE)
    {
        perror("[IPC_ERROR]len is too large");
        return -1;
    }

    key = k;

    msg_id = msgget(key, msg_flg);
    if(msg_id < 0)
    {
        perror("msgget");
        return -1;
    }

    sbuf.mtype = 1;
    memcpy(sbuf.mtext, buf, len);

    ret = msgsnd(msg_id, &sbuf, (unsigned int) len, IPC_NOWAIT);
    if(ret < 0)
    {
        perror("msgsnd");
        return -1;
    }
    else
    {
        printf("Message sent: %s\n", sbuf.mtext);
    }
    return ret;
}

int ipc::recv(char *buf, int k)
{
    int msg_id;
    key_t key;
    message_buf_t rbuf;
    int ret = -1;

    key = k;
    msg_id =  msgget(key, IPC_CREAT | 0666);
    if(msg_id < 0) {
        perror("msgget");
        return -1;
    }
    ret = msgrcv(msg_id, &rbuf, __MB_MSG_SIZE, 1, 0);
    if(ret < 0) {
        perror("msgrcv");
        return -1;
    }
    printf("recv: %s\n", rbuf.mtext);

    memcpy(buf, rbuf.mtext, ret);
    return 0;
}
