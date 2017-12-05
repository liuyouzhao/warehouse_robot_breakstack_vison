#ifndef IPC_H
#define IPC_H

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <stdio.h>
#include <string.h>

#define __MB_MSG_SIZE 128
typedef struct message_buf_s {
    long mtype;
    char mtext[__MB_MSG_SIZE];
} message_buf_t;

class ipc
{
public:
    ipc();

    static int send(const char *buf, int len, int k);
    static int recv(char *buf, int k);
};

#endif // IPC_H
