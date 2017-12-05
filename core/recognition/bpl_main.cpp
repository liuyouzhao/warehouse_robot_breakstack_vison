#include "bpl_main.h"
#include <stdio.h>
#include <string.h>

static void parse_command(char *buf, char parr[16][128])
{
    char *p = buf;
    char *q = p;
    int i = 0;
    int j = 0;
    int mrk = 0;
    int num = 0;

    for( ; j < 16; j ++ )
    {
        memset(parr[j], 0, 128);
    }

    while(*p != '\0')
    {
        while(*p == ' ')
            p ++;

        if(*q != ' ')
        {
            mrk = 1;
        }
        else if((*q == ' ' || *q == '\0') && mrk == 1)
        {
            memcpy(parr[i ++], p, q - p);
            p = q;
            mrk = 0;
        }
        else
        {
            q ++;
        }
    }
}

static int console_run()
{
    char buf[512] = {0};
    char parr[16][128] = {0};
    while(1)
    {
        scanf("%s", buf);
        parse_command(buf, parr);
    }
}

void bpl_main::run()
{

}
