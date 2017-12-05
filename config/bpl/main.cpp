#include <stdio.h>
#include <string.h>

static int s_ccount = 6;
static char *s_commands_info[] = {
    "tplt_gen [jconf file] [-o output filename]",
    "tplt_show [-f tplt filename]",
    "prm_learn [tplt filename] [output id]",
    "prm_show [-i id]",
    "token_learn [prm id]",
    "token_show [prm id] [-f from number] [-t to number]"
};

static char *s_commands[] = {
    "tplt_gen",
    "tplt_show",
    "prm_learn",
    "prm_show",
    "token_learn",
    "token_show"
};

enum en_command
{
    BPL_TPLT_GEN = 0,
    BPL_TPLT_SHOW = 1,
    BPL_PRM_LEARN = 2,
    BPL_PRM_SHOW = 3,
    BPL_TOKEN_LEARN = 4,
    BPL_TOKEN_SHOW = 5
};

static void error_args()
{
    int i = 0;
    printf("Usage: bpl [command] [option]... [argument]...\n");
    printf("[[coommands]]:\n");
    for( ; i < s_ccount; i ++ )
    {
        printf("%s\n", s_commands_info[i]);
    }
}

static int bpl_tplt_gen(char *jconf, char *pout)
{
    printf("%s %s %s\n", __FUNCTION__, jconf, pout);
    return 0;
}

static int bpl_tplt_show(char *pout)
{
    printf("%s %s\n", __FUNCTION__, pout);
    return 0;
}

static int bpl_prm_learn(char *tplt, char *id)
{
    printf("%s %s %s\n", __FUNCTION__, tplt, id);
    return 0;
}

static int bpl_prm_show(char *id)
{
    printf("%s %s\n", __FUNCTION__, id);
    return 0;
}

static int bpl_token_learn(char *id)
{
    printf("%s %s\n", __FUNCTION__, id);
    return 0;
}

static int bpl_token_show(char *id, char *from, char *to)
{
    printf("%s %s %s %s\n", __FUNCTION__, id, from, to);
    return 0;
}


static int parse_args(int argc, char *argv[])
{
    int i = 0;
    int ret = 0;
    int cmd = -1;
    if(argc == 1)
    {
        goto __error_args;
    }
    for(i = 0; i < s_ccount; i ++)
    {
        if(strcmp(argv[1], s_commands[i]) == 0)
        {
            cmd = i;
            break;
        }
    }
    if(cmd == -1)
    {
        goto __error_args;
    }

    switch (cmd) {
    case en_command::BPL_TPLT_GEN:
        if(argc == 3)
            ret = bpl_tplt_gen(argv[2], NULL);
        else if(argc >= 5 && strcmp(argv[3], "-o") == 0)
            ret = bpl_tplt_gen(argv[2], argv[4]);
        else
            goto __error_args;
        break;
    case en_command::BPL_TPLT_SHOW:
        if(argc == 2)
            ret = bpl_tplt_show(NULL);
        else if(argc >= 4 && strcmp(argv[2], "-f") == 0)
            ret = bpl_tplt_show(argv[3]);
        else
            goto __error_args;
        break;
    case en_command::BPL_PRM_LEARN:
        if(argc < 4)
            goto __error_args;
        ret = bpl_prm_learn(argv[2], argv[3]);
        break;
    case en_command::BPL_PRM_SHOW:
        if(argc == 2)
            ret = bpl_prm_show(NULL);
        else if(argc >= 4 && strcmp(argv[2], "-i") == 0)
            ret = bpl_prm_show(argv[3]);
        else
            goto __error_args;
        break;
    case en_command::BPL_TOKEN_LEARN:
        if(argc <= 2)
            goto __error_args;
        else if(argc >= 3)
            ret = bpl_token_learn(argv[2]);
        else
            goto __error_args;
        break;
    case en_command::BPL_TOKEN_SHOW:
        if(argc <= 2)
            goto __error_args;
        else if(argc >= 7 && strcmp(argv[3], "-f") == 0 && strcmp(argv[5], "-t") == 0)
            ret = bpl_token_show(argv[2], argv[4], argv[6]);
        else if(argc >= 5 && strcmp(argv[3], "-f") == 0)
            ret = bpl_token_show(argv[2], argv[4], NULL);
        else if(argc >= 5 && strcmp(argv[3], "-t") == 0)
            ret = bpl_token_show(argv[2], NULL, argv[4]);
        else
            goto __error_args;
    default:
        goto __error_args;
        break;
    }

    return ret;

__error_args:
    error_args();
    return -1;
}


int main(int argc, char *argv[])
{
    //parse_args(argc, argv);
    return 0;
}
