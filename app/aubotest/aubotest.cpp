#include "sys/sys_linux_net_tcp.h"
#include "sys/sys_linux_net_broadcast.h"
#include "hal/aubo_robot_arm.h"
#include "cjson.h"
#include <string.h>
#include <stdio.h>

#define IP "30.11.53.10"
#define PORT 8899
#define PI 3.14159265858
static char s_ipaddr[16];

static int parse_arguments(int argc, char **argv)
{
    if(argc <= 1)
    {
        printf("arguments error\n Usage:aubotest [ip]\n");
        return -1;
    }
    char *ip = argv[1];

}

int main(int argc, char **argv)
{
    int i0, i1, i2, i3, i4, i5;
    int steps = 10;
    float intval = 0.1;
    int rc = 0;

    cainiao_robot::aubo_robot_arm *p_robarm = new cainiao_robot::aubo_robot_arm();
    cainiao_robot::aubo_robot_arm::robarm_config_t conf = {
            IP,
            PORT
    };

    p_robarm->init_sync(&conf);
    cainiao_robot::aubo_robot_arm::robarm_movement_t mov;
    p_robarm->get_pos(&mov);

    printf("joints: \n%f\n %f\n %f\n %f\n %f\n %f\n XYZ: \n%f %f %f PWXYZ:\n %f %f %f %f\n",
            mov.joints[0], mov.joints[1], mov.joints[2], mov.joints[3], mov.joints[4], mov.joints[5],
            mov.vec3[0], mov.vec3[1], mov.vec3[2],
            mov.pos4[0], mov.pos4[1], mov.pos4[2], mov.pos4[3]);

    //p_robarm->move_joint_free();

#if 0
    while(1)
    {
        for( i0 = 0; i0 < steps; i0 ++ )
        {
            mov.joints[0] += intval;
            for( i1 = 0; i1 < steps; i1 ++ )
            {
                mov.joints[1] += intval;
                for( i2 = 0; i2 < steps; i2 ++)
                {
                    mov.joints[2] += intval;
                    for( i3 = 0; i3 < steps; i3 ++)
                    {
                        mov.joints[3] += intval;
                        for( i4 = 0; i4 < steps; i4 ++)
                        {
                            mov.joints[4] += intval;
                            for( i5 = 0; i5 < steps; i5 ++)
                            {
                                mov.joints[5] += intval;
                            }
                        }
                    }

                }
            }
        }
    }

#endif
#if 0

    cJSON *send_json = NULL;
    cJSON *data_json = NULL;
    cainiao_robot::sys_linux_net_tcp *p_net = new cainiao_robot::sys_linux_net_tcp();

    int rc = p_net->net_connect((char*)"30.11.53.10", PORT);
    if(rc)
    {
        printf("cannot connect to 30.11.53.10:8899\n");
        return -1;
    }

    send_json = cJSON_CreateObject();
    data_json = cJSON_CreateObject();

/*
    cJSON_AddItemToObject(data_json, "io_index", cJSON_CreateNumber(0));
    cJSON_AddItemToObject(data_json, "io_value", cJSON_CreateNumber(0));
    cJSON_AddItemToObject(send_json, "command", cJSON_CreateString("setPlcDOStatus"));
    cJSON_AddItemToObject(send_json, "data", data_json);


    name: [
    'shoulder_joint',
    'upperArm_joint',
    'foreArm_joint',
    'wrist1_joint',
    'wrist2_joint',
    'wrist3_joint'
    ]
position: [0.7674682227457782, -0.2153741302964125, 1.253595118293943,
0.40334315035501156, -2.1024120960971495, -0.9962263906148019]

    goal.orientation.x = 0;
    goal.orientation.y = 0;
    goal.orientation.z = 0;
    goal.orientation.w = 1;
    goal.position.x = 0.775884;
    goal.position.y = 0.43172;
    goal.position.z = 1;

*/
    char buf_recv[256] = {0};
    int len = 0;

    memset(buf_recv, 0, 256);
    char *command_ss = (char*)"{\"command\":\"setSpeed\",\"data\":{\"value\":50}}\n";
    p_net->net_send(command_ss, strlen(command_ss));
    p_net->net_recv(buf_recv, &len);
    printf("===> recvd1: %s\n", buf_recv);

    cJSON_AddItemToObject(data_json, "joint1", cJSON_CreateNumber(0.7674682227457782));
    cJSON_AddItemToObject(data_json, "joint2", cJSON_CreateNumber(-0.2153741302964125));
    cJSON_AddItemToObject(data_json, "joint3", cJSON_CreateNumber(1.253595118293943));
    cJSON_AddItemToObject(data_json, "joint4", cJSON_CreateNumber(0.40334315035501156));
    cJSON_AddItemToObject(data_json, "joint5", cJSON_CreateNumber(-2.1024120960971495));
    cJSON_AddItemToObject(data_json, "joint6", cJSON_CreateNumber(-0.9962263906148019));
    cJSON_AddItemToObject(send_json, "command", cJSON_CreateString("movej"));
    cJSON_AddItemToObject(send_json, "data", data_json);
    char *buf = cJSON_Print(send_json);

    memset(buf_recv, 0, 256);
    //p_net->net_send(buf, strlen(buf));
    //p_net->net_recv(buf_recv, &len);

    //printf("===> recvd2: %s\n", buf_recv);

    memset(buf_recv, 0, 256);
    char *command = (char*)"{\"command\":\"getRobotPos\"}\n";
    p_net->net_send(command, strlen(command));

    while(1)
    {
        memset(buf_recv, 0, 256);
        p_net->net_recv(buf_recv, &len);
        printf("%s\n", buf_recv);
    }



    cJSON_Delete(send_json);

    p_net->net_close();

    delete p_net;
#endif
    return 0;
}
