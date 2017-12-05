/*
 * aubo_robot_arm.cpp
 *
 *  Created on: 2017-4-19
 *      Author: hujia
 *  Description:
 */

#include "aubo_robot_arm.h"
#include "sys/sys_linux_net_tcp.h"
#include "sys/utils/cjson.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_COMMAND_MESSAGE_LEN 1024
#define TMP_BUF_LEN 256

namespace cainiao_robot
{

aubo_robot_arm::aubo_robot_arm()
{
}

aubo_robot_arm::~aubo_robot_arm()
{
}

int aubo_robot_arm::init_sync(robarm_config conf)
{
    if(conf == NULL)
    {
        printf("conf is NULL, not corrent\n");
        return -1;
    }
    int rc = m_net.net_connect(conf->ip_addr, conf->port);
    if(rc)
    {
        printf("cannot connect to %s : %d\n", conf->ip_addr, conf->port);
        return -1;
    }

    memset(m_conf.ip_addr, 0, strlen(m_conf.ip_addr));
    memcpy(m_conf.ip_addr, conf->ip_addr, strlen(conf->ip_addr));
    m_conf.port = conf->port;

    m_net.net_close();
}

int aubo_robot_arm::init_async(
        void (*on_pcl_out)(robarm_plc_info),
        void (*on_pcl_in)(robarm_plc_info),
        void (*on_robot_pos)(robarm_movement),
        void (*on_command_done)(robarm_result))
{
    printf("%s %d %s [init_async function not support]\n", __FILE__, __LINE__, __FUNCTION__);
    return -1;
}

int aubo_robot_arm::get_plc_dgtl_in(robarm_plc_info out)
{
}

int aubo_robot_arm::get_plc_dgtl_out(robarm_plc_info out)
{
}
int aubo_robot_arm::set_plc_dgtl_out(robarm_plc_info in)
{
}

int aubo_robot_arm::_recv_response(char *buf, int *length)
{
    char buf_recv[MAX_COMMAND_MESSAGE_LEN] = {0};
    char buf_tmp[TMP_BUF_LEN] = {0};
    int len = 0;
    int pos = 0;

    while(1)
    {
        memset(buf_tmp, 0, TMP_BUF_LEN);
        m_net.net_recv(buf_tmp, &len);

        if(pos + len >= MAX_COMMAND_MESSAGE_LEN)
        {
            printf("[ERROR] command length > MAX_LEN %d > %d", pos + len, MAX_COMMAND_MESSAGE_LEN);
            return -1;
        }
        if(len <= 0)
        {
            if(pos == 0)
            {
                printf("[ERROR] command length > MAX_LEN %s", buf_tmp);
            }
            break;
        }
        else if(len < TMP_BUF_LEN && pos == 0)
        {
            memcpy(buf_recv, buf_tmp, len);
            pos = len;
            break;
        }
        else
        {
            memcpy(buf_recv + pos, buf_tmp, len);
            pos += len;
        }
    }

    memcpy(buf, buf_recv, pos);
    *length = pos;

    return *length;
}

int aubo_robot_arm::_parse_pos(char *buffer, robarm_movement out)
{
    cJSON *item = NULL;
    cJSON *jt = NULL;
    cJSON *rt_json = NULL;
    int i;

    rt_json = cJSON_Parse(buffer);
    item = cJSON_GetObjectItem(rt_json, "response_command");
    if(item == NULL || strcmp("getRobotPos", item->valuestring) != 0)
    {
        printf("%s error response.\n", buffer);
        return -1;
    }

    item = cJSON_GetObjectItem(rt_json, "data");
    if(item == NULL)
    {
       printf("%s error response.\n", buffer);
       return -1;
    }

    for( i = 0; i < 6; i ++ )
    {
        char key[16] = {0};
        sprintf(key, "joint%d", i + 1);
        jt = cJSON_GetObjectItem(item, key);
        if(jt == NULL)
        {
           printf("%s error response.\n", buffer);
           return -1;
        }
        out->joints[i] = jt->valuedouble;
    }

    jt = cJSON_GetObjectItem(item, "X");
    out->vec3[0] = jt->valuedouble;
    jt = cJSON_GetObjectItem(item, "Y");
    out->vec3[1] = jt->valuedouble;
    jt = cJSON_GetObjectItem(item, "Z");
    out->vec3[2] = jt->valuedouble;

    jt = cJSON_GetObjectItem(item, "pose_w");
    out->pos4[0] = jt->valuedouble;
    jt = cJSON_GetObjectItem(item, "pose_x");
    out->pos4[1] = jt->valuedouble;
    jt = cJSON_GetObjectItem(item, "pose_y");
    out->pos4[2] = jt->valuedouble;
    jt = cJSON_GetObjectItem(item, "pose_z");
    out->pos4[3] = jt->valuedouble;

    return 0;
}

int aubo_robot_arm::get_pos(robarm_movement out)
{
    char *command = (char*) "{\"command\":\"getRobotPos\"}\n";
    char buffer[MAX_COMMAND_MESSAGE_LEN] = {0};
    int length = 0;
    int i = 0;

    int rc = m_net.net_connect(m_conf.ip_addr, m_conf.port);
    if (rc)
    {
        printf("%s cannot connect to %s : %d\n", __FUNCTION__,
                m_conf.ip_addr, m_conf.port);
        return -1;
    }
    m_net.net_send(command, strlen(command));
    _recv_response(buffer, &length);
    m_net.net_close();

    printf("%s\n", buffer);
    return _parse_pos(buffer, out);
}

int aubo_robot_arm::get_joints(robarm_movement out)
{
}

int aubo_robot_arm::get_accelerate(robarm_move_state ms)
{
}

int aubo_robot_arm::set_accelerate(robarm_move_state ms)
{
}

int aubo_robot_arm::get_velocity(robarm_move_state ms)
{
}

int aubo_robot_arm::set_velocity(robarm_move_state ms)
{
}

int aubo_robot_arm::reset_movestate()
{
}

int aubo_robot_arm::stop()
{
}

int aubo_robot_arm::move_to_joint_straight(robarm_movement mov)
{
}

int aubo_robot_arm::move_to_locate_straight(robarm_movement mov)
{
}

int aubo_robot_arm::move_sequence_joints_arc(robarm_movement movs)
{
}

int aubo_robot_arm::move_sequence_locates_arc(robarm_movement movs)
{
    int rc = m_net.net_connect(m_conf.ip_addr, m_conf.port);
    if(rc)
    {
        printf("%s cannot connect to %s : %d\n", __FUNCTION__, m_conf.ip_addr, m_conf.port);
        return -1;
    }

    m_net.net_close();
}

int aubo_robot_arm::move_joint_free(robarm_movement mov)
{
    int i = 0, len = 0;
    cJSON *send_json = NULL;
    cJSON *data_json = NULL;
    char *buf_send = NULL;
    char buf_recv[MAX_COMMAND_MESSAGE_LEN] = {0};

    int rc = m_net.net_connect(m_conf.ip_addr, m_conf.port);
    if (rc)
    {
        printf("%s cannot connect to %s : %d\n", __FUNCTION__,
                m_conf.ip_addr, m_conf.port);
        return -1;
    }
    send_json = cJSON_CreateObject();
    data_json = cJSON_CreateObject();

    for( i; i < 6; i ++ )
    {
        char key[16] = {0};
        sprintf(key, "joint%d", i + 1);
        cJSON_AddItemToObject(data_json, key, cJSON_CreateNumber(mov->joints[i]));
    }
    cJSON_AddItemToObject(send_json, "command",  cJSON_CreateString("movej"));
    cJSON_AddItemToObject(send_json, "data", data_json);
    buf_send = cJSON_Print(send_json);
    m_net.net_send(buf_send, strlen(buf_send));

    free(buf_send);
    cJSON_Delete(send_json);

    while (len == 0)
    {
        memset(buf_recv, 0, MAX_COMMAND_MESSAGE_LEN);
        m_net.net_recv(buf_recv, &len);
    }
    m_net.net_close();

    rc = _check_response(buf_recv, len);
    return rc;
}

int aubo_robot_arm::_check_response(char *buf, int len)
{
    cJSON *resp_json = cJSON_Parse(buf);
    cJSON *command = cJSON_GetObjectItem(resp_json, "response_command");
    if(command == NULL || strcmp("movej", command->valuestring) != 0)
    {
        printf("[ERROR] command response error: %s\n", buf);
        return -1;
    }
    command = cJSON_GetObjectItem(resp_json, "error_code");
    if(command == NULL)
    {
        printf("[ERROR] command response format error: %s\n", buf);
        return -1;
    }
    if(command->valueint != 0)
    {
        return command->valueint;
    }
    return 0;
}
} /* namespace cainiao_robot */
