/*
 * aubo_robot_arm.h
 *
 *  Created on: 2017-4-19
 *      Author: hujia
 *  Description:
 */

#ifndef AUBO_ROBOT_ARM_H_
#define AUBO_ROBOT_ARM_H_

#include <sys/sys_linux_net_tcp.h>

namespace cainiao_robot
{

/*
 *
 */
class aubo_robot_arm
{
public:
    typedef struct robarm_config_s
    {
        char ip_addr[16];
        int port;
    } robarm_config_t;
    typedef robarm_config_s* robarm_config;

    typedef struct robarm_movement_s
    {
        float joints[6];
        float vec3[3];
        float pos4[4];
    } robarm_movement_t;
    typedef robarm_movement_t* robarm_movement;

    typedef void* robarm_plc_info;
    typedef void* robarm_result;
    typedef void* robarm_move_state;

public:
    aubo_robot_arm();
    virtual ~aubo_robot_arm();

public:
    virtual int init_sync(robarm_config conf);

    virtual int init_async(
            void (*on_pcl_out)(robarm_plc_info),
            void (*on_pcl_in)(robarm_plc_info),
            void (*on_robot_pos)(robarm_movement),
            void (*on_command_done)(robarm_result));

    virtual int get_plc_dgtl_in(robarm_plc_info out);

    virtual int get_plc_dgtl_out(robarm_plc_info out);
    virtual int set_plc_dgtl_out(robarm_plc_info in);

    virtual int get_joints(robarm_movement out);
    virtual int get_pos(robarm_movement out);

    virtual int get_accelerate(robarm_move_state ms);
    virtual int set_accelerate(robarm_move_state ms);

    virtual int get_velocity(robarm_move_state ms);
    virtual int set_velocity(robarm_move_state ms);

    virtual int reset_movestate();

    virtual int stop();
    virtual int move_to_joint_straight(robarm_movement mov);
    virtual int move_to_locate_straight(robarm_movement mov);
    virtual int move_sequence_joints_arc(robarm_movement movs);
    virtual int move_sequence_locates_arc(robarm_movement movs);
    virtual int move_joint_free(robarm_movement mov);

private:
    int _recv_response(char *buf, int *len);
    int _parse_pos(char *buffer, robarm_movement out);
    int _check_response(char *buf, int len);

    sys_linux_net_tcp m_net;
    robarm_config_t m_conf;
};

} /* namespace cainiao_robot */
#endif /* AUBO_ROBOT_ARM_H_ */
