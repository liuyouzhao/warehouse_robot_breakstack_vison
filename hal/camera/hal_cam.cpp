/*
 * cam.cpp
 *
 *  Created on: 2017-3-21
 *      Author: hujia
 *  Description:
 */

#include "hal_cam.h"
#include <stdio.h>

namespace cainiao_robot
{

hal_cam::hal_cam() :
m_product_id("unknown"),
m_interface_type("none"),
m_inited(0)
{
}

hal_cam::~hal_cam()
{
}

void hal_cam::print_device_info()
{
    printf("\ncamera info:");
    printf("\n[%s]\n", m_product_id);
    printf("\n[%s]\n", m_interface_type);
}

int hal_cam::init()
{
}

int hal_cam::deinit()
{
}

int hal_cam::open()
{
}

int hal_cam::close()
{
}

} /* namespace cainiao_robot */
