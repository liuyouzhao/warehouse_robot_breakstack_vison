/*
 * hal.h
 *
 *  Created on: 2017-3-21
 *      Author: hujia
 *  Description:
 */

#ifndef HAL_H_
#define HAL_H_

namespace cainiao_robot
{

enum hal_status
{
    HAL_STATUS_OK = 0,
    HAL_STATUS_FAIL = -1,
    HAL_ACTIVE = 1,
    HAL_NEGATIVE = 0
};

class hal
{
public:
    virtual void print_device_info() = 0;
    virtual int init() = 0;
    virtual int deinit() = 0;
    virtual int open() = 0;
    virtual int close() = 0;
};
};

#endif /* HAL_H_ */
