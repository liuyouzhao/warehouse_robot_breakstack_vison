/*
 * cam.h
 *
 *  Created on: 2017-3-21
 *      Author: hujia
 *  Description:
 */

#ifndef CAM_H_
#define CAM_H_

#include "hal.h"

namespace cainiao_robot
{

/*
 *
 */
class hal_cam : public hal
{
public:
    hal_cam();
    virtual ~hal_cam();

    virtual void print_device_info();
    virtual int init();
    virtual int deinit();
    virtual int open();
    virtual int close();

protected:
    const char *m_product_id;
    const char *m_interface_type;

    int m_inited;
};

} /* namespace cainiao_robot */
#endif /* CAM_H_ */
