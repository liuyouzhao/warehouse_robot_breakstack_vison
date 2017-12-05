/*
 * hal_gl.h
 *
 *  Created on: 2017-3-21
 *      Author: hujia
 *  Description:
 */

#ifndef HAL_GL_H_
#define HAL_GL_H_

#include "hal.h"
#include <GL/glx.h>
#include <GL/gl.h>
#include <GL/glut.h>

namespace cainiao_robot
{

/*
 *
 */
class hal_gl : public hal
{
public:
    hal_gl();
    virtual ~hal_gl();

    static hal_gl *instance();

    virtual void print_device_info();
    virtual int init();
    virtual int deinit();
    virtual int open();
    virtual int close();

    virtual int swap_buffer();

private:
    int x11_init_gl();

    bool    m_double_buffer;
    static hal_gl     *s_p_self;
};

} /* namespace cainiao_robot */
#endif /* HAL_GL_H_ */
