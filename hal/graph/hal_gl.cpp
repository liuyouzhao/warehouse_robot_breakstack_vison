/*
 * hal_gl.cpp
 *
 *  Created on: 2017-3-21
 *      Author: hujia
 *  Description:
 */

#include <stdio.h>
#include "hal_gl.h"

namespace cainiao_robot
{

hal_gl *hal_gl::s_p_self = 0;

hal_gl::hal_gl()
{
    m_double_buffer = GL_TRUE;
}

hal_gl::~hal_gl()
{
}

int
hal_gl::init()
{
#ifdef CONFIG_HAL_GRAPH_GL_X11
    x11_init_gl();
#else
#endif
}

int
hal_gl::deinit()
{
}

int
hal_gl::open()
{
    /* Pop up the main window */
    this->swap_buffer();
}

int
hal_gl::close()
{
}

hal_gl *hal_gl::instance()
{
    if(hal_gl::s_p_self == 0)
    {
        hal_gl::s_p_self = new hal_gl();
    }
    return hal_gl::s_p_self;
}

void
hal_gl::print_device_info()
{
#ifdef CONFIG_HAL_GRAPH_GL_X11
    printf("OpenGL X11 in linux\n");
#else
#endif
}

static void
fatalError(char *message)
{
    fprintf(stderr, "main: %s\n", message);
}

#ifdef CONFIG_HAL_GRAPH_GL_X11
#include <X11/X.h>    /* X11 constant (e.g. TrueColor) */
#include <X11/keysym.h>
static int snglBuf[] =
{
    GLX_RGBA, GLX_DEPTH_SIZE, 16, None
};
static int dblBuf[] =
{
    GLX_RGBA, GLX_DEPTH_SIZE, 16, GLX_DOUBLEBUFFER, None
};
static Display *dpy;
static Window win;

int hal_gl::x11_init_gl()
{
    XVisualInfo *vi;
    Colormap cmap;
    XSetWindowAttributes swa;
    GLXContext cx;
    XEvent event;
    GLboolean needRedraw = GL_FALSE, recalcModelView = GL_TRUE;
    int dummy;

    /*** (1) open a connection to the X server ***/

    dpy = XOpenDisplay(NULL);
    if (dpy == NULL)
    {
        fatalError((char*)"could not open display");
        return HAL_STATUS_FAIL;
    }

    /*** (2) make sure OpenGL's GLX extension supported ***/

    if (!glXQueryExtension(dpy, &dummy, &dummy))
    {
        fatalError((char*)"X server has no OpenGL GLX extension");
        return HAL_STATUS_FAIL;
    }

    /*** (3) find an appropriate visual ***/

    /* find an OpenGL-capable RGB visual with depth buffer */
    vi = glXChooseVisual(dpy, DefaultScreen(dpy), dblBuf);
    if (vi == NULL)
    {
        vi = glXChooseVisual(dpy, DefaultScreen(dpy), snglBuf);
        if (vi == NULL)
        fatalError((char*)"no RGB visual with depth buffer");
        m_double_buffer = GL_FALSE;
    }
    if (vi->c_class != TrueColor)
    {
        fatalError((char*)"TrueColor visual required for this program");
        return HAL_STATUS_FAIL;
    }

    /*** (4) create an OpenGL rendering context  ***/

    /* create an OpenGL rendering context */
    cx = glXCreateContext(dpy, vi, /* no shared dlists */None,
            /* direct rendering if possible */GL_TRUE);
    if (cx == NULL)
    {
        fatalError((char*)"could not create rendering context");
        return HAL_STATUS_FAIL;
    }

    /*** (5) create an X window with the selected visual ***/

    /* create an X colormap since probably not using default visual */
    cmap = XCreateColormap(dpy, RootWindow(dpy, vi->screen), vi->visual,
            AllocNone);
    swa.colormap = cmap;
    swa.border_pixel = 0;
    swa.event_mask = KeyPressMask | ExposureMask | ButtonPressMask
                     | StructureNotifyMask;

    printf("hal_gl will create XWindow\n");
    win = XCreateWindow(dpy, RootWindow(dpy, vi->screen), 0, 0, 640, 480, 0,
            vi->depth, InputOutput, vi->visual,
            CWBorderPixel | CWColormap | CWEventMask, &swa);

    XSetStandardProperties(dpy, win, "main", "main", None, 0, 0, NULL);

    /*** (6) bind the rendering context to the window ***/

    glXMakeCurrent(dpy, win, cx);

    /*** (7) request the X window to be displayed on the screen ***/

    XMapWindow(dpy, win);

    /*** (8) configure the OpenGL context for rendering ***/

    glEnable(GL_DEPTH_TEST); /* enable depth buffering */
    glDepthFunc(GL_LEQUAL); /* pedantic, GL_LESS is the default */
    glClearDepth(1.0); /* pedantic, 1.0 is the default */

    /* frame buffer clears should be to black */
    //glClearColor(0.0, 0.0, 0.0, 0.0);

    /* set up projection transform */
    //glMatrixMode(GL_PROJECTION);
    //glLoadIdentity();
    //glFrustum(-1.0, 1.0, -1.0, 1.0, 1.0, 10.0);
    /* establish initial viewport */
    /* pedantic, full window size is default viewport */
    glViewport(0, 0, 640, 480);

    return HAL_STATUS_OK;

}
int hal_gl::swap_buffer()
{
    if (m_double_buffer)
    {
        glXSwapBuffers(dpy, win);/* buffer swap does implicit glFlush */
    }
    else
    {
        glFlush(); /* explicit flush for single buffered case */
    }
}
#endif

} /* namespace cainiao_robot */
