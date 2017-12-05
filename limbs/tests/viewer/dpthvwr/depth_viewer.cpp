/* A simple program to show how to set up an X window for OpenGL rendering.
 * X86 compilation: gcc -o -L/usr/X11/lib   main main.c -lGL -lX11
 * X64 compilation: gcc -o -L/usr/X11/lib64 main main.c -lGL -lX11
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "hal_ni_camera3d.h"
#include "hal_gl.h"

using namespace cainiao_robot;

GLfloat xAngle = 42.0, yAngle = 82.0, zAngle = 112.0;
void render(void)
{
    static GLboolean displayListInited = GL_FALSE;

    if (displayListInited)
    {
        /* if display list already exists, just execute it */
        glCallList(1);
    }
    else
    {
        /* otherwise compile and execute to create the display list */
        glNewList(1, GL_COMPILE_AND_EXECUTE);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        /* front face */
        glBegin (GL_QUADS);
        glColor3f(0.0, 0.7, 0.1); /* green */
        glVertex3f(-1.0, 1.0, 1.0);
        glVertex3f(1.0, 1.0, 1.0);
        glVertex3f(1.0, -1.0, 1.0);
        glVertex3f(-1.0, -1.0, 1.0);

        /* back face */
        glColor3f(0.9, 1.0, 0.0); /* yellow */
        glVertex3f(-1.0, 1.0, -1.0);
        glVertex3f(1.0, 1.0, -1.0);
        glVertex3f(1.0, -1.0, -1.0);
        glVertex3f(-1.0, -1.0, -1.0);

        /* top side face */
        glColor3f(0.2, 0.2, 1.0); /* blue */
        glVertex3f(-1.0, 1.0, 1.0);
        glVertex3f(1.0, 1.0, 1.0);
        glVertex3f(1.0, 1.0, -1.0);
        glVertex3f(-1.0, 1.0, -1.0);

        /* bottom side face */
        glColor3f(0.7, 0.0, 0.1); /* red */
        glVertex3f(-1.0, -1.0, 1.0);
        glVertex3f(1.0, -1.0, 1.0);
        glVertex3f(1.0, -1.0, -1.0);
        glVertex3f(-1.0, -1.0, -1.0);
        glEnd();
        glEndList();
        displayListInited = GL_TRUE;
    }
}

/*
 *      x    y
 *       \   |
          \  |
           \ |
z<------------

 * */
int render_points(float *depth, int w, int h)
{
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glNewList(1, GL_COMPILE_AND_EXECUTE);
    //glBegin (GL_QUADS);
    glBegin(GL_POINTS);

#if 1
    int bw = 4.0f;
    int bh = 4.0f;

#if 1
    float dist = 200.0f;
    int ds = 2;
    int ws = w / ds;
    int hs = h / ds;
    for(int i = 0, i2 = 0; i < hs; i += 1, i2 += ds) {
        for(int j = 0, j2 = 0; j < ws; j += 1, j2 += ds) {
            float x = (i - ws / 2.0f) * bw;
            float z = (j - hs / 2.0f) * bh;

            float color = depth[i2 * w + j2] / 256.0f;
            float y = depth[i2 * w + j2] / 256.0f * dist;

            glColor3f(color, 1.0 - color, 1.0 - color);

            glVertex3f(x - 2,     y,      z + 2);
            glVertex3f(x + 2,     y,      z + 2);
            glVertex3f(x + 2,     y,      z - 2);
            glVertex3f(x - 2,     y,      z - 2);
        }
    }
#endif


#else
   glColor3f(0.2, 0.2, 1.0); /* blue */
   glVertex3f(-100.0, 0.0, 100.0);

   glColor3f(1.0, 0.2, 0.0); /* blue */
   glVertex3f(100.0, 0.0, 100.0);

   glColor3f(0.0, 1.0, 0.0); /* blue */
   glVertex3f(100.0, 0.0, -100.0);

   glColor3f(1.0, 1.0, 1.0); /* blue */
   glVertex3f(-100.0, 0.0, -100.0);
#endif

    glEnd();
    glEndList();
    glCallList(1);
}

int main(int argc, char **argv)
{
    float *depth = 0;
    int w = 0;
    int h = 0;
    float range = 256.f;

    /*(1) Initialize OpenGL for global window */
    int rc = hal_gl::instance()->init();
    if (rc != HAL_STATUS_OK)
    {
        printf("%s FatalError after hal_gl init\n", __FUNCTION__);
        return -1;
    }

    /*(2) Open hal of camera3d*/
    hal_ni_camera3d::instance()->init();
    hal_ni_camera3d::instance()->open();
    w = hal_ni_camera3d::instance()->width();
    h = hal_ni_camera3d::instance()->height();
    depth = (float*) malloc(sizeof(float) * w * h);

    while (1)
    {
        glMatrixMode (GL_PROJECTION);
        glLoadIdentity ();
        gluPerspective(60.0f, 640.f / 480.f, 0.0, 10.0);

        /* reset modelview matrix to the identity matrix */
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        gluLookAt(500, 500.0, 0.0,  0.0, 0.0, 0.0,  0.0, 1.0, 0.0);


        rc = hal_ni_camera3d::instance()->read(depth, range);
        if (rc == HAL_STATUS_OK)
        {
            render_points(depth, w, h);
        }
        else
        {
            printf("FataError!\n");
            exit(1);
        }

        hal_gl::instance()->swap_buffer();
        //usleep(1000 * 10);
    }

    return 0;
}
