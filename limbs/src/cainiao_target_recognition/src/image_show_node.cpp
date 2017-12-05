#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <stdlib.h>  
#include <stdio.h>
#include <GL/gl.h>
#include <GL/glut.h>
#define GL_BGR_EXT 0x80E0  
using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
 ros::NodeHandle nh_;
 image_transport::ImageTransport it_;
 image_transport::Subscriber image_sub_;
 image_transport::Publisher image_pub_;

public:
 ImageConverter()
   : it_(nh_)
 {
   // Subscrive to input video feed and publish output video feed
   image_sub_ = it_.subscribe("/camera/image_raw", 1,
     &ImageConverter::imageCb, this);
   image_pub_ = it_.advertise("/image_converter/output_video", 1);

   cv::namedWindow(OPENCV_WINDOW);
 }

 ~ImageConverter()
 {
   cv::destroyWindow(OPENCV_WINDOW);
 }

 void imageCb(const sensor_msgs::ImageConstPtr& msg)
 {
   cv_bridge::CvImagePtr cv_ptr;
   try
   {
     cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
   }
   catch (cv_bridge::Exception& e)
   {
     ROS_ERROR("cv_bridge exception: %s", e.what());
     return;
   }

    image = cv_ptr->image; 

   // Update GUI Window
   cv::imshow(OPENCV_WINDOW, cv_ptr->image);
   cv::waitKey(3);

   // Output modified video stream
   image_pub_.publish(cv_ptr->toImageMsg());
 }

void display(void)
{
    glClear(GL_COLOR_BUFFER_BIT);

    glColor3f(1.0, 1.0, 0.0);
    glBegin(GL_POLYGON);
    glVertex3f(-0.80, 0.80, 0.0);
    glVertex3f(-0.80, -0.80, 0.0);
    glVertex3f(0.80, -0.80, 0.0);
    glVertex3f(0.80, 0.80, 0.0);
    glEnd();

    glFlush();
}

void init(void)
{
    glClearColor(0.0, 1.0, 0.0, 0.0);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-1.0, 1.0, -1.0, 1.0, -1.0, 1.0);
}

private:

Mat image;
/*
int show(const cv::Mat &image)  
{  

    int imagewidth = image.cols;
    int imageheight = image.rows;
    
    //build_projection(cameraMatrix);  //这是建立摄像机内参数矩阵，就是相机矩阵，display函数开始导入的模型就是相机矩阵  
    glutInitDisplayMode(GLUT_DOUBLE|GLUT_RGBA);  
    glutInitWindowPosition(100,100);  
    glutInitWindowSize(imagewidth,imageheight);  
    glutCreateWindow("OPenGL");
    init();  
    //glutDisplayFunc(&display);
    glutDisplayFunc(display);
    glutMainLoop();  
    //-------------------------------------  
    return 0; 


}

*/

 /*

void build_projection(Mat_<float> cameraMatrix)  
{  
    float near = 0.01;  // Near clipping distance  
    float far = 100;  // Far clipping distance  
  
    // Camera parameters  
    //float f_x = cameraMatrix.data[0]; // Focal length in x axis  
    //float f_y = cameraMatrix.data[4]; // Focal length in y axis (usually the same?)  
    //float c_x = cameraMatrix.data[2]; // Camera primary point x  
    //float c_y = cameraMatrix.data[5]; // Camera primary point y  

    float f_x = cameraMatrix(0,0); // Focal length in x axis  
    float f_y = cameraMatrix(1,1); // Focal length in y axis (usually the same?)  
    float c_x = cameraMatrix(0,2); // Camera primary point x  
    float c_y = cameraMatrix(1,2); // Camera primary point y  
  
    projectionMatrix.data[0] =  - 2.0 * f_x / imagewidth;  
    projectionMatrix.data[1] = 0.0;  
    projectionMatrix.data[2] = 0.0;  
    projectionMatrix.data[3] = 0.0;  
  
    projectionMatrix.data[4] = 0.0;  
    projectionMatrix.data[5] = 2.0 * f_y / imageheight;  
    projectionMatrix.data[6] = 0.0;  
    projectionMatrix.data[7] = 0.0;  
  
    projectionMatrix.data[8] = 2.0 * c_x / imagewidth - 1.0;  
    projectionMatrix.data[9] = 2.0 * c_y / imageheight - 1.0;      
    projectionMatrix.data[10] = -( far+near ) / ( far - near );  
    projectionMatrix.data[11] = -1.0;  
  
    projectionMatrix.data[12] = 0.0;  
    projectionMatrix.data[13] = 0.0;  
    projectionMatrix.data[14] = -2.0 * far * near / ( far - near );          
    projectionMatrix.data[15] = 0.0;  
}  
  

void display(void)  
{  
      
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);  
    //绘制图片,第一、二、三、四个参数表示图象宽度、图象高度、像素数据内容、像素数据在内存中的格式,最后一个参数表示用于绘制的像素数据在内存中的位置  
    glDrawPixels(imagewidth,imageheight,GL_BGR_EXT,GL_UNSIGNED_BYTE,pixeldata);  
    
    /* 
     glMatrixMode - 指定哪一个矩阵是当前矩阵 
      
     mode 指定哪一个矩阵堆栈是下一个矩阵操作的目标,可选值: GL_MODELVIEW、GL_PROJECTION、GL_TEXTURE. 
     说明 
     glMatrixMode设置当前矩阵模式: 
     GL_MODELVIEW,对模型视景矩阵堆栈应用随后的矩阵操作. 
     GL_PROJECTION,对投影矩阵应用随后的矩阵操作. 
     GL_TEXTURE,对纹理矩阵堆栈应用随后的矩阵操作. 
     与glLoadIdentity()一同使用 
     glLoadIdentity():该函数的功能是重置当前指定的矩阵为单位矩阵。 
     在glLoadIdentity()之后我们为场景设置了透视图。glMatrixMode(GL_MODELVIEW)设置当前矩阵为模型视图矩阵，模型视图矩阵储存了有关物体的信息。 
     */  
/*    //绘制坐标  ，导入相机内参数矩阵模型
    glMatrixMode(GL_PROJECTION);  
    glLoadMatrixf(projectionMatrix.data);  
    glMatrixMode(GL_MODELVIEW);  
    glLoadIdentity();  
    
    glEnableClientState(GL_VERTEX_ARRAY);  //启用客户端的某项功能
    glEnableClientState(GL_NORMAL_ARRAY);  
  
    glPushMatrix();  
    glLineWidth(3.0f);  
  
    float lineX[] = {0,0,0,1,0,0};  
    float lineY[] = {0,0,0,0,1,0};  
    float lineZ[] = {0,0,0,0,0,1};  
  
    const GLfloat squareVertices[] = {  
        -0.5f, -0.5f,  
        0.5f,  -0.5f,  
        -0.5f,  0.5f,  
        0.5f,   0.5f,  
    };  
    const GLubyte squareColors[] = {  
        255, 255,   0, 255,  
        0,   255, 255, 255,  
        0,     0,   0,   0,  
        255,   0, 255, 255,  
    };  
  
    for (size_t transformationIndex=0; transformationIndex<m_detectedMarkers.size(); transformationIndex++)  
    {  
        const Transformation& transformation = m_detectedMarkers[transformationIndex].transformation;  
        Matrix44 glMatrix = transformation.getMat44();  
        
        //导入相机外参数矩阵模型
        glLoadMatrixf(reinterpret_cast<const GLfloat*>(&glMatrix.data[0]));  //reinterpret_cast:任何类型的指针之间都可以互相转换,修改了操作数类型,仅仅是重新解释了给出的对象的比特模型而没有进行二进制转换
  
        glVertexPointer(2, GL_FLOAT, 0, squareVertices);  //指定顶点数组的位置，2表示每个顶点由三个量构成（x, y），GL_FLOAT表示每个量都是一个GLfloat类型的值。第三个参数0。最后的squareVertices指明了数组实际的位置。
        //这个squareVertices是由第一个参数和要画的图形有几个顶点决定大小，理解。
        glEnableClientState(GL_VERTEX_ARRAY);  //表示启用顶点数组
        glColorPointer(4, GL_UNSIGNED_BYTE, 0, squareColors);  //RGBA颜色，四个顶点
        glEnableClientState(GL_COLOR_ARRAY);  //启用颜色数组
  
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);  
        glDisableClientState(GL_COLOR_ARRAY);  
  
        float scale = 0.5;  
        glScalef(scale, scale, scale);  
  
        glColor4f(1.0f, 0.0f, 0.0f, 1.0f);  
        glVertexPointer(3, GL_FLOAT, 0, lineX);  
        glDrawArrays(GL_LINES, 0, 2);  
  
        glColor4f(0.0f, 1.0f, 0.0f, 1.0f);  
        glVertexPointer(3, GL_FLOAT, 0, lineY);  
        glDrawArrays(GL_LINES, 0, 2);  
  
        glColor4f(0.0f, 0.0f, 1.0f, 1.0f);  
        glVertexPointer(3, GL_FLOAT, 0, lineZ);  
        glDrawArrays(GL_LINES, 0, 2);  
    }     
    glFlush();  
    glPopMatrix();  
  
    glDisableClientState(GL_VERTEX_ARRAY);    
  
    glutSwapBuffers();  
}  
  
int show(const Mat &image,Mat_<float>& cameraMatrix)  
{  

    imagewidth = image.width;
    imageheight = image.height;
    
  build_projection(cameraMatrix);  //这是建立摄像机内参数矩阵，就是相机矩阵，display函数开始导入的模型就是相机矩阵  
    glutInitDisplayMode(GLUT_DOUBLE|GLUT_RGBA);  
    glutInitWindowPosition(100,100);  
    glutInitWindowSize(imagewidth,imageheight);  
    glutCreateWindow("OPenGL");  
    glutDisplayFunc(&display);  
    glutMainLoop();  
    //-------------------------------------  
    //free(pixeldata);  
    return 0;  
}


void readCameraParameter()
{
  camMatrix = Mat::eye(3, 3, CV_64F);
  distCoeff = Mat::zeros(8, 1, CV_64F);

  FileStorage fs("$(find cainiao_robot_service)/limbs/intrinsics.yml",FileStorage::READ);
  if (!fs.isOpened())
    {
      cout << "Could not open the configuration file!" << endl;
      exit(1);
    }
  fs["Camera_Matrix"] >> camMatrix;
  fs["Distortion_Coefficients"] >> distCoeff;
  fs.release();
  cout << camMatrix << endl;
  cout << distCoeff << endl;
}

void readCameraParameter1()
{
  //calibratoin data for iPad 2 
    camMatrix = Mat::eye(3, 3, CV_64F);
    distCoeff = Mat::zeros(8, 1, CV_64F); 

    camMatrix(0,0) = 569.46306454976900;  
    camMatrix(1,1) = 568.55359971960002;  
    camMatrix(0,2) = 326.13149875115442; //640  
    camMatrix(1,2) = 234.45787177933926; //
  
    for (int i=0; i<4; i++)  
        distCoeff(i,0) = 0;
}


 private:

  static GLint imagewidth;  
  static GLint imageheight;  
  //static GLint pixellength;  
  //static GLubyte* pixeldata;  


  Matrix44 projectionMatrix;  
  GLuint defaultFramebuffer, colorRenderbuffer;

//  int cameraNumber = 0;
  Mat_<float>  camMatrix;
  Mat_<float>  distCoeff;

  */
};

int main(int argc, char** argv)
{
 ros::init(argc, argv, "image_converter");
 glutInit(&argc,argv);
 //readCameraParameter1();
 //cout << camMatrix << endl;

 ImageConverter ic;


 // int imagewidth = image.cols;
 //   int imageheight = image.rows;
    
    //build_projection(cameraMatrix);  //这是建立摄像机内参数矩阵，就是相机矩阵，display函数开始导入的模型就是相机矩阵  
    glutInitDisplayMode(GLUT_DOUBLE|GLUT_RGBA);  
    glutInitWindowPosition(100,100);  
    glutInitWindowSize(640,480);  
    glutCreateWindow("OPenGL");
    ic.init();  
    //glutDisplayFunc(&display);
    glutDisplayFunc(&ic.display);
    glutMainLoop(); 
 ros::spin();
 return 0;
}