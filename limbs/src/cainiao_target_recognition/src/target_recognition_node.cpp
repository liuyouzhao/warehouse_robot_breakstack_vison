#include <ros/ros.h>
#include "tf/tf.h"
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "vt_io.h"
#include "vt_visual.h"
#include "vt_pointCloud.h"
#include "opencv2/opencv.hpp"
#include "vt_depthTrans.h"
#include <time.h>
#include <algorithm>
#include "boxIsolate.h"
#include "PlaneFit2.h"
#include "Rotation.h"
#include "aubo_msgs/gettargetpose.h"
//#include <pcl/visualization/cloud_viewer.h>
//typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
#define PI 3.14159265359
// namspace
using namespace std;
using namespace sensor_msgs;
using namespace vt;
using namespace std;
using namespace cv;

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include "ipc.h"

typedef struct rgb_box_s
{
    float x;
    float y;
    float z;
    float box_width;
    float box_height;
} rgb_box_t;

class targetRecognition
{

    
public:
    targetRecognition()
    {
        target_service = nh.advertiseService("target_recognition", &targetRecognition::PosePublish, this);
   
        pcl_sub = nh.subscribe("/camera/depth/points", 10, &targetRecognition::cloudCB, this);
       // pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_", 1);

    }
       
    void cloudCB(const sensor_msgs::PointCloud2& input)
    {

      //  pcl::PCLPointCloud2 pcl_pc2;
      //  pcl_conversions::toPCL(input, pcl_pc2);
    
       // pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      //  pcl::fromPCLPointCloud2(pcl_pc2,*cloud);
       // sensor_cloud_msg_ = sensor_msgs::PointCloud2(*input);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg (input, *cloud);
       // pcl::visualization::CloudViewer viewer("abc");
       // viewer.showCloud(cloud);

        int pointID = 0;
        Mat result = Mat::zeros(cloud->height, cloud->width, CV_32FC1);
        int count =0;

        if (cloud->isOrganized()) 
        {
           
            if (!cloud->empty()) 
            {

                for (int h=0; h<result.rows; h++)
                 {
                    float *data = result.ptr<float>(h);

                    for (int w=0; w<result.cols; w++) 
                    {
                       // pcl::PointXYZ point = cloud->at(w, h);
                        if(isnan(cloud->points[pointID].z))
                        {
                            data[w] = 0.0;

                        }
                        else
                        {   
                          //  cout<< "打印点云数据：" << cloud->points[pointID].z << endl;
                            if(cloud->points[pointID].z>1.9)
                                count++;
                             if(cloud->points[pointID].z > 2.0)
                             {
                                data[w] = 0.0;

                             }
                             else
                             {
                                 data[w] = cloud->points[pointID].z * 1000;


                             }
                             // if(data[w] > 2000.0)
                             //   count++;

                        }
                       
                      //  cout<< data[w] <<endl;
                        pointID ++;
                    }
                 }
            }
        }
printf("errorpoint count = %d\n", count);
        //printf("wh: %d %d\n", cloud->height, cloud->width);

        Mat_<short> depth_;

      //  imshow("result", result);
      //  waitKey(1);
        result.convertTo(depth_, CV_16S);
        vector<Mat_<short> > depths_;
        vector<float> pkpt_, pkv_;
        depths_.push_back(depth_);

        detect(depths_, pkpt_, pkv_);
        
        
//cout<< pkpt_[0] <<endl;
      //  ROS_INFO("%s %d %s", __FILE__, __LINE__, __FUNCTION__);
        
        //ROS_INFO("%s %d %s", __FILE__, __LINE__, __FUNCTION__);
        if(pkpt_.empty())
          return;
        goal_.position.x = pkpt_[0];
        
        goal_.position.y = pkpt_[1];
        goal_.position.z = pkpt_[2];
        goal_.orientation.x = pkpt_[3];
        goal_.orientation.y = pkpt_[4];
        goal_.orientation.z = pkpt_[5];
        goal_.orientation.w = pkpt_[6];


        // new cloud formation 
       //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        
       

       // free(depth);
      //  pcl::toROSMsg(cloud_filtered, output);
    	//output.header.frame_id = "camera_depth_optical_frame";

       // pcl_pub.publish(output);
    }


     bool PosePublish(aubo_msgs::gettargetpose::Request  &req, aubo_msgs::gettargetpose::Response &res)
    {
       // sensor_msgs::PointCloud2 msg = sensor_cloud_msg_;
        

        ROS_INFO("%s %d %s", __FILE__, __LINE__, __FUNCTION__);
        bool flag = req.pick_start;

        ROS_INFO("go here");
        geometry_msgs::Pose goal;
        //goal = goal_;
        ROS_INFO("%s %d %s", __FILE__, __LINE__, __FUNCTION__);
       // cout<< "goal_.position.x = " <<goal_.position.x <<endl;


	/* IPC receive */
        ipc::send("ready", 5, 1888);
	char buf[__MB_MSG_SIZE] = {0};
	int r = ipc::recv(buf, 1888);
	ROS_INFO("ipc received already: %d\n", r);
	if(r > 0)
	{
	    rgb_box_t t;
	    memcpy(&t, buf, sizeof(rgb_box_t));
	    ROS_INFO("ipc received: %f %f %f %f %f\n", t.x, t.y, t.z, t.box_width, t.box_height);

	    double pid = PI / 180.0;
            tf::Quaternion q = tf::createQuaternionFromRPY( -pid *178 , pid*3.05,pid  * 40.71 ); // roll , pitch ,yaw
            goal.position.x = -0.4329;
            goal.position.y = 0.3865;
            goal.position.z = -0.05;
            
            goal.orientation.x = q.x();
            goal.orientation.y = q.y();
            goal.orientation.z = q.z();
            goal.orientation.w = q.w();
	}


	

        if(flag)
        {
            goal = goal_;
        }
      /*  else
        {
            double pid = PI / 180.0;
            tf::Quaternion q = tf::createQuaternionFromRPY( -pid *178 , pid*3.05,pid  * 40.71 ); // roll , pitch ,yaw
          //  tf::Quaternion q = tf::createQuaternionFromRPY( pid  * 9.95 ,pid  * 63.9 ,pid  * 7.31 ); // roll , pitch ,yaw
         
            goal.position.x = -0.4329;
            goal.position.y = 0.3865;
            goal.position.z = -0.05;
            
            goal.orientation.x = q.x();
            goal.orientation.y = q.y();
            goal.orientation.z = q.z();
            goal.orientation.w = q.w();
        }
        
    */
        res.target_pose = goal;
        res.succeeded = true;

        return true;
   }


  void detect(vector<Mat_<short> >& depths, vector<float>& pkpt, vector<float>& pkv)
  {    
    //const float pi = 3.14159265;
    int width = depths[0].cols;
    int height = depths[0].rows;
    int boxWidth = 520;
    int boxHeight = 360;
    Mat boxbottom;
    float bottom[3];
  
    double handeye[24];
    loadBinary("handeye", handeye);
    saveBinary("depth.raw",depths[0].data, depths[0].total()*2);
   // getchar();
    Mat R, t, tR, tt;
    {
        float temp[24];
        for (int i = 0; i < 24; i++)
            temp[i] = handeye[i];
        R = Mat(3, 3, CV_32F, temp).clone();
        t = Mat(3, 1, CV_32F, temp + 9).clone();
        tR = Mat(3, 3, CV_32F, temp + 12).clone();
        tt = Mat(3, 1, CV_32F, temp + 21).clone();
    } 
  //  ROS_INFO("%s %d %s", __FILE__, __LINE__, __FUNCTION__);
    loadBinary("bottom", bottom);
    printf("bottom %f %f %f ", bottom[0], bottom[1], bottom[2]);
   // ROS_INFO("%s %d %s", __FILE__, __LINE__, __FUNCTION__);
    Mat intri[2];
    {
     //   ROS_INFO("%s %d %s", __FILE__, __LINE__, __FUNCTION__);
        FileStorage fs("intrinsics.yml", FileStorage::READ);
      //  ROS_INFO("%s %d %s", __FILE__, __LINE__, __FUNCTION__);
        fs["M1"] >> intri[0];
        fs["D1"] >> intri[1];
        for (int i = 0; i < 2; i++)
        {
            intri[i].convertTo(intri[i], CV_32F);
        }
    }
    cout << intri[0];
  //  ROS_INFO("%s %d %s", __FILE__, __LINE__, __FUNCTION__);
    Mat_<Vec3f> world(height, width);  
    boxbottom = Mat(3, 1, CV_32F, bottom);
    vector<Mat_<Vec3f> > worlds;       
   // ROS_INFO("%s %d %s", __FILE__, __LINE__, __FUNCTION__);
    for (int j = 0; j < depths.size(); j++)
    {
        Mat_<Vec3f> world(height, width);
        depth2World(depths[j], world, (float*)intri[0].data, (float*)intri[1].data);
        worlds.push_back(world);
    }
   //imshow("depths", toGray(depths[0]));
    //waitKey(0);
    multiFrames(worlds, world, 10);
    for(int j=0;j<world.total();j++)
    {
        if(world(j)(2)<10)
            world(j)(2) = world(j)(1) =world(j)(0)=NAN;
    }
        //continue;
        vector<Mat> wall;
     //   ROS_INFO("%s %d %s", __FILE__, __LINE__, __FUNCTION__);
        Rect rt;
        {
            rt = boxEst((float*)world.data, (float*)boxbottom.data, world.cols, world.rows, 280, 20, 620, 440, wall,55,25);
            wall.clear();
     //       ROS_INFO("%s %d %s", __FILE__, __LINE__, __FUNCTION__);
            if (rt.width == 0)
                return;
         //   ROS_INFO("%s %d %s", __FILE__, __LINE__, __FUNCTION__);
            //rt=Rect(250,150,250,200);
            world = Mat(world, rt).clone();
       //     ROS_INFO("%s %d %s", __FILE__, __LINE__, __FUNCTION__);
            setBorder(world);
     //       ROS_INFO("%s %d %s", __FILE__, __LINE__, __FUNCTION__);
            removePointsOnPlane(world, boxbottom);
     //       ROS_INFO("%s %d %s", __FILE__, __LINE__, __FUNCTION__);
        }
        Mat_<short> depth(world.size(), CV_16S);

    //    ROS_INFO("%s %d %s", __FILE__, __LINE__, __FUNCTION__);
        for (int i = 0; i < world.total(); i++)
        {
            float& x = world(i)(2);
            if (isnan(x))
                depth(i) = -10000;
            else
                depth(i) = x;
        }
       // imshow("depthmat", toGray(depth));
        imshow("depthmat", depth);
        waitKey(1);
      //  ROS_INFO("%s %d %s", __FILE__, __LINE__, __FUNCTION__);
        PlaneFit2 ns(depth, world, wall);
        //vector<float> pkpt, pkv;
       // ROS_INFO("%s %d %s", __FILE__, __LINE__, __FUNCTION__);
        Mat_<double> ptR;
        ns.run(pkpt, ptR, pkv);
        if(!pkpt.empty())
        {
          Mat ptR2;
          ptR.convertTo(ptR2, CV_32F);
          Mat Re = R*ptR2*tR.t();
          Mat P = (Mat_<float>(3, 1) << pkpt[0], pkpt[1], pkpt[2]);
          Mat newP = (R*P + t) / 1000;
          //tt.at<float>(0)=0;
          //tt.at<float>(1)=0;
          //newP = (R*P + t - Re*tt) / 1000;

          pkpt[0] = newP.at<float>(0);
          pkpt[1] = newP.at<float>(1);
          pkpt[2] = newP.at<float>(2);
          Re.convertTo(ptR, CV_64F);
          Vec4d q = eulerAngles2quaternion(rotationMatrixToEulerAngles(ptR));
          pkpt[3] = q[1]; pkpt[4] = q[2]; pkpt[5] = q[3]; pkpt[6] = q[0];
          for (int t = 0; t < 4; t++)
          {
              pkv[t*2] += rt.x;
              pkv[2 * t + 1] += rt.y;
          }
          cout<<newP<<"; "<<q<<endl;
        }
        
        
    //ROS_INFO("%s %d %s", __FILE__, __LINE__, __FUNCTION__);
   
    //    _CrtDumpMemoryLeaks();
  }


protected:

    ros::NodeHandle nh;
    geometry_msgs::Pose goal_;
    //sensor_msgs::PointCloud2 sensor_cloud_msg_;
    ros::Subscriber pcl_sub;
   // ros::Publisher pcl_pub;
    ros::ServiceServer target_service;

    
};

main(int argc, char** argv)
{
    ros::init(argc, argv, "target_recognition");

    ROS_INFO("Started target_recognition Node");
    targetRecognition handler;

    ros::spin();

    return 0;
}

