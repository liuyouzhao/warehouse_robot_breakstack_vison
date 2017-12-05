#include "jconf.h"
#include "boxIsolate.h"
#include "PlaneFit2.h"
#include "CameraPara.h"
#include "vt_depthTrans.h"
#include "vt_pointCloud.h"

#include "bjam_depth_proc.h"

using namespace cv;

bjam_depth_proc *bjam_depth_proc::s_p_self = NULL;

bjam_depth_proc *bjam_depth_proc::singleton()
{
    if(s_p_self == NULL)
    {
        s_p_self = new bjam_depth_proc();
    }
    return s_p_self;
}

bjam_depth_proc::bjam_depth_proc()
{
    /**
      [1] Read rectangle configure, with screen size
      */
    jconf jc("../../params/region.ini");
    double x = jc.get_number("x");
    double y = jc.get_number("y");
    double w = jc.get_number("w");
    double h = jc.get_number("h");
    double sw = jc.get_number("sw");
    double sh = jc.get_number("sh");
    m_target_region.x       = (int)x;
    m_target_region.y       = (int)y;
    m_target_region.width   = (int)w;
    m_target_region.height  = (int)h;
    m_screen_width = (int)sw;
    m_screen_height = (int)sh;

    /**
      [2] Load crutal params
      */
    //t3d::loadAstraParams("../../params/camera_params.ini", m_l2r, m_intri);
    t3d::loadIntri("../../params/intrinsics_stereo.yml", m_intri);
    {
        Mat left44 = Mat::zeros(4, 4, CV_32F);
        m_intri[0].copyTo(Mat(left44, Range(0, 3), Range(0, 3)));
        left44.at<float>(3, 3) = 1;
        Mat right44 = Mat::zeros(4, 4, CV_32F);
        m_intri[1].copyTo(Mat(right44, Range(0, 3), Range(0, 3)));
        right44.at<float>(3, 3) = 1;
        Mat LW2RW = Mat::zeros(4, 4, CV_32F);
        m_intri[2].copyTo(Mat(LW2RW, Range(0, 3), Range(0, 3)));
        m_intri[3].copyTo(Mat(LW2RW, Range(0, 3), Range(3, 4)));
        LW2RW.at<float>(3, 3) = 1;
        m_l2r = right44*LW2RW*left44.inv();
    }
    memset(m_sku_siz, 0, sizeof(int) * 2);

    m_p_ns = NULL;
    m_camera = m_intri[1].clone();
    m_camera.at<float>(2) -=x;
    m_camera.at<float>(5) -=y;

}

int bjam_depth_proc::inform_color_depth_map(cv::Mat color, cv::Mat depth)
{
    m_color = color.clone();
    m_rt_color = Mat(m_color, m_target_region).clone();

    m_depth = depth.clone();

    Mat_<Vec3f> world(m_screen_height, m_screen_width);
    Mat_<Vec3f> world1(m_screen_height, m_screen_width);
    Mat depthNew;
    vt::mappingDepth2Color(m_depth, depthNew, (float*)m_l2r.data);
    float k[5];
    memset(k,0,sizeof(float)*5);
    vt::depth2World((Mat_<short>)depthNew, world1,
                    (float*)m_intri[1].data, k);
    m_fxy = (m_intri[1].at<float>(0) + m_intri[1].at<float>(4)) / 2;


    for (int j = 0; j < (int) world1.total(); j++)
    {
        if (world1(j)(2) < 10)
            world1(j)(2) = world1(j)(0) = world1(j)(1) = NAN;
    }
    world = Mat(world1, m_target_region).clone();
    vt::setBorder(world);

    m_world = world.clone();
    m_world1 = world1.clone();

    if(m_p_ns)
    {
        delete m_p_ns;
    }
    if(m_sku_siz[0] == 0 && m_sku_siz[1] == 0)
    {
        printf("[ERROR] %s %d\nNo prior data box size.\n", __FUNCTION__, __LINE__);
        return -1;
    }
    int f1 = m_sku_siz[0] > m_sku_siz[1] ? m_sku_siz[1] : m_sku_siz[0];
    int f2 = m_sku_siz[0] > m_sku_siz[1] ? m_sku_siz[0] : m_sku_siz[1];

    Size boxSize(f1, f2);
    m_p_ns = new PlaneFit2(m_rt_color, m_world, boxSize, m_camera, 3, 20);

    return 0;
}

int bjam_depth_proc::run()
{
    /**
     * Depth segmentation!
     */
    if(m_p_ns == NULL)
    {
        printf("[ERROR] %s %d\ninform_color_depth_map not called\n", __FUNCTION__, __LINE__);
        return -1;
    }
    m_info_result = m_p_ns->run(m_result_image);

    m_damn_right = m_p_ns->damnRight;
    m_length_scale = m_p_ns->planeDistance / m_fxy;

    if(!m_p_ns->found)
        return -1;
    return 0;
}

int bjam_depth_proc::recalculate(std::vector<cv::Point> points)
{
    if(m_p_ns == NULL)
    {
        printf("[ERROR] %s %d\ninform_color_depth_map not called\n", __FUNCTION__, __LINE__);
        return -1;
    }

    m_p_ns->reCalculate(points);

    printf("points %f %f %f %f %f %f %f\n",
            m_p_ns->pickPoint[0],
            m_p_ns->pickPoint[1],
            m_p_ns->pickPoint[2],
            m_p_ns->pickPoint[3],
            m_p_ns->pickPoint[4],
            m_p_ns->pickPoint[5],
            m_p_ns->pickPoint[6]);
    return 0;
}
