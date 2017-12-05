#include "bjam_state.h"
#include "jconf.h"

sku_attribute_t g_default_sku = {
    {
        cv::Point2f(20.0f, 20.0f),
        cv::Point2f(20.0f, 20.0f),
        cv::Point2f(20.0f, 20.0f)
    },
    0
};

int bjam_state_load_sku(sku_attribute_t &sku, unsigned long uuid)
{
    char filename[256] = {0};
    sprintf(filename, "../../params/sku/box%08d", uuid);
    jconf jc(filename);
    double l = jc.get_number("l");
    double w = jc.get_number("w");
    double h = jc.get_number("h");
    double i = jc.get_number("i");
    sku.box_rect_size[0].x = (int)l;
    sku.box_rect_size[0].y = (int)w;
    sku.box_rect_size[1].x = (int)l;
    sku.box_rect_size[1].y = (int)h;
    sku.box_rect_size[2].x = (int)w;
    sku.box_rect_size[2].y = (int)h;
    sku.upper_index = (int)i;

    memset(sku.train_1, 0, 64);
    memset(sku.train_2, 0, 64);

    char *t1 = jc.get_string("train1");
    char *t2 = jc.get_string("train2");
    memcpy(sku.train_1, t1, strlen(t1));
    memcpy(sku.train_2, t2, strlen(t2));
}
