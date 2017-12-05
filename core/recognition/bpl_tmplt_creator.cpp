#include "bpl_tmplt_creator.h"
#include "jconf.h"
#include "mm.h"

bpl_tmplt_creator::bpl_tmplt_creator(const char *jconffile)
{
    jconf conf(jconffile);
    cJSON *json = conf.ptrj();
    cJSON *jarr = cJSON_GetObjectItem(json, "t");
    int num = cJSON_GetArraySize(jarr);

    m_vec_templates.clear();

    for(int i = 0; i < num; i ++)
    {
        cJSON *t = cJSON_GetArrayItem(jarr, i);
        double w = cJSON_GetObjectItem(t, "w")->valuedouble;
        double h = cJSON_GetObjectItem(t, "h")->valuedouble;
        double n = cJSON_GetObjectItem(t, "n")->valuedouble;

        origin_template_t *tmp = (origin_template_t*) DEBUG_MALLOC(sizeof(origin_template_t));
    }
}

bpl_tmplt_creator::~bpl_tmplt_creator()
{
}
