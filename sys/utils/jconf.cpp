#include "jconf.h"
#include <stdio.h>
#include <string.h>
#include "mm.h"

#define JCONF_MAX_BUF_LEN 2048

jconf::jconf(const char *workspace, const char *file, int id):
    m_pjson(NULL),
    m_filename(NULL)
{
    int len = strlen(workspace) + strlen(file) + 32;
    m_filename = (char*)DEBUG_MALLOC(len);
    memset(m_filename, 0, len);
    sprintf(m_filename, "%s/%s%d", workspace, file, id);

    char *buffer = (char*) DEBUG_MALLOC(JCONF_MAX_BUF_LEN);
    memset(buffer, 0, JCONF_MAX_BUF_LEN);

    FILE *fd = fopen(m_filename, "r");
    if(fd == NULL)
    {
        fd = fopen(m_filename, "w+");
    }
    fread(buffer, JCONF_MAX_BUF_LEN, 1, fd);
    fclose(fd);

    m_pjson = cJSON_Parse(buffer);
    if(m_pjson == NULL)
    {
        m_pjson = cJSON_CreateObject();
    }

    DEBUG_FREE(buffer);
}

jconf::jconf(const char *workspace, const char *file):
    m_pjson(NULL),
    m_filename(NULL)
{
    int len = strlen(workspace) + strlen(file);
    m_filename = (char*)DEBUG_MALLOC(len);
    memset(m_filename, 0, len);
    sprintf(m_filename, "%s/%s", workspace, file);

    char *buffer = (char*) DEBUG_MALLOC(JCONF_MAX_BUF_LEN);
    memset(buffer, 0, JCONF_MAX_BUF_LEN);

    FILE *fd = fopen(m_filename, "r");
    if(fd == NULL)
    {
        fd = fopen(m_filename, "w+");
    }
    fread(buffer, JCONF_MAX_BUF_LEN, 1, fd);
    fclose(fd);

    m_pjson = cJSON_Parse(buffer);
    if(m_pjson == NULL)
    {
        m_pjson = cJSON_CreateObject();
    }

    DEBUG_FREE(buffer);
}

jconf::jconf(const char *full):
    m_pjson(NULL),
    m_filename(NULL)
{
    int len = strlen(full);
    m_filename = (char*)DEBUG_MALLOC(len);
    memcpy(m_filename, full, len);

    char *buffer = (char*) DEBUG_MALLOC(JCONF_MAX_BUF_LEN);
    memset(buffer, 0, JCONF_MAX_BUF_LEN);

    FILE *fd = fopen(m_filename, "r");
    if(fd == NULL)
    {
        fd = fopen(m_filename, "w+");
    }
    fread(buffer, JCONF_MAX_BUF_LEN, 1, fd);
    fclose(fd);

    m_pjson = cJSON_Parse(buffer);
    if(m_pjson == NULL)
    {
        m_pjson = cJSON_CreateObject();
    }

    DEBUG_FREE(buffer);
}

jconf::~jconf()
{
    cJSON_Delete(m_pjson);
    DEBUG_FREE(m_filename);
}

cJSON *jconf::ptrj()
{
    return m_pjson;
}

void jconf::print()
{
    char *p = cJSON_Print(m_pjson);
    printf("%s\n", p);
    free(p);
}

int jconf::sync()
{
    char *p = cJSON_PrintUnformatted(m_pjson);
    int len = strlen(p);

    FILE *fd = fopen(m_filename, "w+");
    int num = fwrite(p, strlen(p), 1, fd);
    fclose(fd);

    free(p);
    return num == len ? 0 : -1;
}

int jconf::syncf()
{
    char *p = cJSON_Print(m_pjson);
    int len = strlen(p);

    FILE *fd = fopen(m_filename, "w+");
    int num = fwrite(p, strlen(p), 1, fd);
    fclose(fd);

    free(p);
    return num == len ? 0 : -1;
}

/* TODO: To make below functions' return values associate with actual judgement */
int jconf::set_root_string(const char *key, const char *value)
{
    cJSON_AddItemToObject(m_pjson, key, cJSON_CreateString(value));
    return 0;
}

int jconf::set_root_double(const char *key, double value)
{
    cJSON_AddItemToObject(m_pjson, key, cJSON_CreateNumber(value));
    return 0;
}

int jconf::add_root_array(const char *key)
{
    cJSON_AddItemToObject(m_pjson, key, cJSON_CreateArray());
    return 0;
}

int jconf::add_array_item(const char *arrkey)
{
    cJSON *parr = cJSON_GetObjectItem(m_pjson, arrkey);
    cJSON_AddItemToArray(parr, cJSON_CreateObject());
    return 0;
}

int jconf::set_array_item_string(const char *arrkey, int index, const char *key, const char *value)
{
    cJSON *parr = cJSON_GetObjectItem(m_pjson, arrkey);
    cJSON *pobj = cJSON_GetArrayItem(parr, index);
    cJSON_AddItemToObject(pobj, key, cJSON_CreateString(value));
    return 0;
}

int jconf::set_array_item_double(const char *arrkey, int index, const char *key, double value)
{
    cJSON *parr = cJSON_GetObjectItem(m_pjson, arrkey);
    cJSON *pobj = cJSON_GetArrayItem(parr, index);
    cJSON_AddItemToObject(pobj, key, cJSON_CreateNumber(value));
    return 0;
}

double jconf::get_number(const char *key)
{
    return cJSON_GetObjectItem(m_pjson, key)->valuedouble;
}

char *jconf::get_string(const char *key)
{
    return cJSON_GetObjectItem(m_pjson, key)->valuestring;
}
