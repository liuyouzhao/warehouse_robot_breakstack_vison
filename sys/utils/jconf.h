#ifndef JCONF_H
#define JCONF_H

#include "cjson.h"

class jconf
{
public:
    jconf(const char *workspace, const char *file, int id);
    jconf(const char *workspace, const char *file);
    jconf(const char *full);
    ~jconf();

    cJSON *ptrj();

    void print();
    int sync();
    int syncf();

    int set_root_string(const char *key, const char *value);
    int set_root_double(const char *key, double value);
    int add_root_array(const char *key);

    int add_array_item(const char *arrkey);
    int set_array_item_string(const char *arrkey, int index, const char *key, const char *value);
    int set_array_item_double(const char *arrkey, int index, const char *key, double value);

    /**
     * @brief get_number
     * @param key
     * Get double of the most shallow layer attribute
     * @return
     */
    double get_number(const char *key);
    char* get_string(const char *key);

private:
    cJSON *m_pjson;
    char *m_buffer;
    char *m_filename;
};

#endif // JCONF_H
