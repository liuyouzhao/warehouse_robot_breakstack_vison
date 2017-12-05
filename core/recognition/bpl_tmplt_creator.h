#ifndef BPL_TMPLT_CREATOR_H
#define BPL_TMPLT_CREATOR_H

#include <vector>

class bpl_tmplt;

typedef struct origin_template_s
{
    double w;
    double h;
    int n;
    int id;
} origin_template_t;

class bpl_tmplt_creator
{
public:
    bpl_tmplt_creator(const char *jconffile);
    ~bpl_tmplt_creator();

    bpl_tmplt *result();
private:
    bpl_tmplt *m_inner;

    std::vector< origin_template_t* > m_vec_templates;
};

#endif // BPL_TMPLT_CREATOR_H
