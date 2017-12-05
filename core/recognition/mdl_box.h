/*
 * mdl_box.h
 *
 *  Created on: May 3, 2017
 *      Author: hujia
 */

#ifndef CORE_RECOGNITION_MDL_BOX_H_
#define CORE_RECOGNITION_MDL_BOX_H_
#include <stdlib.h>
#include <string.h>
#include <vector>

typedef struct junction_s
{
    void *owner;
    std::vector<void*> pointers;
    int jid;
} junction_t;

typedef struct joints_s
{
    int jid;
    int offsets[32];
    int num;
} joints_t;

typedef struct face_template_s
{
    face_template_s(float w, float h, int _id):
        width(w),
        height(h),
        id(_id)
    {
    }

    int width;
    int height;
    int id;
    int locked;

    struct joints_s joints[4];

} face_template_t;

typedef struct face_state_s
{
    int x;
    int y;
    int width;
    int height;
} face_state_t;

/*     w
 * ---------vert
 * |       |
 * |       |h
 * |       |
 * |       |junctions->close to other faces
 * ---------
 *
 */
typedef struct face_entity_s
{
    face_entity_s():
        parent(0),
        ptmplt(0)
    {
        memset(juncts, 0, sizeof(void*) * 4);
        memset(id, 0, 32);
    }

    void __construct()
    {
        memset(juncts, 0, sizeof(void*) * 4);
        memset(id, 0, 32);
        parent = NULL;
        ptmplt = NULL;
    }

    char id[32];
    struct junction_s *juncts[4];
    struct face_entity_s *parent;

    face_template_t *ptmplt;
    face_state_t state;

} face_entity_t;

#define IS_LEAF_ENTITY(e) !(e->juncts[0]) && !(e->juncts[1]) && !(e->juncts[2]) &&!(e->juncts[3])

face_entity_t *face_clone(face_entity_t *src);
face_entity_s *face_entity_create();
void face_entity_destroy(face_entity_s **p);
void face_tree_destroy(face_entity_t **root);
void face_tree_copy(face_entity_t *f, face_entity_t **fn, face_entity_t **root);
int face_belong_junct(face_entity_t *e, face_entity_t *p);
void face_tree_print(face_entity_t *root);


void prm_open_file(const char *filename);
void prm_open_file_read(const char *filename);
void prm_begin_append();
void prm_append(int from, int to, int junction);
void prm_end_append();
void prm_close_file();
void prm_set_templates(std::vector< face_template_t* > tmplts);
face_entity_t *prm_getnext_entity();

#if CONFIG_SUPPORT_3D_BOX
typedef struct tmplt_box_s
{
    /*   /-------------/
     *  /      w      /d
     *  --------------
     *  |            |h
     *  --------------
     *
     *  Let the longest slide be the 'w' value;
     *  The shortest slide be the 'h' value;
     *  The slide left naturely gotta be 'd' value
     * */
    float w;
    float h;
    float d;

    /*        /
     * lt--> -----------
     *       |_________
     *
     * The left and top point vector
     * */
    float lt_vec3[3];

    /*
     * The center means the distance from which to each
     * corner point be the same.
     */
    float center_vec3[3];

    /* upper face normal vec3 */
    float up_norm_vec3[3];

} tmplt_box_t;
#endif

#endif /* CORE_RECOGNITION_MDL_BOX_H_ */
