/*
 * bpl_tmplt_mngr.h
 *
 *  Created on: May 5, 2017
 *      Author: hujia
 */

#ifndef CORE_RECOGNITION_BPL_TMPLT_H_
#define CORE_RECOGNITION_BPL_TMPLT_H_

#include "mdl_box.h"
#include <vector>

class bpl_tmplt {
public:
    bpl_tmplt();
    virtual ~bpl_tmplt();

    int load_templates_from_file(const char *file, int w, int h);
	void destroy_templates();

	face_template_t *pick_next_template(std::vector<face_template_t*> vtemps);
	int has_next_template(std::vector<face_template_t*> vtemps);
	int get_unlocked_template_number();

	void unlock_all_box_templates(std::vector< face_template_t* > m_face_templates);

	inline face_template_t *tat(int i) {	return m_face_templates[i]; }
	inline std::vector< face_template_t* > tmplts() {    return m_face_templates; }

	void get_ids(int **ids, int *num);

private:
	std::vector< face_template_t* > m_face_templates;
};

#endif /* CORE_RECOGNITION_BPL_TMPLT_H_ */
