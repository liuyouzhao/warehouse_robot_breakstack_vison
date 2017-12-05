/*
 * bpl_main.h
 *
 *  Created on: May 3, 2017
 *      Author: hujia
 */

#ifndef CORE_RECOGNITION_BPL_MAIN_H_
#define CORE_RECOGNITION_BPL_MAIN_H_

class bpl_main
{
public:
	~bpl_main();

	static bpl_main *instance();

    void run();
private:
	bpl_main();
};



#endif /* CORE_RECOGNITION_BPL_MAIN_H_ */
