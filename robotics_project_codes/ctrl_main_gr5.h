/*! 
 * \file ctrl_main_gr5.h
 * \brief main header of the controller
 */

#ifndef _CTRL_MAIN_GR5_H_
#define _CTRL_MAIN_GR5_H_

#include "CtrlStruct_gr5.h"
#include "namespace_ctrl.h"
#include <stdlib.h>



NAMESPACE_INIT(ctrlGr5);

void controller_init(CtrlStruct *cvs);
void controller_loop(CtrlStruct *cvs);
void controller_finish(CtrlStruct *cvs);

// Our prototypes functions. All prototypes must be defined here



NAMESPACE_CLOSE();

#endif
