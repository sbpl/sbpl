/*
 * Copyright (c) 2008, Maxim Likhachev
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Pennsylvania nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __HEADERS_H_
#define __HEADERS_H_

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <vector>
#include <stdlib.h>

#include "sbpl_exception.h"
#include "config.h"


#if MEM_CHECK == 1
#define _CRTDBG_MAP_ALLOC 
#define CRTDBG_MAP_ALLOC
#endif

#include <stdlib.h> //have to go after the defines above

#if MEM_CHECK == 1
#include <crtdbg.h>
#endif

#include "../utils/key.h"
#include "../utils/mdpconfig.h"
#include "../utils/mdp.h"
#include "../utils/utils.h"
#include "../planners/planner.h"
#include "../discrete_space_information/environment.h"
#include "../discrete_space_information/template/environment_XXX.h"
#include "../discrete_space_information/nav2d/environment_nav2D.h"
#include "../discrete_space_information/navxythetalat/environment_navxythetalat.h"
#include "../discrete_space_information/navxythetalat/environment_navxythetamlevlat.h"
#include "../discrete_space_information/robarm/environment_robarm.h"
#include "../discrete_space_information/nav2d_uu/environment_nav2Duu.h"
#include "../utils/list.h"
#include "../utils/heap.h"
#include "../planners/VI/viplanner.h"
#include "../planners/ARAStar/araplanner.h"
#include "../planners/ADStar/adplanner.h"
#include "../utils/2Dgridsearch.h"
#include "../planners/PPCP/ppcpplanner.h"
#include "../planners/RStar/rstarplanner.h"


#endif

