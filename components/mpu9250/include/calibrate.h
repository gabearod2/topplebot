/*****************************************************************************
 *                                                                           *
 *  Copyright 2018 Simon M. Werner                                           *
 *                                                                           *
 *  Licensed under the Apache License, Version 2.0 (the "License");          *
 *  you may not use this file except in compliance with the License.         *
 *  You may obtain a copy of the License at                                  *
 *                                                                           *
 *      http://www.apache.org/licenses/LICENSE-2.0                           *
 *                                                                           *
 *  Unless required by applicable law or agreed to in writing, software      *
 *  distributed under the License is distributed on an "AS IS" BASIS,        *
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. *
 *  See the License for the specific language governing permissions and      *
 *  limitations under the License.                                           *
 *                                                                           *
 *****************************************************************************/

#ifndef __CALIBRATE_H
#define __CALIBRATE_H

void calibrate_gyro(vector_t *vg_sum_holder);
void calibrate_accel(vector_t *offset_holder, vector_t *scale_lo_holder, vector_t *scale_hi_holder);
void calibrate_mag(vector_t *v_min_holder, vector_t *v_max_holder, vector_t *v_scale_holder);

#endif