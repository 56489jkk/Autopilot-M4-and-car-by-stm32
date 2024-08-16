/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-29     CGY       the first version
 */

#include "rtthread.h"
#include "stdbool.h"
#include "math.h"
#include "CL_Vector.h"
#include "CL_Math.h"

float length_v3(Vector3f *v)
{
    return sqrtf(sq(v->x)+sq(v->y)+sq(v->z));
}
float length_v2(Vector2f *v)
{
    return sqrtf(sq(v->x)+sq(v->y));
}






