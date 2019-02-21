#include <stdio.h>
#include "quaternions.h"

int main (void)
{
    float vector[3] = {1.0f, 0.0f, 0.0f};
    float angle = 90.0f;
    float vector_rot[3] = {0.0f, 1.0f, 0.0f};

    float quat_rot[4] = {0};

    float result[4] = {0};

    quat_from_axis_angle(quat_rot, vector_rot, angle);

    printf("w:%f, x:%f, y:%f, z:%f\n", quat_rot[0], quat_rot[1], quat_rot[2], quat_rot[3]);

    quat_vect_mult(result, quat_rot, vector);

    printf("w:%f, x:%f, y:%f, z:%f\n", result[0], result[1], result[2], result[3]);


    
    return 0;
}
