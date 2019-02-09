#include <math.h>
#include <stdint.h>
#include "quaternions.h"
#include "return_values.h"


#define QUAT_STD_TOLERANCE 0.00001



int8_t 
quat_normalize (float * const p_tuple, int8_t const tuple_size, float const tolerance)
{
    int8_t result = EXIT_FAILURE;
    float length = 0.0f;

    if(NULL != p_tuple)
    {
        for(int8_t i = 0; i < tuple_size;i++)
        {
            length += p_tuple[i] * p_tuple[i];
        }    
    
        length = sqrtf(length);

        if(length - 1.0f > tolerance)
        {
            for(int8_t i = 0; i < tuple_size; i++)
            {
                p_tuple[i] = p_tuple[i] / length;        
            }
        }   
        
        result = EXIT_SUCCESS;
    }

    return result;
}

int8_t 
quat_mult(float * const p_result, float const * const p_quat_l, float const * const p_quat_r)
{
    int8_t result = EXIT_FAILURE;

    if(NULL != p_result
    && NULL != p_quat_l
    && NULL != p_quat_r)
    {
        int8_t const w = 0;
        int8_t const x = 1;
        int8_t const y = 2;
        int8_t const z = 3;

        p_result[w] = p_quat_l[w] * p_quat_r[w] - p_quat_l[x] * p_quat_r[x] - p_quat_l[y] * p_quat_r[y] - p_quat_l[z] * p_quat_r[z];
        p_result[x] = p_quat_l[w] * p_quat_r[x] + p_quat_l[x] * p_quat_r[w] + p_quat_l[y] * p_quat_r[z] - p_quat_l[z] * p_quat_r[y];
        p_result[y] = p_quat_l[w] * p_quat_r[y] + p_quat_l[y] * p_quat_r[w] + p_quat_l[z] * p_quat_r[x] - p_quat_l[x] * p_quat_r[z];
        p_result[z] = p_quat_l[w] * p_quat_r[z] + p_quat_l[z] * p_quat_r[w] + p_quat_l[x] * p_quat_r[y] - p_quat_l[y] * p_quat_r[x];
        
        result = EXIT_SUCCESS;
    }
    
    return result; 
}

int8_t
quat_from_axis_angle (float * const p_result, float const * const p_vect, float theta)
{
    int8_t result = EXIT_FAILURE;

    if(NULL != p_result
    && NULL != p_vect)
    {
        quat_normalize(p_vect, 3, QUAT_STD_TOLERANCE);
        
        theta /= 2;
        float sin_theta = sinf(theta);

        p_result[0] = cos(theta);
    
        for(int8_t i = 0; i < 3; i++)
        {
         p_result[i+1] = p_pect[i] * sin_theta;
        }
        
        result = EXIT_SUCCESS;
    }

    return result;
}

int8_t
quat_conjugate (float * const p_result, float const * const p_quat)
{
    int8_t result = EXIT_FAILURE;

    if(NULL != p_result
    && NULL != p_quat)
    { 
        p_result[0] = p_quat[0];

        for(int8_t i = 1; i < 4; i++)
        {   
            p_result[i] = -p_quat[i];
        }
    }
    
    return result;
}

int8_t
quat_vect_mult(float * const p_result, float const * const p_quat, float const * const p_vect)
{
    int8_t result = EXIT_FAILURE;

    if(NULL != p_result
    && NULL != p_quat
    && NULL != p_vect)
    {
        float vect_quat[4] = {0.0f};
        float quat_intermediate[4] = {0.0f};
        float quat_inv[4] = {0.0f};

        for(int8_t i = 0; i < 3; i++)
        {
            vect_quat[i+1] = p_vect[i];
        }
        
        quat_conjugate(quat_inv, p_quat);
                
        quat_mult(quat_intermediate, p_quat, vect_quat);

        quat_mult(p_result, quat_intermediate, quat_inv);
        
        result = EXIT_SUCCESS;
    }

    return result;
}

int8_t
quat_to_axis_angle(float * const p_vect, float const * const p_theta, float const * const p_quat)
{
    int8_t result = EXIT_FAILURE;
    
    if(NULL != p_vect
    && NULL != p_theta
    && NULL != p_quat)
    {
        *p_theta = acosf(p_quat[0]) * 2.0f;
        
        for(int8_t i = 0; i < 3; i++)
        {
            p_vect[i] = p_quat[i+1];
        }
        
        quat_normalize(p_vect, 3, QUAT_STD_TOLERANCE);

        result = EXIT_SUCCESS;
    } 

    return result;
}
