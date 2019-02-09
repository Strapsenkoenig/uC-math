/****************************************************************************//**
\brief          This function normalizes a tupel.

\details        This function normalizes all kind of tuples based on the norm2. 
                Because the dimension (length) of the tuple can be specified by 
                the parameter \p tuple_size, this function can be used on 
                2 or 3-dimensional vectors and quaternions. 

\param[in,out]  p_tuple The tuple to normalize.
\param[in]      tuple_size The dimension of the tuple.
\param[in]      tolernace The tolerance that determines if the tuple needs 
                re-normalization.

\retval         #EXIT_FAILURE if \p p_tuple is NULL.
                #EXIT_SUCCESS else.
 
********************************************************************************/
int8_t quat_normalize (float * const p_tuple, int8_t const tuple_size, float const tolerance);

/****************************************************************************//**
\brief          This function multiplies two quaternions.

\details        The multiplication is done using the Hamilton production.
                x = (x0 x1*i x2*j x3*k)
                y = (y0 y1*i y2*j y3*k)

\details        xy      = (x0*y0 - x1*y1 - x2*y2 - x3*y3)
                        + (x0*y1 + x1*y0 + x2*y3 - x3*y2)i
                        + (x0*y2 - x1*y3 + x2*y0 + x3*y1)j
                        + (x0*y3 + x1*y2 - x2*y1 + x3*y0)k

\param[out]     p_result A pointer that points to the location the result 
                will be stored
\param[in]      p_quat_l A pointer pointing to the lefthand coefficient
\param[in]      p_quat_r A pointer pointing to the righthand coefficient

\retval         #EXIT_FAILURE if \p p_result if NULL,
                or if \p p_quat_l is NULL,
                or if \p p_quat_r is NULL.
                #EXIT_SUCCESS else.

\note           The Hamilton product is not commutative so the order of 
                coefficients matters.
********************************************************************************/
int8_t quat_mult(float * const p_result, float const * const p_quat_l, float const * const p_quat_r);

/****************************************************************************//**
\brief          Creates a quaternion from an axis angle representation.

\details        (axis, angle) = (x y z, theta)
                The axis is a unit vector representing the axis of rotation.
                
\details        w = cos(theta/2)
                x = x*sin(theta/2)
                y = y*sin(theta/2)
                z = z*sin(theta/2)

\details        q = (w x y z)

\param[out]     p_result A pointer that points to the location the resulting 
                quaternion will be stored. 
\param[in]      p_vect A pointer that points to the vector that represents 
                the axis of rotation.
\param[in]      theta The angle of rotation represented in degrees.

\retval         #EXIT_FAILURE if \p p_result if NULL, 
                or if \p p_vect is NULL.
                #EXIT_SUCCESS else.

\note           The vector \p p_vect does not have to be a unit vector. It 
                will be scaled appropriately inside the function.          
********************************************************************************/
int8_t quat_from_axis_angle (float * const p_result, float const * const p_vect, float theta);

/****************************************************************************//**
\brief          This function creates a conjugate of a quaternion.

\details        The vector part of the quaternion will be returned with a 
                negative sign. Note that the conjugate of a unit quaternion 
                is equal to its inverse.

\param[out]     p_result A pointer that points to the location the resulting 
                quaternion will be stored. 
\param[in]      p_quat The quaternion from which the conjugate will be created.

\retval         #EXIT_FAILURE if \p p_result is NULL, 
                or if \p p_quat is NULL.
                #EXIT_SUCCESS else.
                
********************************************************************************/
int8_t quat_conjugate (float * const p_result, float const * const p_quat);

/****************************************************************************//**
\brief          This function does a full rotation of a vector using a 
                quaternion.

\details        The function does a rotation using the following equation
                v' = qvq^-1.

\param[out]     p_result A pointer that points to the location the resulting 
                quaternion will be stored. 
\param[in]      p_quat The quaternion that will be used for the rotation.
\param[in]      p_vect The vector that will be rotated.

\retval         #EXIT_FAILURE if \p p_result is NULL, 
                or if \p p_quat is NULL, 
                or if \p p_vect is NULL.
                #EXIT_SUCCESS else.

********************************************************************************/
int8_t quat_vect_mult(float * const p_result, float const * const p_quat, float const * const p_vect);

/****************************************************************************//**
\brief          This function converts a quaternion to an axis angle 
                representation.

\details        q = (w x y z)

\details        axis = (x y z)
                angle = acos(w) * 2

\param[out]     p_vect A pointer that points to the location where the rotation 
                axis vector will be stored.
\param[out]     p_theta A pointer to the location the angle of rotation will 
                be stored.
\param[in]      p_quat A pointer to the location where the input quaternion 
                is stored.  

\retval         #EXIT_FAILURE if \p p_vect is NULL, 
                or if \p p_theta is NULL, 
                or if \p p_quat is NULL.
                #EXIT_SUCCESS else.

\note           The rotation axis vector will be normalized before stored 
                in \p p_vect.          
********************************************************************************/
int8_t quat_to_axis_angle(float * const p_vect, float const * const p_theta, float const * const p_quat);
