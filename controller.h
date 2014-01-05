/**
 * @file controller.h
 * @brief P, PI and PID controllers implementation.
 * @note This library does not use floating points. As such, one should use fixed-point
 *       arithmetic if fractional gains are to be used. 
 */

#ifndef DEF_CONTROLLER_H
#define DEF_CONTROLLER_H

/*
 * CONFIGURATION MACROS 
 */ 

/** @brief If set, the controller verifies that the computed setpoint is in the admissible range of values. */
#define SATURATION
/** @brief If set, anti-windup is activated for PI and PID controllers. */
#define ANTI_INTEGRATOR_WINDUP

/**  
 * @brief Represents a proportional (P) controller. 
 */
typedef struct {
    unsigned int p;     /**< Proportional gain value. */ 
}p_controller_s; 

/** 
 * @brief Represents a proportional-integral (PI) controller. 
 */
typedef struct {
    unsigned int p;     /**< Proportional gain value. */ 
    unsigned int i;     /**< Integral gain value. */
    unsigned int dt;    /**< Time elapsed between two iterations of the control loop. */
    
    #ifdef SATURATION
    int sat_min;        /**< Lower saturation bound (minimum setpoint value). */ 
    int sat_max;        /**< Upper saturation bound (maximum setpoint value). */
    #endif  /* SATURATION */
    
    unsigned int _dti;  /**< Integral gain multiplied by the time step value. */
}pi_controller; 

#ifdef SATURATION

/**
 * @brief Saturates the given @p value according to the given bounds.
 *
 */
static inline int _saturation(int value, int lower_bound, int upper_bound); 

#ifdef ANTI_INTEGRATOR_WINDUP

/**
 * @brief Saturates the given @p value according to the given bounds.
 * @details Similarly to @ref _saturation, this function saturates the given @p value. 
 *          It sets the @p is_saturated flag if the value was out of bounds and had to be
 *          modified. 
 * @param[in]   value   Value to compare to the @p min and @p max bounds.
 * @param[in]   min     Lower admissible bound for the given @p value. 
 * @param[in]   max     Upper admissible bound for the given @p value. 
 * @param[in]   is_saturated    Boolean flag indicating if the value was modified. 
 * @returns The saturated value of the input parameter. 
 */
static inline int _integrator_saturation(int value, int min, int max, int *is_saturated); 

#endif  /* ANTI_INTEGRATOR_WINDUP */

#endif  /* SATURATION */

/**
 * @brief Represents a proportional-integral-derivative (PID) controller.
 */
typedef struct {
    unsigned int p;     /**< Proportional gain value. */
    unsigned int i;     /**< Integral gain value. */
    unsigned int d;     /**< Derivative gain value. */
    unsigned int dt;    /**< Time elapsed between two iterations of the control loop. */ 
}pid_controller; 

/**
 * @brief Initializes a PI controller with the given parameters. 
 *
 */
static inline void init_pi_controller(pi_controller *c,
                                      unsigned int kp, 
                                      unsigned int ki,
                                      unsigned int dt, 
                                      int sat_min, 
                                      int sat_max);

/**
 * @brief Control loop for P controllers.
 * @details This function updates the setpoint of the controlled variable using 
 *          a proportional control scheme. 
 * @param[in]   c       Pointer to the structure representing the parameters of the P controller. 
 * @param[in]   error   Difference between the last setpoint and the measured data.
 * @returns New setpoint value. 
 */
static inline unsigned int p_control_loop(const p_controller_s *c, int error);
/**
 * @brief Control loop for PI controllers.
 * @details This function updates the setpoint of the controlled variable using 
 *          a proportional-integral control scheme. 
 * @param[in]   c       Pointer to the structure representing the parameters of the PI controller. 
 * @param[in]   error   Difference between the last setpoint and the measured data.
 * @returns New setpoint value. 
 */
static inline int pi_control_loop(const pi_controller_s *c, int error);

/**
 * @brief Control loop for P controllers.
 * @details This function updates the setpoint of the controlled variable using 
 *          a proportional control scheme. 
 * @param[in]   c       Pointer to the structure representing the parameters of the PID controller. 
 * @param[in]   error   Difference between the last setpoint and the measured data.
 * @returns New setpoint value.
 * @note As the computation performs a division by the time step of the controller, beware
 *       of imprecisions regarding the derivative term for large time step values. 
 */
static inline int pid_control_loop(const pid_controller_s *c, int error);

static inline 

static inline int p_control_loop(const p_controller_s *c, int error) {
    
    return (c->p) * error;  
}

static inline int pi_control_loop(const pi_controller_s *c, int error) {
    
    static int integral = 0; 
    
    integral += (c->_dti) * error; 
    
    #ifdef ANTI_INTEGRATOR_WINDUP

    #endif  /* ANTI_INTEGRATOR_WINDUP */

    return (c->p) * error + integral; 
}

static inline int pid_control_loop(const pid_controller_s *c, int error) {

    static int integral   = 0; 
    static int prev_error = 0;

    int derivative = 0; 

    integral  += error * (c->dt);
    derivative = (error - prev_error) / (c->dt); 
    
    return (c->p) * error + (c->i) * integral + (c->d) * derivative; 
}

#ifdef SATURATION

static inline int saturation(int value, int min, int max) {
    
    if(value > max) {
        return max; 
    }

    if(value < min) {
        return min; 
    }

    return value; 
}

#ifdef ANTI_INTEGRATOR_WINDUP 

static inline int integrator_saturation(int value, int min, int max, int *is_saturated) {

    if(value > max) {
        is_saturated = 1; 
        return max; 
    }

    if(value < min) {
        is_saturated = 1; 
        return min; 
    }

    is_saturated = 0; 
    return value; 
}

#endif  /* ANTI_INTEGRATOR_WINDUP */

#endif  /* SATURATION */

#endif  /* DEF_CONTROLLER_H */
