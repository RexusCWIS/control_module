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

#ifdef SATURATION
/** @brief If set, anti-windup is activated for PI and PID controllers. */
#define ANTI_INTEGRATOR_WINDUP
#endif  /* SATURATION */

/*
 * FUNCTIONS
 */

/**  
 * @brief Represents a proportional (P) controller. 
 */
typedef struct {
    uint32_t p;     /**< Proportional gain value. */
} p_controller_s;

/** 
 * @brief Represents a proportional-integral (PI) controller. 
 */
typedef struct {
    /* Public parameters */
    uint32_t p;     /**< Proportional gain value. */
    uint32_t dti;   /**< Integral gain multiplied by the time step value. */
    
    #ifdef SATURATION
    int32_t sat_min;        /**< Lower saturation bound (minimum setpoint value). */
    int32_t sat_max;        /**< Upper saturation bound (maximum setpoint value). */
    #endif  /* SATURATION */

    /* Private parameters */
    int32_t _integral;      /**< Integrated error. */

    #ifdef ANTI_INTEGRATOR_WINDUP
    int32_t _is_saturated;  /**< Status of the integrator (-1 or 1 means saturation) */
    #endif  /* ANTI_INTEGRATOR_WINDUP */
} pi_controller_s;

#ifdef SATURATION

/**
 * @brief Saturates the given @p value according to the given bounds.
 * @param[in]   value   Value to compare to the @p min and @p max bounds.
 * @param[in]   min     Lower admissible bound for the given @p value. 
 * @param[in]   max     Upper admissible bound for the given @p value. 
 */
static inline int32_t _saturation(int32_t value, int32_t lower_bound, int32_t upper_bound);

#ifdef ANTI_INTEGRATOR_WINDUP

/**
 * @brief Saturates the given @p value according to the given bounds.
 * @details Similarly to @ref _saturation, this function saturates the given @p value. 
 *          It sets the @p is_saturated flag if the value was out of bounds and had to be
 *          modified. 
 * @param[in]   value   Value to compare to the @p min and @p max bounds.
 * @param[in]   min     Lower admissible bound for the given @p value. 
 * @param[in]   max     Upper admissible bound for the given @p value. 
 * @param[in]   is_saturated    Flag indicating if saturation is active. 
 * @returns The saturated value of the input parameter. The @p is_saturated flag is set 
 *          to 1 if @p value was greater than @p max or -1 if it was lower than @p min. 
 *          It is set to 0 if the integrator is not saturating. 
 */
static inline int32_t _integrator_saturation(int32_t value, int32_t min, int32_t max, int32_t *is_saturated);

#endif  /* ANTI_INTEGRATOR_WINDUP */

#endif  /* SATURATION */

/**
 * @brief Represents a proportional-integral-derivative (PID) controller.
 */
typedef struct {
    uint32_t p;     /**< Proportional gain value. */
    uint32_t i;     /**< Integral gain value. */
    uint32_t d;     /**< Derivative gain value. */
    uint32_t dt;    /**< Time elapsed between two iterations of the control loop. */
} pid_controller_s;

/**
 * @brief Initializes a PI controller with the given parameters.
 * @param[in]   c   Pointer to the PI controller to initialize. 
 * @param[in]   kp  Proportional gain of the controller. 
 * @param[in]   ki  Integral gain of the controller. 
 * @param[in]   dt  Period of the control loop. 
 * @param[in]   sat_min Lower saturation bound. 
 * @param[in]   sat_max Upper saturation bound.
 * @note The @sat_min and @sat_max parameters may not be taken into account if 
 *       the @ref SATURATION configuration macro is not defined. 
 */
static inline void init_pi_controller(pi_controller_s *c,
                                      uint32_t kp,
                                      uint32_t dti,
                                      int32_t sat_min,
                                      int32_t sat_max);

/**
 * @brief Control loop for P controllers.
 * @details This function updates the setpoint of the controlled variable using 
 *          a proportional control scheme. 
 * @param[in]   c       Pointer to the structure representing the parameters of the P controller. 
 * @param[in]   error   Difference between the last setpoint and the measured data.
 * @returns New setpoint value. 
 */
static inline uint32_t p_control_loop(const p_controller_s *c, int32_t error);
/**
 * @brief Control loop for PI controllers.
 * @details This function updates the setpoint of the controlled variable using 
 *          a proportional-integral control scheme. 
 * @param[in]   c       Pointer to the structure representing the parameters of the PI controller. 
 * @param[in]   error   Difference between the last setpoint and the measured data.
 * @returns New setpoint value. 
 */
static inline int32_t pi_control_loop(pi_controller_s *c, int32_t error);

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
static inline int32_t pid_control_loop(pid_controller_s *c, int32_t error);


static inline void init_pi_controller(pi_controller_s *c,
                                      uint32_t kp,
                                      uint32_t dti,
                                      int32_t sat_min,
                                      int32_t sat_max) {

    c->p  = kp; 
    c->dti = dti;
    
    c->_integral = 0;

    #ifdef SATURATION
    c->sat_min = sat_min;
    c->sat_max = sat_max; 
    #endif  /* SATURATION */

    #ifdef ANTI_INTEGRATOR_WINDUP
    c->_is_saturated = 0; 
    #endif  /* ANTI_INTEGRATOR_WINDUP */
}


static inline int32_t p_control_loop(const p_controller_s *c, int32_t error) {
    
    return (c->p) * error;  
}

static inline int32_t pi_control_loop(pi_controller_s *c, int32_t error) {
    
    int32_t setpoint;

    /* Update the integrated error only if the integrator is not saturated. */
    if((error * c->_is_saturated) <= 0) {
        c->_integral += (c->dti) * error; 
        
        #ifdef ANTI_INTEGRATOR_WINDUP
        c->_integral = _integrator_saturation(c->_integral, c->sat_min, c->sat_max, &c->_is_saturated);
        #endif  /* ANTI_INTEGRATOR_WINDUP */
    }
    
    setpoint = (c->p) * error + c->_integral;

    #ifdef SATURATION
    setpoint = _saturation(setpoint, c->sat_min, c->sat_max);
    #endif  /* SATURATION */

    return setpoint; 
}

static inline int32_t pid_control_loop(pid_controller_s *c, int32_t error) {

    static int32_t integral   = 0;
    static int32_t prev_error = 0;

    int32_t derivative = 0;

    integral  += error * (c->dt);
    derivative = (error - prev_error) / (c->dt); 
    
    return (c->p) * error + (c->i) * integral + (c->d) * derivative; 
}

#ifdef SATURATION

static inline int32_t _saturation(int32_t value, int32_t min, int32_t max) {
    
    if(value > max) {
        return max; 
    }

    if(value < min) {
        return min; 
    }

    return value; 
}

#endif  /* SATURATION */

#ifdef ANTI_INTEGRATOR_WINDUP 

static inline int32_t _integrator_saturation(int32_t value, int32_t min, int32_t max, int32_t *is_saturated) {

    if(value > max) {
        *is_saturated = 1;
        return max;
    }

    if(value < min) {
        *is_saturated = -1;
        return min;
    }

    *is_saturated = 0;
    return value;
}

#endif  /* ANTI_INTEGRATOR_WINDUP */

#endif  /* DEF_CONTROLLER_H */

