/**
 * @file controller.h
 * @brief P, PI and PID controllers implementation.
 * @note This library does not use floating points. As such, one should use fixed-point
 *       arithmetic if fractional gains are to be used. 
 */

#ifndef DEF_CONTROLLER_H
#define DEF_CONTROLLER_H

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
    unsigned int step;  /**< Time elapsed between two iterations of the control loop. */
}pi_controller; 

/**
 * @brief Represents a proportional-integral-derivative (PID) controller.
 */
typedef struct {
    unsigned int p;     /**< Proportional gain value. */
    unsigned int i;     /**< Integral gain value. */
    unsigned int d;     /**< Derivative gain value. */
    unsigned int step;  /**< Time elapsed between two iterations of the control loop. */ 
}pid_controller; 

/**
 * @brief Control loop for P controllers.
 * @details This function updates the setpoint of the controlled variable using 
 *          a proportional control scheme. 
 * @param[in]   c       Pointer to the structure representing the parameters of the P controller. 
 * @param[in]   error   Difference between the last setpoint and the measured data.
 * @returns New setpoint value. 
 */
static inline unsigned int p_control_loop(const p_controller_s *c, unsigned int error);
/**
 * @brief Control loop for PI controllers.
 * @details This function updates the setpoint of the controlled variable using 
 *          a proportional-integral control scheme. 
 * @param[in]   c       Pointer to the structure representing the parameters of the PI controller. 
 * @param[in]   error   Difference between the last setpoint and the measured data.
 * @returns New setpoint value. 
 */
static inline unsigned int pi_control_loop(const pi_controller_s *c, unsigned int error);

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
static inline unsigned int pid_control_loop(const pid_controller_s *c, unsigned int error);



static inline unsigned int p_control_loop(const p_controller_s *c, unsigned int error) {
    
    return (c->p) * error;  
}

static inline unsigned int pi_control_loop(const pi_controller_s *c, unsigned int error) {
    
    static unsigned int integral = 0; 
    
    integral += error * (c->step); 
    return (c->p) * error + (c->i) * integral; 
}

static inline unsigned int pid_control_loop(const pid_controller_s *c, unsigned int error) {

    static unsigned int integral   = 0; 
    static unsigned int prev_error = 0;

    unsigned int derivative = 0; 

    integral  += error * (c->step);
    derivative = (error - prev_error) / (c->step); 
    
    return (c->p) * error + (c->i) * integral + (c->d) * derivative; 
}

#endif  /* DEF_CONTROLLER_H */

