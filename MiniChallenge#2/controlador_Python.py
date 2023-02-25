#PI controller c++

#//Global variables
#double e=0.0,e_1=0.0,u=0.0,u_1=0.0;
#double kp,ti,q0,q1;
#
#void PID_Initialize( void ){
#    double T = ____;//			sample rate: T < theta/4
#    //Complete here with the identified parameters of the plant
#    double k = ____;
#    double tao = ____;
#    double theta = ____;        
#    
#    //Calculate the coefficients for the discrete PID controller
#    kp = (0.9 * tao) / (k * theta);
#    ti = 3.33 * theta;
#    q0 = kp + ((kp * T)/(2.0 * ti));
#    q1 = ((kp * T)/(2.0 * ti)) - kp;
#}
#double PID_Discrete( double yM ){
#    double R = _____;//			set-point
#    
#    e = R - yM;//				calculate the error
#    u = u_1 + q0*e + q1*e_1;//		discrete PID controller
#    
#    //Saturate the controller with upper and lower limits
#    if(u >= _____)// 			maximum duty cycle value        
#        u = _____;
#    if(u <= _____)//			minimum duty cycle value
#        u = _____;    
#    //Update the values for the next iteration
#    e_1 = e;
#    u_1 = u;
#
#    return u;
#}

#PI CONTROLLER PYTHON
# Global variables
e = 0.0
e_1 = 0.0
u = 0.0
u_1 = 0.0
kp = 0.0
ti = 0.0
q0 = 0.0
q1 = 0.0

def PID_Initialize(T, k, tao, theta):
    global kp, ti, q0, q1
    
    # Calculate the coefficients for the discrete PID controller
    kp = (0.9 * tao) / (k * theta)
    ti = 3.33 * theta
    q0 = kp + ((kp * T)/(2.0 * ti))
    q1 = ((kp * T)/(2.0 * ti)) - kp

def PID_Discrete(yM, max_val, min_val, set_point):
    global e, e_1, u, u_1
    
    # Calculate the error
    e = set_point - yM
    
    # Calculate the discrete PID controller
    u = u_1 + q0 * e + q1 * e_1
    
    # Saturate the controller with upper and lower limits
    if u >= max_val:
        u = max_val
    elif u <= min_val:
        u = min_val
    
    # Update the values for the next iteration
    e_1 = e
    u_1 = u
    
    return u

  
  
