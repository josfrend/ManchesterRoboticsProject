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

#PI controller python

e=0.0, e_1=0.0, u=0.0, u_1=0.0          
kp = 0.011509                           #Kp parameter
ti = 0.015025                           #Ti parameter
q0,q1                                   

def PI_Initialize():
  t = 0.01                              #Sample rate
  
  q0 = kp + ((kp * T)/(2.0 * ti));
  q1 = ((kp * T)/(2.0 * ti)) - kp;
  
  
