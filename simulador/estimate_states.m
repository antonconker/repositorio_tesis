% estimate_states_model_inversion
%   - estimate the MAV states using gyros, accels, pressure sensors, and
%   GPS.
%



function xhat = estimate_states(uu, P)

    % rename inputs
   y_gyro_x      = uu(1);
   y_gyro_y      = uu(2);
   y_gyro_z      = uu(3);
   y_accel_x     = uu(4);
   y_accel_y     = uu(5);
   y_accel_z     = uu(6);
   y_static_pres = uu(7);
   y_diff_pres   = uu(8);
   y_gps_n       = uu(9);
   y_gps_e       = uu(10);
%   y_gps_h       = uu(11);
   y_gps_Vg      = uu(12);
   y_gps_course  = uu(13);
   t             = uu(14);
   
    % define persistent variables
    persistent alpha  % constant for low pass filter - only compute once
    persistent lpf_gyro_x     % low pass filter of x-gyro
    persistent lpf_gyro_y     % low pass filter of y-gyro
    persistent lpf_gyro_z     % low pass filter of z-gyro
    persistent lpf_static     % low pass filter of static pressure sensor
    persistent lpf_diff       % low pass filter of diff pressure sensor
    persistent lpf_accel_x    % low pass filter of x-accelerometer
    persistent lpf_accel_y    % low pass filter of y-accelerometer
    persistent lpf_accel_z    % low pass filter of z-accelerometer
    persistent lpf_gps_n      % low pass filter GPS North
    persistent lpf_gps_e      % low pass filter GPS East
    persistent lpf_gps_chi    % low pass filter GPS Course
    persistent lpf_gps_Vg     % low pass filter GPS Groundspeed
    
    % initialize persistent variables
    lpf_a = 50;
    if t==0,
        alpha = exp(-lpf_a*P.Ts);
        lpf_gyro_x   = 0;
        lpf_gyro_y   = 0;
        lpf_gyro_z   = 0;
        lpf_static   = P.rho*P.gravity*(-P.pd0);
        lpf_diff     = 1/2*P.rho*P.Va^2;
        lpf_accel_x  = 0;
        lpf_accel_y  = 0;
        lpf_accel_z  = 0;      
        lpf_gps_n    = P.pn0;
        lpf_gps_e    = P.pe0;
        lpf_gps_chi  = P.psi0;
        lpf_gps_Vg   = P.Va0;
    end
    
    %------------------------------------------------------------------
    % low pass filter gyros to estimate angular rates
    lpf_gyro_x = alpha*lpf_gyro_x + (1-alpha)*y_gyro_x;
    lpf_gyro_y = alpha*lpf_gyro_y + (1-alpha)*y_gyro_y;
    lpf_gyro_z = alpha*lpf_gyro_z + (1-alpha)*y_gyro_z;
    phat = lpf_gyro_x;
    qhat = lpf_gyro_y;
    rhat = lpf_gyro_z;
    
    %------------------------------------------------------------------
    % low pass filter static pressure sensor and invert to estimate
    % altitude
    lpf_static = alpha*lpf_static + (1-alpha)*y_static_pres;
    hhat = lpf_static/P.rho/P.gravity;
    
    % low pass filter diff pressure sensor and invert to estimate Va
    lpf_diff = alpha*lpf_diff + (1-alpha)*y_diff_pres;
    Vahat = sqrt(2/P.rho*lpf_diff);
    
    %------------------------------------------------------------------
    % low pass filter accelerometers
    lpf_accel_x = alpha*lpf_accel_x + (1-alpha)*y_accel_x;
    lpf_accel_y = alpha*lpf_accel_y + (1-alpha)*y_accel_y;
    lpf_accel_z = alpha*lpf_accel_z + (1-alpha)*y_accel_z;
    
    % invert accels to estimate phi and theta
    if abs(lpf_accel_z)<.001,
        phihat_accel = 0;
    else
        phihat_accel = atan(lpf_accel_y/lpf_accel_z);
    end
    thetahat_accel = asin(lpf_accel_x/P.gravity);
    phihat = phihat_accel;
    thetahat = thetahat_accel;
        
    %------------------------------------------------------------------
    % low pass filter GPS
    lpf_gps_n   = alpha*lpf_gps_n   + (1-alpha)*y_gps_n;
    lpf_gps_e   = alpha*lpf_gps_e   + (1-alpha)*y_gps_e;
    lpf_gps_chi = alpha*lpf_gps_chi + (1-alpha)*y_gps_course;
    lpf_gps_Vg  = alpha*lpf_gps_Vg  + (1-alpha)*y_gps_Vg;
     
    pnhat    = lpf_gps_n;
    pehat    = lpf_gps_e;
    Vghat    = lpf_gps_Vg;
    chihat   = lpf_gps_chi; 
    wnhat    = 0;
    wehat    = 0;
    psihat   = lpf_gps_chi;

    % not estimating these states 
    alphahat = 0;
    betahat  = 0;
    bxhat    = 0;
    byhat    = 0;
    bzhat    = 0;
    
      xhat = [...
        pnhat;...
        pehat;...
        hhat;...
        Vahat;...
        alphahat;...
        betahat;...
        phihat;...
        thetahat;...
        chihat;...
        phat;...
        qhat;...
        rhat;...
        Vghat;...
        wnhat;...
        wehat;...
        psihat;...
        bxhat;...
        byhat;...
        bzhat;...
        ];
end
