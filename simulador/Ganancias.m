% ganancias para el roll
    % obtenemos la funcion de transferencia de delta aleron a phi
    [num,den]=tfdata(T_phi_delta_a,'v');
    a_phi2 = num(3);
    a_phi1 = den(2);
    % deflexion maxima del aleron
    delta_a_max = 45*pi/180;
    
    %roll deseado cuando delta_a_max es alcanzado
    phi_max = 15*pi/180;
    
    zeta_roll = 0.7;
    wn_roll = sqrt(a_phi2*delta_a_max/phi_max);
    %wn_roll = sqrt(a_phi2*delta_a_max*sqrt(1-zeta_roll^2)/phi_max);
    
   
    %ganancias basadas en zeta y wn
    P.roll_kp = wn_roll^2/a_phi2;
    P.roll_kd =(2*zeta_roll*wn_roll - a_phi1)/a_phi2; %*1.1
    P.roll_ki = 1;
        
    
% ganancias para el lazo cerrado del curzo
   zeta_course = 0.8;
   wn_course = wn_roll/30;
   P.course_kp = 2*zeta_course*wn_course*P.Va/P.gravity;
   P.course_ki = wn_course^2*P.Va/P.gravity;
   P.course_kd = 0;
  

%ganancias para el pitch 
   [num,den]=tfdata(T_theta_delta_e,'v');
   a_theta1 = den(2);
   a_theta2 = den(3);
   a_theta3 = num(3);
   % maximo elevador
   delta_e_max = 45*pi/180;
   % theta deseada cuando max elevador es alcanzado
   theta_max = 15*pi/180;
   % 
   zeta_pitch = 0.9;
   %wn_pitch = sqrt(abs(a_theta3)*delta_e_max*sqrt(1-zeta_pitch^2)/theta_max);
   wn_pitch = sqrt(a_theta2 + abs(a_theta3)*(delta_e_max)/theta_max);
  
   P.pitch_kp = (wn_pitch^2-a_theta2)/a_theta3;
   P.pitch_kd = (2*zeta_pitch*wn_pitch - a_theta1)/a_theta3;
   P.pitch_ki = 0.0;
   P.K_theta_DC = P.pitch_kp*a_theta3/(a_theta2+P.pitch_kp*a_theta3);

% ganancias para el lazo de altura
   zeta_altitude = 0.7;
   wn_altitude = wn_pitch/40;
   P.altitude_kp = 1.1*2*zeta_altitude*wn_altitude/(P.K_theta_DC*P.Va);
   P.altitude_ki = wn_altitude^2/(P.K_theta_DC*P.Va);
   P.altitude_kd = 0;
 

% usando pitch se mantiene la vel
   [num,den]=tfdata(T_Va_theta,'v');
   a_V1 = den(2);
   zeta_airspeed_pitch = 0.707;
   wn_airspeed_pitch = wn_pitch/10;
   P.airspeed_pitch_kp = (a_V1-2*zeta_airspeed_pitch*wn_airspeed_pitch)/(P.K_theta_DC*P.gravity);
   P.airspeed_pitch_ki = -wn_airspeed_pitch^2/(P.K_theta_DC*P.gravity);
   P.airspeed_pitch_kd = 0;
 
% control de vel
   [num,den]=tfdata(T_Va_delta_t,'v');
   a_Vt1 = den(2);
   a_Vt2 = num(2);
   zeta_airspeed_throttle = 0.707;
%    wn_airspeed_throttle = 5;   % debe ser menor de 5
   wn_airspeed_throttle = 3;
   P.airspeed_throttle_kp = (2*zeta_airspeed_throttle*wn_airspeed_throttle-a_Vt1)/a_Vt2;
   P.airspeed_throttle_ki = wn_airspeed_throttle^2/a_Vt2;
   P.airspeed_throttle_kd = 0;
   P.airspeed_throttle_integrator_gain = a_Vt1/a_Vt2/P.airspeed_throttle_ki;
 



