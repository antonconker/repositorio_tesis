function y = autopilot(uu,P)
%
% autopiloto para UAV
% 

    % entradas
    NN = 0;
%    pn       = uu(1+NN);  % N
%    pe       = uu(2+NN);  % E
    h        = uu(3+NN);  % alttura
    Va       = uu(4+NN);  % Va
%    alpha    = uu(5+NN);  % angulo de ataque
%    beta     = uu(6+NN);  % ángulo de ladeo
    phi      = uu(7+NN);  % alabeo
    theta    = uu(8+NN);  % cabeceo
    chi      = uu(9+NN);  % ángulo de curso
    p        = uu(10+NN); % p
    q        = uu(11+NN); % q
    r        = uu(12+NN); % r
%    Vg       = uu(13+NN); % velocidad en tierra
%    wn       = uu(14+NN); % viento norte
%    we       = uu(15+NN); % viento este
%    psi      = uu(16+NN); % guiñada
%    bx       = uu(17+NN); % x-gyro bias
%    by       = uu(18+NN); % y-gyro bias
%    bz       = uu(19+NN); % z-gyro bias
    NN = NN+19;
    Va_c     = uu(1+NN);  % Va deseada (m/s)
    h_c      = uu(2+NN);  % altitud deseada (m)
    %theta_c=uu(2+NN);   % cabeceo deseado (rad)
    chi_c    = uu(3+NN);  % curso deseado (rad)
    %roll_c=uu(4+NN);  %roll deseado (rad)
    NN = NN+3;
    t        = uu(1+NN);   % tiempo
    
    
           [delta, x_command] = autopilot_uavbook(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P);
        
    y = [delta; x_command];
end
    
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autopilot versions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [delta, x_command] = autopilot_uavbook(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P)

    %----------------------------------------------------------
    % control plano lateral 
    if t==0,
        delta_r = 0;%coordinated_turn_hold(beta, 1, P);
        phi_c   = course_hold(chi_c, chi, r, 1, P);
          
    else
        phi_c   = course_hold(chi_c, chi, r, 0, P);
        delta_r = 0;%coordinated_turn_hold(beta, 0, P);
        %delta_a = roll_hold(phi_c, phi, p,0, P);     

    end
    delta_a = roll_hold(phi_c, phi, p,1, P);     
  
    
    %----------------------------------------------------------
    % Conrtrol longitudinal  
 
    persistent altitude_state;
    persistent initialize_integrator;
    % inicializamos variables persistentes
    if t==0,
        if h<=P.altitude_take_off_zone,     
            altitude_state = 1;
        elseif h<=h_c-P.altitude_hold_zone, 
            altitude_state = 2;
        elseif h>=h_c+P.altitude_hold_zone, 
            altitude_state = 3;
        else
            altitude_state = 4;
        end
        initialize_integrator = 1;
    end
    
    % maquina de estados
    switch altitude_state,
        case 1,  % zona de despegue, empuje maximo y regular theta deseada
             delta_t = 1;
            theta_c = 30*pi/180;
            if h>=P.altitude_take_off_zone,
                altitude_state = 2;
                initialize_integrator = 1;
            else
                initialize_integrator = 0;
            end
        case 2,  % zona de ascenso empuje al maximo y 
                %regulamos la vel del viento comandando el angulo de cabeceo
            delta_t = 1;
            theta_c = airspeed_with_pitch_hold(Va_c, Va, initialize_integrator, P);
            if h>=h_c-P.altitude_hold_zone,
                altitude_state = 4;
                initialize_integrator = 1;
            elseif h<=P.altitude_take_off_zone,
                altitude_state = 1;
                initialize_integrator = 1;
            else
                initialize_integrator = 0;
            end
             
        case 3, % zona de descenso, empuje cero y regulamos la vel del viento con el cabeceo
            delta_t = 0;
            theta_c = airspeed_with_pitch_hold(Va_c, Va, initialize_integrator, P);
            if h<=h_c+P.altitude_hold_zone,
                altitude_state = 4;
                initialize_integrator = 1;
            else
                initialize_integrator = 0;
            end
        case 4, % zona de altura 
            %regulamos la altura comandando el pitch, cabeceo
            %regulamos la velocidad comandando el empuje del motor
            delta_t = airspeed_with_throttle_hold(Va_c, Va, initialize_integrator, P);
            theta_c = altitude_hold(h_c, h, initialize_integrator, P);
            if h<=h_c-P.altitude_hold_zone,
                altitude_state = 2;
                initialize_integrator = 1;
            elseif h>=h_c+P.altitude_hold_zone,
                altitude_state = 3;
                initialize_integrator = 1;
            else
                initialize_integrator = 0;
            end     
    end
    if t==0,
        delta_e = pitch_hold(theta_c, theta, q, 1, P);

    else
        delta_e = pitch_hold(theta_c, theta, q,0, P);
    end
 
    % saturación
    delta_t = sat(delta_t,1,0);
 
    
    %----------------------------------------------------------
    % salidas
    
    % salidas de control, elevador, aleron, timon, motor
    delta = [delta_e; delta_a; delta_r; delta_t];
    % estados deseados
    x_command = [...
        0;...                    % pn
        0;...                    % pe
        h_c;...                  % h
        Va_c;...                 % Va
        0;...                    % alpha
        0;...                    % beta
        phi_c;...                % phi
        %theta_c*P.K_theta_DC;... % theta
        theta_c;
        chi_c;...                % chi
        0;...                    % p
        0;...                    % q
        0;...                    % r
        ];
            
     
    end

%funciones de control
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% course_hold
%   - regula la guiñada usando el roll
%   - Entrega el roll deseado en base al error en la guiñada
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function phi_c = course_hold(chi_c, chi, r, flag, P)
  persistent integrator;
  persistent error_d1; %error previo
  % inicializamos 
  if flag==1,
      integrator = 0; 
      error_d1   = 0; 
  end
 
  % calculo del error
  error = chi_c - chi;
  
  % integrador
  if abs(error)>15*pi/180,
      integrator = 0;
  else
      integrator = integrator + (P.Ts/2)*(error + error_d1); % regla trapezoidal
  end
  
  % termino proporcional
  up = P.course_kp * error;
  
  % termino integral
  ui = P.course_ki * integrator;
  
  % termino derivativo
  ud = -P.course_kd*r;
  
  
  % control PID
  phi_c = sat(up + ui + ud, 45*pi/180, -45*pi/180);
  
  % implementando anti-windup
  if P.course_ki~=0,
    phi_c_unsat = up+ui+ud;
    k_antiwindup = P.Ts/P.course_ki;
    integrator = integrator + k_antiwindup*(phi_c-phi_c_unsat);
  end

  % actualizando
  error_d1 = error;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% roll_hold
%   - regula el alabeo usando el aleron
%   - regresa el alerón una vez recibe el roll deseado
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_a = roll_hold(phi_c, phi, p, flag, P)
  persistent integrator;
  persistent error_d1;
  % initialize persistent variables at beginning of simulation
  if flag==1,
      integrator = 0; 
      error_d1   = 0; % error at last sample (d1-delayed by one sample)
  end
 
  % calculo del error de yaw
  error = phi_c - phi;
  
  % calculo de la integral
  integrator = integrator + (P.Ts/2)*(error + error_d1); 
  
  % proporcional 
  up = P.roll_kp * error*2;
  
  % integral
  ui = P.roll_ki * integrator;
  
  % derivativo
  ud = -P.roll_kd*p;
  
  
  % PID
  delta_a = sat(up + ui + ud, 45*pi/180, -45*pi/180);
  
  % nti-windup
  if P.roll_ki~=0,
    delta_a_unsat = up + ui + ud;
    k_antiwindup=P.Ts/P.roll_ki;
    integrator = integrator + k_antiwindup*(delta_a - delta_a_unsat);
  end
  
  % actualizamos las variables
  error_d1 = error;

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% airspeed_with_pitch_hold
%   - controla la velocidad usando el ángulo de cabeceo
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function theta_c = airspeed_with_pitch_hold(Va_c, Va, flag, P)
  persistent integrator;
  persistent differentiator;
  persistent differentiator_d1;
  persistent error_d1;
  % inicializamos 
  if flag==1,
      integrator = 0; 
      differentiator = 0;
      differentiator_d1 = 0;
      error_d1   = 0; 
  end
 
  % calculo del error error
  error = Va_c - Va;
  
  % calculo de la integral
  integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule
  
  % calculo del la derivada (derivada sucia o aproximada)
  differentiator = (2*P.tau-P.Ts)/(2*P.tau+P.Ts)*differentiator_d1...
      + (2/(2*P.tau+P.Ts))*(error - error_d1);
  
  % termino proporcional
  up = P.airspeed_pitch_kp * error;
  
  % termino de la intergral
  ui = P.airspeed_pitch_ki * integrator;
  
  % termino derivativo
  ud = P.airspeed_pitch_kd * differentiator;
  
  
  % PID
  theta_c = sat(up + ui + ud, 30*pi/180, -30*pi/180);
  
  % antiwindup
  if P.airspeed_pitch_ki~=0,
    theta_c_unsat = up + ui + ud;
    k_antiwindup = P.Ts/P.airspeed_pitch_ki;
    integrator = integrator + k_antiwindup*(theta_c-theta_c_unsat);
  end

  % actualizamos
  error_d1 = error;
  differentiator_d1 = differentiator;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% airspeed_with_throttle_hold
%   - controla la velocidad usando el thrust
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_t = airspeed_with_throttle_hold(Va_c, Va, flag, P)
  persistent integrator;
  persistent differentiator;
  persistent differentiator_d1;
  persistent error_d1;
  % inicializamos
  if flag==1,
      integrator = 0; 
      differentiator = 0;
      differentiator_d1 = 0;
      error_d1   = 0; 
  end
 
  % calculo del error
  error = Va_c - Va;
  
  % integrador
  integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule
  
  % derivada sucia o aproximada
  differentiator = (2*P.tau-P.Ts)/(2*P.tau+P.Ts)*differentiator_d1...
      + (2/(2*P.tau+P.Ts))*(error - error_d1);
  
  % parte proporcional
  up = P.airspeed_throttle_kp * error;
  
  % parte integral
  ui = P.airspeed_throttle_ki * integrator;
  
  % parte derivativa
  ud = P.airspeed_throttle_kd * differentiator;  
  
  % PID 
  delta_t = sat(P.u_trim(4)+(up + ui + ud), 1, 0);
  
  % anti-windup
  if P.airspeed_throttle_ki~=0,
    delta_t_unsat = P.u_trim(4) + up + ui + ud;
    k_antiwindup = P.Ts/P.airspeed_throttle_ki;
    integrator = integrator + k_antiwindup*(delta_t-delta_t_unsat);
  end
  
  % actualizacion
  error_d1 = error;
  differentiator_d1 = differentiator;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% altitude_hold
%   - controla la altura usando el cabeceo
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function theta_c = altitude_hold(h_c, h, flag, P)
  persistent integrator;
  persistent differentiator;
  persistent differentiator_d1;
  persistent error_d1;
  % inicializamos
  if flag==1,
      integrator = 0; 
      differentiator = 0;
      differentiator_d1 = 0;
      error_d1   = 0; 
  end
 
  % calculo del error
  error = h_c - h;
  
  % integrador
  integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule
  
  % derivada sucia
  differentiator = (2*P.tau-P.Ts)/(2*P.tau+P.Ts)*differentiator_d1...
      + (2/(2*P.tau+P.Ts))*(error - error_d1);
  
  % parte proporcional
  up = P.altitude_kp * error*5;
  
  % parte integral
  ui = P.altitude_ki * integrator*5;
  
  % parte derivativa
  ud = P.altitude_kd * differentiator;
  
  
  % PID
  theta_c = sat(up + ui + ud, 30*pi/180, -30*pi/180);
  
  % anti-windup
  if P.altitude_ki~=0,
    theta_c_unsat = up + ui + ud;
    k_antiwindup = P.Ts/P.altitude_ki;
    integrator = integrator + k_antiwindup*(theta_c-theta_c_unsat);
  end
  
  % actualizacion
  error_d1 = error;
  differentiator_d1 = differentiator;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% pitch_hold
%   - controla el cabeceo usando los elevadores
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_e = pitch_hold(theta_c, theta, q, flag, P)
  persistent integrator;
  persistent error_d1;
  % initialize persistent variables at beginning of simulation
  if flag==1,
      integrator = 0; 
      error_d1   = 0; % error at last sample (d1-delayed by one sample)
  end
 
  % compute the current error
  error = theta_c - theta;
  
  % update the integrator
  integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule
  
  % proportional term
  up = P.pitch_kp * error;
  
  % integral term
  ui = P.pitch_ki * integrator;
  
  % derivative term
  ud = -P.pitch_kd * q*0.6;
  
  
  % implement PID control
  delta_e = sat(up + ui + ud, 45*pi/180, -45*pi/180);
  
  % implement integrator anti-windup
  if P.pitch_ki~=0,
    delta_e_unsat = up + ui + ud;
    k_antiwindup = P.Ts/P.pitch_ki;
    integrator = integrator + k_antiwindup*(delta_e-delta_e_unsat);
  end
  
  % update persistent variables
  error_d1 = error;

end



  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% saturación
%   - funcion de saturación
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function out = sat(in, up_limit, low_limit)
  if in > up_limit,
      out = up_limit;
  elseif in < low_limit;
      out = low_limit;
  else
      out = in;
  end
end
  
 