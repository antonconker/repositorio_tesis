
P.gravity = 9.8; %gravedad

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%parámetros de la aeronave
P.mass = 13.5;
P.Jx   = 0.8244;
P.Jy   = 1.135;
P.Jz   = 1.759;
P.Jxz  = .1204;
% coeficientes aerodinámicos
P.S_wing        = 0.55;
P.b             = 2.8956;
P.c             = 0.18994;
P.S_prop        = 0.2027;
P.rho           = 1.2682;
P.k_motor       = 80;
P.k_T_P         = 0;
P.k_Omega       = 0;
P.e             = 0.9;

P.C_L_0         = 0.28;
P.C_L_alpha     = 3.45;
P.C_L_q         = 0.0;
P.C_L_delta_e   = -0.36;
P.C_D_0         = 0.03;
P.C_D_alpha     = 0.30;
P.C_D_p         = 0.0437;
P.C_D_q         = 0.0;
P.C_D_delta_e   = 0.0;
P.C_m_0         = -0.02338;
P.C_m_alpha     = -0.38;
P.C_m_q         = -3.6;
P.C_m_delta_e   = -0.5;
P.C_M_0         = 0.0;
P.C_M_alpha     = -0.38;
P.C_M_q         = -3.6;
P.C_M_delta_e   = -0.5;
P.C_Y_0         = 0.0;
P.C_Y_beta      = -0.98;
P.C_Y_p         = 0.0;
P.C_Y_r         = 0.0;
P.C_Y_delta_a   = 0.0;
P.C_Y_delta_r   = -0.17;
P.C_ell_0       = 0.0;
P.C_ell_beta    = -0.12;
P.C_ell_p       = -0.26;
P.C_ell_r       = 0.14;
P.C_ell_delta_a = 0.08;
P.C_ell_delta_r = 0.105;
P.C_n_0         = 0.0;
P.C_n_beta      = 0.25;
P.C_n_p         = 0.022;
P.C_n_r         = -0.35;
P.C_n_delta_a   = 0.06;
P.C_n_delta_r   = -0.032;
P.C_prop        = 1.0;
P.M             = 50;
P.epsilon       = 0.1592;
P.alpha0        = 0.4712;

%Paramétros del motor

P.D_prop = 20*(0.0254);     % prop diameter in m

P.KV = 145;                   % from datasheet RPM/V
P.KQ = (1./P.KV)*60/(2*pi);  % KQ in N-m/A, V-s/rad
P.R_motor = 0.042;              % ohms
P.i0 = 1.5;                     % no-load (zero-torque) current (A)


%Inputs
P.ncells = 12.3;
P.V_max = 3.7 * P.ncells;  % max voltage for specified number of battery cells

% Coeffiecients from prop_data fit
% P.C_Q2 = -0.01664;
% P.C_Q1 = 0.004970;
% P.C_Q0 = 0.005230;
% P.C_T2 = -0.1079;
% P.C_T1 = -0.06044;
% P.C_T0 = 0.09357;

% Coeficientes ajustados a datos experimentales UIUC (APC 20x10E, Vol. 4)
   P.C_Q2 = -0.025461; P.C_Q1 = 0.012308; P.C_Q0 = 0.004010;
   P.C_T2 = -0.203043; P.C_T1 = -0.026331; P.C_T0 = 0.102397;


% parámetros del viento
P.wind_n = 0;
P.wind_e = 0;
P.wind_d = 0;
P.L_u = 200;
P.L_v = 200;
P.L_w = 50;
P.sigma_u = 1.06; 
P.sigma_v = 1.06;
P.sigma_w = .7;
P.Va = 17;
%calculo de condiciones de equilibrio usando mav_trim

%velocidad inicial del viento

P.Va0=25;     %m/s (~85 mph)
%gamma = 10*pi/180;  % desired flight path angle (radianes)
%R     = 150;         % desired radius (m) - use (+) for right handed orbit,
                     %                          (-) for left handed orbit
gamma = 0*pi/180; %linea recta
R=0; %radio cero

% tiempo de muestreo
P.Ts = 0.01;

% condiciones iniciales

 P.pn0    = 0;  % N
 P.pe0    = 0;  % E
 P.pd0    = -200;  % D (-h)
 P.u0     = P.Va0; % u0
 P.v0     = 0;  % v0
 P.w0     = 0;  % 
 P.phi0   = 0;  % 
 P.theta0 = 0;  % 
 P.psi0   = 0;  % 
 P.p0     = 0;  % 
 P.q0     = 0;  % 
 P.r0     = 0;  % 

                   

% modelo de propulsion coherente con la validacion de la helice (H1):
% empuje con C_T(J) y corriente con el modelo del motor (ver fym.m)
P.modelo_empuje    = 'helice';
P.modelo_corriente = 'motor';

% trim comandos
[x_trim, u_trim]=compute_trim('mavsim_trim',P.Va0,gamma,R);
P.u_trim = u_trim;
P.x_trim = x_trim;

%condiciones iniciales del trim
P.pn0    = 0;  % N
P.pe0    = 0;  % E
P.pd0    = -200;  % D (negativo para la altura)
P.u0     = x_trim(4);  % u0
P.v0     = x_trim(5);  % v0
P.w0     = x_trim(6);  % w0
P.phi0   = x_trim(7);  % phi
P.theta0 = x_trim(8);  % pitch
 P.psi0   = x_trim(9);  % yaw
 P.p0     = x_trim(10);  % p0
 P.q0     = x_trim(11);  % q0
 P.r0     = x_trim(12);  % r0

% compute different transfer functions
% calculo de las funciones de transferencia
[T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r]...
    = compute_tf_model(x_trim,u_trim,P);
% compute_tf_model calcula a_V2 con el empuje original (k_motor); con el
% modelo de helice se obtiene por diferencias finitas (igual que en
% experimento_montecarlo.m), para que las ganancias del lazo de velocidad
% sean coherentes con el empuje que usa la simulacion
if strcmp(P.modelo_empuje,'helice')
    hh = 1e-3;
    fp = fym(x_trim, u_trim + [0;0;0;hh], zeros(6,1), P);
    fm = fym(x_trim, u_trim - [0;0;0;hh], zeros(6,1), P);
    a_V2 = (fp(1) - fm(1))/(2*hh)/P.mass;
    [~, den] = tfdata(T_Va_delta_t,'v');
    T_Va_delta_t = tf(a_V2, den);
end


% linearización a lo largo de las condiciones de equilibrio
[A_lon, B_lon, A_lat, B_lat] = compute_ss_model('mavsim_trim',x_trim,u_trim);

% ganancia derivada sucia
P.tau = 5;


P.altitude_take_off_zone = 15;
P.altitude_hold_zone = 20;
Ganancias;
P.phi_max = 15*pi/180;

%sensores
P.sigma_gyro = 0.13*pi/180;      % standard deviation of gyros in rad/sec
P.bias_gyro_x = 0.1*pi/180*rand; % bias on x_gyro
P.bias_gyro_y = 0.1*pi/180*rand; % bias on y_gyro
P.bias_gyro_z = 0.1*pi/180*rand; % bias on z_gyro
P.sigma_accel = 0.0025*9.8;      % standard deviation of accelerometers in m/s^2
P.sigma_static_pres = 0.01*1000; % standard deviation of static pressure sensor in Pascals
P.sigma_diff_pres = 0.002*1000;  % standard deviation of diff pressure sensor in Pascals
%GPS
P.Ts_gps = 1;                              % sample rate of GPS in s
P.beta_gps = 1/1100;                       % 1/s
P.sigma_n_gps = 0.21;
P.sigma_e_gps = 0.21; 
P.sigma_h_gps = 0.40;
P.sigma_Vg_gps = 0.05;
P.sigma_course_gps = P.sigma_Vg_gps/P.Va;



% chapter 11 - path manager
% number of waypoints in data structure
P.size_waypoint_array = 10;
P.R_min = P.Va0^2/P.gravity/tan(P.phi_max);

% create random city map
city_width      = 7000;  % the city is of size (width)x(width)
building_height = 300;   % maximum height of buildings
%building_height = 1;   % maximum height of buildings (for camera)
num_blocks      = 1;    % number of blocks in city
street_width    = .8;   % percent of block that is street.
P.pd0           = P.pd0;  % initial height of MAV
P.map = createWorld(city_width, building_height, num_blocks, street_width);

