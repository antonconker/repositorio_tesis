function mav_dynamics_v2(block)
% MAV_DYNAMICS_V2 Level-2 MATLAB S-Function (version corregida de mav_dynamics.m)
% Correccion: en qdot el coeficiente de (p^2 - r^2) es Jxz/Jy (Gamma6 de
% Beard & McLain); el original usaba Jxz/Gamma.
% Modelo dinámico de un vehículo aéreo (MAV) 6 DoF

  setup(block);

% ==========================================================
function setup(block)

  % Número de parámetros esperados en el bloque
  block.NumDialogPrms = 17; % mass, Jx, Jy, Jz, Jxz, pn0,..., r0

  % Número de puertos
  block.NumInputPorts  = 1;
  block.NumOutputPorts = 1;

  % Tamaño de puertos
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;
  
  block.InputPort(1).Dimensions  = 6;  % fx, fy, fz, ell, m, n
  block.OutputPort(1).Dimensions = 12; % estados

  block.InputPort(1).DirectFeedthrough = false;

  % Número de estados
  block.NumContStates = 12;
 

  % Tiempo de muestreo continuo
  block.SampleTimes = [0 0];

  % Métodos de la S-Function
  block.RegBlockMethod('InitializeConditions', @InitializeConditions);
  block.RegBlockMethod('Derivatives',          @Derivatives);
  block.RegBlockMethod('Outputs',              @Outputs);
  block.RegBlockMethod('Terminate',            @Terminate);

% ==========================================================
function InitializeConditions(block)
  % Extraer parámetros
  pn0    = block.DialogPrm(6).Data;
  pe0    = block.DialogPrm(7).Data;
  pd0    = block.DialogPrm(8).Data;
  u0     = block.DialogPrm(9).Data;
  v0     = block.DialogPrm(10).Data;
  w0     = block.DialogPrm(11).Data;
  phi0   = block.DialogPrm(12).Data;
  theta0 = block.DialogPrm(13).Data;
  psi0   = block.DialogPrm(14).Data;
  p0     = block.DialogPrm(15).Data;
  q0     = block.DialogPrm(16).Data;
  r0     = block.DialogPrm(17).Data;

  % Inicializar estados
  block.ContStates.Data = [pn0; pe0; pd0; u0; v0; w0; ...
                           phi0; theta0; psi0; p0; q0; r0];

% ==========================================================
function Derivatives(block)
  % Parámetros
  mass = block.DialogPrm(1).Data;
  Jx   = block.DialogPrm(2).Data;
  Jy   = block.DialogPrm(3).Data;
  Jz   = block.DialogPrm(4).Data;
  Jxz  = block.DialogPrm(5).Data;

  % Estados
  x     = block.ContStates.Data;
  pn    = x(1); pe = x(2); pd = x(3);
  u     = x(4); v  = x(5); w  = x(6);
  phi   = x(7); theta = x(8); psi = x(9);
  p     = x(10); q = x(11); r  = x(12);

  % Entradas
  uu = block.InputPort(1).Data;
  fx  = uu(1); fy = uu(2); fz = uu(3);
  ell = uu(4); m  = uu(5); n  = uu(6);

  % Seguridad numérica
  cos_theta = max(cos(theta), eps);
  tan_theta = sin(theta) / cos_theta;

  % Constantes de inercia
  Gamma  = Jx*Jz - Jxz^2;
  Gamma1 = (Jxz*(Jx - Jy + Jz)) / Gamma;
  Gamma2 = (Jz*(Jz - Jy) + Jxz^2) / Gamma;
  Gamma3 = Jz / Gamma;
  Gamma4 = Jxz / Gamma;
  Gamma5 = (Jz - Jx) / Jy;
  Gamma6 = (Jx*(Jx - Jy) + Jxz^2) / Gamma;
  Gamma7 = Jx / Gamma;

  % Cinemática de posición
  pndot = cos(theta)*cos(psi)*u + ...
          (sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi))*v + ...
          (cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi))*w;

  pedot = cos(theta)*sin(psi)*u + ...
          (sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi))*v + ...
          (cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi))*w;

  pddot = -sin(theta)*u + sin(phi)*cos(theta)*v + cos(phi)*cos(theta)*w;

  % Dinámica de velocidad
  udot = r*v - q*w + fx/mass;
  vdot = p*w - r*u + fy/mass;
  wdot = q*u - p*v + fz/mass;

  % Cinemática angular
  phidot   = p + sin(phi)*tan_theta*q + cos(phi)*tan_theta*r;
  thetadot = cos(phi)*q - sin(phi)*r;
  psidot   = (sin(phi)/cos_theta)*q + (cos(phi)/cos_theta)*r;

  % Dinámica angular
  pdot = Gamma1*p*q - Gamma2*q*r + Gamma3*ell + Gamma4*n;
  qdot = Gamma5*p*r - (Jxz/Jy)*(p^2 - r^2) + m/Jy;   % CORRECCION
  rdot = Gamma6*p*q - Gamma1*q*r + Gamma4*ell + Gamma7*n;

  % Asignar derivadas
  block.Derivatives.Data = [pndot; pedot; pddot; udot; vdot; wdot; ...
                            phidot; thetadot; psidot; pdot; qdot; rdot];

% ==========================================================
function Outputs(block)
  block.OutputPort(1).Data = block.ContStates.Data;

% ==========================================================
function Terminate(~)
  % Nada que hacer al terminar
