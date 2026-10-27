function [x_trim,u_trim] = compute_trim(filename, Va, gamma, R)
% Va velocidad deseada (m/s)
% gamma Es el ángulo de trayectoria de vuelo deseado (radians)
% R radio de giro deseado           (+) oribita con direccion mano derecha, 
%                                   (-) orbita con direccion mano izquierda

% condiociones iniciales
x0=[0; %pn
    0;  %pe
    0;  %pd
    Va; %u
    0;  %v
    0;  %w
    0;  %phi
    gamma;%theta
    0;  %psi
    0;  %p
    0;  %q
    0]; %r
%especificamos que variebles deben mantenerse igual a las condiciones
%iniciales
ix=[];

%condiciones iniciales de entrada
u0=[0;0;0;1;];
%especificamos condiciones iniciales de entrada para mantener
iu=[];
%definimos salidas constantes
y0=[Va;gamma;0;];
%especificamos que variables deben de estar constantes de la salida
iy=[1,3];
%definimos las variables constantes
dx0=[0;                 % pndot
    0;                  % pedot
    -Va*sin(gamma);     % pddot
    0;                  % udot
    0;                  % vdot
    0;                  % wdot 
    0;                  % phidot
    0;                  % thetadot
    0;                  % psidot
    0;                  % pdot
    0;                  % qdot
    0;];                % rdot
if R~=0, dx0(9) = Va/R; end  % cambiamos la condicion psidot si R=0
% especificamos cuales se mantienen constantes en el algoritmo de
% equilibrio
idx = [3; 4; 5; 6; 7; 8; 9; 10; 11; 12];
% calculamos el trim
[x_trim,u_trim,y_trim,dx_trim] = trim(filename,x0,u0,y0,ix,iu,iy,dx0,idx);

% norma de la linearización (debe ser pequeña)
norm(dx_trim(3:end)-dx0(3:end));

