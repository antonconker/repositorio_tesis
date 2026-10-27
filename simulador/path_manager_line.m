
function out = path_manager_line(in,P,start_of_simulation)

  NN = 0;
  num_waypoints = in(1+NN);
  waypoints = reshape(in(2+NN:5*P.size_waypoint_array+1+NN),5,P.size_waypoint_array);%matriz cuadrada transpuesta a la original
  NN = NN + 1 + 5*P.size_waypoint_array;
  pn        = in(1+NN);
  pe        = in(2+NN);
  h         = in(3+NN);
  % Va      = in(4+NN);
  % alpha   = in(5+NN);
  % beta    = in(6+NN);
  % phi     = in(7+NN);
  % theta   = in(8+NN);
  % chi     = in(9+NN);
  % p       = in(10+NN);
  % q       = in(11+NN);
  % r       = in(12+NN);
  % Vg      = in(13+NN);
  % wn      = in(14+NN);
  % we      = in(15+NN);
  % psi     = in(16+NN);
  state     =  in(1+NN:16+NN);
  NN = NN + 16;
  t         = in(1+NN);
 
  
  p = [pn; pe; -h];

  persistent waypoints_old   % almacen waypoints pasados
  persistent ptr_a           % apuntador de waypoint
  persistent flag_need_new_waypoints % flag que solicita nuevos waypoints
  persistent waypointCounter;
  
  
  if start_of_simulation || isempty(waypoints_old),%isempty regresa 1 si hay un arreglo vacio
      waypoints_old = zeros(5,P.size_waypoint_array);%matriz de 5 por 5 de ceros
      flag_need_new_waypoints = 0;
      waypointCounter = 0;
     
  end
  
  % if the waypoints have changed, update the waypoint pointer
  %si los waypoints cambian, actualizamos el apuntador.
  if min(min(waypoints==waypoints_old))==0,
      ptr_a = 1;
      waypoints_old = waypoints;
      flag_need_new_waypoints = 0;
      waypointCounter = 0;
  end
  
   % define the current and next two waypoints
  
  % find unit vector along current waypoint
  if ptr_a==num_waypoints, 
      ptr_b = 1;
      ptr_c = 2;
  elseif ptr_a==num_waypoints-1,
      ptr_b = num_waypoints;
      ptr_c = 1;
  else
      ptr_b = ptr_a+1;
      ptr_c = ptr_b+1;
  end
  wpp_a = waypoints(1:3,ptr_a);
  wpp_b = waypoints(1:3,ptr_b);
  wpp_c = waypoints(1:3,ptr_c);
 
   % definición de trayectoria recta
  flag   = 1;                  
  Va_d   = waypoints(5,ptr_a); 
  r      = wpp_a;
  q      = wpp_b-wpp_a;
  q      = q/norm(q);
  c      = [0; 0; 0];
  rho    = 0;
  lambda = 0;
  
 

   % determinamos si fue alcanzado el waypoint.
  q_next   = wpp_c - wpp_b;
  q_next   = q_next/norm(q_next);
  n = (q+q_next)/2;
  if norm(n) < 0.001
      n = q; 
  else
      n = n/norm(n);
  end
  if (p-wpp_b)'*n >= 0,  % se verifica si el avión esta en el semiplano
      waypointCounter=waypointCounter+1;
      % actualizamos
      if ptr_a==num_waypoints,
          ptr_a = 1;
      else
          ptr_a = ptr_a+1;
      end
  end
   out = [flag; Va_d; r; q; c; rho; lambda; state; flag_need_new_waypoints; waypointCounter];
end