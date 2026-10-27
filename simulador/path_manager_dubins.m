% path_manager_dubins 
%   - follow Dubins paths between waypoint configurations
%
% input is:
%   num_waypoints - Número de waypoints
%   waypoints    - un arreglo de dimensión 5 dada por P.size_waypoint_array.
%                
%                - configuración de los waypoints:
%                  [wn, we, wd, chi_d, Va_d]
%                  
%
% Salidas:
%   flag - if flag==1, seguimiento de linea recta
%          if flag==2, seguimiento de orbita
%   
%   Va^d - viento relativo, o velocidad aerea
%   r    - vector de posición del origen del waypoint
%   q    - vector unitario que define el ángulo entre los waypoints
%   c    - Centro de la órbita
%   rho  - Radio de la órbita
%   lambda = Dirección de la orbita
%
function out = path_manager_dubins(in,P,start_of_simulation)

  NN = 0;
  num_waypoints = in(1+NN);
  waypoints = reshape(in(2+NN:5*P.size_waypoint_array+1+NN),5,P.size_waypoint_array);
  NN = NN + 1 + 5*P.size_waypoint_array;
  pn        = in(1+NN);
  pe        = in(2+NN);
  h         = in(3+NN);
  % Va      = in(4+NN);
  % alpha   = in(5+NN);
  % beta    = in(6+NN);
  % phi     = in(7+NN);
  % theta   = in(8+NN);
  chi     = in(9+NN);
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

  persistent waypoints_old   % variable que almacena los waypoints
  persistent ptr_a           % apuntador de waypoint 
  persistent state_transition % estaso de transición a través de una maquina de estados
  persistent dubinspath
  persistent flag_need_new_waypoints % bandera para solicitar un waypoint nuevo
  persistent flag_first_time_in_state
  
  if start_of_simulation,
      waypoints_old = zeros(5,P.size_waypoint_array);
      flag_need_new_waypoints = 0;
      state_transition = 0;
      flag_first_time_in_state = 1;
  end
  
  
  % su los waypoints han cambiado, actualiza a los nuevos waypoints con sus
  % respectivos apuntadores
  if min(min(waypoints==waypoints_old))==0,
      waypoints_old = waypoints;
      state_transition = 1;
      ptr_a = 1;
      ptr_b = 2;
      start_node = [waypoints(1:4,ptr_a)', 0, 0];
      end_node   = [waypoints(1:4,ptr_b)', 0, 0];      
      dubinspath = dubinsParameters(start_node, end_node, P.R_min);  
      flag_need_new_waypoints = 0;
      flag_first_time_in_state = 1;
  end
  
  % definimos una maquina de estados que nos permita cambiar entre los
  % estados de transición
 switch state_transition,
      case 0, % iniciando primera ruta
          flag   = 1;
          Va_d   = P.Va0;
          r      = p;
          q      = [cos(chi); sin(chi); 0];
          c      = [0;0;0];
          rho    = 0;
          lambda = 0;
          if flag_first_time_in_state,
              flag_first_time_in_state =0;
          end
      
      case 1, % siguiendo la primer ruta con dubins
          flag   = 2;  % seguimiento de orbita
          Va_d   = waypoints(5,ptr_a); % velocidad deseada en la ruta
          r      = [0;0;0];
          q      = [0;0;0];
          c      = dubinspath.cs;
          rho    = P.R_min;
          lambda = dubinspath.lams;
          
          if ((p-dubinspath.w1)'*dubinspath.q1 >= 0)&&(flag_first_time_in_state==1), % start in H1
              state_transition = 2;
              flag_first_time_in_state = 1;
          elseif (p-dubinspath.w1)'*dubinspath.q1 >= 0, % entering H1
              state_transition = 3;
              flag_first_time_in_state = 1;
          else
              flag_first_time_in_state = 0;
          end
          
      case 2, % siguiendo la orbita pasando del lado derecho del semiplano h
          flag   = 2;   
          Va_d   = waypoints(5,ptr_a); 
          r      = [0;0;0];
          q      = [0;0;0];
          c      = dubinspath.cs;
          rho    = P.R_min;
          lambda = dubinspath.lams;
          
          if (p-dubinspath.w1)'*dubinspath.q1 < 0, % get to right side H1
              state_transition = 1;
              flag_first_time_in_state = 1;
          else
              flag_first_time_in_state = 0;
          end
          
      case 3, % follow straight line on Dubins path until intersect H2
          flag   = 1;  % following straight line path
          Va_d   = waypoints(5,ptr_a); % desired airspeed along waypoint path
          r      = dubinspath.w1;
          q      = dubinspath.q1;
          c      = [0; 0; 0];
          rho    = 0;
          lambda = 0;
          flag_first_time_in_state = 0;
          
          if (p-dubinspath.w2)'*dubinspath.q1 >= 0, % entering H2
              state_transition = 4;
              flag_first_time_in_state = 1;
          end
              
      case 4, % follow second orbit on Dubins path until intersect H3
          flag   = 2;  % following orbit
          Va_d   = waypoints(5,ptr_a); % desired airspeed along waypoint path
          r      = [0;0;0];
          q      = [0;0;0];
          c      = dubinspath.ce;
          rho    = P.R_min;
          lambda = dubinspath.lame;
          flag_first_time_in_state = 0;
          
          if ((p-dubinspath.w3)'*dubinspath.q3 >= 0)&&(flag_first_time_in_state==1), % start in H3
              state_transition = 5;
              flag_first_time_in_state=1;
          elseif (p-dubinspath.w3)'*dubinspath.q3 >= 0, % entering H3
              % increase the waypoint pointer
              if ptr_a==num_waypoints-1,
                  flag_need_new_waypoints = 1;
                  ptr_b = ptr_a+1;
              else
                  ptr_a = ptr_a+1;
                  ptr_b = ptr_a+1;
                  state_transition = 1;
                  flag_first_time_in_state = 1;
              end
              % plan new Dubin's path to next waypoint configuration
              start_node = [waypoints(1:4,ptr_a)', 0, 0];
              end_node   = [waypoints(1:4,ptr_b)', 0, 0];      
              dubinspath = dubinsParameters(start_node, end_node, P.R_min);    
          else
              flag_first_time_in_state = 0;
          end

      case 5, % follow first orbit on Dubins path until on right side of H3
          flag   = 2;  % following orbit
          Va_d   = waypoints(5,ptr_a); % desired airspeed along waypoint path
          r      = [0;0;0];
          q      = [0;0;0];
          c      = dubinspath.ce;
          rho    = P.R_min;
          lambda = dubinspath.lame;
          flag_first_time_in_state = 0;
          
          if (p-dubinspath.w3)'*dubinspath.q3 < 0, % get to right side of H3
              state_transition = 4;
              flag_first_time_in_state = 1;
          else
              flag_first_time_in_state = 0;
          end
          
              
  end
  
  
  out = [flag; Va_d; r; q; c; rho; lambda; state; flag_need_new_waypoints];

end

