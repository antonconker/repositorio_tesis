% path planner

function out = path_planner_chap11(in,P,map)

  NN = 0;
  % pn        = in(1+NN);
  % pe        = in(2+NN);
  % h         = in(3+NN);
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
  % flag_new_waypoints =  in(17+NN);
  NN = NN + 17;
  % t         =  in(1+NN);


  num_waypoints = 10;
  % format for each point is [pn, pe, pd, chi, Va^d] where the position
  % of the waypoint is (pn, pe, pd), the desired course at the waypoint
  % is chi, and the desired airspeed between waypoints is Va
  % if chi!=-9999, then Dubins paths will be used between waypoints.
  if 1,   %1 para dar inicio a trayectorias con suavizado o lineas rectas, cualquioer otro para los dubins
           % 0,   0,   -100, -9999, P.Va0;...                            #Nodo5
            %7.15316868e+02,   3.46107314e+03, -200, -9999, P.Va0;...    #nodo 9
            %3.72410150e+03,   1.84141530e+03, -200, -9999, P.Va0;...    #nodo 3
            %3.84598537e+03,  -1.21462650e+03, -200, -9999, P.Va0;...    #nodo 1
            %2.53475639e+03,  -5.41696984e+03, -200, -9999, P.Va0;...    #nodo 10
            %3.20315333e+02,  -6.54094552e+03, -200, -9999, P.Va0;...    #nodo 7
            %4.83143200e+03,  -7.61794264e+03, -200, -9999, P.Va0;...    #nodo 2
            %-3.72650750e+03, -1.66724889e+03, -200, -9999, P.Va0;...    #nodo 8
            %-6.32543123e+03,  5.82643639e+03, -200, -9999, P.Va0;...    #nodo 6
            %-1.09379176e+03,  6.41052979e+03, -200, -9999, P.Va0;...    #nodo 4
            % wpp=[...
            %  0,   0,   -200, -9999, P.Va0;... %Nodo5
            %  %9138.70923091/2, 44217.80398078/2,   -200, -9999, P.Va0;... %nodo 9
            %  100, 100,   -200, -9999, P.Va0;... %nodo 9
            %  200, 250,   -200, -9999, P.Va0;... %nodo 9
            %  300, -600,   -200, -9999, P.Va0;... %nodo 9
            %  ];
          
      wpp= [...
            0,   0,   -200, -9999, P.Va0;... %Nodo5
            7.15316868e+02, 3.46107314e+03,   -200, -9999, P.Va0;... %nodo 9
            3.72410150e+03,   1.84141530e+03, -200, -9999, P.Va0;...    %nodo 3...
            3.84598537e+03,  -1.21462650e+03, -200, -9999, P.Va0;...    %nodo 1
            2.53475639e+03,  -5.41696984e+03, -200, -9999, P.Va0;...    %nodo 10
            3.20315333e+02,  -6.54094552e+03, -200, -9999, P.Va0;...    #nodo 7
            4.83143200e+03,  -7.61794264e+03, -200, -9999, P.Va0;...    #nodo 2
            -3.72650750e+03, -1.66724889e+03, -200, -9999, P.Va0;...    #nodo 8
            -6.32543123e+03,  5.82643639e+03, -200, -9999, P.Va0;...    #nodo 6
            -1.09379176e+03,  6.41052979e+03, -200, -9999, P.Va0;...    #nodo 4
           ];
  else  % Dubins
    wpp = [...
            0,    0,    -200,  0,           P.Va0;...
            1200, 0,    -200,  45*pi/180,   P.Va0;...
            0,    1200, -200,  45*pi/180,   P.Va0;...
            1200, 1200, -200, -135*pi/180,  P.Va0;...
           ];
  end      
  %
%   for i=6:P.size_waypoint_array,
%       wpp = [...
%           wpp;...
%           -9999, -9999, -9999, -9999, -9999;...
%           ];
%   end
  
  out = [num_waypoints; reshape(wpp', 5*P.size_waypoint_array, 1)]; 

end