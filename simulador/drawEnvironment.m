function drawEnvironment(uu,P)
    % process inputs to function
    NN = 0;
    pn       = uu(1+NN);       % inertial North position     
    pe       = uu(2+NN);       % inertial East position
    pd       = uu(3+NN);       % inertial Down position
    u        = uu(4+NN);       % body frame velocities
    v        = uu(5+NN);       
    w        = uu(6+NN);       
    phi      = uu(7+NN);       % roll angle         
    theta    = uu(8+NN);       % pitch angle     
    psi      = uu(9+NN);       % yaw angle     
    p        = uu(10+NN);      % roll rate
    q        = uu(11+NN);      % pitch rate     
    r        = uu(12+NN);      % yaw rate    
    t        = uu(13+NN);      % time
    
    NN = NN + 13;
    path     = uu(1+NN:13+NN); 
    NN = NN + 13;
    num_waypoints = uu(1+NN);
    waypoints     = reshape(uu(2+NN:5*num_waypoints+1+NN),5,num_waypoints)'; 
    
    % define persistent variables 
    persistent aircraft_handle;  % figure handle for MAV
    %persistent path_handle;     % handle for straight-line or orbit path
    persistent waypoint_handle;  % handle for waypoints
    persistent Faces
    persistent Vertices
    persistent facecolors
    persistent traj_handle;
    S = 8000; % plot size
    
    % first time function is called, initialize plot and persistent vars
    if t==0
        figure(1);
        clf;
        scale = 15;
        [Vertices, Faces, facecolors] = defineAircraftBody(scale);                              
        
        aircraft_handle = drawBody(Vertices, Faces, facecolors,...
                                   pn, pe, pd, phi, theta, psi, []);
        hold on;
        
        waypoint_handle = drawWaypoints(waypoints, P.R_min, []); % Draw all the route
        hold on;
        
        traj_handle = plot3(pe, pn, -pd, 'r--');
        
        % path_handle = drawPath(path, S, []); % show the actual path to follow
        % drawMap(P.map); % Draw the buildings
        
        title('Seguimiento de trayectoria del UAV');
        xlabel('Este (m)');
        ylabel('Norte (m)');
        zlabel('Altitud (m)');
        
        % axis([-S/5,S,-S/5,S,0,3*P.map.MaxHeight]);
        axis([-S, S, -S, S, 0, 3*P.map.MaxHeight]);
        % view(-40,70)  % set the view angle for figure
        view(2);
        grid on;
        
        % Creado aquí para no sobrecargar el bucle (Mejora de rendimiento)
        legend([traj_handle, waypoint_handle], 'Trayectoria de la aeronave', 'Trayectoria deseada');
        
    % at every other time step, redraw MAV and update trajectory
    else 
        drawBody(Vertices, Faces, facecolors,...
                 pn, pe, pd, phi, theta, psi, aircraft_handle);
                 
        drawWaypoints(waypoints, P.R_min, waypoint_handle);
        % drawPath(path, S, path_handle); % draw actual path
        
        % Update trajectory history
        set(traj_handle, 'XData', [get(traj_handle,'XData') pe]);
        set(traj_handle, 'YData', [get(traj_handle,'YData') pn]);
        set(traj_handle, 'ZData', [get(traj_handle,'ZData') -pd]);
    end
end

%=======================================================================
% drawBody
%=======================================================================
function handle = drawBody(V, F, colors, pn, pe, pd, phi, theta, psi, handle)
  V = rotate(V', phi, theta, psi)';  % rotate rigid body  
  V = translate(V', pn, pe, pd)';    % translate after rotation
  % transform vertices from NED to XYZ (for matlab rendering)
  R = [0, 1, 0;
       1, 0, 0;
       0, 0, -1];
  V = V * R;
  
  if isempty(handle)
    handle = patch('Vertices', V, 'Faces', F,...
                 'FaceVertexCData', colors,...
                 'FaceColor', 'flat');
  else
    set(handle, 'Vertices', V, 'Faces', F);
    drawnow limitrate;
  end
end 

%=======================================================================
% drawPath
%=======================================================================
function handle = drawPath(path, S, handle)
    flag = path(1); 
    r    = [path(3); path(4); path(5)];
    q    = [path(6); path(7); path(8)];
    c    = [path(9); path(10); path(11)];
    rho  = path(12);
    % lam  = path(13); % Variable no usada
    
    switch flag
        case 1
            XX = [r(1), r(1)+S*q(1)];
            YY = [r(2), r(2)+S*q(2)];
            ZZ = [r(3), r(3)+S*q(3)];
        case 2
            N = 100;
            th = 0:2*pi/N:2*pi;
            XX = c(1) + rho*cos(th);
            YY = c(2) + rho*sin(th);
            ZZ = c(3)*ones(size(th));
    end
    
    if isempty(handle)
        handle = plot3(YY, XX, -ZZ, 'r');
    else
        set(handle, 'XData', YY, 'YData', XX, 'ZData', -ZZ);
        drawnow limitrate;
    end
end 

%=======================================================================
% drawWaypoints
%=======================================================================
function handle = drawWaypoints(waypoints, R_min, handle)
    if waypoints(1,4) == -9999 % check to see if Dubins paths
        XX = waypoints(:,1);
        YY = waypoints(:,2);
        ZZ = waypoints(:,3);
    else
        XX = [];
        YY = [];
        for i = 2:size(waypoints,1)
            dubinspath = dubinsParameters(waypoints(i-1,:), waypoints(i,:), R_min);
            [tmpX, tmpY] = pointsAlongDubinsPath(dubinspath, 0.1);
            XX = [XX; tmpX];
            YY = [YY; tmpY];     
        end
        ZZ = waypoints(i,3) * ones(size(XX));
    end
    
    if isempty(handle)
        handle = plot3(YY, XX, -ZZ, 'b');
        % Adding waypoints text labels
        for i = 1:size(waypoints,1)
            text(YY(i), XX(i), -ZZ(i), ['WP' num2str(i)], 'FontSize', 10, 'Color', 'k');
        end
    else
        set(handle, 'XData', YY, 'YData', XX, 'ZData', -ZZ);
        drawnow limitrate;
    end
end 

%=======================================================================
% rotate
%=======================================================================
function XYZ = rotate(XYZ, phi, theta, psi)
  % define rotation matrix
  R_roll = [1, 0, 0;
            0, cos(phi), -sin(phi);
            0, sin(phi), cos(phi)];
            
  R_pitch = [cos(theta), 0, sin(theta);
             0, 1, 0;
             -sin(theta), 0, cos(theta)];
             
  R_yaw = [cos(psi), -sin(psi), 0;
           sin(psi), cos(psi), 0;
           0, 0, 1];
           
  % rotate vertices
  XYZ = R_yaw * R_pitch * R_roll * XYZ;
end

%=======================================================================
% translate
%=======================================================================
function XYZ = translate(XYZ, pn, pe, pd)
  XYZ = XYZ + repmat([pn; pe; pd], 1, size(XYZ, 2));
end

%=======================================================================
% drawMap
%=======================================================================
function drawMap(map)
  % draw buildings 
  V = [];
  F = [];
  patchcolors = [];
  count = 0;
  
  for i = 1:map.NumBlocks
      for j = 1:map.NumBlocks
        [Vtemp, Ftemp, patchcolorstemp] = buildingVertFace(map.buildings_n(i),...
            map.buildings_e(j), map.BuildingWidth, map.heights(j,i));
        V = [V; Vtemp];
        Ftemp = Ftemp + count;
        F = [F; Ftemp];
        count = count + 8;
        patchcolors = [patchcolors; patchcolorstemp];
      end
  end
  
  patch('Vertices', V, 'Faces', F,...
        'FaceVertexCData', patchcolors,...
        'FaceColor', 'flat');
end

%=======================================================================
% buildingVertFace
%=======================================================================
function [V, F, patchcolors] = buildingVertFace(n, e, width, height)
  % vertices of the building
  V = [ e+width/2, n+width/2, 0;
        e+width/2, n-width/2, 0;
        e-width/2, n-width/2, 0;
        e-width/2, n+width/2, 0;
        e+width/2, n+width/2, height;
        e+width/2, n-width/2, height;
        e-width/2, n-width/2, height;
        e-width/2, n+width/2, height ];    
        
  % define faces of building
  F = [ 1, 4, 8, 5; % North Side
        1, 2, 6, 5; % East Side
        2, 3, 7, 6; % South Side
        3, 4, 8, 7; % West Side
        5, 6, 7, 8; % Top
      ];   
      
  mygreen  = [0, 1, 0];
  myyellow = [1, 1, 0];
  
  patchcolors = [ mygreen;  % North
                  mygreen;  % East
                  mygreen;  % South
                  mygreen;  % West
                  myyellow; % Top
                ];
end

%=======================================================================
% pointsAlongDubinsPath
%=======================================================================
function [X, Y] = pointsAlongDubinsPath(dubinspath, Del)
  % points along start circle
  th1 = mod(atan2(dubinspath.ps(2)-dubinspath.cs(2), dubinspath.ps(1)-dubinspath.cs(1)), 2*pi);
  th2 = mod(atan2(dubinspath.w1(2)-dubinspath.cs(2), dubinspath.w1(1)-dubinspath.cs(1)), 2*pi);
  
  if dubinspath.lams > 0
      if th1 >= th2
        th = [th1:Del:2*pi, 0:Del:th2];
      else
        th = th1:Del:th2;
      end
  else
      if th1 <= th2
        th = [th1:-Del:0, 2*pi:-Del:th2];
      else
        th = th1:-Del:th2;
      end
  end
  
  X = [];
  Y = [];
  for i = 1:length(th)
    X = [X; dubinspath.cs(1) + dubinspath.R*cos(th(i))]; 
    Y = [Y; dubinspath.cs(2) + dubinspath.R*sin(th(i))];
  end
  
  % points along straight line 
  sig = 0;
  while sig <= 1
      X = [X; (1-sig)*dubinspath.w1(1) + sig*dubinspath.w2(1)];
      Y = [Y; (1-sig)*dubinspath.w1(2) + sig*dubinspath.w2(2)];
      sig = sig + Del;
  end
    
  % points along end circle
  th2 = mod(atan2(dubinspath.pe(2)-dubinspath.ce(2), dubinspath.pe(1)-dubinspath.ce(1)), 2*pi);
  th1 = mod(atan2(dubinspath.w2(2)-dubinspath.ce(2), dubinspath.w2(1)-dubinspath.ce(1)), 2*pi);
  
  if dubinspath.lame > 0
      if th1 >= th2
        th = [th1:Del:2*pi, 0:Del:th2];
      else
        th = th1:Del:th2;
      end
  else
      if th1 <= th2
        th = [th1:-Del:0, 2*pi:-Del:th2];
      else
        th = th1:-Del:th2;
      end
  end
  
  for i = 1:length(th)
    X = [X; dubinspath.ce(1) + dubinspath.R*cos(th(i))]; 
    Y = [Y; dubinspath.ce(2) + dubinspath.R*sin(th(i))];
  end
end

%=======================================================================
% defineAircraftBody
%=======================================================================
function [V, F, colors] = defineAircraftBody(scale)
  fuse_l1    = 7;
  fuse_l2    = 4;
  fuse_l3    = 15;
  fuse_w     = 2;
  wing_l     = 6;
  wing_w     = 20;
  tail_l     = 3;
  tail_h     = 3;
  tailwing_w = 10;
  tailwing_l = 3;
  
  red     = [1, 0, 0];
  green   = [0, 1, 0];
  blue    = [0, 0, 1];
  yellow  = [1, 1, 0];
  
  V = [ fuse_l1,             0,             0;
        fuse_l2,            -fuse_w/2,     -fuse_w/2;
        fuse_l2,             fuse_w/2,     -fuse_w/2;
        fuse_l2,             fuse_w/2,      fuse_w/2;
        fuse_l2,            -fuse_w/2,      fuse_w/2;
       -fuse_l3,             0,             0;
        0,                   wing_w/2,      0;
       -wing_l,              wing_w/2,      0;
       -wing_l,             -wing_w/2,      0;
        0,                  -wing_w/2,      0;
       -fuse_l3+tailwing_l,  tailwing_w/2,  0;
       -fuse_l3,             tailwing_w/2,  0;
       -fuse_l3,            -tailwing_w/2,  0;
       -fuse_l3+tailwing_l, -tailwing_w/2,  0;
       -fuse_l3+tailwing_l,  0,             0;
       -fuse_l3+tailwing_l,  0,             -tail_h;
       -fuse_l3,             0,             -tail_h ];
  
  F = [ 1,  2,  3,  1; 
        1,  3,  4,  1; 
        1,  4,  5,  1; 
        1,  5,  2,  1; 
        2,  3,  6,  2; 
        3,  6,  4,  3; 
        4,  6,  5,  4; 
        2,  5,  6,  2; 
        7,  8,  9, 10; 
       11, 12, 13, 14; 
        6, 15, 17, 17 ];  
  
  colors = [ yellow; yellow; yellow; yellow; 
             blue; blue; red; blue; 
             green; green; blue ];
             
  V = scale * V;   
end