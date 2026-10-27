% PATH_PLANNER_MC  Planificador para el experimento Monte Carlo.
% Entrega los waypoints definidos en P.wpp (filas [pn pe pd chi Va]) en el
% mismo formato que path_planner_chap11.m. Las filas no usadas se rellenan
% con -9999 hasta completar P.size_waypoint_array.
function out = path_planner_mc(in, P, map) %#ok<INUSD>
    wpp = P.wpp;
    num_waypoints = size(wpp,1);
    relleno = -9999*ones(P.size_waypoint_array - num_waypoints, 5);
    wpp = [wpp; relleno];
    out = [num_waypoints; reshape(wpp', 5*P.size_waypoint_array, 1)];
end
