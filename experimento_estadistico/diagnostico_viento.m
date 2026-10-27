%% DIAGNOSTICO: verifica que el viento medio llegue al modelo
% Ejecutar en la misma sesion de MATLAB despues de experimento_montecarlo
% (necesita la variable P en el espacio de trabajo base). Simula 60 s.
mdl = 'mavsim_manager_mc';
if ~exist('P','var'), error('Primero ejecuta experimento_montecarlo para crear P.'); end
load_system(mdl);
P.wind_n = 10; P.wind_e = 0; P.wind_d = 0;
P.wpp = [0 0 -200 -9999 P.Va0; 45152 0 -200 -9999 P.Va0]; P.wp_stop = 1;
if ~isfield(P,'seed_u'), P.seed_u = 1; P.seed_v = 2; P.seed_w = 3; end
blk = [mdl '/Controlled MAV/Wind1/Constant2'];
fprintf('Parametro del bloque Constant2: %s\n', get_param(blk,'Value'));
fprintf('P.wind en el espacio base:      [%g %g %g]\n', P.wind_n, P.wind_e, P.wind_d);
fprintf('Modo de simulacion: %s | FastRestart: %s\n', get_param(mdl,'SimulationMode'), get_param(mdl,'FastRestart'));
clear path_manager path_manager_line autopilot
so = sim(mdl,'ReturnWorkspaceOutputs','on','StopTime','60');
ad = so.logsout.get('airdata').Values; d = squeeze(ad.Data);
if size(d,2) ~= 6, d = d.'; end
vg = squeeze(so.logsout.get('Vg').Values.Data);
fprintf('Viento registrado en el modelo: w_n = %.2f, w_e = %.2f m/s  (esperado: 10 y 0)\n', mean(d(:,4)), mean(d(:,5)));
fprintf('Va media = %.2f m/s, Vg media = %.2f m/s  (esperado: Va ~ 25, Vg ~ 35)\n', mean(d(:,1)), mean(vg));
