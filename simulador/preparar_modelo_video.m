%% PREPARAR UNA COPIA DEL MODELO CON NOMBRES EN ESPAÑOL (SOLO PARA EL VIDEO)
%
% Crea mavsim_manager_video.slx a partir de mavsim_manager.slx y cambia los
% nombres de los bloques a español. El modelo original NO se modifica:
% experimento_montecarlo.m depende de los nombres originales de los bloques
% ('Wind Gusts', 'Path Manager', 'draw environment', etc.), por lo que no
% deben renombrarse en mavsim_manager.slx.
%
% También activa el registro de señales que usa graficas_video.m.
%
% Ejecutar desde la carpeta del simulador:
%     preparar_modelo_video

orig = 'mavsim_manager';
mdl  = 'mavsim_manager_video';
if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist([mdl '.slx'], 'file'), delete([mdl '.slx']); end
load_system(orig);
save_system(orig, mdl);          % copia; a partir de aquí se trabaja sobre la copia
close_system(orig, 0);
load_system(mdl);

% {nombre original, nombre nuevo}; primero los bloques internos y al final
% los subsistemas que los contienen
nombres = {
    'autopilot',                               'Leyes de control'
    sprintf('estimate states\nfrom sensors'),  'Estimación de estados'
    'true states',                             'Estados reales'
    'Forces & Moments',                        'Fuerzas y momentos'
    sprintf('Level-2 MATLAB\nS-Function'),     sprintf('Dinámica 6 GDL\n(S-function)')
    'autopilot sensors',                       'Sensores del piloto automático'
    'sensors',                                 'Sensores'
    'Wind Gusts',                              'Ráfagas de Dryden'
    'Current comsumsion A-s',                  'Carga consumida (A s)'
    'Distance m',                              'Distancia recorrida (m)'
    'Autopilot',                               'Piloto automático'
    'MAV1',                                    'Dinámica de la aeronave'
    'Wind1',                                   'Viento'
    'Controlled MAV',                          'UAV controlado'
    'Path Follower',                           'Seguidor de trayectoria'
    'Path Manager',                            'Gestor de trayectoria'
    'Path Planner',                            'Planificador de rutas'
    'Simulation Time',                         'Tiempo de simulación'
    'draw environment',                        'Visualización'
};

% registro de señales para graficas_video.m (antes de renombrar: se usan
% las rutas originales)
set_param(mdl,'SignalLogging','on','SignalLoggingName','logsout');
sysFM = [mdl '/Controlled MAV/MAV1/Forces & Moments'];
ph = get_param([sysFM '/Demux'],'PortHandles');
registrar(ph.Outport(2),'airdata');
registrar(ph.Outport(3),'corriente');
ph = get_param([sysFM '/delta'],'PortHandles');       registrar(ph.Outport(1),'delta');
ph = get_param([sysFM '/Integrator'],'PortHandles');  registrar(ph.Outport(1),'distancia_m');
ph = get_param([mdl '/Controlled MAV/MAV1/' sprintf('Level-2 MATLAB\nS-Function')],'PortHandles');
registrar(ph.Outport(1),'estados');

% se reúnen todos los bloques a renombrar y se renombran del más profundo al
% menos profundo, para que cambiar el nombre de un subsistema no invalide la
% ruta de los bloques que contiene
rutas = {}; nuevos = {};
for k = 1:size(nombres,1)
    b = find_system(mdl, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'Name', nombres{k,1});
    if isempty(b)
        fprintf('Aviso: no se encontró el bloque "%s"\n', strrep(nombres{k,1}, newline, ' '));
    end
    rutas  = [rutas; b(:)];                                   %#ok<AGROW>
    nuevos = [nuevos; repmat(nombres(k,2), numel(b), 1)];     %#ok<AGROW>
end
prof = cellfun(@(r) numel(get_param(r,'Parent')), rutas);
[~, orden] = sort(prof, 'descend');
h = get_param(rutas, 'Handle');                               % los handles no cambian al renombrar
for j = orden(:)'
    try
        set_param(h{j}, 'Name', nuevos{j});
    catch err
        fprintf('No se pudo renombrar "%s": %s\n', strrep(get_param(h{j},'Name'), newline, ' '), err.message);
    end
end

save_system(mdl);
open_system(mdl);
fprintf('Listo: %s.slx abierto. Úsalo solo para grabar el video.\n', mdl);
fprintf('Al terminar la simulación ejecuta graficas_video para ver estados y energía.\n');

function registrar(h, nombre)
    set_param(h,'DataLogging','on','DataLoggingNameMode','Custom','DataLoggingName',nombre);
end
