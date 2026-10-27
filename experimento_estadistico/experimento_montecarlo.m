%% EXPERIMENTO MONTE CARLO PARA LAS HIPOTESIS H2 Y H3
%
% Ejecutar desde la carpeta del simulador (donde estan param_chap11.m,
% mavsim_manager.slx y mavsim_trim.slx), con esta carpeta en el path:
%     addpath('ruta/a/experimento_estadistico')
%     experimento_montecarlo
%
% El script NO modifica los modelos originales: crea copias
% (mavsim_manager_mc.slx y mavsim_trim_v2.slx) y trabaja sobre ellas.
%
% Diseno: para cada replica i = 1..N se usa la misma semilla de turbulencia
% en los tres escenarios (numeros aleatorios comunes), lo que permite un
% analisis pareado:
%   A0: ruta RGPPM,          sin viento medio, turbulencia Dryden
%   A1: ruta RGPPM,          viento medio W en direccion theta_i, turbulencia
%   B0: recta equivalente,   sin viento medio, turbulencia (misma longitud)
%   H2: d_i = E(A1_i) - E(A0_i)      H3: d_i = E(A0_i) - E(B0_i)
%
% Los resultados se guardan fila por fila en resultados_mc/resultados.csv;
% si la ejecucion se interrumpe, al volver a correr el script continua
% donde se quedo.

clear; clc;

%% ---------------- configuracion ----------------
cfg.N              = 30;        % 5 = prueba piloto; despues 30 (o el N calculado)
cfg.semilla_base   = 1000;     % semilla de la replica i: semilla_base + i
cfg.W              = 10;       % magnitud del viento medio en A1 [m/s]
cfg.direccion      = 'aleatoria';  % 'aleatoria' (U[0,360)) o 'fija'
cfg.dir_fija_deg   = 0;        % direccion hacia donde sopla (0 = norte, 90 = este)
cfg.turbulencia    = true;     % Dryden de baja altitud (sigmas de param_chap11)
cfg.usar_v2        = true;     % fym_v2 y mav_dynamics_v2 (modelos corregidos)
cfg.modelo_empuje  = 'helice';     % 'helice' (recomendado) o 'original' (ver fym_v2.m)
cfg.modelo_corriente = 'motor';    % 'motor' o 'polinomio'
cfg.escenarios     = {'A0','A1','B0'};
cfg.mdl_base       = 'mavsim_manager';
cfg.mdl            = 'mavsim_manager_mc';
cfg.carpeta        = 'resultados_mc_v2';
cfg.t_max          = 20000;    % tiempo maximo de simulacion [s] (seguridad)

if ~exist(cfg.carpeta,'dir'), mkdir(cfg.carpeta); end
archivo_csv = fullfile(cfg.carpeta,'resultados.csv');
save(fullfile(cfg.carpeta,'configuracion.mat'),'cfg');

%% ---------------- parametros, trim y ganancias ----------------
param_chap11;                                  % crea P (trim con el modelo original)
P.modelo_empuje    = cfg.modelo_empuje;
P.modelo_corriente = cfg.modelo_corriente;
if cfg.usar_v2
    % trim y ganancias recalculados con los modelos corregidos
    preparar_modelo_trim_v2();
    [x_trim, u_trim] = compute_trim('mavsim_trim_v2', P.Va0, 0, 0);
    P.u_trim = u_trim;  P.x_trim = x_trim;
    P.u0 = x_trim(4); P.v0 = x_trim(5); P.w0 = x_trim(6);
    P.phi0 = x_trim(7); P.theta0 = x_trim(8); P.psi0 = x_trim(9);
    P.p0 = x_trim(10);  P.q0 = x_trim(11);    P.r0 = x_trim(12);
    [T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r] ...
        = compute_tf_model(x_trim, u_trim, P);
    if strcmp(P.modelo_empuje,'helice')
        % compute_tf_model calcula a_V2 con el modelo de empuje original
        % (k_motor); con el modelo de helice se obtiene por diferencias finitas.
        h  = 1e-3;
        fp = fym_v2(x_trim, u_trim + [0;0;0;h], zeros(6,1), P);
        fm = fym_v2(x_trim, u_trim - [0;0;0;h], zeros(6,1), P);
        a_V2 = (fp(1) - fm(1))/(2*h)/P.mass;
        [~, den] = tfdata(T_Va_delta_t,'v');
        T_Va_delta_t = tf(a_V2, den);
        fprintf('a_V2 (modelo de helice) = %.3f\n', a_V2);
    end
    Ganancias;                                 % actualiza las ganancias en P
end
% variables que el bloque S-Function toma del espacio de trabajo base
mass = P.mass; Jx = P.Jx; Jy = P.Jy; Jz = P.Jz; Jxz = P.Jxz;
pn0 = P.pn0; pe0 = P.pe0; pd0 = P.pd0; u0 = P.u0; v0 = P.v0; w0 = P.w0;
phi0 = P.phi0; theta0 = P.theta0; psi0 = P.psi0; p0 = P.p0; q0 = P.q0; r0 = P.r0;
sigmas = [P.sigma_u P.sigma_v P.sigma_w];
if ~cfg.turbulencia, sigmas = [0 0 0]; end

%% ---------------- rutas ----------------
% Ruta A: waypoints de RGPPM (path_planner_chap11.m)
wpA = [   0,               0;
          7.15316868e+02,  3.46107314e+03;
          3.72410150e+03,  1.84141530e+03;
          3.84598537e+03, -1.21462650e+03;
          2.53475639e+03, -5.41696984e+03;
          3.20315333e+02, -6.54094552e+03;
          4.83143200e+03, -7.61794264e+03;
         -3.72650750e+03, -1.66724889e+03;
         -6.32543123e+03,  5.82643639e+03;
         -1.09379176e+03,  6.41052979e+03];
L_A = sum(vecnorm(diff(wpA),2,2));             % 45 152.3 m
rutaA = [wpA, -200*ones(10,1), -9999*ones(10,1), P.Va0*ones(10,1)];
% Ruta B: recta y nivelada de la misma longitud, rumbo norte
rutaB = [0, 0, -200, -9999, P.Va0;  L_A, 0, -200, -9999, P.Va0];
fprintf('Longitud de la ruta A: %.1f m\n', L_A);

%% ---------------- modelo para el experimento ----------------
preparar_modelo_mc(cfg);

%% ---------------- plan de corridas ----------------
plan = {};
for i = 1:cfg.N
    semilla = cfg.semilla_base + i;
    rng(semilla);                              % direccion del viento reproducible
    if strcmp(cfg.direccion,'aleatoria'), theta_deg = 360*rand; else, theta_deg = cfg.dir_fija_deg; end
    for k = 1:numel(cfg.escenarios)
        plan(end+1,:) = {i, semilla, theta_deg, cfg.escenarios{k}}; %#ok<SAGROW>
    end
end

hechos = table();
if exist(archivo_csv,'file'), hechos = readtable(archivo_csv); end

%% ---------------- ejecucion ----------------
for r = 1:size(plan,1)
    [i, semilla, theta_deg, esc] = plan{r,:};
    if ~isempty(hechos) && any(hechos.replica==i & strcmp(hechos.escenario,esc))
        continue;                              % ya ejecutada
    end
    Pr = P;
    Pr.seed_u = semilla; Pr.seed_v = semilla + 7919; Pr.seed_w = semilla + 15838;  % semillas distintas por eje
    Pr.sigma_u = sigmas(1); Pr.sigma_v = sigmas(2); Pr.sigma_w = sigmas(3);
    switch esc
        case 'A0', Pr.wpp = rutaA; Pr.wp_stop = 9; W = 0;
        case 'A1', Pr.wpp = rutaA; Pr.wp_stop = 9; W = cfg.W;
        case 'B0', Pr.wpp = rutaB; Pr.wp_stop = 1; W = 0;
    end
    Pr.wind_n = W*cosd(theta_deg);  Pr.wind_e = W*sind(theta_deg);  Pr.wind_d = 0;
    P = Pr;                                    % el modelo lee P del espacio de trabajo base

    % reiniciar variables persistentes de las funciones del lazo
    clear path_manager path_manager_line path_manager_dubins autopilot seguimiento gps sensors estimate_states

    fprintf('[%s] replica %d/%d, escenario %s, theta = %.1f deg ... ', ...
        datestr(now,'HH:MM:SS'), i, cfg.N, esc, theta_deg);
    tic;
    simOut = sim(cfg.mdl, 'ReturnWorkspaceOutputs', 'on', 'StopTime', num2str(cfg.t_max));
    m = calcular_metricas(simOut, Pr, cfg.t_max);
    if W > 0 && hypot(m.wn_media, m.we_media) < 0.5*W
        error(['El viento medio no llego al modelo (w = %.2f m/s, esperado %.1f m/s). ' ...
               'Ejecuta diagnostico_viento y avisame.'], hypot(m.wn_media, m.we_media), W);
    end
    fprintf('%.0f s de computo; E = %.1f Wh, t = %.0f s\n', toc, m.E_Wh, m.t_vuelo_s);

    fila = table(i, semilla, theta_deg, {esc}, m.t_vuelo_s, m.distancia_m, m.carga_Ah, ...
        m.E_Wh, m.P_media_W, m.E_por_km_Wh, m.Va_media, m.wn_media, m.we_media, m.completa, ...
        'VariableNames', {'replica','semilla','theta_deg','escenario','t_vuelo_s', ...
        'distancia_m','carga_Ah','E_Wh','P_media_W','E_por_km_Wh','Va_media', ...
        'wn_media','we_media','completa'});
    hechos = [hechos; fila]; %#ok<AGROW>
    writetable(hechos, archivo_csv);
end
disp('Experimento terminado. Ejecuta analisis_estadistico.m');

%% ======================= funciones locales =======================
function preparar_modelo_trim_v2()
    if exist('mavsim_trim_v2.slx','file')
        load_system('mavsim_trim_v2');
        set_param('mavsim_trim_v2','InitFcn','');   % no volver a ejecutar param_chap11
        return;
    end
    load_system('mavsim_trim');
    save_system('mavsim_trim','mavsim_trim_v2');   % copia; el original queda intacto
    mdl = 'mavsim_trim_v2';
    reemplazar_fym_y_dinamica(mdl);
    set_param(mdl,'InitFcn','');
    save_system(mdl);
end

function reemplazar_fym_y_dinamica(mdl)
    fcn = find_system(mdl,'LookUnderMasks','all','FollowLinks','on','BlockType','MATLABFcn');
    for k = 1:numel(fcn)
        expr = get_param(fcn{k},'MATLABFcn');
        if startsWith(strtrim(expr),'fym(')
            set_param(fcn{k},'MATLABFcn',strrep(expr,'fym(','fym_v2('));
        end
    end
    sf = find_system(mdl,'LookUnderMasks','all','FollowLinks','on','BlockType','M-S-Function');
    for k = 1:numel(sf)
        if strcmp(get_param(sf{k},'FunctionName'),'mav_dynamics')
            set_param(sf{k},'FunctionName','mav_dynamics_v2');
        end
    end
end

function preparar_modelo_mc(cfg)
    mdl = cfg.mdl;
    if bdIsLoaded(mdl), close_system(mdl,0); end
    if exist([mdl '.slx'],'file'), delete([mdl '.slx']); end
    load_system(cfg.mdl_base);
    save_system(cfg.mdl_base, mdl);              % copia de trabajo
    % 0) el modelo original ejecuta param_chap11 en su InitFcn; eso reiniciaba
    %    P (viento = 0, trim y ganancias originales) al inicio de cada corrida.
    %    En la copia se desactiva para que se usen los parametros del experimento.
    set_param(mdl,'InitFcn','');
    % 1) conectar las rafagas de Dryden (en el original iban a un Terminator)
    %    (si ya estan conectadas en el modelo original, no se modifica nada)
    sysW = [mdl '/Controlled MAV/Wind1'];
    phG  = get_param([sysW '/Wind Gusts'],'PortHandles');
    lG   = get_param(phG.Outport(1),'Line');
    conectado = false;
    if lG ~= -1
        destinos  = get_param(get_param(lG,'DstBlockHandle'),'Name');
        conectado = any(strcmp(destinos,'Mux1'));
    end
    if ~conectado
        try, delete_line(sysW,'Constant1/1','Mux1/2'); catch, end
        try, delete_line(sysW,'Wind Gusts/1','Terminator/1'); catch, end
        add_line(sysW,'Wind Gusts/1','Mux1/2','autorouting','on');
        disp('Rafagas de Dryden conectadas en la copia del modelo.');
    else
        disp('Las rafagas de Dryden ya estaban conectadas.');
    end
    % 2) semillas y sigmas parametrizadas (una semilla distinta por eje)
    sysG = [sysW '/Wind Gusts'];
    set_param([sysG '/' sprintf('Band-Limited\nWhite Noise')], 'seed','P.seed_u');
    set_param([sysG '/' sprintf('Band-Limited\nWhite Noise1')],'seed','P.seed_v');
    set_param([sysG '/' sprintf('Band-Limited\nWhite Noise2')],'seed','P.seed_w');
    % 3) modelos corregidos
    if cfg.usar_v2, reemplazar_fym_y_dinamica(mdl); end
    % 4) planificador parametrizado y condicion de paro
    set_param([mdl '/Path Planner/Path Planner'],'MATLABFcn','path_planner_mc(u,P,P.map)');
    set_param([mdl '/Path Manager/' sprintf('Compare\nTo Constant')],'const','P.wp_stop');
    % 5) sin animacion (acelera cada corrida)
    set_param([mdl '/draw environment'],'Commented','on');
    % 6) registro de senales
    set_param(mdl,'SignalLogging','on','SignalLoggingName','logsout');
    sysFM = [mdl '/Controlled MAV/MAV1/Forces & Moments'];
    ph = get_param([sysFM '/Demux'],'PortHandles');
    registrar(ph.Outport(2),'airdata');
    registrar(ph.Outport(3),'corriente');
    registrar(ph.Outport(4),'Vg');
    ph = get_param([sysFM '/delta'],'PortHandles');       registrar(ph.Outport(1),'delta');
    ph = get_param([sysFM '/Integrator1'],'PortHandles'); registrar(ph.Outport(1),'carga_As');
    ph = get_param([sysFM '/Integrator'],'PortHandles');  registrar(ph.Outport(1),'distancia_m');
    save_system(mdl);
end

function registrar(h, nombre)
    set_param(h,'DataLogging','on','DataLoggingNameMode','Custom','DataLoggingName',nombre);
end

function m = calcular_metricas(simOut, P, t_max)
    lg = simOut.logsout;
    I  = lg.get('corriente').Values;
    dl = lg.get('delta').Values;
    ad = lg.get('airdata').Values;
    ti = I.Time;  i_A = squeeze(I.Data);
    dt = interp1(dl.Time, squeeze(dl.Data(:,4)), ti, 'previous', 'extrap');
    Vin = P.V_max*dt;
    Pel = i_A(:).*Vin(:);                      % potencia electrica [W]
    m.t_vuelo_s   = ti(end);
    m.carga_Ah    = trapz(ti, i_A)/3600;
    m.E_Wh        = trapz(ti, Pel)/3600;
    m.P_media_W   = m.E_Wh*3600/m.t_vuelo_s;
    dist          = squeeze(lg.get('distancia_m').Values.Data);
    m.distancia_m = dist(end);
    m.E_por_km_Wh = m.E_Wh/(m.distancia_m/1000);
    da = squeeze(ad.Data);
    if size(da,2) ~= 6, da = da.'; end          % muestras x 6 (Va, alpha, beta, w_n, w_e, w_d)
    m.Va_media    = mean(da(:,1));
    m.wn_media    = mean(da(:,4));
    m.we_media    = mean(da(:,5));
    m.completa    = m.t_vuelo_s < t_max - 1;    % termino por la condicion de paro, no por tiempo
end
