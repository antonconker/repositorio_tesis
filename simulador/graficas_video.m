%% GRÁFICAS DE ESTADOS Y ENERGÍA PARA EL VIDEO
%
% Ejecutar después de simular mavsim_manager_video.slx (botón Run).
% Usa las señales que registra preparar_modelo_video.m y calcula la energía
% igual que experimento_montecarlo.m: P = I * V_max * delta_t.

% --- datos registrados (out.logsout en versiones recientes, logsout en antiguas)
lg = [];
try, lg = evalin('base', 'out.logsout'); catch, end
if isempty(lg)
    try, lg = evalin('base', 'logsout'); catch, end
end
if isempty(lg)
    error('No hay datos registrados. Corre primero preparar_modelo_video y luego la simulación.');
end
P = evalin('base', 'P');

% --- estados de la aeronave: [pn pe pd u v w phi theta psi p q r]
X  = lg.get('estados').Values;  tx = X.Time;  x = squeeze(X.Data);
if size(x,1) == 12 && size(x,2) ~= 12, x = x.'; end
pn = x(:,1); pe = x(:,2); h = -x(:,3);
phi = rad2deg(x(:,7)); theta = rad2deg(x(:,8)); psi = rad2deg(x(:,9));

% --- datos de aire: [Va alpha beta wn we wd]
A  = lg.get('airdata').Values;  ta = A.Time;  a = squeeze(A.Data);
if size(a,1) == 6 && size(a,2) ~= 6, a = a.'; end
Va = a(:,1);  viento = hypot(a(:,4), a(:,5));

% --- energía
I  = lg.get('corriente').Values;  ti = I.Time;  iA = squeeze(I.Data);  iA = iA(:);
D  = lg.get('delta').Values;  dd = squeeze(D.Data);
if size(dd,1) == 4 && size(dd,2) ~= 4, dd = dd.'; end
dt  = interp1(D.Time, dd(:,4), ti, 'previous', 'extrap');
Pel = iA .* (P.V_max * dt(:));                       % potencia eléctrica [W]
E   = cumtrapz(ti, Pel) / 3600;                      % energía acumulada [Wh]
dist = squeeze(lg.get('distancia_m').Values.Data);  dist = dist(:);

% --- diagnóstico de las señales eléctricas
fprintf('Corriente: %d muestras, %d no finitas, mín %.1f A, mediana %.1f A, máx %.1f A\n', ...
    numel(iA), sum(~isfinite(iA)), min(iA), median(iA,'omitnan'), max(iA));
fprintf('Potencia:  %d muestras, %d no finitas, mín %.1f W, mediana %.1f W, máx %.1f W\n', ...
    numel(Pel), sum(~isfinite(Pel)), min(Pel), median(Pel,'omitnan'), max(Pel));

% --- figura 1: trayectoria y estados
figure('Name','Estados de la aeronave','Color','w','Position',[80 80 1200 650]);
subplot(2,3,1); plot(pe/1000, pn/1000, 'LineWidth', 1.5); axis equal; grid on;
xlabel('Este (km)'); ylabel('Norte (km)'); title('Trayectoria recorrida');
subplot(2,3,2); plot(tx/60, h, 'LineWidth', 1.5); grid on;
xlabel('Tiempo (min)'); ylabel('Altitud (m)'); title('Altitud');
subplot(2,3,3); plot(ta/60, Va, 'LineWidth', 1.5); grid on;
xlabel('Tiempo (min)'); ylabel('V_a (m/s)'); title('Velocidad aerodinámica');
subplot(2,3,4); plot(tx/60, phi, 'LineWidth', 1.2); grid on;
xlabel('Tiempo (min)'); ylabel('\phi (°)'); title('Ángulo de alabeo');
subplot(2,3,5); plot(tx/60, theta, 'LineWidth', 1.2); grid on;
xlabel('Tiempo (min)'); ylabel('\theta (°)'); title('Ángulo de cabeceo');
subplot(2,3,6); plot(tx/60, psi, 'LineWidth', 1.2); grid on;
xlabel('Tiempo (min)'); ylabel('\psi (°)'); title('Ángulo de guiñada');

% --- figura 2: propulsión y energía
figure('Name','Consumo energético','Color','w','Position',[120 120 1200 650]);
subplot(2,2,1); plot(ti/60, 100*dt, 'LineWidth', 1.2); grid on;
xlabel('Tiempo (min)'); ylabel('\delta_t (%)'); title('Posición del acelerador');
subplot(2,2,2); plot(ti/60, iA, 'LineWidth', 1.2); grid on; ajustar_ylim(iA);
xlabel('Tiempo (min)'); ylabel('Corriente (A)'); title('Corriente del motor');
subplot(2,2,3); plot(ti/60, Pel, 'LineWidth', 1.2); grid on; ajustar_ylim(Pel);
xlabel('Tiempo (min)'); ylabel('Potencia (W)'); title('Potencia eléctrica');
subplot(2,2,4); plot(ti/60, E, 'LineWidth', 2); grid on;
xlabel('Tiempo (min)'); ylabel('Energía (Wh)'); title('Energía consumida acumulada');

% --- resumen en la ventana de comandos
fprintf('\n===== Resumen de la misión =====\n');
fprintf('Tiempo de vuelo:      %8.1f min\n', ti(end)/60);
fprintf('Distancia recorrida:  %8.2f km\n', dist(end)/1000);
fprintf('Energía consumida:    %8.1f Wh\n', E(end));
fprintf('Carga consumida:      %8.2f Ah\n', trapz(ti, iA)/3600);
fprintf('Potencia media:       %8.1f W\n', E(end)*3600/ti(end));
fprintf('Energía por km:       %8.2f Wh/km\n', E(end)/(dist(end)/1000));
fprintf('Velocidad del viento: %8.1f m/s (media)\n', mean(viento));

function ajustar_ylim(y)
% Escala el eje vertical con los percentiles 0.5-99.5 para que un pico
% transitorio (por ejemplo, el arranque del motor en t = 0) no aplaste la curva.
    y = y(isfinite(y));
    if isempty(y), return; end
    ys = sort(y);  n = numel(ys);
    lo = ys(max(1, round(0.005*n)));  hi = ys(min(n, round(0.995*n)));
    if hi > lo
        m = 0.1*(hi - lo);  ylim([lo - m, hi + m]);
    end
end
