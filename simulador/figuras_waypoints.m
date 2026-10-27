%% FIGURAS DE SEGUIMIENTO DE WAYPOINTS PARA LA TESIS (CAPÍTULO 7)
%
% Simula la ruta RGPPM completa con mavsim_manager_video.slx (sin animación)
% y exporta en PDF vectorial, con etiquetas en español:
%   wpRute.pdf   ruta tridimensional
%   wpRute2.pdf  vista en planta con los waypoints
%   wp2.pdf ... wp9.pdf  acercamiento al paso por cada waypoint intermedio
%
% Requisitos: haber corrido preparar_modelo_video (crea el modelo y activa el
% registro de la señal 'estados'). Ejecutar desde la carpeta del simulador.
% Las figuras se guardan en la subcarpeta figuras_tesis.

mdl = 'mavsim_manager_video';
if ~exist([mdl '.slx'], 'file')
    error('No existe %s.slx: ejecuta primero preparar_modelo_video.', mdl);
end
load_system(mdl);

% --- ruta completa y sin animación (solo en esta corrida; no se guarda el modelo)
cmp = find_system(mdl, 'LookUnderMasks', 'all', 'MaskType', 'Compare To Constant');
if isempty(cmp), warning('No se encontró el bloque Compare To Constant; la simulación podría detenerse antes del final.'); end
for k = 1:numel(cmp), set_param(cmp{k}, 'const', '9'); end
vis = find_system(mdl, 'SearchDepth', 1, 'Name', 'Visualización');
for k = 1:numel(vis), set_param(vis{k}, 'Commented', 'on'); end

fprintf('Simulando la ruta completa (puede tardar algunos minutos)...\n');
out = sim(mdl, 'ReturnWorkspaceOutputs', 'on');
close_system(mdl, 0);                           % descarta los cambios de esta corrida

X = out.logsout.get('estados').Values;
x = squeeze(X.Data);  if size(x,1) == 12 && size(x,2) ~= 12, x = x.'; end
pn = x(:,1);  pe = x(:,2);  h = -x(:,3);
fprintf('Tiempo simulado: %.0f s | distancia en planta: %.1f km\n', X.Time(end), ...
        sum(hypot(diff(pn), diff(pe)))/1000);

% --- waypoints de la ruta RGPPM (norte, este), como en path_planner_chap11.m
wp = [    0,               0;
          7.15316868e+02,  3.46107314e+03;
          3.72410150e+03,  1.84141530e+03;
          3.84598537e+03, -1.21462650e+03;
          2.53475639e+03, -5.41696984e+03;
          3.20315333e+02, -6.54094552e+03;
          4.83143200e+03, -7.61794264e+03;
         -3.72650750e+03, -1.66724889e+03;
         -6.32543123e+03,  5.82643639e+03;
         -1.09379176e+03,  6.41052979e+03];
hwp = 200*ones(size(wp,1),1);

carpeta = 'figuras_tesis';
if ~exist(carpeta, 'dir'), mkdir(carpeta); end
azul = [0.16 0.47 0.84];  naranja = [0.92 0.41 0.20];
estilo = @(ax) set(ax, 'FontName', 'Helvetica', 'FontSize', 10, 'Box', 'on');

% --- 1) ruta tridimensional
f = figure('Color', 'w', 'Position', [100 100 760 520]);
plot3(wp(:,2)/1000, wp(:,1)/1000, hwp, '-', 'Color', azul, 'LineWidth', 1.0); hold on;
plot3(pe/1000, pn/1000, h, '-', 'Color', naranja, 'LineWidth', 1.4);
plot3(wp(:,2)/1000, wp(:,1)/1000, hwp, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 4);
for k = 1:size(wp,1)
    text(wp(k,2)/1000, wp(k,1)/1000, hwp(k) + 25, sprintf('WP%d', k), 'FontSize', 9);
end
grid on; view(-35, 30); zlim([0 400]);
xlabel('Este (km)'); ylabel('Norte (km)'); zlabel('Altitud (m)');
legend('Trayectoria deseada', 'Trayectoria de la aeronave', 'Location', 'northeast');
estilo(gca);
exportgraphics(f, fullfile(carpeta, 'wpRute.pdf'), 'ContentType', 'vector');

% --- 2) vista en planta
f = figure('Color', 'w', 'Position', [100 100 760 560]);
plot(wp(:,2)/1000, wp(:,1)/1000, '-', 'Color', azul, 'LineWidth', 1.0); hold on;
plot(pe/1000, pn/1000, '-', 'Color', naranja, 'LineWidth', 1.4);
plot(wp(:,2)/1000, wp(:,1)/1000, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 4);
for k = 1:size(wp,1)
    text(wp(k,2)/1000 + 0.15, wp(k,1)/1000 + 0.25, sprintf('WP%d', k), 'FontSize', 9);
end
axis equal; grid on;
xlabel('Este (km)'); ylabel('Norte (km)');
legend('Trayectoria deseada', 'Trayectoria de la aeronave', 'Location', 'southwest');
estilo(gca);
exportgraphics(f, fullfile(carpeta, 'wpRute2.pdf'), 'ContentType', 'vector');

% --- 3) acercamiento a cada waypoint intermedio
semiancho = 450;                                % m alrededor del waypoint
for k = 2:9
    f = figure('Color', 'w', 'Position', [100 100 520 440]);
    plot(wp(:,2), wp(:,1), '-', 'Color', azul, 'LineWidth', 1.0); hold on;
    plot(pe, pn, '-', 'Color', naranja, 'LineWidth', 1.4);
    plot(wp(k,2), wp(k,1), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 5);
    text(wp(k,2) + 25, wp(k,1) + 25, sprintf('WP%d', k), 'FontSize', 10);
    axis equal; grid on;
    xlim(wp(k,2) + [-1 1]*semiancho);  ylim(wp(k,1) + [-1 1]*semiancho);
    xlabel('Este (m)'); ylabel('Norte (m)');
    legend('Trayectoria deseada', 'Trayectoria de la aeronave', 'Location', 'best');
    estilo(gca);
    exportgraphics(f, fullfile(carpeta, sprintf('wp%d.pdf', k)), 'ContentType', 'vector');
end
fprintf('Listo: figuras en la carpeta %s\n', fullfile(pwd, carpeta));
