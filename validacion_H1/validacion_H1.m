%% VALIDACION DEL MODELO DE PROPULSION (H1)
% Compara el modelo cuadratico C_T(J), C_P(J) del simulador con datos
% experimentales independientes de la helice del caso de estudio:
%   APC Thin Electric 20x10 (20x10E), UIUC Propeller Data Site, Vol. 4
%   (Dantsker, Caccamo, Deters & Selig, 2022). Archivo: uiuc_apce_20x10.csv
% Criterio de aceptacion (definido antes del analisis): NRMSE <= 10 % en
% validacion cruzada dejando fuera una corrida de RPM completa.
% Compatible con MATLAB y Octave.

clear; clc;
d   = csvread('uiuc_apce_20x10.csv', 1, 0);   % rpm, J, CT, CP
rpm = d(:,1); J = d(:,2); CT = d(:,3); CP = d(:,4);
eps_acept = 0.10;
sel = rpm >= 2979;                 % corridas con Reynolds cercano a la operacion
R   = unique(rpm(sel));

% --- 1) coeficientes del libro (Beard & McLain) frente a UIUC
CT_lib = @(x) 0.09357 - 0.06044*x - 0.1079*x.^2;
CP_lib = @(x) 2*pi*(0.005230 + 0.004970*x - 0.01664*x.^2);
fprintf('Coeficientes de Beard & McLain frente a UIUC (corridas >= 2979 rpm):\n');
reporta('C_T', CT(sel), CT_lib(J(sel)));
reporta('C_P', CP(sel), CP_lib(J(sel)));

% --- 2) validacion cruzada del ajuste cuadratico con datos UIUC
fprintf('\nValidacion cruzada (se deja fuera una corrida de RPM completa):\n');
nombres = {'C_T','C_P'}; Y = {CT, CP}; acepta = true;
for k = 1:2
    y = Y{k}; e = [];
    for r = R'
        tr = sel & rpm ~= r;  te = rpm == r;
        c  = polyfit(J(tr), y(tr), 2);
        e  = [e; polyval(c, J(te)) - y(te)]; %#ok<AGROW>
    end
    yy = y(sel); rmse = sqrt(mean(e.^2)); nrmse = rmse/(max(yy)-min(yy));
    r2 = 1 - sum(e.^2)/sum((yy-mean(yy)).^2);
    fprintf('  %s: RMSE = %.5f, NRMSE = %.1f %%, R^2 = %.3f  ->  %s\n', nombres{k}, rmse, 100*nrmse, r2, ...
            ternario(nrmse <= eps_acept, 'cumple', 'NO cumple'));
    acepta = acepta && nrmse <= eps_acept;
end

% --- 3) coeficientes finales (todas las corridas >= 2979 rpm)
cT = polyfit(J(sel), CT(sel), 2);  cP = polyfit(J(sel), CP(sel), 2);  cQ = cP/(2*pi);
fprintf('\nCoeficientes para param_chap11.m:\n');
fprintf('P.C_T2 = %.6f; P.C_T1 = %.6f; P.C_T0 = %.6f;\n', cT);
fprintf('P.C_Q2 = %.6f; P.C_Q1 = %.6f; P.C_Q0 = %.6f;\n', cQ);
fprintf('\nVeredicto H1 (NRMSE <= %.0f %%): %s\n', 100*eps_acept, ternario(acepta,'ACEPTADA','RECHAZADA'));


%% ---------------- PARTE B: banco de pruebas propio (helice del Bixler 3) ----------------
% Verifica el procedimiento de caracterizacion C_T(J) con mediciones propias
% (tunel de viento del laboratorio, tres velocidades de flujo).
b  = csvread('banco_bixler3.csv', 1, 0);   % esc, rpm, V, T, rho, J, CT
b  = b(b(:,4) > 0, :);                     % se excluye el punto con empuje negativo (molinete)
Jb = b(:,6); CTb = b(:,7);
cb = polyfit(Jb, CTb, 2); eb = polyval(cb, Jb) - CTb;
fprintf('\nBanco propio (helice Bixler 3, D = %.3f m): %d puntos, J = %.2f-%.2f\n', ...
        mean(b(:,3)./(b(:,2)/60.*b(:,6))), numel(Jb), min(Jb), max(Jb));
fprintf('  Ajuste C_T = %.5f J^2 %+.5f J %+.5f\n', cb);
fprintf('  Ajuste completo: NRMSE = %.1f %%, R^2 = %.3f\n', 100*sqrt(mean(eb.^2))/(max(CTb)-min(CTb)), ...
        1 - sum(eb.^2)/sum((CTb-mean(CTb)).^2));
% validacion cruzada en 5 particiones (semilla fija)
rand('seed', 1); idx = randperm(numel(Jb)); k = 5; e5 = zeros(size(Jb));
for f = 1:k
    te = idx(f:k:end); tr = setdiff(idx, te);
    cc = polyfit(Jb(tr), CTb(tr), 2); e5(te) = polyval(cc, Jb(te)) - CTb(te);
end
nb = sqrt(mean(e5.^2))/(max(CTb)-min(CTb));
fprintf('  Validacion cruzada (5 particiones): NRMSE = %.1f %%  ->  %s\n', 100*nb, ternario(nb <= eps_acept,'cumple','NO cumple'));
% repetibilidad de las mediciones repetidas (mismo flujo y mismo ESC)
[g, ~, id] = unique(b(:,[3 1]), 'rows'); cvs = [];
for q = 1:size(g,1)
    v = CTb(id == q); if numel(v) > 1, cvs(end+1) = std(v)/mean(v); end %#ok<AGROW>
end
fprintf('  Repetibilidad: coeficiente de variacion medio de C_T = %.2f %% (%d grupos repetidos)\n', 100*mean(cvs), numel(cvs));

% --- 4) figura
Jg = linspace(0.1, 0.68, 200);
figure('Color','w','Position',[100 100 900 360]);
subplot(1,2,1); hold on; grid on;
plot(J(sel), CT(sel), 'k.', 'MarkerSize', 8);
plot(Jg, polyval(cT,Jg), 'b-', 'LineWidth', 1.5);
plot(Jg, CT_lib(Jg), 'r--', 'LineWidth', 1.2);
xlabel('Número de avance J'); ylabel('Coeficiente de empuje C_T');
legend({'UIUC (experimental)','Ajuste cuadrático (simulador)','Beard & McLain'},'Location','southwest');
subplot(1,2,2); hold on; grid on;
plot(J(sel), CP(sel), 'k.', 'MarkerSize', 8);
plot(Jg, polyval(cP,Jg), 'b-', 'LineWidth', 1.5);
plot(Jg, CP_lib(Jg), 'r--', 'LineWidth', 1.2);
xlabel('Número de avance J'); ylabel('Coeficiente de potencia C_P');
print(gcf, 'validacion_H1.pdf', '-dpdf');

figure('Color','w','Position',[100 100 450 360]); hold on; grid on;
V = unique(b(:,3)); mk = {'o','s','^'};
for q = 1:numel(V)
    m = b(:,3) == V(q); plot(Jb(m), CTb(m), mk{q}, 'MarkerSize', 5);
end
plot(Jg, polyval(cb,Jg), 'b-', 'LineWidth', 1.5);
xlabel('Número de avance J'); ylabel('Coeficiente de empuje C_T');
legend([arrayfun(@(v) sprintf('V = %.1f m/s', v), V, 'UniformOutput', false); {'Ajuste cuadrático'}], 'Location','southwest');
print(gcf, 'validacion_banco_propio.pdf', '-dpdf');

function reporta(nombre, y, yh)
    e = yh - y; rmse = sqrt(mean(e.^2));
    fprintf('  %s: RMSE = %.5f, NRMSE = %.1f %%, sesgo = %+.5f\n', nombre, rmse, 100*rmse/(max(y)-min(y)), mean(e));
end
function s = ternario(c, a, b)
    if c, s = a; else, s = b; end
end
