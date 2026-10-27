%% DIAGNÓSTICO DE LA CORRIENTE, LA POTENCIA Y LA ENERGÍA
%
% Ejecutar después de simular mavsim_manager_video.slx (Run), en lugar de
% graficas_video o además de él. Comprueba:
%   1) con qué configuración de propulsión corrió el modelo (P en el espacio base);
%   2) qué valores tienen el acelerador, la velocidad y la corriente registrados;
%   3) si la corriente registrada coincide con la que se obtiene al aplicar
%      las ecuaciones de fym.m a la velocidad y al acelerador registrados.

lg = [];
try, lg = evalin('base', 'out.logsout'); catch, end
if isempty(lg), try, lg = evalin('base', 'logsout'); catch, end, end
if isempty(lg), error('No hay datos registrados: corre preparar_modelo_video y luego Run.'); end
P = evalin('base', 'P');

% --- 1) configuración
fprintf('\n===== 1) Configuración de P en el espacio de trabajo =====\n');
if isfield(P,'modelo_empuje'),    fprintf('P.modelo_empuje    = %s\n', P.modelo_empuje);
else,                             fprintf('P.modelo_empuje    = (no existe -> fym usa ''original'')\n'); end
if isfield(P,'modelo_corriente'), fprintf('P.modelo_corriente = %s\n', P.modelo_corriente);
else,                             fprintf('P.modelo_corriente = (no existe -> fym usa ''motor'')\n'); end
fprintf('delta_t de trim    = %.3f\n', P.u_trim(4));
fprintf('V_max = %.1f V | KQ = %.4f N m/A | R = %.3f ohm | i0 = %.2f A | D = %.3f m\n', ...
        P.V_max, P.KQ, P.R_motor, P.i0, P.D_prop);
fprintf('C_T = [%.4f %.4f %.4f] | C_Q = [%.5f %.5f %.5f]\n', P.C_T0, P.C_T1, P.C_T2, P.C_Q0, P.C_Q1, P.C_Q2);

% --- 2) señales registradas
I  = lg.get('corriente').Values;  ti = I.Time;  iA = double(squeeze(I.Data));  iA = iA(:);
D  = lg.get('delta').Values;  dd = double(squeeze(D.Data));
if size(dd,1) == 4 && size(dd,2) ~= 4, dd = dd.'; end
A  = lg.get('airdata').Values;  aa = double(squeeze(A.Data));
if size(aa,1) == 6 && size(aa,2) ~= 6, aa = aa.'; end
dt = interp1(D.Time, dd(:,4), ti, 'previous', 'extrap');
Va = interp1(A.Time, aa(:,1), ti, 'linear',   'extrap');

fprintf('\n===== 2) Señales registradas =====\n');
fprintf('Duración: %.1f s, %d muestras de corriente\n', ti(end), numel(ti));
fprintf('delta_t: mín %.3f | mediana %.3f | máx %.3f\n', min(dt), median(dt), max(dt));
fprintf('Va:      mín %.1f | mediana %.1f | máx %.1f m/s\n', min(Va), median(Va), max(Va));
fprintf('I:       mín %.2f | mediana %.2f | máx %.2f A | muestras con I = 0: %.1f %%\n', ...
        min(iA), median(iA), max(iA), 100*mean(iA <= 0));
fprintf('Primeras muestras (t, delta_t, Va, I):\n');
disp([ti(1:min(8,end)), dt(1:min(8,end)), Va(1:min(8,end)), iA(1:min(8,end))]);

% --- 3) corriente recalculada con las ecuaciones de fym.m
Dp = P.D_prop;  Vin = P.V_max*dt;
a = P.rho*Dp^5*P.C_Q0/(2*pi)^2;
b = P.rho*Dp^4*P.C_Q1*Va/(2*pi) + P.KQ^2/P.R_motor;
c = P.rho*Dp^3*P.C_Q2*Va.^2 - (P.KQ/P.R_motor)*Vin + P.KQ*P.i0;
Om = max((-b + sqrt(max(b.^2 - 4*a*c, 0)))/(2*a), 0);
Ir = max((Vin - P.KQ*Om)/P.R_motor, 0);
J  = 2*pi*Va./max(Om*Dp, 1e-6);
fprintf('\n===== 3) Corriente recalculada con fym.m =====\n');
fprintf('I recalculada: mediana %.2f A | Omega mediana %.0f rpm | J mediana %.2f\n', ...
        median(Ir), median(Om)*60/(2*pi), median(J(Om > 1)));
fprintf('Diferencia media |I registrada - I recalculada| = %.3f A\n', mean(abs(iA - Ir)));

Pel = iA.*Vin;   E = trapz(ti, Pel)/3600;
Pr  = Ir.*Vin;   Er = trapz(ti, Pr)/3600;
fprintf('Energía con I registrada: %.2f Wh | con I recalculada: %.2f Wh\n', E, Er);

figure('Name','Diagnóstico de energía','Color','w','Position',[100 100 1200 650]);
subplot(2,2,1); plot(ti/60, dt, 'LineWidth', 1.2); grid on; xlabel('Tiempo (min)'); ylabel('\delta_t'); title('Acelerador registrado');
subplot(2,2,2); plot(ti/60, Va, 'LineWidth', 1.2); grid on; xlabel('Tiempo (min)'); ylabel('V_a (m/s)'); title('Velocidad aerodinámica');
subplot(2,2,3); plot(ti/60, iA, 'LineWidth', 1.2); hold on; plot(ti/60, Ir, '--', 'LineWidth', 1.2); grid on;
legend('Registrada', 'Recalculada con fym.m'); xlabel('Tiempo (min)'); ylabel('Corriente (A)'); title('Corriente del motor');
ys = sort(Ir(isfinite(Ir))); if numel(ys) > 10 && ys(round(0.995*end)) > 0, ylim([0, 1.2*ys(round(0.995*end))]); end
subplot(2,2,4); plot(ti/60, Om*60/(2*pi), 'LineWidth', 1.2); grid on; xlabel('Tiempo (min)'); ylabel('\Omega (rpm)'); title('Velocidad de la hélice');
