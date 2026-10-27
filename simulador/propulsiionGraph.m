% --- 1. Definir los Parámetros de la Hélice ---

% Coeficientes de ajuste para Empuje (Thrust)
P.C_T2 = -0.1079;
P.C_T1 = -0.06044;
P.C_T0 = 0.09357;

% Coeficientes de ajuste para Torque
P.C_Q2 = -0.01664;
P.C_Q1 = 0.004970;
P.C_Q0 = 0.005230;

% --- 2. Crear el Dominio (Ratio de Avance, J) ---
J = linspace(0, 0.7, 100);

% --- 3. Calcular C_T y C_Q ---
CT = P.C_T2 * J.^2 + P.C_T1 * J + P.C_T0;
CQ = P.C_Q2 * J.^2 + P.C_Q1 * J + P.C_Q0;

% --- 4. Crear textos de los polinomios ---
CT_text = sprintf('C_T = %.4fJ^2 %+0.4fJ %+0.4f', ...
                  P.C_T2, P.C_T1, P.C_T0);

CQ_text = sprintf('C_Q = %.5fJ^2 %+0.5fJ %+0.5f', ...
                  P.C_Q2, P.C_Q1, P.C_Q0);

% --- 5. Graficar con Dos Ejes Y ---
figure;

% Ajustar tamaño de la ventana
set(gcf, 'Color', 'w');

hold on;

% ==========================================================
% EJE IZQUIERDO -> C_T
% ==========================================================
yyaxis left;

plot(J, CT, ...
    'b-', ...
    'LineWidth', 4, ...
    'DisplayName', CT_text);

ylabel('C_T (Coeficiente de Empuje)', ...
       'FontSize', 15, ...
       'FontWeight', 'bold');

% Color del eje izquierdo
ax = gca;
ax.YColor = 'b';

% ==========================================================
% EJE DERECHO -> C_Q
% ==========================================================
yyaxis right;

plot(J, CQ, ...
    'r--', ...
    'LineWidth', 4, ...
    'DisplayName', CQ_text);

ylabel('C_Q (Coeficiente de Torque)', ...
       'FontSize', 15, ...
       'FontWeight', 'bold');

% Color del eje derecho
ax = gca;
ax.YColor = 'r';

hold off;

% ==========================================================
% 6. TÍTULOS Y FORMATO GENERAL
% ==========================================================
title('Curvas de Rendimiento de la Hélice (C_T y C_Q vs. J)', ...
      'FontSize', 18, ...
      'FontWeight', 'bold');

xlabel('Ratio de Avance (J)', ...
       'FontSize', 16, ...
       'FontWeight', 'bold');

% Leyenda con ecuaciones
legend('show', ...
       'Location', 'NorthEast', ...
       'FontSize', 12);

% Mejorar apariencia de ejes
set(gca, ...
    'FontSize', 14, ...
    'FontWeight', 'bold', ...
    'LineWidth', 2);

grid on;
box on;

% Límites del eje X
xlim([0, max(J)]);

% ==========================================================
% 7. Mostrar ecuaciones en consola
% ==========================================================
disp('Polinomio de Empuje:');
disp(CT_text);

disp('Polinomio de Torque:');
disp(CQ_text);