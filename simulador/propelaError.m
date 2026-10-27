% Datos experimentales
J  = [0.257544 0.271600 0.288206 0.306338 0.321786 0.338747 0.356564 0.372573 0.388613 0.404398 ...
      0.418644 0.433247 0.447601 0.463198 0.479032 0.496369 0.513081 0.529811 0.545398 0.569711 ...
      0.582643 0.598384 0.613118 0.636422];

CT = [0.085100 0.083269 0.081441 0.079450 0.076397 0.074090 0.071641 0.068948 0.066418 0.063626 ...
      0.060887 0.058136 0.054974 0.051048 0.047177 0.043292 0.038986 0.034816 0.030820 0.024491 ...
      0.021307 0.016585 0.013616 0.008203];

% Coeficientes del polinomio de empuje
CT2 = -0.1079;
CT1 = -0.06044;
CT0 = 0.09357;

% Modelo polinómico
CT_fit = CT2*J.^2 + CT1*J + CT0;

% Calcular error RMS (raíz del error cuadrático medio)
error_RMS = sqrt(mean((CT - CT_fit).^2));

% Crear texto del polinomio para la leyenda
polinomio_txt = sprintf('C_T = %.4fJ^2 %+0.4fJ %+0.4f', CT2, CT1, CT0);

% Graficar
figure;

% Datos experimentales
plot(J, CT, 'o', ...
    'MarkerFaceColor', 'b', ...
    'MarkerEdgeColor', 'k', ...
    'MarkerSize', 8, ...
    'LineWidth', 2);
hold on;

% Ajuste polinómico
plot(J, CT_fit, 'r-', ...
    'LineWidth', 3);

% Etiquetas
xlabel('J', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('C_T', 'FontSize', 14, 'FontWeight', 'bold');

% Leyenda con ecuación del polinomio
legend('Datos proveedor', ...
       ['Ajuste polinómico: ', polinomio_txt], ...
       'Location', 'best', ...
       'FontSize', 11);

% Título
title(['Ajuste de C_T vs J   |   Error RMS = ', ...
       num2str(error_RMS, '%.4e')], ...
       'FontSize', 14, ...
       'FontWeight', 'bold');

% Configuración de ejes
set(gca, 'LineWidth', 1.5, ...
         'FontSize', 12, ...
         'FontWeight', 'bold');

grid on;
box on;

% Mostrar error en consola
disp(['Error RMS = ', num2str(error_RMS)]);
disp(['Polinomio ajustado: ', polinomio_txt]);