%% ANALISIS ESTADISTICO DE H2 Y H3 (diseno pareado)
%
% Lee resultados_mc/resultados.csv (generado por experimento_montecarlo.m) y,
% para cada hipotesis y cada indicador, calcula:
%   - diferencia media pareada, desviacion estandar e IC 95 %
%   - diferencia relativa media (%) e IC 95 %
%   - normalidad de las diferencias (Lilliefors)
%   - prueba t pareada unilateral (H1: mu_d > 0) y Wilcoxon de rangos con signo
%   - tamano de efecto d_z de Cohen
%   - prueba de relevancia practica: diferencia relativa > umbral (%)
%   - correccion de Holm para las dos hipotesis (indicador principal E_Wh)
%   - tamano de muestra necesario (si se corre como prueba piloto)
% Requiere Statistics and Machine Learning Toolbox.

clear; clc;
carpeta  = 'resultados_mc_v2';
alpha    = 0.05;
potencia = 0.80;
umbral_pct = 1.0;        % diferencia minima con relevancia practica [%]
indicadores = {'E_Wh','carga_Ah','P_media_W','t_vuelo_s','E_por_km_Wh'};
nombres     = {'Energía (Wh)','Carga (Ah)','Potencia media (W)','Tiempo de vuelo (s)','Energía por km (Wh/km)'};

T = readtable(fullfile(carpeta,'resultados.csv'));
if any(~T.completa)
    warning('%d corridas no terminaron por la condicion de paro; se excluyen.', sum(~T.completa));
    T = T(T.completa==1,:);
end

hip = struct('id',{'H2','H3'}, ...
             'tratamiento',{'A1','A0'}, 'control',{'A0','B0'}, ...
             'desc',{'Viento (A1) vs. sin viento (A0)','Ruta RGPPM (A0) vs. recta equivalente (B0)'});

filas = {};
pvals_principal = zeros(1,numel(hip));
for h = 1:numel(hip)
    Tt = T(strcmp(T.escenario,hip(h).tratamiento),:);
    Tc = T(strcmp(T.escenario,hip(h).control),:);
    [rep, it, ic] = intersect(Tt.replica, Tc.replica);   % emparejar por replica
    n = numel(rep);
    fprintf('\n=== %s: %s  (n = %d pares) ===\n', hip(h).id, hip(h).desc, n);
    for k = 1:numel(indicadores)
        yt = Tt.(indicadores{k})(it);  yc = Tc.(indicadores{k})(ic);
        d  = yt - yc;                  dr = 100*d./yc;
        md = mean(d); sd = std(d); se = sd/sqrt(n); tc = tinv(1-alpha/2, n-1);
        mr = mean(dr); sr = std(dr);
        [~, p_t] = ttest(d, 0, 'Tail','right');
        p_w = signrank(d, 0, 'tail','right');
        if n >= 4, [~, p_norm] = lillietest(d); else, p_norm = NaN; end
        [~, p_rel] = ttest(dr - umbral_pct, 0, 'Tail','right');
        dz = md/sd;
        n_req = ceil(((tinv(1-alpha, max(n-1,1)) + tinv(potencia, max(n-1,1)))*sr/umbral_pct)^2);
        if k == 1, pvals_principal(h) = p_t; end
        fprintf('%-24s media A=%10.3f  B=%10.3f  d=%9.3f [%9.3f, %9.3f]  d%%=%6.2f [%6.2f, %6.2f]  p_t=%.2g  p_W=%.2g  p_norm=%.2g  dz=%.2f  p(d%%>%.1f)=%.2g  n_req=%d\n', ...
            nombres{k}, mean(yt), mean(yc), md, md-tc*se, md+tc*se, mr, mr-tc*sr/sqrt(n), mr+tc*sr/sqrt(n), ...
            p_t, p_w, p_norm, dz, umbral_pct, p_rel, n_req);
        filas(end+1,:) = {hip(h).id, nombres{k}, n, mean(yt), mean(yc), md, md-tc*se, md+tc*se, ...
                          mr, mr-tc*sr/sqrt(n), mr+tc*sr/sqrt(n), p_t, p_w, p_norm, dz, p_rel, n_req}; %#ok<SAGROW>
    end
end

% Holm-Bonferroni sobre el indicador principal (energia) de H2 y H3
[ps, orden] = sort(pvals_principal);
m = numel(ps); p_holm = zeros(size(ps));
for j = 1:m, p_holm(orden(j)) = min(1, max(ps(1:j).*(m-(1:j)+1))); end
fprintf('\nHolm (E_Wh):  H2 p_ajust = %.3g   H3 p_ajust = %.3g\n', p_holm(1), p_holm(2));

R = cell2table(filas, 'VariableNames', {'hipotesis','indicador','n','media_trat','media_ctrl', ...
    'dif_media','IC95_inf','IC95_sup','dif_pct','IC95pct_inf','IC95pct_sup', ...
    'p_t','p_wilcoxon','p_normalidad','d_z','p_relevancia','n_requerido'});
writetable(R, fullfile(carpeta,'resumen_estadistico.csv'));

%% tabla LaTeX (indicador principal y secundarios)
fid = fopen(fullfile(carpeta,'tabla_estadistica.tex'),'w','n','UTF-8');
fprintf(fid,'\\begin{table}[H]\n\\centering\n\\small\n');
fprintf(fid,'\\caption{Resultados de las pruebas pareadas para H2 y H3 ($n=%d$ réplicas, $\\alpha=%.2f$).}\n', n, alpha);
fprintf(fid,'\\label{tab:pruebas_estadisticas}\n');
fprintf(fid,'\\begin{tabular}{l l r r r r}\n\\toprule\n');
fprintf(fid,'Hip. & Indicador & $\\bar d$ [IC 95\\,\\%%] & $\\bar d$ (\\%%) & $p$ ($t$) & $d_z$ \\\\\n\\midrule\n');
for r = 1:height(R)
    fprintf(fid,'%s & %s & %.2f [%.2f, %.2f] & %.2f & %s & %.2f \\\\\n', R.hipotesis{r}, R.indicador{r}, ...
        R.dif_media(r), R.IC95_inf(r), R.IC95_sup(r), R.dif_pct(r), formato_p(R.p_t(r)), R.d_z(r));
end
fprintf(fid,'\\bottomrule\n\\end{tabular}\n\\end{table}\n');
fclose(fid);

%% figura: diferencias relativas pareadas en energia
figure('Color','w','Position',[100 100 700 380]);
for h = 1:numel(hip)
    Tt = T(strcmp(T.escenario,hip(h).tratamiento),:); Tc = T(strcmp(T.escenario,hip(h).control),:);
    [~, it, ic] = intersect(Tt.replica, Tc.replica);
    dr = 100*(Tt.E_Wh(it) - Tc.E_Wh(ic))./Tc.E_Wh(ic);
    subplot(1,2,h); boxplot(dr); hold on;
    plot(1 + 0.08*randn(size(dr)), dr, 'k.', 'MarkerSize', 10);
    yline(umbral_pct,'--','Umbral de relevancia');
    title(sprintf('%s: %s', hip(h).id, hip(h).desc),'FontWeight','normal','FontSize',9);
    ylabel('Diferencia de energía (%)'); set(gca,'XTick',[]); grid on;
end
try
    exportgraphics(gcf, fullfile(carpeta,'diferencias_energia.pdf'),'ContentType','vector');
catch
    print(gcf, fullfile(carpeta,'diferencias_energia.pdf'),'-dpdf','-bestfit');
end
disp('Listo: resumen_estadistico.csv, tabla_estadistica.tex y diferencias_energia.pdf en resultados_mc/');

function s = formato_p(p)
    if p < 0.001, s = '$<0.001$'; else, s = sprintf('%.3f', p); end
end
