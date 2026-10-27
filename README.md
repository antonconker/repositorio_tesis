# Entorno de simulación integrado para evaluar el consumo energético de UAV de ala fija

Código y datos de la tesis doctoral *«Entorno de simulación integrado para evaluar el consumo energético de aeronaves no tripuladas de ala fija durante el seguimiento de trayectorias logísticas»* (Doctorado en Ciencias del Software, Universidad de Xalapa), de Antonio de Jesús Flores Moreno.

El entorno, desarrollado en MATLAB/Simulink, integra planificación de trayectorias, gestión y seguimiento de trayectorias, piloto automático, dinámica de vuelo de 6 grados de libertad, propulsión eléctrica (motor y hélice), perturbaciones atmosféricas (viento medio y turbulencia de Dryden) y estimación del consumo energético. El caso de estudio es el UAV Aerosonde.

**Video de demostración:** https://youtu.be/Q9u4ppzfUhI

## Estructura

| Carpeta | Contenido |
|---|---|
| `simulador/` | Modelo de Simulink (`mavsim_manager.slx`), parámetros (`param_chap11.m`), dinámica (`mav_dynamics.m`), fuerzas y momentos (`fym.m`), piloto automático, gestión y seguimiento de trayectorias, y visualización. |
| `simulador/resultados_mc_v2/` | Resultados de las 90 simulaciones de la tesis (30 réplicas × 3 escenarios) y el resumen estadístico. |
| `experimento_estadistico/` | Experimento Monte Carlo de las hipótesis H2 y H3 y su análisis estadístico. |
| `validacion_H1/` | Validación del modelo de propulsión (hipótesis H1) con datos de túnel de viento de la UIUC y del banco de pruebas propio. |

## Requisitos

- MATLAB y Simulink (desarrollado en R2025a).
- Statistics and Machine Learning Toolbox (solo para `analisis_estadistico.m` y `validacion_H1.m`).

## Uso

### 1. Simulación de una misión

```matlab
cd simulador
param_chap11            % carga los parámetros en la estructura P
open('mavsim_manager.slx')
% ejecutar la simulación desde Simulink (botón Run)
```

### 2. Experimento Monte Carlo (H2 y H3)

```matlab
cd simulador
addpath('../experimento_estadistico')
experimento_montecarlo   % crea copias de los modelos y simula los escenarios A0, A1 y B0
analisis_estadistico     % pruebas t pareadas, Wilcoxon, IC 95 %, Holm, tamaño de efecto
```

- Los resultados se guardan fila por fila en `simulador/resultados_mc_v2/resultados.csv`. Si la ejecución se interrumpe, al volver a correr el script continúa donde se quedó.
- El repositorio incluye los resultados de la tesis; por eso `experimento_montecarlo` no vuelve a simular nada y `analisis_estadistico` reproduce directamente las tablas. Para repetir el experimento desde cero, cambia `cfg.carpeta` en `experimento_montecarlo.m` y en `analisis_estadistico.m`.
- Cada réplica *i* usa la semilla `1000 + i` en los tres escenarios (números aleatorios comunes), así que los resultados son reproducibles.

### 3. Validación del modelo de propulsión (H1)

```matlab
cd validacion_H1
validacion_H1
```

## Créditos y referencias

- La estructura base del simulador proviene del material de R. W. Beard y T. W. McLain, *Small Unmanned Aircraft: Theory and Practice*, Princeton University Press, 2012 (capítulo 11). Sobre ella se desarrollaron el modelo de propulsión eléctrica y de consumo, las correcciones al modelo, la integración del planificador de rutas y los scripts de experimentación.
- Datos de la hélice APC 20×10E: UIUC Propeller Database, vol. 4 (Dantsker et al., 2022, AIAA 2022-4020, doi:10.2514/6.2022-4020).

## Cómo citar

<!-- Agrega aquí la referencia de la tesis y, si lo archivas en Zenodo, el DOI -->
Flores Moreno, A. J. (2026). *Entorno de simulación integrado para evaluar el consumo energético de aeronaves no tripuladas de ala fija durante el seguimiento de trayectorias logísticas* [Tesis doctoral]. Universidad de Xalapa.
