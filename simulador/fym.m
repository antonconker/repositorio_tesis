%   FYM_V2  Fuerzas, momentos y consumo electrico (version corregida de fym.m)
%
%   Cambios respecto a fym.m (el original no se modifica):
%     1. Arrastre inducido con el alargamiento real AR = b^2/S (antes AR = 2 fijo)
%        y coeficiente de arrastre parasito C_D_p, como en Beard & McLain.
%     2. Voltaje de entrada Vin = P.V_max*delta_t (antes 11.1*delta_t, que no
%        corresponde a la bateria de P.ncells celdas).
%     3. Corriente obtenida del modelo del motor, I = (Vin - K_Q*Omega)/R,
%        coherente con la ecuacion usada para calcular Omega (opcion 'motor').
%        La opcion 'polinomio' conserva el calculo original con ef_prop.
%     4. Opcion de calcular el empuje con el modelo de helice C_T(J) ('helice'),
%        coherente con el torque y la corriente. Por defecto se conserva el
%        empuje original ('original'), porque cambiarlo exige reajustar el trim
%        y las ganancias del lazo de velocidad.
%
%   Salida: misma estructura que fym.m, para no modificar el Demux [6;6;1;1]:
%       out = [F(3); M(3); Va; alpha; beta; w_n; w_e; w_d; I_motor; Vg]

function out = fym(x, delta, wind, P)

    if isfield(P,'modelo_empuje'),   modelo_empuje   = P.modelo_empuje;   else, modelo_empuje   = 'original'; end
    if isfield(P,'modelo_corriente'), modelo_corriente = P.modelo_corriente; else, modelo_corriente = 'motor';    end

    u = x(4); v = x(5); w = x(6);
    phi = x(7); theta = x(8); psi = x(9);
    p = x(10); q = x(11); r = x(12);
    delta_e = delta(1); delta_a = delta(2); delta_r = delta(3); delta_t = delta(4);
    w_ns = wind(1); w_es = wind(2); w_ds = wind(3);
    u_wg = wind(4); v_wg = wind(5); w_wg = wind(6);

    % rotacion inercial -> cuerpo
    R = [1, 0, 0; 0, cos(phi), sin(phi); 0, -sin(phi), cos(phi)] * ...
        [cos(theta), 0, -sin(theta); 0, 1, 0; sin(theta), 0, cos(theta)] * ...
        [cos(psi), sin(psi), 0; -sin(psi), cos(psi), 0; 0, 0, 1];

    % viento en el marco del cuerpo (estacionario + rafagas)
    u_w = u_wg + R(1,:)*[w_ns; w_es; w_ds];
    v_w = v_wg + R(2,:)*[w_ns; w_es; w_ds];
    w_w = w_wg + R(3,:)*[w_ns; w_es; w_ds];
    % viento en el marco inercial NED
    w_n = w_ns + R(:,1)'*[u_wg; v_wg; w_wg];
    w_e = w_es + R(:,2)'*[u_wg; v_wg; w_wg];
    w_d = w_ds + R(:,3)'*[u_wg; v_wg; w_wg];

    % velocidad relativa al aire
    ur = u - u_w;  vr = v - v_w;  wr = w - w_w;
    Va    = sqrt(ur^2 + vr^2 + wr^2);
    alpha = atan2(wr, ur);
    beta  = atan2(vr, sqrt(ur^2 + wr^2));
    qbar  = 0.5*P.rho*Va^2;
    ca = cos(alpha); sa = sin(alpha);

    % gravedad
    Force = zeros(3,1);
    Force(1) = -P.mass*P.gravity*sin(theta);
    Force(2) =  P.mass*P.gravity*cos(theta)*sin(phi);
    Force(3) =  P.mass*P.gravity*cos(theta)*cos(phi);

    % sustentacion y arrastre (modelo sigmoidal de Beard & McLain)
    tmp1  = exp(-P.M*(alpha - P.alpha0));
    tmp2  = exp( P.M*(alpha + P.alpha0));
    sigma = (1 + tmp1 + tmp2)/((1 + tmp1)*(1 + tmp2));
    CL = (1 - sigma)*(P.C_L_0 + P.C_L_alpha*alpha) + sigma*2*sign(alpha)*sa*sa*ca;
    AR = P.b^2/P.S_wing;                                        % CORRECCION 1
    CD = P.C_D_p + (P.C_L_0 + P.C_L_alpha*alpha)^2/(pi*P.e*AR);  % CORRECCION 1

    Force(1) = Force(1) + qbar*P.S_wing*(-CD*ca + CL*sa);
    Force(1) = Force(1) + qbar*P.S_wing*(-P.C_D_q*ca + P.C_L_q*sa)*P.c*q/(2*Va);
    Force(2) = Force(2) + qbar*P.S_wing*(P.C_Y_0 + P.C_Y_beta*beta);
    Force(2) = Force(2) + qbar*P.S_wing*(P.C_Y_p*p + P.C_Y_r*r)*P.b/(2*Va);
    Force(3) = Force(3) + qbar*P.S_wing*(-CD*sa - CL*ca);
    Force(3) = Force(3) + qbar*P.S_wing*(-P.C_D_q*sa - P.C_L_q*ca)*P.c*q/(2*Va);

    % superficies de control
    Force(1) = Force(1) + qbar*P.S_wing*(-P.C_D_delta_e*ca + P.C_L_delta_e*sa)*delta_e;
    Force(2) = Force(2) + qbar*P.S_wing*(P.C_Y_delta_a*delta_a + P.C_Y_delta_r*delta_r);
    Force(3) = Force(3) + qbar*P.S_wing*(-P.C_D_delta_e*sa - P.C_L_delta_e*ca)*delta_e;

    % momentos aerodinamicos
    Torque = zeros(3,1);
    Torque(1) = qbar*P.S_wing*P.b*(P.C_ell_0 + P.C_ell_beta*beta) ...
              + qbar*P.S_wing*P.b*(P.C_ell_p*p + P.C_ell_r*r)*P.b/(2*Va);
    Torque(2) = qbar*P.S_wing*P.c*(P.C_M_0 + P.C_M_alpha*alpha) ...
              + qbar*P.S_wing*P.c*P.C_M_q*P.c*q/(2*Va);
    Torque(3) = qbar*P.S_wing*P.b*(P.C_n_0 + P.C_n_beta*beta) ...
              + qbar*P.S_wing*P.b*(P.C_n_p*p + P.C_n_r*r)*P.b/(2*Va);
    Torque(1) = Torque(1) + qbar*P.S_wing*P.b*(P.C_ell_delta_a*delta_a + P.C_ell_delta_r*delta_r);
    Torque(2) = Torque(2) + qbar*P.S_wing*P.c*P.C_M_delta_e*delta_e;
    Torque(3) = Torque(3) + qbar*P.S_wing*P.b*(P.C_n_delta_a*delta_a + P.C_n_delta_r*delta_r);

    % ---------------- sistema de propulsion ----------------
    Vin = P.V_max*delta_t;                                        % CORRECCION 2
    D   = P.D_prop;
    a = P.rho*D^5*P.C_Q0/(2*pi)^2;
    b = P.rho*D^4*P.C_Q1*Va/(2*pi) + P.KQ^2/P.R_motor;
    c = P.rho*D^3*P.C_Q2*Va^2 - (P.KQ/P.R_motor)*Vin + P.KQ*P.i0;
    disc    = max(b^2 - 4*a*c, 0);
    Omega_p = max((-b + sqrt(disc))/(2*a), 0);                   % rad/s
    if Omega_p > 1e-3
        J   = 2*pi*Va/(Omega_p*D);
        C_T = P.C_T2*J^2 + P.C_T1*J + P.C_T0;
        C_Q = P.C_Q2*J^2 + P.C_Q1*J + P.C_Q0;
    else
        C_T = 0; C_Q = 0;
    end
    T_p = P.rho*D^4*Omega_p^2*C_T/(2*pi)^2;                      % empuje [N]
    Q_p = P.rho*D^5*Omega_p^2*C_Q/(2*pi)^2;                      % torque [N m]

    switch modelo_empuje                                          % CORRECCION 4
        case 'helice'
            Force(1) = Force(1) + T_p;
        otherwise   % 'original' (Beard & McLain, 1a ed.)
            Force(1) = Force(1) + 0.5*P.rho*P.S_prop*P.C_prop*(P.k_motor^2*delta_t^2 - Va^2);
    end
    Torque(1) = Torque(1) - Q_p;

    switch modelo_corriente                                       % CORRECCION 3
        case 'polinomio'
            ef_prop = -1E-6*Omega_p^2 + 0.0022*Omega_p - 0.3854;
            Pot = Q_p*Omega_p/max(ef_prop, 1e-3);
            I_motor = Pot/max(Vin, 1e-3);
        otherwise   % 'motor'
            I_motor = max((Vin - P.KQ*Omega_p)/P.R_motor, 0);    % sin regeneracion
    end

    Vg = sqrt(u^2 + v^2 + w^2);
    out = [Force; Torque; Va; alpha; beta; w_n; w_e; w_d; I_motor; Vg];
end