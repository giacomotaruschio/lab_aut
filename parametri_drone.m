function p = parametri_drone()
% PARAMETRI_DRONE Costanti fisiche calibrate sul banco e parametri migliorati

%% Costanti fisiche del telaio (da ESC.h e Relazione Sez. 1.3.2)
p.g = 9.81;                 % Accelerazione di gravita' [m/s^2]
p.l = 0.30;                 % Distanza motore-baricentro [m]
p.b = 0.00001163;           % Coefficiente di spinta [N*s^2/rad^2]
p.d = 0.00008;              % Coefficiente di resistenza aerodinamica

% Parametri inerziali del banco di prova ridotto (calibrati sui test reali)
p.m       = 1.15;           % Massa equivalente [kg]
p.Ixx     = 0.065;          % Momento di inerzia asse roll sul supporto [kg*m^2]
p.Iyy     = 0.070;          % Momento di inerzia asse pitch sul supporto [kg*m^2]
p.c_phi   = 0.080;          % Attrito viscoso supporto roll [N*m*s/rad]
p.c_theta = 0.095;          % Attrito viscoso supporto pitch [N*m*s/rad]
p.eff_M1  = 0.985;          % Asimmetria Motore 1 (~1.5%, come da plot hovering)
p.tau_m   = 0.030;          % Costante di tempo attuatori ESC + elica [s]

%% Limiti attuatori e ESC (da Disegna Dati e Appendice B)
p.duty_hover = 5.70;        % Duty cycle medio in hovering [%]
p.duty_min   = 5.39;        % Soglia minima di sicurezza ESC [%]
p.duty_max   = 6.30;        % Soglia massima di sicurezza ESC [%]
p.U1_hover   = p.m * p.g;   % Spinta totale di hovering [N]
p.w2_hover   = p.U1_hover / (4 * p.b);

%% Temporizzazione firmware STM32 e sensore BNO055
p.dt_ctrl = 0.005;          % Ciclo principale 200 Hz (5 ms)
p.dt_traj = 0.020;          % Aggiornamento punti traiettoria 50 Hz (20 ms)
p.dt_imu  = 0.010;          % Frequenza fusione NDOF BNO055 (100 Hz)

%% Controllo anello esterno di posizione - Baseline (main.c)
p.wn_x   = 0.38;            % Pulsazione naturale X baseline [rad/s]
p.zeta_x = 1.00;            % Smorzamento X baseline
p.Kp_x   = p.wn_x^2;
p.Kd_x   = 2 * p.zeta_x * p.wn_x;

p.wn_y   = 0.38;            % Pulsazione naturale Y baseline [rad/s]
p.zeta_y = 1.00;            % Smorzamento Y baseline
p.Kp_y   = p.wn_y^2;
p.Kd_y   = 2 * p.zeta_y * p.wn_y;

p.acc_max = 1.30;           % POS_ACC_MAX [m/s^2]

% Parametri anello esterno ritarati (C5)
p.wn_opt   = 1.05;          % Nuova pulsazione naturale [rad/s]
p.zeta_opt = 1.05;          % Leggermente sovrasmorzato (zero overshoot)

%% Controllo anello interno PID assetto (PID.h)
p.filter_size = 15;         % Baseline: media mobile 15 campioni (35 ms ritardo)
p.fc_iir      = 18.0;       % Miglioramento: taglio filtro IIR 1° ordine [Hz]

% Roll PID baseline (ingresso in gradi, uscita U2 in N*m)
p.KPR = 0.055;  p.KIR = 0.0001;  p.KDR = 0.07;
p.U2_lim = 1.30;

% Pitch PID baseline (ingresso in gradi, uscita U3 in N*m)
p.KPP = 0.050;  p.KIP = 0.00055; p.KDP = 0.09;
p.U3_lim = 1.30;

%% Parametri C6 (Anello interno ritarato + esterno veloce con integrale)
p.KPR_opt  = 0.14;  p.KDR_opt  = 0.020;
p.KPP_opt  = 0.14;  p.KDP_opt  = 0.022;
p.wn_c6    = 1.80;  p.zeta_c6  = 1.05;
p.Ki_pos   = 0.45;  p.Ipos_lim = 0.40;  % Anti-windup integrale posizione [m/s^2]

%% Rumore e bias residuo sensore BNO055 (modalita' NDOF)
p.imu_noise_std  = 0.025;   % Deviazione standard rumore angolare fuso [deg]
p.imu_bias_phi   = 0.015;   % Residuo offset post-hovering roll [deg]
p.imu_bias_theta = -0.020;  % Residuo offset post-hovering pitch [deg]
end