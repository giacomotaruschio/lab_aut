function res = simula_quadricottero(traj, p, opz)
% SIMULA_QUADRICOTTERO Digital twin discreto a 200 Hz con opzioni di ablation study

    if nargin < 3, opz = struct(); end
    if ~isfield(opz, 'fix_timing_est'),     opz.fix_timing_est     = false; end
    if ~isfield(opz, 'disaccoppia_esatto'), opz.disaccoppia_esatto = false; end
    if ~isfield(opz, 'usa_filtro_iir'),     opz.usa_filtro_iir     = false; end
    if ~isfield(opz, 'usa_pid_opt'),        opz.usa_pid_opt        = false; end
    if ~isfield(opz, 'wn'),                 opz.wn                 = p.wn_x; end
    if ~isfield(opz, 'zeta'),               opz.zeta               = p.zeta_x; end
    if ~isfield(opz, 'Ki_pos'),             opz.Ki_pos             = 0.0; end

    Kp_pos = opz.wn^2;
    Kd_pos = 2 * opz.zeta * opz.wn;

    if opz.usa_pid_opt
        KPR = p.KPR_opt; KDR = p.KDR_opt;
        KPP = p.KPP_opt; KDP = p.KDP_opt;
    else
        KPR = p.KPR; KDR = p.KDR;
        KPP = p.KPP; KDP = p.KDP;
    end

    dt = p.dt_ctrl;                 % 0.005 s (200 Hz)
    T_after = 4.0;                  % Stato MAT_AFT (4 s dopo fine traiettoria)
    T_sim = traj.t(end) + T_after;
    t_sim = (0:dt:T_sim)';
    N = length(t_sim);

    % Stati fisici reali del drone sul banco
    phi = 0; dphi = 0;
    theta = 0; dtheta = 0;
    x_true = 0; vx_true = 0;
    y_true = 0; vy_true = 0;

    % Stati stima STM32 (est_update)
    est_x = 0; est_vx = 0;
    est_y = 0; est_vy = 0;
    ax_est_prev = 0; ay_est_prev = 0;
    vx_est_prev = 0; vy_est_prev = 0;
    tick_prev = 0;

    % Integrale anello esterno posizione (C6)
    Ipos_x = 0;
    Ipos_y = 0;

    % Buffer media mobile a 15 campioni (PID.c)
    buf_roll  = zeros(p.filter_size, 1);
    buf_pitch = zeros(p.filter_size, 1);
    idx_buf = 1;
    filt_roll_prev  = 0;
    filt_pitch_prev = 0;

    % Pre-carica integrale pitch accumulato nei 30 s di hovering (MAT_HOV)
    U3_eq = -2 * p.l * p.b * p.w2_hover * (1 - p.eff_M1) / (1 + p.eff_M1);
    Iterm_roll  = 0;
    Iterm_pitch = U3_eq;

    % Stato dinamico attuatori (inizializzato all'equilibrio di hovering)
    w2_act = [p.w2_hover - U3_eq / (2 * p.l * p.b);
              p.w2_hover;
              p.w2_hover + U3_eq / (2 * p.l * p.b);
              p.w2_hover];

    % Variabili campionamento IMU BNO055 (100 Hz)
    imu_step = max(1, round(p.dt_imu / dt));
    roll_meas_deg  = 0;
    pitch_meas_deg = 0;

    % Log variabili
    log.t = t_sim;
    log.refX = zeros(N,1); log.refY = zeros(N,1);
    log.estX = zeros(N,1); log.estY = zeros(N,1);
    log.trueX = zeros(N,1); log.trueY = zeros(N,1);
    log.roll = zeros(N,1); log.pitch = zeros(N,1);
    log.refRoll = zeros(N,1); log.refPitch = zeros(N,1);
    log.U2 = zeros(N,1); log.U3 = zeros(N,1);
    log.motors_duty = zeros(N,4);
    log.sat_flag = false(N,1);

    rng(42); % Riproducibilita' rumore IMU

    for k = 1:N
        tk = t_sim(k);

        %% 1. Aggiornamento traiettoria (mat_trajUpdate)
        idx_tr = floor(tk / traj.dt) + 1;
        if idx_tr <= length(traj.t)
            refX  = traj.x(idx_tr);  refY  = traj.y(idx_tr);
            refVX = traj.vx(idx_tr); refVY = traj.vy(idx_tr);
            refAX = traj.ax(idx_tr); refAY = traj.ay(idx_tr);
        else
            % Stato MAT_AFT: mantenimento posizione finale
            refX  = traj.x(end); refY  = traj.y(end);
            refVX = 0; refVY = 0;
            refAX = 0; refAY = 0;
        end

        %% 2. Lettura IMU BNO055 (aggiornamento NDOF a 100 Hz)
        if mod(k - 1, imu_step) == 0
            roll_meas_deg  = rad2deg(phi)   + p.imu_bias_phi   + p.imu_noise_std * randn();
            pitch_meas_deg = rad2deg(theta) + p.imu_bias_theta + p.imu_noise_std * randn();
        end

        %% 3. Stima posizione STM32 (est_update)
        r_est = deg2rad(roll_meas_deg);
        p_est = deg2rad(pitch_meas_deg);
        cp = max(0.1, cos(p_est));
        ax_est = p.g * tan(p_est);
        ay_est = p.g * tan(r_est) / cp;

        if ~opz.fix_timing_est
            % Legacy: HAL_GetTick() a 1 ms con jitter + integrazione di Eulero
            tick_now = floor((tk + 0.00045 * randn()) * 1000) * 0.001;
            dt_est = tick_now - tick_prev;
            tick_prev = tick_now;
            if dt_est <= 0 || dt_est > 0.1
                dt_est = dt;
            end
            est_vx = est_vx + ax_est * dt_est;
            est_vy = est_vy + ay_est * dt_est;
            est_x  = est_x  + est_vx * dt_est;
            est_y  = est_y  + est_vy * dt_est;
        else
            % Migliorato: timer hardware a 1 us (dt esatto) + integrazione trapezoidale
            est_vx = est_vx + 0.5 * (ax_est + ax_est_prev) * dt;
            est_vy = est_vy + 0.5 * (ay_est + ay_est_prev) * dt;
            est_x  = est_x  + 0.5 * (est_vx + vx_est_prev) * dt;
            est_y  = est_y  + 0.5 * (est_vy + vy_est_prev) * dt;
            ax_est_prev = ax_est; ay_est_prev = ay_est;
            vx_est_prev = est_vx; vy_est_prev = est_vy;
        end

        %% 4. Controllo anello esterno posizione (pos_control)
        errX = refX - est_x;
        errY = refY - est_y;

        if opz.Ki_pos > 0
            Ipos_x = min(max(Ipos_x + opz.Ki_pos * errX * dt, -p.Ipos_lim), p.Ipos_lim);
            Ipos_y = min(max(Ipos_y + opz.Ki_pos * errY * dt, -p.Ipos_lim), p.Ipos_lim);
        end

        ax_des = refAX + Kd_pos * (refVX - est_vx) + Kp_pos * errX + Ipos_x;
        ay_des = refAY + Kd_pos * (refVY - est_vy) + Kp_pos * errY + Ipos_y;

        % Saturazione POS_ACC_MAX
        ax_des = min(max(ax_des, -p.acc_max), p.acc_max);
        ay_des = min(max(ay_des, -p.acc_max), p.acc_max);

        % Modello inverso accelerazione -> angoli desiderati [deg]
        pitchRef = rad2deg(atan(ax_des / p.g));
        if opz.disaccoppia_esatto
            rollRef = rad2deg(atan((ay_des * cos(deg2rad(pitchRef))) / p.g));
        else
            rollRef = rad2deg(atan(ay_des / p.g));
        end

        %% 5. Controllo anello interno PID assetto (PID_controller)
        if ~opz.usa_filtro_iir
            % Media mobile circolare a 15 campioni (identica a PID.c)
            buf_roll(idx_buf)  = roll_meas_deg;
            buf_pitch(idx_buf) = pitch_meas_deg;
            idx_buf = mod(idx_buf, p.filter_size) + 1;
            roll_filt  = mean(buf_roll);
            pitch_filt = mean(buf_pitch);
        else
            % Filtro passa-basso IIR del 1° ordine (ritardo ~8.8 ms a 18 Hz)
            alpha = 1 - exp(-2 * pi * p.fc_iir * dt);
            roll_filt  = filt_roll_prev  + alpha * (roll_meas_deg  - filt_roll_prev);
            pitch_filt = filt_pitch_prev + alpha * (pitch_meas_deg - filt_pitch_prev);
        end

        % PID Roll -> U2
        e_r = rollRef - roll_filt;
        P_r = KPR * e_r;
        newI_r = Iterm_roll + p.KIR * dt * e_r;
        D_r = -(KDR / dt) * (roll_filt - filt_roll_prev);
        U2_cmd = P_r + newI_r + D_r;
        if U2_cmd > p.U2_lim
            U2_cmd = p.U2_lim;
        elseif U2_cmd < -p.U2_lim
            U2_cmd = -p.U2_lim;
        else
            Iterm_roll = newI_r;
        end

        % PID Pitch -> U3
        e_p = pitchRef - pitch_filt;
        P_p = KPP * e_p;
        newI_p = Iterm_pitch + p.KIP * dt * e_p;
        D_p = -(KDP / dt) * (pitch_filt - filt_pitch_prev);
        U3_cmd = P_p + newI_p + D_p;
        if U3_cmd > p.U3_lim
            U3_cmd = p.U3_lim;
        elseif U3_cmd < -p.U3_lim
            U3_cmd = -p.U3_lim;
        else
            Iterm_pitch = newI_p;
        end

        filt_roll_prev  = roll_filt;
        filt_pitch_prev = pitch_filt;

        %% 6. Mixing ESC (SpeedCompute), ritardo attuatori e saturazione
        w2_cmd = zeros(4,1);
        w2_cmd(1) = p.w2_hover - U3_cmd / (2 * p.l * p.b);
        w2_cmd(2) = p.w2_hover - U2_cmd / (2 * p.l * p.b);
        w2_cmd(3) = p.w2_hover + U3_cmd / (2 * p.l * p.b);
        w2_cmd(4) = p.w2_hover + U2_cmd / (2 * p.l * p.b);

        % Conversione in duty cycle [%] e saturazione fisica ESC
        duty = 5.0 + (w2_cmd / p.w2_hover) * (p.duty_hover - 5.0);
        sat_now = any(duty <= p.duty_min) || any(duty >= p.duty_max);
        duty_sat = min(max(duty, p.duty_min), p.duty_max);

        % Dinamica del 1° ordine degli ESC + motori (tau_m)
        w2_target = ((duty_sat - 5.0) / (p.duty_hover - 5.0)) * p.w2_hover;
        alpha_m = dt / (p.tau_m + dt);
        w2_act = w2_act + alpha_m * (w2_target - w2_act);

        % Asimmetria fisica Motore 1
        w2_eff = w2_act;
        w2_eff(1) = w2_eff(1) * p.eff_M1;

        U2_eff = p.l * p.b * (-w2_eff(2) + w2_eff(4));
        U3_eff = p.l * p.b * (-w2_eff(1) + w2_eff(3));

        %% 7. Integrazione dinamica fisica reale del drone sul banco
        ddphi   = (U2_eff - p.c_phi * dphi) / p.Ixx;
        ddtheta = (U3_eff - p.c_theta * dtheta) / p.Iyy;
        dphi    = dphi + ddphi * dt;
        dtheta  = dtheta + ddtheta * dt;
        phi     = phi + dphi * dt;
        theta   = theta + dtheta * dt;

        % Cinematica reale corrispondente
        ax_true = p.g * tan(theta);
        ay_true = p.g * tan(phi) / max(0.1, cos(theta));
        vx_true = vx_true + ax_true * dt;
        vy_true = vy_true + ay_true * dt;
        x_true  = x_true  + vx_true * dt;
        y_true  = y_true  + vy_true * dt;

        %% Salvataggio log
        log.refX(k) = refX;   log.refY(k) = refY;
        log.estX(k) = est_x;  log.estY(k) = est_y;
        log.trueX(k)= x_true; log.trueY(k)= y_true;
        log.roll(k) = rad2deg(phi); log.pitch(k) = rad2deg(theta);
        log.refRoll(k) = rollRef;   log.refPitch(k) = pitchRef;
        log.U2(k) = U2_eff;   log.U3(k) = U3_eff;
        log.motors_duty(k,:) = duty_sat';
        log.sat_flag(k) = sat_now;
    end

    %% Calcolo indici oggettivi di prestazione (solo finestra traiettoria)
    idx_m = t_sim <= traj.t(end);
    ex = log.refX(idx_m) - log.estX(idx_m);
    ey = log.refY(idx_m) - log.estY(idx_m);
    err_eucl = hypot(ex, ey);

    res.log = log;
    res.RMSE_x   = rms(ex);
    res.RMSE_y   = rms(ey);
    res.RMSE_xy  = rms(err_eucl);
    res.err_max  = max(err_eucl);
    res.err_end  = err_eucl(end);
    res.sat_perc = 100 * mean(log.sat_flag(idx_m));
end