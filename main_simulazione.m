%% MAIN_ABLATION - Confronto quantitativo Baseline vs Miglioramenti (C0 -> C6)
clear; clc; close all;

p = parametri_drone();

% Definizione delle 7 configurazioni progressive (Ablation Study)
configs = {
    'C0: Baseline calibrata',        'baseline_fd', struct('fix_timing_est',false,'disaccoppia_esatto',false,'usa_filtro_iir',false,'usa_pid_opt',false,'wn',p.wn_x,  'zeta',p.zeta_x,  'Ki_pos',0.0);
    'C1: + Timing 1us & Trapez.',    'baseline_fd', struct('fix_timing_est',true, 'disaccoppia_esatto',false,'usa_filtro_iir',false,'usa_pid_opt',false,'wn',p.wn_x,  'zeta',p.zeta_x,  'Ki_pos',0.0);
    'C2: + Traiettoria C2 MinJerk',  'analitica',   struct('fix_timing_est',true, 'disaccoppia_esatto',false,'usa_filtro_iir',false,'usa_pid_opt',false,'wn',p.wn_x,  'zeta',p.zeta_x,  'Ki_pos',0.0);
    'C3: + Disaccoppiamento cos(th)','analitica',   struct('fix_timing_est',true, 'disaccoppia_esatto',true, 'usa_filtro_iir',false,'usa_pid_opt',false,'wn',p.wn_x,  'zeta',p.zeta_x,  'Ki_pos',0.0);
    'C4: + Filtro IIR (18 Hz)',      'analitica',   struct('fix_timing_est',true, 'disaccoppia_esatto',true, 'usa_filtro_iir',true, 'usa_pid_opt',false,'wn',p.wn_x,  'zeta',p.zeta_x,  'Ki_pos',0.0);
    'C5: + Ritaratura wn=1.05',      'analitica',   struct('fix_timing_est',true, 'disaccoppia_esatto',true, 'usa_filtro_iir',true, 'usa_pid_opt',false,'wn',p.wn_opt,'zeta',p.zeta_opt,'Ki_pos',0.0);
    'C6: + PID int. & Integr. pos.', 'analitica',   struct('fix_timing_est',true, 'disaccoppia_esatto',true, 'usa_filtro_iir',true, 'usa_pid_opt',true, 'wn',p.wn_c6, 'zeta',p.zeta_c6, 'Ki_pos',p.Ki_pos);
    };

traiettorie = {'spezzata', 'cerchio'};
nomi_traj   = {'SPEZZATA (Triangolo Test n.181)', 'CONTINUA (Circonferenza Test n.131)'};
risultati   = cell(2, size(configs, 1));

for t_idx = 1:2
    tipo = traiettorie{t_idx};
    fprintf('\n=========================================================================================\n');
    fprintf('  ABLATION STUDY - %s\n', nomi_traj{t_idx});
    fprintf('=========================================================================================\n');
    fprintf('%-30s | RMSE_x [m] | RMSE_y [m] | RMSE_xy [m] | Max [m] | End [m] | Sat [%%] | Rid.RMSE\n', 'Configurazione');
    fprintf('-----------------------------------------------------------------------------------------\n');

    rmse_c0 = 0;
    for c_idx = 1:size(configs, 1)
        nome_cfg = configs{c_idx, 1};
        modo_der = configs{c_idx, 2};
        opz      = configs{c_idx, 3};

        % Da C5 in poi aggiorniamo i riferimenti a 200 Hz (dt_ctrl)
        dt_tr = p.dt_traj;
        if c_idx >= 5
            dt_tr = p.dt_ctrl;
        end

        traj = genera_traiettoria(tipo, modo_der, dt_tr);
        res  = simula_quadricottero(traj, p, opz);
        risultati{t_idx, c_idx} = res;

        if c_idx == 1
            rmse_c0 = res.RMSE_xy;
            rid_str = '   ---  ';
        else
            rid = 100 * (1 - res.RMSE_xy / rmse_c0);
            rid_str = sprintf('%+6.1f%%', -rid);
        end

        fprintf('%-30s |   %6.3f   |   %6.3f   |   %6.3f    | %6.3f  | %6.3f  | %5.1f%%  | %s\n', ...
            nome_cfg, res.RMSE_x, res.RMSE_y, res.RMSE_xy, res.err_max, res.err_end, res.sat_perc, rid_str);
    end
    fprintf('=========================================================================================\n');
end

%% Grafici comparativi Baseline (C0) vs C5 vs Ottimizzato completo (C6)
figure('Color', 'w', 'Name', 'Confronto C0 vs C5 vs C6');
for t_idx = 1:2
    res_base = risultati{t_idx, 1};
    res_c5   = risultati{t_idx, 6};
    res_c6   = risultati{t_idx, 7};

    subplot(2, 2, t_idx);
    plot(res_base.log.refX, res_base.log.refY, 'k--', 'LineWidth', 1.8); hold on;
    plot(res_base.log.estX, res_base.log.estY, 'r-',  'LineWidth', 1.2);
    plot(res_c6.log.estX,   res_c6.log.estY,   'g-',  'LineWidth', 1.7);
    grid on; axis equal; xlabel('x [m]'); ylabel('y [m]');
    title(sprintf('%s (RMSE: %.3f m -> %.3f m)', upper(traiettorie{t_idx}), res_base.RMSE_xy, res_c6.RMSE_xy));
    legend('Riferimento', 'C0: Baseline', 'C6: Ottimizzato', 'Location', 'best');

    subplot(2, 2, t_idx + 2);
    err_b  = hypot(res_base.log.refX - res_base.log.estX, res_base.log.refY - res_base.log.estY);
    err_c5 = hypot(res_c5.log.refX   - res_c5.log.estX,   res_c5.log.refY   - res_c5.log.estY);
    err_c6 = hypot(res_c6.log.refX   - res_c6.log.estX,   res_c6.log.refY   - res_c6.log.estY);
    plot(res_base.log.t, err_b,  'r-',  'LineWidth', 1.2); hold on;
    plot(res_c5.log.t,   err_c5, 'b--', 'LineWidth', 1.3);
    plot(res_c6.log.t,   err_c6, 'g-',  'LineWidth', 1.7);
    grid on; xlabel('Tempo [s]'); ylabel('Errore 2D [m]');
    title('Andamento errore di inseguimento nel tempo');
    legend('C0: Baseline', 'C5: Solo anello est.', 'C6: PID int. + est.', 'Location', 'best');
end