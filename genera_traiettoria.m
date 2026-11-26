function traj = genera_traiettoria(tipo, modo_deriv, dt_traj)
% GENERA_TRAIETTORIA Crea traiettorie di prova (spezzata o continua)
%   tipo:       'spezzata' (triangolo Test 181) | 'cerchio' (Test 131)
%   modo_deriv: 'baseline_fd' (differenze finite STM32) | 'analitica' (smooth)
%   dt_traj:    passo temporale traiettoria [s] (default 0.020 s)

    if nargin < 3, dt_traj = 0.020; end

    switch lower(tipo)
        case 'spezzata'
            % Waypoint triangolo Test n.181: (0,0) -> (-3,3) -> (0,6) -> (0,0)
            wp = [ 0.0,  0.0;
                  -3.0,  3.0;
                   0.0,  6.0;
                   0.0,  0.0];
            T_seg = [10.0, 10.0, 12.0]; % Durata ciascun lato [s]
            
            t = []; x = []; y = [];
            vx_an = []; vy_an = []; ax_an = []; ay_an = [];
            t_curr = 0;

            for i = 1:size(wp, 1)-1
                N_s = round(T_seg(i) / dt_traj);
                ts = (0:N_s-1)' * dt_traj;
                tau = ts / T_seg(i);
                
                dx = wp(i+1,1) - wp(i,1);
                dy = wp(i+1,2) - wp(i,2);

                if strcmpi(modo_deriv, 'baseline_fd')
                    % Spezzata lineare pura (velocita' costante a tratti, vertici vivi)
                    s   = tau;
                    ds  = ones(size(tau)) / T_seg(i);
                    dds = zeros(size(tau));
                else
                    % Profilo Minimum Jerk (polinomio 5° grado) per ogni segmento
                    s   = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;
                    ds  = (30*tau.^2 - 60*tau.^3 + 30*tau.^4) / T_seg(i);
                    dds = (60*tau - 180*tau.^2 + 120*tau.^3) / (T_seg(i)^2);
                end

                t     = [t; t_curr + ts]; %#ok<AGROW>
                x     = [x; wp(i,1) + dx * s]; %#ok<AGROW>
                y     = [y; wp(i,2) + dy * s]; %#ok<AGROW>
                vx_an = [vx_an; dx * ds]; %#ok<AGROW>
                vy_an = [vy_an; dy * ds]; %#ok<AGROW>
                ax_an = [ax_an; dx * dds]; %#ok<AGROW>
                ay_an = [ay_an; dy * dds]; %#ok<AGROW>
                t_curr = t_curr + T_seg(i);
            end
            % Punto finale esatto
            t(end+1) = t_curr; x(end+1) = wp(end,1); y(end+1) = wp(end,2);
            vx_an(end+1) = 0; vy_an(end+1) = 0; ax_an(end+1) = 0; ay_an(end+1) = 0;

        case 'cerchio'
            % Circonferenza Test n.131: centro (0, -5), raggio R = 5 m, partenza (0,0)
            R = 5.0;
            T_giro = 20.0;
            T_ramp = 4.0;
            T_tot  = T_giro + T_ramp;
            t = (0:dt_traj:T_tot)';
            
            % Parametro temporale con raccordo quintico
            tau = zeros(size(t));
            dtau = zeros(size(t));
            ddtau = zeros(size(t));
            
            for k = 1:length(t)
                tk = t(k);
                if tk < T_ramp
                    s = tk / T_ramp;
                    tau(k)   = T_ramp * (2.5*s^4 - 3*s^5 + s^6);
                    dtau(k)  = 10*s^3 - 15*s^4 + 6*s^5;
                    ddtau(k) = (30*s^2 - 60*s^3 + 30*s^4) / T_ramp;
                elseif tk <= T_tot - T_ramp
                    tau(k)   = T_ramp/2 + (tk - T_ramp);
                    dtau(k)  = 1.0;
                    ddtau(k) = 0.0;
                else
                    s = max(0, (T_tot - tk) / T_ramp);
                    tau(k)   = T_giro - T_ramp * (2.5*s^4 - 3*s^5 + s^6);
                    dtau(k)  = 10*s^3 - 15*s^4 + 6*s^5;
                    ddtau(k) = -(30*s^2 - 60*s^3 + 30*s^4) / T_ramp;
                end
            end
            
            w = 2*pi / T_giro;
            x = R * sin(w * tau);
            y = R * (cos(w * tau) - 1);
            
            vx_an = R * w * cos(w * tau) .* dtau;
            vy_an = -R * w * sin(w * tau) .* dtau;
            ax_an = R * w * (-w * sin(w * tau) .* (dtau.^2) + cos(w * tau) .* ddtau);
            ay_an = -R * w * (w * cos(w * tau) .* (dtau.^2) + sin(w * tau) .* ddtau);
    end

    %% Calcolo derivate: replica esattamente mat_trajBuildDerivs() di main.c
    N = length(t);
    if strcmpi(modo_deriv, 'baseline_fd')
        vx = zeros(N,1); vy = zeros(N,1);
        ax = zeros(N,1); ay = zeros(N,1);
        dt2 = dt_traj^2;
        for k = 2:N-1
            vx(k) = (x(k+1) - x(k-1)) / (2 * dt_traj);
            vy(k) = (y(k+1) - y(k-1)) / (2 * dt_traj);
            ax(k) = (x(k+1) - 2*x(k) + x(k-1)) / dt2;
            ay(k) = (y(k+1) - 2*y(k) + y(k-1)) / dt2;
        end
    else
        vx = vx_an; vy = vy_an;
        ax = ax_an; ay = ay_an;
    end

    traj.t  = t;
    traj.x  = x;  traj.y  = y;
    traj.vx = vx; traj.vy = vy;
    traj.ax = ax; traj.ay = ay;
    traj.dt = dt_traj;
end