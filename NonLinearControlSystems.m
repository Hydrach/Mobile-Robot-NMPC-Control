%% Proje: FINAL KARŞILAŞTIRMA (AKICI HEDEF + HATA ANALİZİ)
% Amaç: Cascaded vs NMPC kıyaslaması, Gelişmiş Hedef Hareketi
clc;
clear;
close all;

%% 1. ORTAK SİMÜLASYON PARAMETRELERİ
dt = 0.05;         
T_final = 60;      
t = 0:dt:T_final;  
n_steps = length(t);

% --- ÇEVRE ---
obs_centers_init = {[15; 15; 2], [30; 10; 1.5], [10; 30; 2.5], [35; 35; 1]};
obs_sizes = [4, 3, 4, 5];
obs_range = [5, 0, 8, 6]; 
obs_speed = [0.4, 0, 0.6, 0.3]; 
num_obs = numel(obs_centers_init);
curr_obs = obs_centers_init; 

% Hedef (Daha Akıcı Hareket)
target_pos = [25; 25; 4]; 
target_dir = [1; 0.5; 0.2]; % Rastgele bir ilk yön
target_dir = target_dir / norm(target_dir);
target_speed = 0.6; % Hedefin hızı
arena_lims = [-5 50; -5 50; 1 8]; 
target_rad = 1.0;

%% 2. ROBOT FİZİĞİ (İKİSİ İÇİN AYNI)
z1 = [0; -3; pi/6; 0; 0]; % Robot 1: Cascaded (Mavi)
z2 = [0;  3; pi/6; 0; 0]; % Robot 2: NMPC (Mor)

m = 10; I = 5; mu_v = 1.5; mu_w = 0.8; 
rob_rad = 1.5; rob_width = 1.7; teker_r = 0.5; teker_w = 0.4;
taret_len = 2.5; taret_z = teker_r + rob_rad;

%% 3. KONTROLCÜ PARAMETRELERİ
% Cascaded
kv_c = 0.9; kw_c = 1.8; kF_c = 50.0; kTau_c = 20.0; k_rep_c = 200.0; det_r_c = 8.0; 
% NMPC
N_hor = 15; K_sam = 60; u_F_max = 80; u_Tau_max = 40; 
W_dist = 80; W_head = 30; W_coll = 100000; W_eff = 0.005;
last_best_U = zeros(2, N_hor);

%% 4. VERİ SAKLAMA
h1_hist = zeros(5, n_steps); h1_hist(:,1)=z1; 
h2_hist = zeros(5, n_steps); h2_hist(:,1)=z2; 
U1_hist = zeros(2, n_steps); U2_hist = zeros(2, n_steps); 
En1 = zeros(1, n_steps); En2 = zeros(1, n_steps); 
err1_dist = zeros(1, n_steps); err2_dist = zeros(1, n_steps);
err1_ang  = zeros(1, n_steps); err2_ang  = zeros(1, n_steps);
predicted_path_x = []; predicted_path_y = []; 

%% 5. GÖRSELLEŞTİRME HAZIRLIĞI
[Xr, Yr, Zr] = sphere(10); [Xh, Yh, Zh] = sphere(10); [Xt, Yt, Zt] = cylinder([teker_r teker_r], 8);

% --- PENCERE 1: CASCADED ---
fig_casc = figure('Name', '1. Cascaded Kontrol (Mavi)', 'Position', [20 50 500 450]);
hold on; grid on; axis equal; axis([-10 55 -10 55 0 12]); view(30, 45);
xlabel('X'); ylabel('Y'); title('Robot 1: Cascaded (Reaktif)');
h_targ1 = surf(nan(2), nan(2), nan(2), 'FaceColor', 'r', 'EdgeColor', 'none');
h_r1 = surf(nan(2), nan(2), nan(2), 'FaceColor', 'b', 'EdgeColor', 'none', 'FaceAlpha', 0.8);
h_path1 = plot3(nan, nan, nan, 'b--', 'LineWidth', 1);
h_tr1 = quiver3(0,0,0,0,0,0, 'b', 'LineWidth', 2);
h_obs1 = cell(1, num_obs);
for i=1:num_obs, h_obs1{i} = patch('Vertices', [], 'Faces', [], 'FaceColor', [0.5 0.5 0.5], 'FaceAlpha', 0.4); end

% --- PENCERE 2: NMPC ---
fig_nmpc = figure('Name', '2. NMPC Kontrol (Mor)', 'Position', [530 50 500 450]);
hold on; grid on; axis equal; axis([-10 55 -10 55 0 12]); view(30, 45);
xlabel('X'); ylabel('Y'); title('Robot 2: NMPC (Prediktif)');
h_targ2 = surf(nan(2), nan(2), nan(2), 'FaceColor', 'r', 'EdgeColor', 'none');
h_r2 = surf(nan(2), nan(2), nan(2), 'FaceColor', 'm', 'EdgeColor', 'none', 'FaceAlpha', 0.8);
h_path2 = plot3(nan, nan, nan, 'm-', 'LineWidth', 1);
h_pred = plot3(nan, nan, nan, 'g:', 'LineWidth', 1.5); 
h_tr2 = quiver3(0,0,0,0,0,0, 'm', 'LineWidth', 2);
h_obs2 = cell(1, num_obs);
for i=1:num_obs, h_obs2{i} = patch('Vertices', [], 'Faces', [], 'FaceColor', [0.5 0.5 0.5], 'FaceAlpha', 0.4); end

% --- PENCERE 3: PERFORMANS ---
fig_anl = figure('Name', '3. Performans ve Enerji', 'Position', [1050 50 400 450]);
subplot(3,1,1); h_v1=plot(nan,'b'); hold on; h_v2=plot(nan,'m','LineWidth',1.5); title('Hız (v)'); legend('Cascaded','NMPC'); grid on;
subplot(3,1,2); h_w1=plot(nan,'b'); hold on; h_w2=plot(nan,'m','LineWidth',1.5); title('Dönüş (\omega)'); grid on;
subplot(3,1,3); h_e1=plot(nan,'b'); hold on; h_e2=plot(nan,'m','LineWidth',2); title('Toplam Enerji (J)'); legend('Cascaded', 'NMPC'); grid on; xlabel('Zaman');

% --- PENCERE 4: HATA ANALİZİ ---
fig_err = figure('Name', '4. Hata Analizi', 'Position', [1050 550 400 300]);
subplot(2,1,1); h_err_dist1 = plot(nan, 'b'); hold on; h_err_dist2 = plot(nan, 'm', 'LineWidth', 1.5); 
title('Mesafe Hatası (m)'); legend('Cascaded', 'NMPC'); grid on;
subplot(2,1,2); h_err_ang1 = plot(nan, 'b'); hold on; h_err_ang2 = plot(nan, 'm', 'LineWidth', 1.5); 
title('Açı Hatası (Derece)'); legend('Cascaded', 'NMPC'); grid on;

%% === SİMÜLASYON ===
for k = 1:n_steps-1
    t_now = t(k);
    
    % --- A. ORTAM GÜNCELLEMESİ ---
    for i=1:num_obs
        init = obs_centers_init{i};
        curr_x = init(1) + obs_range(i) * sin(obs_speed(i) * t_now);
        curr_obs{i} = [curr_x, init(2), init(3)];
    end
    
    % --- B. HEDEF HAREKETİ (DAHA AKICI VE DOĞAL) ---
    % 1. Yönü hafifçe saptır (Rastgelelik)
    random_turn = (rand(3,1) - 0.5) * 0.2; % Küçük rastgele değişim
    target_dir = target_dir + random_turn;
    target_dir = target_dir / norm(target_dir); % Normalize et
    
    % 2. Potansiyel yeni konumu hesapla
    pot_pos = target_pos + target_dir * target_speed;
    
    % 3. Duvarlardan Sekme Kontrolü (Bilardo Topu Gibi)
    if pot_pos(1) < arena_lims(1,1) || pot_pos(1) > arena_lims(1,2)
        target_dir(1) = -target_dir(1); % X yönünü ters çevir
        pot_pos(1) = max(arena_lims(1,1), min(arena_lims(1,2), pot_pos(1))); % Sınır içine çek
    end
    if pot_pos(2) < arena_lims(2,1) || pot_pos(2) > arena_lims(2,2)
        target_dir(2) = -target_dir(2); % Y yönünü ters çevir
        pot_pos(2) = max(arena_lims(2,1), min(arena_lims(2,2), pot_pos(2)));
    end
    if pot_pos(3) < arena_lims(3,1) || pot_pos(3) > arena_lims(3,2)
        target_dir(3) = -target_dir(3); % Z yönünü ters çevir
        pot_pos(3) = max(arena_lims(3,1), min(arena_lims(3,2), pot_pos(3)));
    end
    
    target_pos = pot_pos;

    % --- C. ROBOT 1: CASCADED ---
    x=z1(1); y=z1(2); th=z1(3); v=z1(4); w=z1(5);
    rep_vec = [0;0]; min_d = inf;
    for i=1:num_obs
        vec = curr_obs{i}(1:2) - [x; y]; d = norm(vec); min_d = min(min_d, d - obs_sizes(i)/2);
        if d < det_r_c, rep_vec = rep_vec - (vec/d) * k_rep_c * (1/d - 1/det_r_c) * (1/d^2); end
    end
    vec_tar = [target_pos(1)-x; target_pos(2)-y];
    final_dir = (vec_tar / (norm(vec_tar)+eps)) + rep_vec;
    rho = norm(vec_tar); alpha = atan2(sin(atan2(final_dir(2), final_dir(1))-th), cos(atan2(final_dir(2), final_dir(1))-th));
    v_ist = kv_c * rho; if min_d < rob_rad + 1.5, v_ist = v_ist * 0.3; end
    w_ist = kw_c * alpha;
    F1 = max(-u_F_max, min(u_F_max, kF_c * (v_ist - v))); Tau1 = max(-u_Tau_max, min(u_Tau_max, kTau_c * (w_ist - w)));
    a_lin = (F1 - mu_v * v) / m; a_ang = (Tau1 - mu_w * w) / I;
    z1 = z1 + [v*cos(th); v*sin(th); w; a_lin; a_ang] * dt;
    h1_hist(:, k+1) = z1; U1_hist(:, k) = [F1; Tau1];
    P1 = abs(F1*v) + abs(Tau1*w); if k==1, En1(k)=P1*dt; else, En1(k)=En1(k-1)+P1*dt; end
    
    % Hata 1
    err1_dist(k) = sqrt((target_pos(1)-z1(1))^2 + (target_pos(2)-z1(2))^2);
    t_ang1 = atan2(target_pos(2)-z1(2), target_pos(1)-z1(1));
    err1_ang(k) = rad2deg(abs(atan2(sin(t_ang1-z1(3)), cos(t_ang1-z1(3)))));

    % --- D. ROBOT 2: NMPC ---
    x=z2(1); y=z2(2); th=z2(3); v=z2(4); w=z2(5);
    best_J = inf; best_U = zeros(2, N_hor); pred_x = zeros(1, N_hor); pred_y = zeros(1, N_hor);
    for s = 1:K_sam
        noise = [randn(1, N_hor)*20; randn(1, N_hor)*10];
        U_seq = (s==1)*last_best_U + (s>1)*(last_best_U + noise);
        U_seq(1,:) = max(-u_F_max, min(u_F_max, U_seq(1,:))); U_seq(2,:) = max(-u_Tau_max, min(u_Tau_max, U_seq(2,:)));
        sx=x; sy=y; sth=th; sv=v; sw=w; J=0; tmp_x = zeros(1, N_hor); tmp_y = zeros(1, N_hor);
        for n=1:N_hor
            uF=U_seq(1,n); uT=U_seq(2,n);
            sv=sv+(uF-mu_v*sv)/m*dt; sw=sw+(uT-mu_w*sw)/I*dt; sth=sth+sw*dt; sx=sx+sv*cos(sth)*dt; sy=sy+sv*sin(sth)*dt;
            tmp_x(n)=sx; tmp_y(n)=sy;
            d_tar = sqrt((target_pos(1)-sx)^2 + (target_pos(2)-sy)^2);
            h_err = abs(atan2(sin(atan2(target_pos(2)-sy, target_pos(1)-sx)-sth), cos(atan2(target_pos(2)-sy, target_pos(1)-sx)-sth)));
            d_obs_pen = 0;
            for i=1:num_obs
                d = sqrt((sx-curr_obs{i}(1))^2 + (sy-curr_obs{i}(2))^2);
                if d < obs_sizes(i)/2 + rob_rad + 1.5, d_obs_pen = d_obs_pen + W_coll*(1/d)^4; end
            end
            J = J + W_dist*d_tar + W_head*h_err + d_obs_pen + W_eff*(uF^2+uT^2);
        end
        if J < best_J, best_J=J; best_U=U_seq; pred_x=tmp_x; pred_y=tmp_y; end
    end
    F2 = best_U(1,1); Tau2 = best_U(2,1); last_best_U = [best_U(:,2:end), [0;0]];
    a_lin = (F2 - mu_v * v) / m; a_ang = (Tau2 - mu_w * w) / I;
    z2 = z2 + [v*cos(th); v*sin(th); w; a_lin; a_ang] * dt;
    h2_hist(:, k+1) = z2; U2_hist(:, k) = [F2; Tau2];
    P2 = abs(F2*v) + abs(Tau2*w); if k==1, En2(k)=P2*dt; else, En2(k)=En2(k-1)+P2*dt; end
    
    % Hata 2
    err2_dist(k) = sqrt((target_pos(1)-z2(1))^2 + (target_pos(2)-z2(2))^2);
    t_ang2 = atan2(target_pos(2)-z2(2), target_pos(1)-z2(1));
    err2_ang(k) = rad2deg(abs(atan2(sin(t_ang2-z2(3)), cos(t_ang2-z2(3)))));

    % --- E. ÇİZİM (Her 5 adımda bir - Hızlandırılmış) ---
    if mod(k, 5) == 0
        % Engeller
        for i=1:num_obs
           c = curr_obs{i}; s = obs_sizes(i)/2;
           v = [c(1)-s c(2)-s 0; c(1)+s c(2)-s 0; c(1)+s c(2)+s 0; c(1)-s c(2)+s 0; c(1)-s c(2)-s s*2; c(1)+s c(2)-s s*2; c(1)+s c(2)+s s*2; c(1)-s c(2)+s s*2];
           set(h_obs1{i}, 'Vertices', v, 'Faces', [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8]); 
           set(h_obs2{i}, 'Vertices', v, 'Faces', [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8]); 
        end
        % Hedef
        set(h_targ1, 'XData', Xh*target_rad+target_pos(1), 'YData', Yh*target_rad+target_pos(2), 'ZData', Zh*target_rad+target_pos(3));
        set(h_targ2, 'XData', Xh*target_rad+target_pos(1), 'YData', Yh*target_rad+target_pos(2), 'ZData', Zh*target_rad+target_pos(3));
        
        % Robot 1
        set(h_r1, 'XData', Xr*rob_rad+z1(1), 'YData', Yr*rob_rad+z1(2), 'ZData', Zr*rob_rad+teker_r);
        set(h_path1, 'XData', h1_hist(1,1:k), 'YData', h1_hist(2,1:k), 'ZData', zeros(1,k));
        d1 = norm([target_pos(1)-z1(1), target_pos(2)-z1(2)]); p1 = atan2(target_pos(3)-taret_z, d1);
        v1 = [cos(z1(3))*cos(p1), sin(z1(3))*cos(p1), sin(p1)] * taret_len;
        set(h_tr1, 'XData', z1(1), 'YData', z1(2), 'ZData', taret_z, 'UData', v1(1), 'VData', v1(2), 'WData', v1(3));
        
        % Robot 2
        set(h_r2, 'XData', Xr*rob_rad+z2(1), 'YData', Yr*rob_rad+z2(2), 'ZData', Zr*rob_rad+teker_r);
        set(h_path2, 'XData', h2_hist(1,1:k), 'YData', h2_hist(2,1:k), 'ZData', zeros(1,k));
        set(h_pred, 'XData', pred_x, 'YData', pred_y, 'ZData', zeros(1, N_hor)); 
        d2 = norm([target_pos(1)-z2(1), target_pos(2)-z2(2)]); p2 = atan2(target_pos(3)-taret_z, d2);
        v2 = [cos(z2(3))*cos(p2), sin(z2(3))*cos(p2), sin(p2)] * taret_len;
        set(h_tr2, 'XData', z2(1), 'YData', z2(2), 'ZData', taret_z, 'UData', v2(1), 'VData', v2(2), 'WData', v2(3));

        % Grafikler
        set(h_v1, 'XData', t(1:k), 'YData', h1_hist(4,1:k)); set(h_v2, 'XData', t(1:k), 'YData', h2_hist(4,1:k));
        set(h_w1, 'XData', t(1:k), 'YData', h1_hist(5,1:k)); set(h_w2, 'XData', t(1:k), 'YData', h2_hist(5,1:k));
        set(h_e1, 'XData', t(1:k), 'YData', En1(1:k)); set(h_e2, 'XData', t(1:k), 'YData', En2(1:k));
        
        set(h_err_dist1, 'XData', t(1:k), 'YData', err1_dist(1:k)); set(h_err_dist2, 'XData', t(1:k), 'YData', err2_dist(1:k));
        set(h_err_ang1, 'XData', t(1:k), 'YData', err1_ang(1:k)); set(h_err_ang2, 'XData', t(1:k), 'YData', err2_ang(1:k));
        
        drawnow;
    end
end
disp('Karşılaştırma Bitti.');