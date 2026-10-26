%% Proje: Gelişmiş Saldırı Robotu Sim. (Dinamik Hedef ve Robot APF Engel Kaçınma V6)
clc;
clear;
close all;

%% 1. Simülasyon Parametreleri
dt = 0.05;         
T_final = 70;      
t = 0:dt:T_final;  

%% 2. Hedef (Düşman) Yörünge Tanımı ve Dinamikleri
hedef_merkez = [25; 25]; 
hedef_genlik_x = 15.0; 
hedef_genlik_y = 20.0; 
hedef_hiz_xy = 0.2; 
hedef_z_merkez = 4.0; 
hedef_z_genlik1 = 2.0; 
hedef_z_genlik2 = 1.5; 
hedef_z_hiz1 = 0.3; 
hedef_z_hiz2 = 0.7; 
hedef_pos = [hedef_merkez(1) + hedef_genlik_x; hedef_merkez(2); hedef_z_merkez]; % Mevcut hedef konumu
% Hedef Dinamikleri
hedef_velocity = [0; 0; 0]; % Hedefin başlangıç hızı
m_target = 2.0;            % Hedefin kütlesi (kg)
mu_target = 0.8;           % Hedef için sürtünme katsayısı (artırıldı)
max_target_speed = 4.0;    % Hedefin maksimum hızı (biraz düşürüldü)

%% 3. Robot Başlangıç Durumu ve Dinamik Parametreler
z = [0; 0; pi/6; 0; 0]; % x, y, theta, v, w

% 3D Şekil Parametreleri
robot_radius = 1.5; robot_genislik = 1.7; teker_r = 0.5; teker_w = 0.4;        
hedef_radius = 1.0; taret_z_baslangic = teker_r + robot_radius; taret_uzunluk = robot_radius * 1.5;

% Dinamik Parametreler (Robot)
m = 10; I = 5; mu_v = 1.5; mu_w = 0.8; 

%% Engeller (Hareketli)
obstacles_initial_centers = {[15; 15; 2], [30; 10; 1.5], [10; 30; 2.5], [35; 35; 1]};
obstacles_sizes = [4, 3, 4, 5];
obstacles_motion_range_x = [5, 0, 8, 6]; 
obstacles_speed_x = [0.4, 0, 0.6, 0.3]; 
num_obstacles = numel(obstacles_initial_centers); 
obstacle_cubes = cell(1, num_obstacles); 
obstacle_current_centers = obstacles_initial_centers; 

%% 4. Kontrolcü Kazançları (Robot)
kv = 0.9; kw = 1.8; kF = 50.0; kTau = 20.0; 

% Engel Kaçınma Parametreleri (Robot için)
k_rep_robot = 250.0; % Artırıldı
detection_radius_robot = 7.0; 
safety_margin_robot = 1.2;  

% Engel Kaçınma Parametreleri (Hedef için - APF Kuvvet)
k_att_target = 6.0;           % Hedef için çekim kazancı (artırıldı)
k_rep_target = 200.0;         % Hedef için itme kazancı (artırıldı)
detection_radius_target = 6.0; % Algılama mesafesi (artırıldı)

%% 5. Simülasyon Döngüsü ve Görselleştirme
z_gecmis = zeros(5, length(t)); z_gecmis(:, 1) = z;
U_hist = zeros(2, length(t)); 

% 3D Şekilleri Oluştur...
disp('3D şekiller oluşturuluyor...');
[X_r_base, Y_r_base, Z_r_base] = sphere(20); robot_shapes.X_r_base = X_r_base; robot_shapes.Y_r_base = Y_r_base; robot_shapes.Z_r_base = Z_r_base;
[X_h_base, Y_h_base, Z_h_base] = sphere(20); robot_shapes.X_h_base = X_h_base; robot_shapes.Y_h_base = Y_h_base; robot_shapes.Z_h_base = Z_h_base;
[Xt_base, Yt_base, Zt_base] = cylinder([teker_r teker_r], 10); robot_shapes.Xt_base = Xt_base; robot_shapes.Yt_base = Yt_base; robot_shapes.Zt_base = (Zt_base - 0.5) * teker_w; 

% Grafik handle'ları...
h_robot_body = []; h_yon_taret = []; h_yol = []; h_teker_R = []; h_teker_L = [];
h_obstacles = cell(1, num_obstacles); 

% Pencere 1: 3D Görselleştirme...
fig_3d = figure('Name', '1. Dinamik Hedef ve Robot APF Engel Kaçınma V6', 'Position', [50 100 800 800]); 
hold on; grid on; axis equal; axis([-10 50 -10 50 0 10]); 
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Gelişmiş 3D Simülasyon (Dinamik Hedef ve Robot APF V6)'); view(30, 45);

% Hedefi Çiz...
h_hedef = surf(robot_shapes.X_h_base * hedef_radius + hedef_pos(1), ...
               robot_shapes.Y_h_base * hedef_radius + hedef_pos(2), ...
               robot_shapes.Z_h_base * hedef_radius + hedef_pos(3), ... 
               'FaceColor', 'r', 'EdgeColor', 'none', 'DisplayName', 'Düşman Hedef');

% Engelleri Çiz...
disp('Engeller ilk kez çiziliyor...');
for obs_idx = 1:num_obstacles
    center = obstacle_current_centers{obs_idx}; 
    sz = obstacles_sizes(obs_idx) / 2; 
    corners = [center(1)-sz, center(2)-sz, 0; center(1)+sz, center(2)-sz, 0; center(1)+sz, center(2)+sz, 0; center(1)-sz, center(2)+sz, 0; 
               center(1)-sz, center(2)-sz, sz*2; center(1)+sz, center(2)-sz, sz*2; center(1)+sz, center(2)+sz, sz*2; center(1)-sz, center(2)+sz, sz*2]; 
    obstacle_cubes{obs_idx} = corners; 
    faces = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];
    h_obstacles{obs_idx} = patch('Vertices', corners, 'Faces', faces, 'FaceColor', [0.5 0.5 0.5], 'FaceAlpha', 0.7, 'EdgeColor', 'k', 'DisplayName', sprintf('Engel %d', obs_idx)); 
end
legend('Location', 'northwest');

% Pencere 2: Hız Takibi Grafiği...
fig_vel = figure('Name', '2. Hız Takibi (Düşük Seviye Kontrolcü)', 'Position', [900 100 600 600]); 
subplot(2,1,1); h_v_d = plot(0, 0, 'r--', 'LineWidth', 2, 'DisplayName', 'v_{istenen}'); hold on; h_v_a = plot(0, 0, 'b-', 'LineWidth', 2, 'DisplayName', 'v_{gerçek}'); title('Lineer Hız Takibi'); ylabel('Hız (m/s)'); grid on; legend;
subplot(2,1,2); h_w_d = plot(0, 0, 'r--', 'LineWidth', 2, 'DisplayName', '\omega_{istenen}'); hold on; h_w_a = plot(0, 0, 'b-', 'LineWidth', 2, 'DisplayName', '\omega_{gerçek}'); title('Açısal Hız Takibi'); ylabel('Hız (rad/s)'); xlabel('Zaman (s)'); grid on; legend;

% === SİMÜLASYON ANA DÖNGÜSÜ ===
for i = 1:length(t) - 1
    t_anlik = t(i); % Mevcut zaman
    t_sonraki = t_anlik + dt; % Sonraki zaman (ideal hedef için)
    
    % --- Engel Konumlarını Güncelle ---
    for obs_idx = 1:num_obstacles
        initial_center = obstacles_initial_centers{obs_idx};
        range = obstacles_motion_range_x(obs_idx);
        speed = obstacles_speed_x(obs_idx);
        current_x = initial_center(1) + range * sin(speed * t_anlik);
        obstacle_current_centers{obs_idx}(1) = current_x; 
        obstacle_current_centers{obs_idx}(2) = initial_center(2); 
        obstacle_current_centers{obs_idx}(3) = initial_center(3); 
    end
    % ---

    % --- HEDEF KONUM GÜNCELLEME (Dinamik Model ve APF ile - V6) ---
    
    % 1. Hedefin bir sonraki adımdaki *ideal* konumunu hesapla (yörünge formülü)
    ideal_next_hedef_x = hedef_merkez(1) + hedef_genlik_x * cos(hedef_hiz_xy * t_sonraki);
    ideal_next_hedef_y = hedef_merkez(2) + hedef_genlik_y * sin(2 * hedef_hiz_xy * t_sonraki); 
    ideal_next_hedef_z = hedef_z_merkez + hedef_z_genlik1 * sin(hedef_z_hiz1 * t_sonraki) + ...
                              + hedef_z_genlik2 * cos(hedef_z_hiz2 * t_sonraki);
    ideal_next_hedef_pos = [ideal_next_hedef_x; ideal_next_hedef_y; ideal_next_hedef_z];
    
    % 2. İdeal yörüngeye doğru çekici kuvveti hesapla
    F_att_target_3d = k_att_target * (ideal_next_hedef_pos - hedef_pos); 
    
    % 3. Hedefin *mevcut* konumuna göre engellerden gelen 3D itici kuvveti hesapla
    F_rep_target_3d = [0; 0; 0]; 
    for obs_idx = 1:num_obstacles
        obs_center = obstacle_current_centers{obs_idx}; % Güncel engel konumu
        obs_size = obstacles_sizes(obs_idx);
        obs_half_size = obs_size / 2;
        
        % Engelin kutusuna en yakın noktayı bul (3D'de)
         closest_point_on_obs = [max(obs_center(1)-obs_half_size, min(hedef_pos(1), obs_center(1)+obs_half_size)); ...
                                max(obs_center(2)-obs_half_size, min(hedef_pos(2), obs_center(2)+obs_half_size)); ...
                                max(0, min(hedef_pos(3), obs_size*2))]; 

        vec_to_obs_3d = closest_point_on_obs - hedef_pos; 
        dist_to_obs_surface_3d = norm(vec_to_obs_3d);
        
        if dist_to_obs_surface_3d < detection_radius_target && dist_to_obs_surface_3d > 1e-6 
            repulsion_force_magnitude = k_rep_target * (1/dist_to_obs_surface_3d - 1/detection_radius_target) * (1/dist_to_obs_surface_3d^2);
            repulsive_force_vec = - (vec_to_obs_3d / dist_to_obs_surface_3d) * repulsion_force_magnitude; 
            F_rep_target_3d = F_rep_target_3d + repulsive_force_vec;
        end
    end
    
    % 4. Toplam kuvveti (çekici + itici) bul
    F_total_target_3d = F_att_target_3d + F_rep_target_3d;
    
    % 5. Hedefin ivmesini hesapla (Sürtünmeyi de ekle)
    acceleration_target = (F_total_target_3d - mu_target * hedef_velocity) / m_target;
    
    % 6. Yeni hedef hızını hesapla (Euler) ve sınırla
    hedef_velocity = hedef_velocity + acceleration_target * dt;
    current_speed_target = norm(hedef_velocity);
    if current_speed_target > max_target_speed
        hedef_velocity = hedef_velocity * (max_target_speed / current_speed_target);
    end
        
    % 7. Yeni hedef konumunu hesapla (Euler)
    hedef_pos = hedef_pos + hedef_velocity * dt; 
    % --- HEDEF GÜNCELLEME SONU ---

    % Mevcut Durum (Robot)...
    x = z(1); y = z(2); theta = z(3); v_gercek = z(4); w_gercek = z(5);
    
    % Engel Kaçınma Hesaplamaları (Robot için - APF)...
    total_repulsive_vector_robot_xy = [0; 0]; 
    min_dist_to_obstacle_robot = inf; 
    for obs_idx = 1:num_obstacles
        obs_center_xy = obstacle_current_centers{obs_idx}(1:2); % Güncel engel konumu
        obs_size = obstacles_sizes(obs_idx);
        vec_to_obs = obs_center_xy - [x; y];
        dist_to_obs = norm(vec_to_obs);
        current_dist_to_surface = dist_to_obs - obs_size/2; 
        min_dist_to_obstacle_robot = min(min_dist_to_obstacle_robot, current_dist_to_surface); 
        if dist_to_obs < detection_radius_robot && dist_to_obs > 1e-6 
            repulsion_strength_robot = k_rep_robot * (1/dist_to_obs - 1/detection_radius_robot) * (1/dist_to_obs^2);
            repulsive_vector = - (vec_to_obs / dist_to_obs) * repulsion_strength_robot; 
            total_repulsive_vector_robot_xy = total_repulsive_vector_robot_xy + repulsive_vector;
        end
    end
    
    % Kontrolcü Adım 1: Yüksek Seviye (Robot için - APF)...
    vec_to_target = [hedef_pos(1) - x; hedef_pos(2) - y]; % hedef -> hedef_pos
    dist_to_target_xy = norm(vec_to_target);
    rho = dist_to_target_xy; 
    attractive_vector_xy = vec_to_target / (dist_to_target_xy + eps); 
    final_direction_vector_xy = attractive_vector_xy + total_repulsive_vector_robot_xy; 
    target_angle_combined = atan2(final_direction_vector_xy(2), final_direction_vector_xy(1));
    alpha = target_angle_combined - theta;
    alpha = atan2(sin(alpha), cos(alpha)); 
    v_istenen = kv * rho;
    w_istenen = kw * alpha;
    
    % Hız Azaltma (Robot için - APF)...
    effective_safety_margin_robot = robot_radius + safety_margin_robot; 
    if min_dist_to_obstacle_robot < effective_safety_margin_robot * 1.5 
       speed_scale = max(0, min(1, (min_dist_to_obstacle_robot - effective_safety_margin_robot*0.5) / (effective_safety_margin_robot * 1.0) )); 
       v_istenen = v_istenen * speed_scale;
       v_istenen = max(v_istenen, 0.05); 
    end

    % Kontrolcü Adım 2: Düşük Seviye...
    e_v = v_istenen - v_gercek; e_w = w_istenen - w_gercek;
    F = kF * e_v; Tau = kTau * e_w;
    
    % Fizik Modeli (Robot)...
    a_linear = (F - mu_v * v_gercek) / m;
    a_angular = (Tau - mu_w * w_gercek) / I;
    v_yeni = v_gercek + a_linear * dt;
    w_yeni = w_gercek + a_angular * dt;
    theta_yeni = theta + w_yeni * dt;
    x_yeni = x + v_yeni * cos(theta_yeni) * dt;
    y_yeni = y + v_yeni * sin(theta_yeni) * dt;
    
    % Durum Güncelleme ve Kayıt (Robot)...
    z = [x_yeni; y_yeni; theta_yeni; v_yeni; w_yeni];
    z_gecmis(:, i+1) = z;
    U_hist(:, i) = [F; Tau];
    
    % Yakalama Koşulu...
    if rho < (robot_radius + hedef_radius) 
        disp(['Hedef Yakalandı (XY Düzleminde)! Süre: ', num2str(t_anlik, '%.2f'), 's']);
        break; 
    end
    
    % Görselleştirme...
    if mod(i, 3) == 0 
        figure(fig_3d);
        % Önceki çizimleri sil (Robot ve Engeller)
        if isgraphics(h_robot_body)
            delete(h_robot_body); delete(h_yon_taret); delete(h_yol);
            delete(h_teker_R); delete(h_teker_L); 
        end
        for obs_idx = 1:num_obstacles
            if isgraphics(h_obstacles{obs_idx})
                delete(h_obstacles{obs_idx});
            end
        end

        % Hedefi yeni konumunda çiz
        set(h_hedef, 'XData', robot_shapes.X_h_base * hedef_radius + hedef_pos(1), ... 
                     'YData', robot_shapes.Y_h_base * hedef_radius + hedef_pos(2), ...
                     'ZData', robot_shapes.Z_h_base * hedef_radius + hedef_pos(3)); 
        
        % Engelleri yeni konumlarında tekrar çiz
        for obs_idx = 1:num_obstacles
            center = obstacle_current_centers{obs_idx}; 
            sz = obstacles_sizes(obs_idx) / 2; 
            corners = [center(1)-sz, center(2)-sz, 0; center(1)+sz, center(2)-sz, 0; center(1)+sz, center(2)+sz, 0; center(1)-sz, center(2)+sz, 0; 
                       center(1)-sz, center(2)-sz, sz*2; center(1)+sz, center(2)-sz, sz*2; center(1)+sz, center(2)+sz, sz*2; center(1)-sz, center(2)+sz, sz*2]; 
            faces = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];
            h_obstacles{obs_idx} = patch('Vertices', corners, 'Faces', faces, 'FaceColor', [0.5 0.5 0.5], 'FaceAlpha', 0.7, 'EdgeColor', 'k', 'HandleVisibility', 'off'); 
        end

        % Robotu çiz
        Xt = robot_shapes.Xt_base; Yt = robot_shapes.Yt_base; Zt = robot_shapes.Zt_base;
        teker_R_pos_x = x + sin(theta) * robot_genislik; teker_R_pos_y = y - cos(theta) * robot_genislik; 
        Xtr = Xt * cos(theta) - Yt * sin(theta) + teker_R_pos_x; Ytr = Xt * sin(theta) + Yt * cos(theta) + teker_R_pos_y;
        Ztr = Zt + teker_r; 
        h_teker_R = surf(Xtr, Ytr, Ztr, 'FaceColor', 'k', 'EdgeColor', 'none', 'DisplayName','Sağ Teker');
        teker_L_pos_x = x - sin(theta) * robot_genislik; teker_L_pos_y = y + cos(theta) * robot_genislik;
        Xtl = Xt * cos(theta) - Yt * sin(theta) + teker_L_pos_x; Ytl = Xt * sin(theta) + Yt * cos(theta) + teker_L_pos_y;
        Ztl = Zt + teker_r; 
        h_teker_L = surf(Xtl, Ytl, Ztl, 'FaceColor', 'k', 'EdgeColor', 'none', 'DisplayName','Sol Teker');
        X_r = robot_shapes.X_r_base * robot_radius; Y_r = robot_shapes.Y_r_base * robot_radius; Z_r = robot_shapes.Z_r_base * robot_radius;
        h_robot_body = surf(X_r + x, Y_r + y, Z_r + teker_r, 'FaceColor', 'b', 'EdgeColor', 'none', 'FaceAlpha', 0.8, 'DisplayName','Robot Gövdesi');
        taret_acisi_xy = theta; hedef_uzaklik_xy = rho; hedef_yukseklik_farki = hedef_pos(3) - taret_z_baslangic; 
        taret_acisi_z = atan2(hedef_yukseklik_farki, hedef_uzaklik_xy); 
        yon_vektoru_x = taret_uzunluk * cos(taret_acisi_xy) * cos(taret_acisi_z);
        yon_vektoru_y = taret_uzunluk * sin(taret_acisi_xy) * cos(taret_acisi_z);
        yon_vektoru_z = taret_uzunluk * sin(taret_acisi_z);
        h_yon_taret = quiver3(x, y, taret_z_baslangic, yon_vektoru_x, yon_vektoru_y, yon_vektoru_z, 'r', 'LineWidth', 2.5, 'MaxHeadSize', 3, 'DisplayName','Taret Yönü');
        h_yol = plot3(z_gecmis(1, 1:i), z_gecmis(2, 1:i), zeros(1,i), 'g--', 'DisplayName','Robot Yolu');
        
        figure(fig_vel);
        zaman_ekseni = t(1:i);
        subplot(2,1,1); set(h_v_d, 'XData', zaman_ekseni, 'YData', z_gecmis(4, 1:i)*0 + v_istenen); set(h_v_a, 'XData', zaman_ekseni, 'YData', z_gecmis(4, 1:i)); 
        subplot(2,1,2); set(h_w_a, 'XData', zaman_ekseni, 'YData', z_gecmis(5, 1:i)); set(h_w_d, 'XData', zaman_ekseni, 'YData', z_gecmis(5, 1:i)*0 + w_istenen); 
        
        drawnow;

        % Legend'ı sadece bir kere güncelle
        if i <= 3 
             figure(fig_3d); 
             legend('show', 'Location', 'northwest'); 
        end
    end
end % === DÖNGÜ SONU ===

hold off;
disp('Simülasyon tamamlandı.');