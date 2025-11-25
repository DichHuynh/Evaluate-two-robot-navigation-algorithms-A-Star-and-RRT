%% ================== SO SÁNH CÔNG BẰNG A* vs RRT ==================
clear; clc; close all;

% ------------------ Thiết lập bản đồ chung ------------------
map = zeros(200, 200);                    % Bản đồ 200x200
%map = addObstacle(map, 60:140, 90, 1);     % Tường dọc giữa
%map = addObstacle(map, 60, 50:140, 1);     % Cửa vào bẫy hình chữ U
%map = addObstacle(map, 140, 50:140, 1);   % Cửa ra bẫy (narrow passage)
%map = addObstacle(map, 20:180, 20, 1);     % Tường ngang trên
%map = addObstacle(map, 20:180, 180, 1);    % Tường ngang dưới
%map(80:100, 20:60) = 1; 

%map = addObstacle(map, 20, 60:160, 1);
%map = addObstacle(map, 80, 1:60, 1);
%map = addObstacle(map, 180, 80:140, 1);

%map = addObstacle(map, 20:60, 60, 1);
%map = addObstacle(map, 140:180, 140, 1);
%map = addObstacle(map, 180:200, 80, 1);

start = [20, 170];
goal  = [180, 10];

% Seed cố định để RRT cho kết quả giống nhau mỗi lần (công bằng!)
rng(42);  

fprintf('=====================================\n');
fprintf('   SO SÁNH A* vs RRT TRÊN CÙNG MAP \n');
fprintf('=====================================\n');

% ------------------ Chạy A* (đã sửa: dùng priority queue nhanh) ------------------
tic;
[pathA, expandedA] = AStar_core(map, start, goal);
timeA = toc;

% ------------------ Chạy RRT (đã sửa: kiểm tra va chạm đoạn thẳng + smoothing) ------------------
tic;
[pathR, treeR, successR] = RRT_core(map, start, goal);
timeR = toc;

% ------------------ In kết quả ------------------
fprintf('\n===== KẾT QUẢ SO SÁNH =====\n');
fprintf('A*   - Thời gian: %.4f s | Node mở rộng: %d | ', ...
    timeA, expandedA);
if ~isempty(pathA), fprintf('Tìm được đích!\n'); else fprintf('THẤT BẠI\n'); end

fprintf('RRT  - Thời gian: %.4f s | Node trong cây: %d | ', ...
    timeR, length(treeR));
if successR, fprintf('Tìm được đích!\n'); else fprintf('THẤT BẠI\n'); end

% ------------------ Vẽ kết quả đẹp ------------------
figure('Position',[100 100 900 800],'Color','w');
imagesc(map'); axis equal tight; colormap(flipud(gray)); hold on;
title('So sánh A* (xanh) vs RRT (đỏ) - Cùng bản đồ, cùng start/goal','FontSize',14);

% Vẽ vật cản
[X,Y] = meshgrid(1:size(map,2),1:size(map,1));
contour(X,Y,map','k','LineWidth',1.5);

plot(start(1), start(2), 'go', 'MarkerSize',12, 'LineWidth',3, 'DisplayName','Start');
plot(goal(1),  goal(2),  'bp', 'MarkerSize',12, 'LineWidth',3, 'DisplayName','Goal');

if ~isempty(pathA)
    plot(pathA(:,1), pathA(:,2), 'c-', 'LineWidth',3, 'DisplayName','A* Path');
end
if ~isempty(pathR)
    plot(pathR(:,1), pathR(:,2), 'r-', 'LineWidth',2.5, 'DisplayName','RRT Path');
end

legend('Location','southoutside','FontSize',12);
grid on;