clear; clc; close all;
rng(42); % Seed cố định

mapSizes = 10:10:200;

timeA_arr = zeros(size(mapSizes));
nodeA_arr = zeros(size(mapSizes));
timeR_arr = zeros(size(mapSizes));
nodeR_arr = zeros(size(mapSizes));

for idx = 1:length(mapSizes)
    N = mapSizes(idx);
    
    % --- Tạo map ---
    map = zeros(N, N);
    
    % Thêm tường dọc cách 20% từ hai bên
    colLeft  = round(0.05*N);
    colRight = round(0.95*N);
    middle = round(0.5*N);
    map(colLeft:colRight, middle) = 1;
    
    % Start / goal
    start = [round(N/2), N-1];
    goal  = [round(N/2), 2];
    
    % --- Chạy A* ---
    tic;
    [pathA, expandedA] = AStar_core(map, start, goal);
    timeA_arr(idx) = toc;
    nodeA_arr(idx) = expandedA;
    
    % --- Chạy RRT ---
    tic;
    [pathR, treeR, successR] = RRT_Connect_core(map, start, goal);
    timeR_arr(idx) = toc;
    nodeR_arr(idx) = length(treeR);
    
    fprintf('Map %d x %d done. A*: %.4fs, RRT: %.4fs\n', N, N, timeA_arr(idx), timeR_arr(idx));
end

%figure('Name','Thời gian A*','Position',[100 100 800 500],'Color','w');
%plot(mapSizes, timeA_arr, 'b-o', ...
%    'LineWidth', 3, 'MarkerSize', 10, 'MarkerFaceColor', 'b');
%xlabel('Kích thước bản đồ (N × N)', 'FontSize', 14, 'FontWeight','bold');
%ylabel('Thời gian thực thi (giây)', 'FontSize', 14, 'FontWeight','bold');
%title('Thời gian thực thi của thuật toán A*', 'FontSize', 16, 'FontWeight','bold');
%grid on; box on;
%set(gca,'FontSize',12,'LineWidth',1.5);
%legend('A*','Location','northwest','FontSize',12);

%% === FIGURE 2: Thời gian chạy của RRT ===
figure('Name','So sánh A* vs RRT-Connect','Position',[100 100 900 600],'Color','w');
%plot(mapSizes, timeR_arr, 'r-s', ...
%    'LineWidth', 3, 'MarkerSize', 10, 'MarkerFaceColor', 'r');
plot(mapSizes, timeA_arr, 'b-o', mapSizes, timeR_arr, 'r-s', 'LineWidth',2);
xlabel('Kích thước bản đồ (N × N)', 'FontSize', 14, 'FontWeight','bold');
ylabel('Thời gian thực thi (giây)', 'FontSize', 14, 'FontWeight','bold');
title('Thời gian thực thi của 2 thuật toán', 'FontSize', 16, 'FontWeight','bold');
grid on; box on;
set(gca,'FontSize',12,'LineWidth',1.5);
legend('A*','RRT_Connect','Location','northwest');

%% === FIGURE 3: Biểu đồ so sánh tổng hợp (thời gian + số node) ===
figure('Name','So sánh A* vs RRT-Connect','Position',[100 100 900 600],'Color','w');

plot(mapSizes, nodeA_arr, 'b-o', mapSizes, nodeR_arr, 'r-s', 'LineWidth',2);
xlabel('Kích thước map (N x N)');
ylabel('Số node mở rộng / cây');
title('Số node A* vs RRT');
legend('A*','RRT-Connect','Location','northwest');
grid on;

%% --- Vẽ map cuối cùng và đường đi ---
N = mapSizes(end); % map lớn nhất
map = zeros(N, N);
colLeft  = round(0.05*N);
colRight = round(0.95*N);
middle = round(0.5*N);
map(colLeft:colRight, middle) = 1;

start = [round(N/2), N-1];
goal  = [round(N/2), 2];

[pathA, expandedA] = AStar_core(map, start, goal);
[pathR, treeR, successR] = RRT_core(map, start, goal);

figure('Color','w','Position',[100 100 800 700]);
imagesc(map'); axis equal tight; colormap(flipud(gray)); hold on;
title(sprintf('Map %d x %d: A* (xanh) vs RRT (đỏ)', N, N),'FontSize',14);

% Vẽ tường
[X,Y] = meshgrid(1:size(map,2),1:size(map,1));
contour(X,Y,map','k','LineWidth',1.5);

% Vẽ start / goal
plot(start(1), start(2), 'go', 'MarkerSize',12, 'LineWidth',3, 'DisplayName','Start');
plot(goal(1),  goal(2),  'bp', 'MarkerSize',12, 'LineWidth',3, 'DisplayName','Goal');

% Vẽ đường đi
if ~isempty(pathA)
    plot(pathA(:,1), pathA(:,2), 'c-', 'LineWidth',3, 'DisplayName','A* Path');
end
if ~isempty(pathR)
    plot(pathR(:,1), pathR(:,2), 'r-', 'LineWidth',2.5, 'DisplayName','RRT Path');
end

legend('Location','southoutside','FontSize',12);
grid on;
