function [pathR, treeR, successR] = RRT_core(map, start, goal)
    %% Tham số thuật toán
    iter      = 6000;    % số lượng vòng lặp tối đa
    stepSize  = 10;       % bước mở rộng mỗi lần
    goalProb  = 0.9;     % xác suất chọn goal làm target
    distThresh= 5;       % khoảng cách để coi là chạm goal
    
    %% Khởi tạo
    emptyNode = 0;
    successR = 0;
    mapSize = size(map);
    
    treeR(1).x = start(1);
    treeR(1).y = start(2);
    treeR(1).parent = 0;
    
    %% Vòng lặp sinh cây RRT
    for i = 1:iter
        target = ChooseTarget(goal, mapSize, goalProb);
        
        % Tìm node gần target nhất
        [nearestNode, nearestNodeID] = NearestNode(treeR, target);
        
        % Mở rộng về target
        extended = Extend(map, nearestNode, target, stepSize);
        
        % Nếu không va chạm -> thêm node mới
        if extended ~= emptyNode
            treeR = AddNode(treeR, extended, nearestNodeID);
            
            % Kiểm tra xem có đạt goal không
            if Dist(extended, goal) < distThresh
                successR = 1;
                break;
            end
        end
    end
    
    %% Backtrack để lấy đường đi nếu thành công
    pathR = [];
    if successR
        currNodeID = length(treeR);
        while currNodeID ~= 0
            pathR = [ [treeR(currNodeID).x, treeR(currNodeID).y]; pathR ];
            currNodeID = treeR(currNodeID).parent;
        end
        pathR = [pathR; goal]; % thêm điểm goal cuối cùng
    end
    
end

%% --- Các hàm phụ ---
function target = ChooseTarget(goal, mapSize, goalProb)
    if rand < goalProb
        target = goal;
    else
        target = [randi(mapSize(1)), randi(mapSize(2))];
    end
end

function [nearestNode, nearestNodeID] = NearestNode(tree, target)
    dists = arrayfun(@(n) sqrt((n.x-target(1))^2 + (n.y-target(2))^2), tree);
    [~, nearestNodeID] = min(dists);
    nearestNode = [tree(nearestNodeID).x, tree(nearestNodeID).y];
end

function extended = Extend(map, nearest, target, stepSize)
    dir = target - nearest;
    dist = norm(dir);
    if dist == 0
        extended = 0;
        return;
    end
    step = stepSize * dir / dist;
    newPos = round(nearest + step);
    
    % Kiểm tra ngoài biên
    if newPos(1) < 1 || newPos(2) < 1 || newPos(1) > size(map,1) || newPos(2) > size(map,2)
        extended = 0;
        return;
    end
    
    % Kiểm tra toàn bộ đoạn thẳng từ nearest -> newPos
    numPoints = ceil(norm(newPos - nearest));
    for i = 0:numPoints
        pt = round(nearest + (i/numPoints)*(newPos - nearest));
        if map(pt(1), pt(2)) == 1
            extended = 0; % va chạm
            return;
        end
    end
    
    extended = newPos;
end


function tree = AddNode(tree, extended, parentNodeID)
    tree(end+1).x = extended(1);
    tree(end).y = extended(2);
    tree(end).parent = parentNodeID;
end

function d = Dist(p1, p2)
    d = sqrt((p1(1)-p2(1))^2 + (p1(2)-p2(2))^2);
end
