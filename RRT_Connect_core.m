function [path, tree, success] = RRT_Connect_core(map, start, goal)
% RRT-CONNECT cho bản đồ 2D dạng grid
% Tương thích 100% với code RRT trước của bạn

    maxIter   = 10000;
    stepSize  = 10;
    goalThresh = 5;

    % Hai cây RRT: từ start và từ goal
    treeA = start;
    parentA = 0;

    treeB = goal;
    parentB = 0;

    success = false;

    for iter = 1:maxIter
        
        % ---- 1. Sample ngẫu nhiên ----
        rnd = [randi(size(map,1)), randi(size(map,2))];

        % ---- 2. Extend tree A ----
        [treeA, parentA, newA, ok1] = extend(treeA, parentA, rnd, map, stepSize);
        if ~ok1, continue; end

        % ---- 3. Connect tree B đến newA ----
        [treeB, parentB, newB, ok2] = connect(treeB, parentB, newA, map, stepSize);

        % ---- 4. Nếu hai cây chạm nhau ----
        if ok2 && norm(newA - newB) < goalThresh
            success = true;
            path = build_path(treeA, parentA, treeB, parentB, newA, newB);
            tree = [treeA; treeB];    % hợp cây để vẽ
            return;
        end

        % ---- 5. Đổi vai 2 cây ----
        [treeA, treeB] = deal(treeB, treeA);
        [parentA, parentB] = deal(parentB, parentA);

    end

    % Nếu thất bại
    path = [];
    tree = [treeA; treeB];
end
function [tree, parent, newNode, ok] = extend(tree, parent, target, map, step)

    ok = false;
    nearestID = findNearest(tree, target);
    nearest = tree(nearestID,:);

    dir = target - nearest;
    dist = norm(dir);
    if dist < 1
        newNode = nearest;
        return;
    end

    dir = dir / dist;
    newNode = round(nearest + dir * step);

    if collisionCheck(nearest, newNode, map)
        return;
    end

    tree = [tree; newNode];
    parent = [parent; nearestID];
    ok = true;
end
function [tree, parent, lastNode, ok] = connect(tree, parent, target, map, step)

    ok = false;
    lastNode = tree(end,:);

    while true
        [treeNew, parentNew, newNode, done] = extend(tree, parent, target, map, step);
        if ~done
            return;
        end

        tree = treeNew;
        parent = parentNew;
        lastNode = newNode;

        if norm(newNode - target) < step
            ok = true;
            return;
        end
    end
end
function idx = findNearest(tree, point)
    diff = tree - point;
    dist2 = sum(diff.^2, 2);
    [~, idx] = min(dist2);
end
function hit = collisionCheck(a, b, map)
    % returns true if any point along line a->b touches an obstacle (value==1)
    pts = bresenhamLine(a(1), a(2), b(1), b(2));
    % clamp pts to map bounds
    [mx,my] = size(map);
    pts(:,1) = max(1, min(mx, pts(:,1)));
    pts(:,2) = max(1, min(my, pts(:,2)));
    ind = sub2ind(size(map), pts(:,1), pts(:,2));
    hit = any(map(ind) == 1);
end

function path = build_path(treeA, parentA, treeB, parentB, a, b)

    idA = find(ismember(treeA, a, 'rows'));
    idB = find(ismember(treeB, b, 'rows'));

    % Truy ngược cây A
    pA = [];
    while idA > 0
        pA = [treeA(idA,:); pA];
        idA = parentA(idA);
    end

    % Truy ngược cây B (nhưng đảo ngược lại)
    pB = [];
    while idB > 0
        pB = [pB; treeB(idB,:)];
        idB = parentB(idB);
    end

    path = [pA; pB];
end
function pts = bresenhamLine(x1,y1,x2,y2)
% Return Nx2 integer points along line from (x1,y1) to (x2,y2)
% Coordinates assumed integer grid. Output rows are [x y].
    x1 = round(x1); y1 = round(y1);
    x2 = round(x2); y2 = round(y2);

    dx = abs(x2 - x1);
    sx = sign(x2 - x1);
    dy = -abs(y2 - y1);
    sy = sign(y2 - y1);
    err = dx + dy;

    x = x1; y = y1;
    pts = [x,y];
    while ~(x == x2 && y == y2)
        e2 = 2*err;
        if e2 >= dy
            err = err + dy;
            x = x + sx;
        end
        if e2 <= dx
            err = err + dx;
            y = y + sy;
        end
        pts(end+1, :) = [x,y]; %#ok<AGROW>
    end
end
