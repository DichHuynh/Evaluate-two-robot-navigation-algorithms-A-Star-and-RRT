% Hàm tạo vật cản nhanh
function map = addObstacle(map, rowRange, colRange, value)
    map(rowRange, colRange) = value;
end