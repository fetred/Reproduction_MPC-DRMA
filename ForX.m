%% 计算与停止线距离函数
function x1 = ForX(car)
global stoplineclass stoplinepoints;

    link = car.AttValue('LINK');
    point = [car.AttValue('POINT').x,car.AttValue('POINT').y];

    %%停止线前后判断，link>=9，停止线后
    if link >= 9
        sign = -1;
    else 
        sign = 1;
    end
    
    %%计算与停止线的距离
    x1 = -1;
    for lane=1:4
        if ismember(link,stoplineclass{lane})
            mystoplinepoint = stoplinepoints(lane);
            if lane == 1 || lane == 3
                x1 = sign * abs(point(2)-mystoplinepoint);
            elseif lane == 2 || lane == 4
                x1 = sign * abs(point(1)-mystoplinepoint);
            break
            end
        end
    end
