clear;
vissim = actxserver('VISSIM.vissim.430');
current_path = 'D:\data\matlab_data\MPC-DRMA_trial';
vissim.LoadNet([current_path,'\trial01.inp']);
sim = vissim.Simulation;
sim.RandomSeed = randi([1,100]);
sim.Period = 600;
sim.Speed = 1;
vnet = vissim.Net;

%% data
global collisionpoints stoplineclass stoplinepoints;
collisionpoints= {[37.3,25.1],[37.3,33.7],[29.5,33.7],[29.5,25.1]};     %碰撞点
%%停止线判断信息
stoplineclass = {[1,2,10000,10001,10002,10003],[3,4,10004,10005,10006,10007],[5,6,10008,10009,10010,10011],[7,8,10012,10013,10014,10015]};
stoplinepoints = [15.5,45,5,40.5,20.5];
collisionignore = [9,10,11,12,13,14,15,16,10003,10007,10011,10015];     %出口和右转
collisionignorepairing1 = {[1,2,10000,10001,10002],[3,4,10004,10005,10006],[5,6,10008,10009,10010],[7,8,10012,10013,10014]};     %同向
collisionignorepairing2 = {[10001,10009],[10005,10013]};    %对向左转
collisionignorepairing = [collisionignorepairing1,collisionignorepairing2];
pointpredpairing1 = {[1,2,13,14,10000,10002],[3,4,15,16,10004,10006],[5,6,10008,10010,9,10],...
    [7,8,10012,10014,11,12],[10003,10013],[10005,10011],[10001,10007],[10009,10015]};
pointpredvec = {[0,1],[-1,0],[0,-1],[1,0],[0.707,0.707],[-0.707,-0.707],0.707*[-1,1],0.707*[1,-1],};
vd = 60/3.6;     %期望速度
vmin = 3;   %速度下界
vmax = 23;  %速度上界
umin = -5;umax = 4; %加速度上下界
Rmin = 7;
G = 1500;
PU = 0.5;
T = 5;      %预测长度
step = 0.4;     %预测步长

%% 开始按步仿真
for ii = 1:6000
    sim.RunSingleStep;
    vehicles = vissim.Net.Vehicles(1);
    vehiclesIDs = vehicles.IDs;
    vehicleNum = vehicles.Count;
    
    %% 转化车辆ID列表
    n = 0;
    ID = 1;
    IDs = zeros(1,vehicleNum);
    while n < vehicleNum
        if isempty(vehicles.GetVehicleByNumber(ID)) ==0
            n = n+1;
            IDs(n) = ID;
        end
        ID = ID + 1;
    end
    
    %% 计算
    if vehicleNum > 1
        % run('caculation.m');
        % load('caculation.mat')
        % run('solver.m');
        % load('solver.mat');
        % 

        %% caculation%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        %% 定义变量
        X = [];
        Point = [];
        X0 = [];
        U = [];
        V = [];
        V0 = [];
        ss03 = [];

        %%生成加速度矩阵U
        for i = 1:vehicleNum
            eval(['U',num2str(i),'J = [];']);
            for j = 1:T
                eval(['syms u',num2str(i),num2str(j)]);
                eval(['U',num2str(i),'J=[U',num2str(i),'J,u',num2str(i),num2str(j),'];']);
            end
            eval(['U = [U;U',num2str(i),'J];']);
        end
        
        
        %% 遍历所有车,读取状态量
        n = 1;
        for veh = IDs
            MyVehicle = vehicles.GetVehicleByNumber(veh);
            % disp('车辆INTERACTION状态');
            % disp(MyVehicle.AttValue('INTERACTION'));
            % MyVehicle.set('AttValue','INTERACTION',dec2hex(0));
            % disp('更改后车辆INTERACTION状态');
            % disp(MyVehicle.AttValue('INTERACTION'));
            MyV = MyVehicle.AttValue('SPEED')/3.6;
            V0 = [V0;MyV];
        
            %% 预测速度表达式
            eval(['V',num2str(n),'T = [MyV];']);
            s = MyV;
            for t = 1:T
                s = s+U(n,t)*step;  %速度模型
                eval(['V',num2str(n),'T = [V',num2str(n),'T,s];']);
            end
            eval(['V = [V;V',num2str(n),'T];'])
        


            %% 预测坐标
            sumpoint = [MyVehicle.AttValue('POINT').x,MyVehicle.AttValue('POINT').y];
            MyVehiclePoint = [MyVehicle.AttValue('POINT').x,MyVehicle.AttValue('POINT').y];
            for pair = 1:length(pointpredpairing1)
                if ismember(MyVehicle.AttValue('LINK'),pointpredpairing1{pair})
                    setpointvec = pointpredvec(pair);
                    for t = 1:T
                        sumpoint = sumpoint + V(end,t).*step.*setpointvec;
                        MyVehiclePoint = [MyVehiclePoint;sumpoint];
                    end
                end
            end
            Point = [Point;MyVehiclePoint];
        
            n = n+1;

            %% 计算与停止线的距离
            x1 = ForX(MyVehicle);
            X = [X;x1];
        end

        %% 预测与停止线距离表达式
        for t = 1:T
            Xn = X(:,end)-V(:,t)*step - 0.5*U(:,t)*step.^2;   %位移模型
            X = [X,Xn];
        end
        
        %% 计算碰撞风险
        S3 = [];
        sign1 = 0;
        n = 1;
        for ID1 = IDs
            s3 = 0;
            eval(['S3',num2str(n),'T=[];']);
            MyVehicle = vehicles.GetVehicleByNumber(ID1);
            if ismember(MyVehicle.AttValue('LINK'),collisionignore)
                eval(['S3',num2str(n),'T=zeros(1,T);']);
                n = n+1;
                break
            end
            
            sign1 = sign1 + 1;
            for ID2 = IDs
                TerVehicle = vehicles.GetVehicleByNumber(ID2); 
                %% 筛同车道
                if MyVehicle.AttValue('LINK') == TerVehicle.AttValue('LINK')
                    continue
                end
                %% 筛其他无冲突车道
                isignore = 0;
                for pairNum = 1:length(collisionignorepairing)
                    if ismember(MyVehicle.AttValue('LINK'),collisionignorepairing{pairNum}) && ...
                            ismember(TerVehicle.AttValue('LINK'),collisionignorepairing{pairNum})
                        isignore = 1;
                        break
                    end
                end
                if isignore == 1
                    continue
                end
        
                %% 计算
                Myx = X(find(IDs == ID1),:);
                Terx = X(find(IDs == ID2),:);
                s03 = (abs(Myx+10)).^2 + (abs(Terx+10)).^2;
                s3 = s3 + 2500*exp(-1*0.005*s03);
                if sign1 == 1
                    ss03 = [ss03;s03(2:end)];   %冲突相位最小间隔距离约束用
                end
            end
           S3 = [S3;s3];
        end
        SS3 = S3(:,1:end-1);
        sumS3 = sum(SS3(:));     %总碰撞风险
        
        %% 计算期望速度偏离
        Vd = ones(vehicles.Count,T)*vd;
        S2 = 0.6*(V(:,1:end-1) - Vd).^2;
        sumS2 = sum(S2(:));     %总速度偏离
        
        %% 计算加速成本
        UU = PU*U.^2;
        sumS1 = sum(UU(:));
        
        %% 计算行车风险
        RiskSign = 1;
        s4 = 0;
        for ID1 = IDs
            MyVehicle = vehicles.GetVehicleByNumber(ID1);
            for ID2 = IDs
                if ID2 == ID1
                    continue
                end
                TerVehicle = vehicles.GetVehicleByNumber(ID2);
                for pairRisk = collisionignorepairing
                    if ismember(MyVehicle.AttValue('LINK'),pairRisk{1}) && ismember(TerVehicle.AttValue('LINK'),pairRisk{1})
                        RiskSign = 0;
                        break
                    end
                end
                if MyVehicle.AttValue('LINK') == TerVehicle.AttValue('LINK')
                    RiskSign = 1;
                end
                if RiskSign == 0
                    continue
                end
                %s4 = s4 + G * TerVehicle.AttValue('WEIGHT')/(ForR(MyVehicle,TerVehicle));
                for iRisk = 1:T+1
                    s4_s = G * TerVehicle.AttValue('WEIGHT')*1000/(ForR1(Point((find(IDs==ID1)-1)*(T+1)+iRisk,:),...
                        Point((find(IDs==ID2)-1)*(T+1)+iRisk,:)))^2;
                    s4 = s4 + s4_s;
                end
            end
        end
        sumS4 = s4;



        
        %% solver%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        Ju = sumS1 + sumS2 + sumS3 + sumS4;
        uvec = reshape(U,[],1);
        fun = matlabFunction(Ju,'Vars',{uvec});


        %% 目标函数
        uNum = numel(U);    %变量数
        lb = ones(uNum,1) * umin;    ub = ones(uNum,1) * umax;    %上下界
        x0 = zeros(uNum,1);     %初始值
        Aeq = [];   beq = 0;    %无等式约束
        ceq = [];
        
        %% fmincon求解
        Uo = fmincon(fun,x0,[],[],[],[],lb,ub,[]);





        disp(Uo);

        for ID = IDs
            SETVehicle = vehicles.GetVehicleByNumber(ID);
            disp('输入前速度');
            disp(SETVehicle.AttValue('SPEED'));
            % disp('输入前速度');
            % disp(SETVehicle.AttValue('SPEED'));
            disp('加速度');
            disp(Uo(find(IDs==ID)));
            vset = round((SETVehicle.AttValue('SPEED')/3.6 + Uo(find(IDs==ID)) * step )*3.6);
            disp('输入速度');
            disp(vset);
            SETVehicle.set('AttValue','SPEED',num2str(vset));
        end
    end

    % %% 两车以下跳过等待
    % if vehicleNum <= 1  
    %     continue
    % end
    % 
    % pause(0.2)    
    sim.RunSingleStep;
    pause(0.2);

end
sim.Stop