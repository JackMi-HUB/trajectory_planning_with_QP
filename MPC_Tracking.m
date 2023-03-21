function[] = MPC_Tracking(Pos,Vel)

% 定义轨迹
steps = size(Pos,1);
dt = 0.1;
fin = steps*dt;
t = linspace(1,fin,steps);

lw=0.1;%lw为两轮距的半长
cw=10;%cw为角速度参数
v_max = 5;

%定义期望轨迹（包括角度）和速度
x = Pos(:,1)';
y = Pos(:,2)';
v = ones(1,steps);
w = ones(1,steps);
theta = Pos(:,3)';
% for i = 1:steps   
%     v(i) = sqrt(Vel(i,1)^2+Vel(i,2)^2);
% end

for i = 1:steps-1
    w(i) = angle_bound(theta(i+1)-theta(i))/dt;%合理性待测试
    vx =  (x(i+1)-x(i))/dt;
    vy =  (y(i+1)-y(i))/dt;
    v(i) =  sqrt(vx^2+vy^2);
%     v(i) = v_max - lw*w(i);
end
w(steps) = 0;
v(steps) = v(steps-1);
v = delete_abnormal_v(v);
w = delete_abnormal_w(w);

% x = ones(1,steps); % reference x
% y = ones(1,steps); % reference y
% for i = 1:steps
%     % 要修改轨迹，改这个地方就行
%     x(i) = 2*cos(0.5*i*dt);
%     y(i) = sin(0.5*i*dt);
% end

% v = ones(1,steps); % reference v
% theta = zeros(1,steps); % reference theta
% w = zeros(1,steps); % reference omiga
% for i =1:steps-1
%     vx =  (x(i+1)-x(i))/dt;
%     vy =  (y(i+1)-y(i))/dt;
%     v(i) =  sqrt(vx^2+vy^2);
%     theta(i)= atan2(vy,vx);
% end
% v(steps) = v(steps-1);%这里是为了考虑连续性？
% theta(steps) = theta(steps-1);
% for i =1:steps-1
%     w(i)= angle_bound(theta(i+1)-theta(i))/dt;
% end
% w(steps) = 0;


% 初始状态
pose = [x(1),y(1),theta(1)]; % x y theta
control = [0,0]; % v w
n = size(pose,2); % 状态变量个数
p = size(control,2); % 控制变量个数

% 机器人模型
I=eye(n);



A = zeros(n); % 3*3
B = zeros(n,p); % 3*2

% 控制
N = 30; % 预测步
Q=[1 0 0;0 1 0;0 0 5]; % 定义Q矩阵，n x n 矩阵，x_err y_err theta_err权重；权重的设定？
R=[1 0;0 1]; % 定义R矩阵，p x p 矩阵 delta_v delta_w 权重

X = [pose,control]; %机器人最新的状态
X_real = []; %记录机器人每一时刻的状态，进行画图
X_real = [X_real;X];

for k = 1 : steps %每个点预测20步，但最后面的点预测的次数会逐渐减少直至0
    N = min(N,steps-k);
    if N == 0
        break
    end

    x0=[X(1)-x(k),X(2)-y(k),angle_bound(X(3)-theta(k))]; %x0存储了deta_State，误差变量

    %离散化：A B矩阵就是Ak Bk矩阵
    A = zeros(n,n,N);
    B = zeros(n,p,N); %n为状态变量的个数，N为预测的步数，p为控制变量个数
    for i = 1:N
        A(1,3,i)=-1*v(k+i-1)*sin(theta(k+i-1));
        A(2,3,i)= v(k+i-1)*cos(theta(k+i-1));
        A(:,:,i) = A(:,:,i)*dt+I;

        B(1,1,i) = cos(theta(k+i-1))*dt;
        B(2,1,i) = sin(theta(k+i-1))*dt;
        B(3,2,i) = 1*dt;
    end

    % Ahat Bhat 就是那个A B矩阵？
    Ahat = zeros(n*N,n); %n为状态变量的个数，N为预测的步数，p为控制变量个数
    Bhat = zeros(n*N,p*N);
    Atemp=1;
    for i=1:N
        Atemp=Atemp*A(:,:,i);
        Ahat(n*(i-1)+1:i*n,:)=Atemp;
        Btemp=1;
        for kk=1:i
            j=i-kk+1;
            Bhat((i-1)*n+1:i*n,(j-1)*p+1:j*p)=Btemp*B(:,:,j);
            Btemp=Btemp*A(:,:,j);
        end
    end

    %二次规划标准式 (X'*(1/2 *H) *X +f'*X 最小值
    Q_bar = kron(eye(N),Q);
    R_bar = kron(eye(N),R);
    
    H = Bhat'*Q_bar*Bhat + R_bar;
    H = (H+H')/2; %确保H阵为对称阵
    f = 2*(Ahat*x0')'*Q_bar*Bhat;
    
    lb=-10*ones(p*N,1);%lb和ub是对欲优化的变量的阈值设定，这里设定控制量在[-10,10]的范围内
    ub=10*ones(p*N,1);
    [dertau, fval, exitflag]=quadprog(H,f',[],[],[],[],lb,ub);%该优化问题暂未考虑不等式约束，需要根据实际的实验对象来设置(最大允许速度、角速度)
    u1=dertau(1)+v(k); %v(k),w(k) are referential v&w;这里可以理解为增量式优化，最终=实际的控制+优化量
    u2=dertau(2)+w(k);

    %将控制量输入对象----X=[x,y,theta,v,w]'
    X(4) = u1; %为什么这里要乘以一个正态分布的随机数+0.05*randn？
    X(5) = u2;
    X(1) = X(1)+ X(4)*cos(X(3))*dt;
    X(2) = X(2)+ X(4)*sin(X(3))*dt;
    X(3) = X(3)+ X(5)*dt;
    X(3) = angle_bound(X(3));
    X_real = [X_real;X];
end

% 画图
% X_real = [X_real;X];
figure(4);
plot(X_real(:,1),X_real(:,2),'ro');
hold on;
plot(x,y,'b-');
title("path");

figure(5);
plot(X_real(:,4),'ro');
hold on;
plot(v,'b-');
title("vel");

figure(6);
plot(X_real(:,5),'ro');
hold on;
plot(w,'b-');
title("AngularVel");

figure(7);
position_err = [];
vel_err = [];
w_err = [];
for i=1:steps
    position_err = [position_err,sqrt((x(i)-X_real(i,1))^2+(y(i)-X_real(i,2))^2)];
    vel_err = [vel_err,v(i)-X_real(i,4)];
    w_err = [w_err,w(i)-X_real(i,5)];
end
plot(position_err,'b-');
hold on;
plot(vel_err,'r-');
hold on;
plot(w_err,'k-');
title('The error');
legend({'P-Err','V-Err','W-Err'},'Location','northeast');

% figure(5);
% plot(t,X_real(:,1),"ro");
% hold on;
% plot(t,x);
% title("X");
% 
% figure(6);
% plot(t,X_real(:,2),"ro");
% hold on;
% plot(t,y);
% title("Y");

end

function res = angle_bound(x)
    res=x;
    for i=1:size(res,2)
        while res(i)<-pi 
            res(i)=res(i)+2*pi;
        end
        while res(i)>pi
           res(i)=res(i)-2*pi;
        end
    end
    for i=1:size(res,1)
        while res(i)<-pi
            res(i)=res(i)+2*pi;
        end
        while res(i)>pi
           res(i)=res(i)-2*pi;
        end
    end
end