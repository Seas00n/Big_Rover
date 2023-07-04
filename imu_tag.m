clear all;clc;close all;
load('data.mat')
% 数据说明：
% data[0,1,2]: aX,aY,aZ去除重力后的加速度
% data[3,4,5]: Gx,Gy,Gz角速度
% data[6,7,8]: Ax,Ay,Az角度
% data[9,10,11]: x,y,z坐标
% data[12,13,14]: tagx,tagy,tagz坐标
% data[15,16,17,18]: qx,qy,qz,qw四元数
% data[19]时间
data = data(2:end,:);
a_imu = data(:,1:3);
a_imu = a_imu-a_imu(1,:);
w_imu = data(:,4:6);
A_imu = data(:,7:9)*pi/180;
A_imu = A_imu-A_imu(1,:);
p_imu = data(:,10:12);
p_tag = data(:,13:15);
q_tag = data(:,16:19);
t = data(:,20);
t = t-t(1);
% 处理apriltag数据
se3_tag = zeros(4,4,size(data,1));
for i=1:size(data,1)
    Q = UnitQuaternion([q_tag(i,4),q_tag(i,1),q_tag(i,2),q_tag(i,3)]);
    T = p_tag(i,:);
    Q_SE3 = SO3ToMatrix(Q.SO3(),T);
    se3_tag(:,:,i) = Q_SE3;
end
%myse3animate(se3_tag,'b');
% 处理imu数据
p_imu = cal_pimu_world(a_imu,A_imu,t);
se3_imu = zeros(4,4,size(data,1));
for i=1:size(data,1)
    R = SO3.convert(eul2rotm(A_imu(i,:),"XYZ"));
    T = p_imu(i,:)+p_tag(1,:);
    se3_imu(:,:,i) = SO3ToMatrix(R,T);
end
%myse3animate(se3_imu,'r');
myse3animatecompare(se3_tag,se3_imu);

%利用imu修正tag朝向
A_imu_new = A_imu;
A_imu_new(:,1) = A_imu_new(:,1)+pi/2;

quan_tag_list = zeros(4,size(data,1));
quan_imu_list = zeros(4,size(data,1));
for i = 1:size(data,1)
    quan_tag =UnitQuaternion([q_tag(i,4),q_tag(i,1),q_tag(i,2),q_tag(i,3)]);
    quan_tag_list(1,i) = quan_tag.s;
    quan_tag_list(2:4,i) = quan_tag.v;
    quan_imu =UnitQuaternion(SO3.convert(eul2rotm(A_imu_new(i,:),"XYZ")));
    quan_imu_list(1,i) = quan_imu.s;
    quan_imu_list(2:4,i) = quan_imu.v;
end
% plot(t,quan_imu_list);
% legend('w','x','y','z')
% ylabel('Quaternion of IMU')
% xlabel('time/s')
% figure(2)
% plot(t,quan_tag_list);
% legend('w','x','y','z')
% ylabel('Quaternion of AprilTag')
% 利用tag修正积分
p_ekf = cal_p_ekf(a_imu,A_imu,p_tag,t);
se3_ekf = zeros(4,4,size(data,1));
for i=1:size(data,1)
    R = SO3.convert(eul2rotm(A_imu(i,:),"XYZ"));
    T = p_ekf(i,:)+p_tag(1,:);
    se3_ekf(:,:,i) = SO3ToMatrix(R,T);
end
figure(2)
myse3animatecompare(se3_tag,se3_ekf);










function Q_SE3 = SE3ToMatrix(se3)
    Q_SE3 = zeros(4,4);
    Q_SE3(4,4) =1;
    Q_SE3(1:3,1)=se3.n;
    Q_SE3(1:3,2)=se3.o;
    Q_SE3(1:3,3)=se3.a;
    Q_SE3(1:3,4)=se3.t;
end
function Q_SE3 = SO3ToMatrix(R,T)
    Q_SE3 = zeros(4,4);
    Q_SE3(4,4) =1;
    Q_SE3(1:3,1)=R.n;
    Q_SE3(1:3,2)=R.o;
    Q_SE3(1:3,3)=R.a;
    Q_SE3(1,4)=T(1);
    Q_SE3(2,4)=T(2);
    Q_SE3(3,4)=T(3);
end
function myse3animate(se3_list,color)
    i = 1;
    xlim([-5,5])
    ylim([-2,10])
    zlim([-5,5])
    while i<size(se3_list,3)
        se3 = se3_list(:,:,i);
        a = trplot(SE3.convert(se3),'length', 0.5, 'retain', 'rgb', 'notext');
        px = [se3(1,4)-0.05,se3(1,4)+0.05];
        py = [se3(2,4)-0.01,se3(2,4)+0.05];
        pz = [se3(3,4)-0.05,se3(3,4)+0.05];
        hold on
        xlim([-5,5])
        ylim([-2,10])
        zlim([-5,5])
        plot3(px,py,pz,'LineWidth',2,'Color',color);
        pause(0.01)
        i = i+10;
        delete(a)
        drawnow limitrate
    end
end
function myse3animatecompare(se3_list1,se3_list2)
    i = 1;
    xlim([-5,5])
    ylim([-2,10])
    zlim([-5,5])
    while i<size(se3_list1,3)
        se3_1 = se3_list1(:,:,i);
        se3_2 = se3_list2(:,:,i);
        a = trplot(SE3.convert(se3_1),'length', 0.5, 'retain', 'rgb', 'notext');
        px = [se3_1(1,4)-0.05,se3_1(1,4)+0.05];
        py = [se3_1(2,4)-0.01,se3_1(2,4)+0.05];
        pz = [se3_1(3,4)-0.05,se3_1(3,4)+0.05];
        b = trplot(SE3.convert(se3_2),'length', 0.5, 'retain', 'rgb', 'notext');
        px2 = [se3_2(1,4)-0.05,se3_2(1,4)+0.05];
        py2 = [se3_2(2,4)-0.01,se3_2(2,4)+0.05];
        pz2 = [se3_2(3,4)-0.05,se3_2(3,4)+0.05];
        hold on
        xlim([-2,2])
        ylim([0,6])
        zlim([-3,2])
        plot3(px,py,pz,'LineWidth',2,'Color','b');
        plot3(px2,py2,pz2,'LineWidth',2,'Color','r');
        pause(0.01)
        i = i+30;
        delete(a)
        delete(b)
        drawnow limitrate
    end
end
function p_imu_world = cal_pimu_world(a_imu,A_imu,t)
    %将加速度转换到世界坐标系下
    a_imu_world = zeros(size(a_imu,1),3);
    for i =1:size(a_imu,1)
        R = eul2rotm(A_imu(i,:),"XYZ");
        temp =R*a_imu(i,:)';
        a_imu_world(i,:) = temp';
    end
    % 梯形积分
    v_imu_world = cumtrapz(t,a_imu_world);
    p_imu_world = cumtrapz(t,v_imu_world);
end
function [vel, pos] = kalman(t, acc, gps)

H = [1 0];                              % 转换矩阵
Q = [1e-4 0; 0 5e-4];                   % 过程噪声协方差，估计一个
R = 10;
P = eye(2);                             % 初始值为 1（可为非零任意数） 
N = length(acc);
x = zeros(2, N);                        % 存储滤波后的数据，分别为位移、速度信息

for k=2:N
    dt = t(k) - t(k-1);
    A = [1 dt; 0 1];                            % 状态转移矩阵
    B = [1/2*dt^2; dt];                         % 输入控制矩阵
    x(:,k) = A * x(:,k-1) + B*acc(k,1);         % 卡尔曼公式1
    P = A * P * A' + Q;                         % 卡尔曼公式2
    K = P*H' * inv(H*P*H' + R);                 % 卡尔曼公式3                                       
    x(:,k) = x(:,k) + K * (gps(k)-H*x(:,k));    % 卡尔曼公式4
    P = (eye(2)-K*H) * P;                       % 卡尔曼公式5
end
pos = x(1,:)';
vel = x(2,:)';
end

function p_ekf = cal_p_ekf(a_imu,A_imu,p_tag,t)
    %将加速度转换到世界坐标系下
    a_imu_world = zeros(size(a_imu,1),3);
    for i =1:size(a_imu,1)
        R = eul2rotm(A_imu(i,:),"XYZ");
        temp =R*a_imu(i,:)';
        a_imu_world(i,:) = temp';
    end
    % tag得到的位置
    p_tag_world = p_tag-p_tag(1,:);
    % 加速度积分得到是速度和位移
    v_imu_world = cumtrapz(t,a_imu_world);
    p_imu_world = cumtrapz(t,v_imu_world);%加上初始位置
    % 对三个频段分别使用kalman滤波
    accx_imu = a_imu_world(:,1);
    posx_tag = p_tag_world(:,1);
    [velx_km,posx_km] = kalman(t,accx_imu,posx_tag);
    accy_imu = a_imu_world(:,2);
    posy_tag = p_tag_world(:,2);
    [vely_km,posy_km] = kalman(t,accy_imu,posy_tag);
    accz_imu = a_imu_world(:,3);
    posz_tag = p_tag_world(:,3);
    [velz_km,posz_km] = kalman(t,accz_imu,posz_tag);
    p_ekf = [posx_km,posy_km,posz_km];
end