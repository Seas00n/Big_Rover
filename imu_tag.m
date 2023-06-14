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
data(1,:) = data(2,:);
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
    Q = UnitQuaternion([q_tag(i,1),q_tag(i,2),q_tag(i,3),q_tag(i,4)]);
    T = p_tag(i,:);
    Q_SE3 = SO3ToMatrix(Q.SO3(),T);
    se3_tag(:,:,i) = Q_SE3;
end
myse3animate(se3_tag,'b');
% 处理imu数据
p_imu = cal_pimu_world(a_imu,A_imu,t);
se3_imu = zeros(4,4,size(data,1));
for i=1:size(data,1)
    R = SO3.eul([A_imu(i,1),A_imu(i,2),A_imu(i,3)]);
    T = p_imu(i,:)+p_tag(1,:);
    se3_imu(:,:,i) = SO3ToMatrix(R,T);
end
myse3animate(se3_imu,'r');


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
    ylim([-5,5])
    zlim([-5,5])
    while i<size(se3_list,3)
        se3 = se3_list(:,:,i);
        a = trplot(SE3.convert(se3),'length', 0.5, 'retain', 'rgb', 'notext');
        px = [se3(1,4)-0.05,se3(1,4)+0.05];
        py = [se3(2,4)-0.01,se3(2,4)+0.05];
        pz = [se3(3,4)-0.05,se3(3,4)+0.05];
        hold on
        xlim([-5,5])
        ylim([-5,5])
        zlim([-5,5])
        plot3(px,py,pz,'LineWidth',2,'Color',color);
        pause(0.01)
        i = i+5;
        delete(a)
        drawnow limitrate
    end
end
function p_imu_world = cal_pimu_world(a_imu,A_imu,t)
    %将加速度转换到世界坐标系下
    a_imu_world = zeros(size(a_imu,1),3);
    for i =1:size(a_imu,1)
        R = eul2rotm(A_imu(i,:),"XYZ");
        temp =R'*a_imu(i,:)';
        a_imu_world(i,:) = temp';
    end
    % 梯形积分
    v_imu_world = cumtrapz(t,a_imu_world);
    p_imu_world = cumtrapz(t,v_imu_world);
end
