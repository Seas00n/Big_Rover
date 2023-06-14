clear allc;close all;clc;
data = load("imu_test.mat");
data = data.data;
data(1,:)=data(2,:);
rpy = data(:,1:3)*pi/180;
rpy = rpy-rpy(1,:);
axyz = data(:,4:6);
agxyz = data(:,7:9);
t = data(:,10);
t = t-t(1);
% 处理imu数据





figure(1)
plot(t,axyz);
figure(2)








p_imu = cal_pimu_world(axyz,rpy,t);
A_imu = rpy;
se3_imu = zeros(4,4,size(data,1));
for i=1:size(data,1)
    R = SO3.eul([A_imu(i,1),A_imu(i,2),A_imu(i,3)]);
    T = p_imu(i,:);
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
    xlim([-10,10])
    ylim([-10,10])
    zlim([-10,10])
    while i<size(se3_list,3)
        se3 = se3_list(:,:,i);
        a = trplot(SE3.convert(se3),'length', 0.5, 'retain', 'rgb', 'notext');
        px = [se3(1,4)-0.05,se3(1,4)+0.05];
        py = [se3(2,4)-0.01,se3(2,4)+0.05];
        pz = [se3(3,4)-0.05,se3(3,4)+0.05];
        hold on
        xlim([-10,10])
        ylim([-10,10])
        zlim([-10,10])
        plot3(px,py,pz,'LineWidth',2,'Color',color);
        pause(0.01)
        i = i+1;
        delete(a)
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
    v_imu_world = zeros(size(a_imu,1),3);
    p_imu_world = zeros(size(a_imu,1),3);
    v_imu_world = cumtrapz(t,a_imu_world);
    p_imu_world = v_imu_world;
   
end
