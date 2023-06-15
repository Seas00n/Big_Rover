clear all;clc;close all;
%% ��ȡ�����봦��
data = importdata('kalman_vel_pos.txt');

t            = data.data(:, 1);
accx_dji = data.data(:, 2);
accy_dji = data.data(:, 3);
gpsx_dji = data.data(:, 5);
gpsy_dji = data.data(:, 6);
velx_dji  = data.data(:, 7);
vely_dji  = data.data(:, 8);
posx_dji = data.data(:, 9);
posy_dji = data.data(:, 10);
 
% ���ٶȻ��ּ����ٶȡ�λ��
dt = [0; t(2:end)-t(1:end-1)];
velx_int = velx_dji(1) + cumsum(accx_dji .* dt);   % ���ּ����ٶ�
vely_int = vely_dji(1) + cumsum(accy_dji .* dt);   % ���ּ����ٶ�
posx_int = posx_dji(1) + cumsum(velx_int .* dt);   % ���ּ���λ��
posy_int = posy_dji(1) + cumsum(vely_int .* dt);   % ���ּ���λ��

% GPSֱ�Ӽ���λ��
gpsx0 = gpsx_dji(1);
gpsy0 = gpsy_dji(1);

earth_radius = 6378137.0;
posx_gps = (gpsx_dji - gpsx0) * pi / 180 * earth_radius .* cos(gpsy_dji*pi/180);        % ֱ��ֻ��GPS�����λ��
posy_gps = (gpsy_dji - gpsy0) * pi / 180 * earth_radius;                                % ֱ��ʹ��GPS�����λ��

posx_gps = posx_gps + posx_dji(1);      % ���ϳ�ʼλ��
posy_gps = posy_gps + posy_dji(1);      % ���ϳ�ʼλ��

% �������˲������ٶȡ�λ��
[velx_km, posx_km] = kalman(t, accx_dji, posx_gps);
[vely_km, posy_km] = kalman(t, accy_dji, posy_gps);

%% �������
%��λ�ã�
figure(1);
plot(posx_int, posy_int);hold on
plot(posx_gps, posy_gps, 'b.');
plot(posx_dji, posy_dji,'g');
plot(posx_km, posy_km,'r'); hold off
legend('���ٶȻ���', 'GPSλ��', 'DJI���','Kalman���');
title('�������˲�����λ��');xlabel('x����λ�� (m)');ylabel('y����λ�� (m)');
text(posx_dji(1), posy_dji(1), '��ʼ��');
text(posx_dji(end),posy_dji(end), '��ֹ��');

%���ٶȣ�
figure(2);subplot(211);
plot(t-t(1), velx_int);hold on
plot(t-t(1), velx_dji);
plot(t-t(1), velx_km);hold off
legend('���ٶȻ���','DJI���','Kalman���');
title('�������˲������ٶ�');ylabel('x���ٶ� (m/s)');

subplot(212);
plot(t-t(1), vely_int);hold on
plot(t-t(1), vely_dji);
plot(t-t(1), vely_km);hold off
legend('���ٶȻ���','DJI���','Kalman���');
xlabel('ʱ��t (s)');ylabel('y���ٶ� (m/s)');

%% �������˲���
% ����: t, ʱ���(s)
%       acc, ���˻����ܺ����ڵ�������ϵ�²����ļ��ٶ�(m/s^2)
%       gps, ��������ϵ��ʹ��GPS�����λ��(m)
% ���: vel, ��������ϵ�µ��ٶ�(m/s)
%       pos, ��������ϵ�µ�λ��(m)

function [vel, pos] = kalman(t, acc, gps)

H = [1 0];                              % ת������
Q = [1e-4 0; 0 5e-4];                   % ��������Э�������һ��
R = 10;
P = eye(2);                             % ��ʼֵΪ 1����Ϊ������������ 
N = length(acc);
x = zeros(2, N);                        % �洢�˲�������ݣ��ֱ�Ϊλ�ơ��ٶ���Ϣ

for k=2:N
    dt = t(k) - t(k-1);
    A = [1 dt; 0 1];                            % ״̬ת�ƾ���
    B = [1/2*dt^2; dt];                         % ������ƾ���
    x(:,k) = A * x(:,k-1) + B*acc(k,1);         % ��������ʽ1
    P = A * P * A' + Q;                         % ��������ʽ2
    K = P*H' * inv(H*P*H' + R);                 % ��������ʽ3                                       
    x(:,k) = x(:,k) + K * (gps(k)-H*x(:,k));    % ��������ʽ4
    P = (eye(2)-K*H) * P;                       % ��������ʽ5
end
pos = x(1,:)';
vel = x(2,:)';
end

%%
