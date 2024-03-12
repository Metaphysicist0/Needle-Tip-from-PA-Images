clc;
clear;
close all;
%% step1 ��ȡ����
% ��ȡ���������
% jsonStr=fileread('./location/001.json');
% jsonData=jsondecode(jsonStr);
% point=jsonData.shapes.points

filepath='./location1/';
files=dir(fullfile(filepath,'*.json'));
numFiles=length(files);

%
points=zeros(numFiles,2);
for i=1:numFiles
   FileName=fullfile(filepath,files(i).name);
   
   jsonStr=fileread(FileName);
   jsonData=jsondecode(jsonStr);
   
   point=jsonData.shapes.points; % ��ȡ��ÿ����
   
   points(i,:)=point; % ��ÿ���������洢��points��
   
end

% figure;
% plot(Point(:,1),Point(:,2),'r-','LineWidth',1.5);

%% step2 �������˲�
% ��ʼ���������˲�������
dt = 1;  % ʱ�䲽��
A = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];  % ״̬ת�ƾ���
H = [1 0 0 0; 0 1 0 0];  % �۲����
Q = eye(4) * 0.01;  % ״̬����Э�������
R = eye(2) * 0.1;  % �۲�����Э�������

% ��ʼ��״̬���ƺ�Э�������
x_hat = [points(1, 1); points(1, 2); 0; 0];  % ��ʼ״̬����
P = eye(4);  % ��ʼЭ�������

% ʹ�ÿ������˲�����Ԥ��
predicted_points = zeros(size(points));
for i = 1:size(points, 1)
    % Ԥ�ⲽ��
    x_hat_minus = A * x_hat;
    P_minus = A * P * A' + Q;
    
    % ���²���
    K = P_minus * H' * inv(H * P_minus * H' + R);
    x_hat = x_hat_minus + K * (points(i, :)' - H * x_hat_minus);
    P = (eye(4) - K * H) * P_minus;
    
    % ����Ԥ����
    predicted_points(i, :) = [x_hat(1), x_hat(2)];
end

% ����ԭʼ���Ԥ���
figure('Units','normalized','Position',[0.1 0.1 0.6 0.6]);  % ����ͼ�δ��ڴ�С��λ��
plot(points(:,1),points(:,2),'b-','LineWidth',1.5);
hold on;
plot(predicted_points(:,1),predicted_points(:,2),'r--','LineWidth',2.0);

% �����������ǩ��ͼ��
xlabel('X','FontName','Times New Roman','FontSize',18);
ylabel('Y','FontName','Times New Roman','FontSize',18);
title('Tip Tracking','FontName','Times New Roman','FontSize',20);
l = legend('Actual Data','Prediction');
l.Box = 'off'; % ȥ��ͼ���߿�
l.Location = 'NorthWest'; % ����ͼ��λ��
l.FontName = 'Times New Roman';
l.FontSize = 16;

% ���������᷶Χ��������
axis tight; % �Զ����������᷶Χ
grid on;
box on; % ��ӱ߿�
set(gca,'GridLineStyle','-'); % ������������ʽΪʵ��
set(gca,'LineWidth',1.2); % �����������߿�
set(gca,'FontName','Times New Roman','FontSize',16); % ��������������

% ������ɫӳ�䷽��
set(gca,'ColorOrder',[0 0.4470 0.7410; 0.8500 0.3250 0.0980]);

% ����ͼ�δ��ڱ�����ɫ
set(gcf,'Color','w'); % ���ñ���Ϊ��ɫ

% �������
errors = points - predicted_points;

% ������������
rmse = sqrt(sum(errors(:).^2) / numel(errors));

fprintf('RMSE: %.4f\n', rmse);

std_error = std(errors(:));

fprintf('STD: %.4f\n', std_error);

mae = mean(abs(errors), 'all');

fprintf('MAE: %.4f\n', mae);