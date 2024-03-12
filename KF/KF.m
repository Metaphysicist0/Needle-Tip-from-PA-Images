clc;
clear;
close all;
%% step1 读取数据
% 提取单个点测试
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
   
   point=jsonData.shapes.points; % 读取的每个点
   
   points(i,:)=point; % 将每个点的坐标存储在points中
   
end

% figure;
% plot(Point(:,1),Point(:,2),'r-','LineWidth',1.5);

%% step2 卡尔曼滤波
% 初始化卡尔曼滤波器参数
dt = 1;  % 时间步长
A = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];  % 状态转移矩阵
H = [1 0 0 0; 0 1 0 0];  % 观测矩阵
Q = eye(4) * 0.01;  % 状态噪声协方差矩阵
R = eye(2) * 0.1;  % 观测噪声协方差矩阵

% 初始化状态估计和协方差矩阵
x_hat = [points(1, 1); points(1, 2); 0; 0];  % 初始状态估计
P = eye(4);  % 初始协方差矩阵

% 使用卡尔曼滤波进行预测
predicted_points = zeros(size(points));
for i = 1:size(points, 1)
    % 预测步骤
    x_hat_minus = A * x_hat;
    P_minus = A * P * A' + Q;
    
    % 更新步骤
    K = P_minus * H' * inv(H * P_minus * H' + R);
    x_hat = x_hat_minus + K * (points(i, :)' - H * x_hat_minus);
    P = (eye(4) - K * H) * P_minus;
    
    % 保存预测结果
    predicted_points(i, :) = [x_hat(1), x_hat(2)];
end

% 绘制原始点和预测点
figure('Units','normalized','Position',[0.1 0.1 0.6 0.6]);  % 设置图形窗口大小和位置
plot(points(:,1),points(:,2),'b-','LineWidth',1.5);
hold on;
plot(predicted_points(:,1),predicted_points(:,2),'r--','LineWidth',2.0);

% 设置坐标轴标签和图例
xlabel('X','FontName','Times New Roman','FontSize',18);
ylabel('Y','FontName','Times New Roman','FontSize',18);
title('Tip Tracking','FontName','Times New Roman','FontSize',20);
l = legend('Actual Data','Prediction');
l.Box = 'off'; % 去除图例边框
l.Location = 'NorthWest'; % 调整图例位置
l.FontName = 'Times New Roman';
l.FontSize = 16;

% 设置坐标轴范围和网格线
axis tight; % 自动调整坐标轴范围
grid on;
box on; % 添加边框
set(gca,'GridLineStyle','-'); % 设置网格线样式为实线
set(gca,'LineWidth',1.2); % 设置坐标轴线宽
set(gca,'FontName','Times New Roman','FontSize',16); % 设置坐标轴字体

% 设置颜色映射方案
set(gca,'ColorOrder',[0 0.4470 0.7410; 0.8500 0.3250 0.0980]);

% 设置图形窗口背景颜色
set(gcf,'Color','w'); % 设置背景为白色

% 计算误差
errors = points - predicted_points;

% 计算均方根误差
rmse = sqrt(sum(errors(:).^2) / numel(errors));

fprintf('RMSE: %.4f\n', rmse);

std_error = std(errors(:));

fprintf('STD: %.4f\n', std_error);

mae = mean(abs(errors), 'all');

fprintf('MAE: %.4f\n', mae);