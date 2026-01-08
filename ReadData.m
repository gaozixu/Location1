%% 用来加载测量的两个imu数据
%  存放在new_data文件夹中，每个csv文件代表一次测试结果
%  1-3列：imu1的加速度，单位（g） 4-6列：imu1的角速度，单位（°/s） 
%  7-9列：imu2的加速度，单位（g） 10-12列：imu2的角速度，单位（°/s） 
%  两个imu的时间长度不同，而且并不是严格对其的，因此需要进行预处理才能给后面使用
clear ;clc;close all;
%% 读取,imu前三列为加速度，后三为角速度
tbl = readtable("new_data\data5.csv");
imu1 = table2array(tbl(:,1:6));
imu1_data = imu1(~all(isnan(imu1), 2), :);
imu2 = table2array(tbl(:,7:12));
imu2_data = imu2(~all(isnan(imu2), 2), :);
clear imu1 imu2;
 g=9.7949 ;
minlength=min([length(imu1_data) length(imu2_data)] );
imu1_data = imu1_data(1:minlength,:);
imu2_data = imu2_data(1:minlength,:);

plot(vecnorm(imu1_data(:,1:3).*9.8*5,2,2),'r')
hold on;
plot(vecnorm(imu1_data(:,4:6),2,2),'b')

figure(1);
plot(imu1_data(:,1:3).*g);
figure(2);
plot(imu1_data(:,4:6));

figure(3);
plot(imu2_data(:,1:3).*g);
figure(4);
plot(imu2_data(:,4:6));

  
 