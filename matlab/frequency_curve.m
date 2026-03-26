clc;
clear;
close all;

%% 1. 读取 CSV
filename = 'digital.csv';
T = readtable(filename, 'VariableNamingRule', 'preserve');

time = T.('Time [s]');
varNames = T.Properties.VariableNames;

%% 2. 自动寻找有跳变的通道
channelName = '';
sig = [];

for k = 2:length(varNames)   % 第1列是时间列
    tmp = T.(varNames{k});
    if any(diff(tmp) ~= 0)
        channelName = varNames{k};
        sig = tmp;
        break;
    end
end

if isempty(channelName)
    error('没有找到有跳变的通道。');
end

time = time(:);
sig  = sig(:);

%% 3. 检测上升沿
rising_idx = find(sig(1:end-1) == 0 & sig(2:end) == 1) + 1;
rising_time = time(rising_idx);

if length(rising_time) < 2
    error('上升沿数量不足，无法计算频率。');
end

%% 4. 计算频率
period = diff(rising_time);
freq = 1 ./ period;
freq_time = rising_time(1:end-1);

%% 5. 绘图
figure('Color','w');

subplot(2,1,1);
stairs(time, sig, 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Logic Level');
title(['原始数字波形 - ' channelName]);
ylim([-0.2, 1.2]);

subplot(2,1,2);
stairs(freq_time, freq, 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Frequency (Hz)');
title(['频率波形 - ' channelName]);

%% 6. 显示结果
fprintf('自动选择通道: %s\n', channelName);
fprintf('平均频率: %.3f Hz\n', mean(freq));
fprintf('最大频率: %.3f Hz\n', max(freq));
fprintf('最小频率: %.3f Hz\n', min(freq));