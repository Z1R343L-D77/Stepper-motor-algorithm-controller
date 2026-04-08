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

for k = 2:length(varNames)
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

% 默认频率范围
defaultMinFreq = min(freq);
defaultMaxFreq = max(freq);

%% 5. 创建主图窗 + 大标题
figMain = figure('Color', [0.95 0.96 0.98], ...
                 'Name', ['数字信号分析 - ' channelName], ...
                 'Position', [80 60 1480 780], ...
                 'MenuBar', 'none');

% 顶部大标题
titlePanel = uipanel(figMain, 'Position', [0.02 0.94 0.96 0.05], ...
                     'BackgroundColor', [0.88 0.92 0.96], ...
                     'BorderType', 'none');

uicontrol(titlePanel, 'Style', 'text', ...
          'String', '数字信号频率分析工具', ...
          'FontSize', 18, 'FontWeight', 'bold', ...
          'ForegroundColor', [0.00 0.35 0.75], ...
          'HorizontalAlignment', 'center', ...
          'BackgroundColor', [0.88 0.92 0.96], ...
          'Position', [0 0 1400 35]);

leftPanel = uipanel(figMain, 'Position', [0.02 0.04 0.67 0.88], ...
                    'BackgroundColor', 'w', 'BorderType', 'none');

rightPanel = uipanel(figMain, 'Position', [0.71 0.04 0.27 0.88], ...
                     'Title', '分析信息与操作记录', ...
                     'BackgroundColor', [0.97 0.97 0.99], ...
                     'FontSize', 12, 'FontWeight', 'bold', ...
                     'TitlePosition', 'lefttop', ...
                     'BorderType', 'etchedin', ...
                     'HighlightColor', [0.7 0.7 0.7]);

%% 6. 左侧：波形图
ax1 = subplot(2,1,1, 'Parent', leftPanel);
stairs(time, sig, 'LineWidth', 1.6, 'Color', [0.00 0.48 1.00]);
grid on;
xlabel('Time (s)');
ylabel('Logic Level');
title(['原始数字波形 - ' channelName], 'FontSize', 13);
ylim([-0.2, 1.2]);

ax2 = subplot(2,1,2, 'Parent', leftPanel);
hFreq = stairs(freq_time, freq, 'LineWidth', 1.6, 'Color', [1.00 0.40 0.00]);
grid on;
xlabel('Time (s)');
ylabel('Frequency (Hz)');
title(['频率波形 - ' channelName], 'FontSize', 13);

axtoolbar(ax1, {'zoomin', 'zoomout', 'restoreview', 'pan'});
axtoolbar(ax2, {'zoomin', 'zoomout', 'restoreview', 'pan'});
addToolbarExplorationButtons(figMain);

%% 7. 右侧：控件布局
infoText = uicontrol(rightPanel, 'Style', 'edit', ...
                     'Max', 100, ...
                     'Min', 0, ...
                     'HorizontalAlignment', 'left', ...
                     'FontName', 'Menlo', ...
                     'FontSize', 10.5, ...
                     'BackgroundColor', [0.99 0.99 1.00], ...
                     'Position', [15 325 325 340]);

% 自定义频率范围输入
uicontrol(rightPanel, 'Style', 'text', ...
          'String', '自定义频率有效范围 (Hz)：', ...
          'FontSize', 11, 'FontWeight', 'bold', ...
          'HorizontalAlignment', 'left', ...
          'BackgroundColor', [0.97 0.97 0.99], ...
          'Position', [20 280 300 25]);

minEdit = uicontrol(rightPanel, 'Style', 'edit', ...
                    'String', num2str(defaultMinFreq, '%.3f'), ...
                    'FontSize', 11, ...
                    'BackgroundColor', [1 1 1], ...
                    'Position', [20 245 140 30]);

uicontrol(rightPanel, 'Style', 'text', ...
          'String', '~', ...
          'FontSize', 13, ...
          'Position', [165 247 20 25]);

maxEdit = uicontrol(rightPanel, 'Style', 'edit', ...
                    'String', num2str(defaultMaxFreq, '%.3f'), ...
                    'FontSize', 11, ...
                    'BackgroundColor', [1 1 1], ...
                    'Position', [185 245 140 30]);

% 仅显示有效范围复选框
showRangeCheck = uicontrol(rightPanel, 'Style', 'checkbox', ...
                           'String', '仅显示有效范围内的频率点', ...
                           'FontSize', 10.5, ...
                           'Value', 0, ...
                           'BackgroundColor', [0.97 0.97 0.99], ...
                           'Position', [20 205 300 25], ...
                           'Callback', @(src,evt) toggleFreqRange(ax2, hFreq, freq, freq_time, str2double(minEdit.String), str2double(maxEdit.String), src.Value));

% 应用自定义范围按钮
uicontrol(rightPanel, 'Style', 'pushbutton', ...
          'String', '应用自定义范围', ...
          'FontSize', 11, 'FontWeight', 'bold', ...
          'BackgroundColor', [0.00 0.48 1.00], ...
          'ForegroundColor', [1 1 1], ...
          'Position', [20 160 305 40], ...
          'Callback', @(src,evt) applyCustomRange(ax2, hFreq, freq, freq_time, minEdit, maxEdit, showRangeCheck));

% 坐标轴缩放按钮
uicontrol(rightPanel, 'Style', 'pushbutton', ...
          'String', 'X 放大', 'FontSize', 11, 'BackgroundColor', [0.92 0.92 0.94], ...
          'Position', [20 115 145 42], 'Callback', @(src,evt) axisZoom(ax1, ax2, 'x', 1.6));

uicontrol(rightPanel, 'Style', 'pushbutton', ...
          'String', 'X 缩小', 'FontSize', 11, 'BackgroundColor', [0.92 0.92 0.94], ...
          'Position', [180 115 145 42], 'Callback', @(src,evt) axisZoom(ax1, ax2, 'x', 0.65));

uicontrol(rightPanel, 'Style', 'pushbutton', ...
          'String', 'Y 放大', 'FontSize', 11, 'BackgroundColor', [0.92 0.92 0.94], ...
          'Position', [20 65 145 42], 'Callback', @(src,evt) axisZoom(ax1, ax2, 'y', 1.6));

uicontrol(rightPanel, 'Style', 'pushbutton', ...
          'String', 'Y 缩小', 'FontSize', 11, 'BackgroundColor', [0.92 0.92 0.94], ...
          'Position', [180 65 145 42], 'Callback', @(src,evt) axisZoom(ax1, ax2, 'y', 0.65));

% 恢复全部视图按钮
uicontrol(rightPanel, 'Style', 'pushbutton', ...
          'String', '恢复全部视图', ...
          'FontSize', 12, 'FontWeight', 'bold', ...
          'BackgroundColor', [0.00 0.48 1.00], 'ForegroundColor', [1 1 1], ...
          'Position', [20 20 305 45], ...
          'Callback', @(src,evt) resetAllView(ax1, ax2, hFreq, freq, freq_time, showRangeCheck, minEdit, maxEdit));

%% 8. 初始信息显示
avgFreq = mean(freq);
initialInfo = {sprintf('通道: %s', channelName); ...
               ''; ...
               sprintf('平均频率: %.3f Hz', avgFreq); ...
               sprintf('计算得到的范围: %.3f ~ %.3f Hz', defaultMinFreq, defaultMaxFreq); ...
               ''; ...
               '=== 操作记录 ==='; ...
               '界面已加载成功'; ...
               'R 键为AUTO '; ...
               '可手动输入频率范围并应用'};

set(infoText, 'String', initialInfo);

%% 9. 使用说明
disp(' ');
disp('📌 使用说明：');
disp('   • 顶部显示大标题');
disp('   • 右侧可手动输入频率有效范围下限和上限');
disp('   • 输入后点击“应用自定义范围”按钮生效');
disp('   • 勾选“仅显示有效范围内的频率点”可隐藏超出范围的点');
disp('   • 按键盘 **R** 键可快速恢复全部视图');

% ==================== 键盘快捷键 ====================
set(figMain, 'KeyPressFcn', @(src, evt) keyPressCallback(src, evt, ax1, ax2, infoText, hFreq, freq, freq_time, showRangeCheck, minEdit, maxEdit));

% ==================== 辅助函数 ====================
function applyCustomRange(ax2, hFreq, freq, freq_time, minEdit, maxEdit, checkBox)
    minR = str2double(minEdit.String);
    maxR = str2double(maxEdit.String);
    if isnan(minR) || isnan(maxR) || minR >= maxR
        disp('⚠️ 请输入有效的范围（下限必须小于上限）');
        return;
    end
    if checkBox.Value
        filteredFreq = freq;
        filteredFreq(freq < minR | freq > maxR) = NaN;
        set(hFreq, 'YData', filteredFreq);
    end
    disp(['✅ 已应用自定义范围：' num2str(minR,'%.3f') ' ~ ' num2str(maxR,'%.3f') ' Hz']);
end

function toggleFreqRange(ax2, hFreq, freq, freq_time, minR, maxR, enabled)
    if enabled
        filteredFreq = freq;
        filteredFreq(freq < minR | freq > maxR) = NaN;
        set(hFreq, 'YData', filteredFreq);
        disp('✅ 已启用仅显示有效频率范围');
    else
        set(hFreq, 'YData', freq);
        disp('✅ 已恢复显示全部频率点');
    end
end

function axisZoom(ax1, ax2, direction, factor)
    if strcmpi(direction, 'x')
        linkaxes([ax1 ax2], 'x');
        xlim1 = xlim(ax1); center = mean(xlim1); width = diff(xlim1)/factor;
        newLim = [center - width/2, center + width/2];
        xlim(ax1, newLim); xlim(ax2, newLim);
        linkaxes([ax1 ax2], 'off');
    else
        ylim1 = ylim(ax1); c1 = mean(ylim1); w1 = diff(ylim1)/factor; ylim(ax1, [c1-w1/2, c1+w1/2]);
        ylim2 = ylim(ax2); c2 = mean(ylim2); w2 = diff(ylim2)/factor; ylim(ax2, [c2-w2/2, c2+w2/2]);
    end
    zoomStr = iif(factor > 1, '放大', '缩小');
    disp(['✅ 已对 ' direction ' 轴进行 ' zoomStr]);
end

function resetAllView(ax1, ax2, hFreq, freq, freq_time, checkBox, minEdit, maxEdit)
    linkaxes([ax1 ax2], 'x');
    axis(ax1, 'auto'); axis(ax2, 'auto');
    ylim(ax1, [-0.2, 1.2]);
    linkaxes([ax1 ax2], 'off');
    
    set(hFreq, 'YData', freq);
    checkBox.Value = 0;
    minEdit.String = num2str(min(freq), '%.3f');
    maxEdit.String = num2str(max(freq), '%.3f');
    
    disp('✅ 已恢复全部视图到原始范围');
end

function keyPressCallback(~, evt, ax1, ax2, infoText, hFreq, freq, freq_time, checkBox, minEdit, maxEdit)
    if strcmpi(evt.Key, 'r')
        resetAllView(ax1, ax2, hFreq, freq, freq_time, checkBox, minEdit, maxEdit);
        
        currentStr = get(infoText, 'String');
        if ischar(currentStr), currentStr = cellstr(currentStr); end
        newLog = sprintf('%s 已通过按 R 键恢复全部视图', datestr(now, 'HH:MM:SS'));
        set(infoText, 'String', [currentStr; {newLog}]);
    end
end

% 小辅助函数
function res = iif(cond, trueVal, falseVal)
    if cond, res = trueVal; else, res = falseVal; end
end