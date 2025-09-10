function modelPath = generate_inverter_model()
% 生成一个离散时间的“逆变器近似”仿真模型（PI + 一阶离散被控对象），并保存到固定路径。
%
% 设计动机（为何如此建模）:
% - 为了与 Python 侧 MatlabSimulator 解耦联调，避免对电力电子专用工具箱的强依赖，
%   这里用“离散 PI + 一阶离散对象(离散传函)”近似逆变侧滤波与负载的主动态。
% - 控制目标：让离散对象输出 V_out 跟踪参考 V_ref。
%
% 与 Python 配置的接口约定（如何对齐 Python 侧 I/O）:
% - 输入动作: Kp -> base.Kp_val, Ki -> base.Ki_val
% - 输出信号: V_out, V_ref, error 通过 To Workspace 以 Array 格式写入 base 工作区
% - 采样时间: Ts (秒)，StopTime 由 Python 每步设置（此处默认 0.02）
%
% 使用:
% - 在 MATLAB 命令行运行：  modelPath = generate_inverter_model();
% - 运行结束后，会在命令行打印保存路径，且返回该绝对路径字符串

    % 目标保存目录与模型名（请根据你的电脑用户名/路径确认）
    targetDir = 'C:/Users/tiany/PycharmProjects/analog TopoBench/simulink';
    modelName = 'inverter_model';
    mdl = modelName;

    if ~exist(targetDir, 'dir')
        mkdir(targetDir);
    end

    % 若模型已加载则先关闭（不保存）
    if bdIsLoaded(mdl)
        close_system(mdl, 0);
    end

    % 新建并打开模型
    new_system(mdl);
    open_system(mdl);

    % =============== 基本参数注入到 base 工作区（为何放这里：保证可重复生成） ===============
    % 采样时间与被控对象时间常数，可与 Python 配置对应
    assignin('base', 'Ts', 0.001);
    assignin('base', 'Tau', 0.01);
    % PI 初值（可被 Python 每步覆盖）
    assignin('base', 'Kp_val', 1.0);
    assignin('base', 'Ki_val', 0.1);
    % 参考值（可由 Python 更改或在 MATLAB 命令行覆盖）
    assignin('base', 'V_ref_src', 1.0);

    % 一阶离散对象: y[k] = alpha * u[k] + (1-alpha) * y[k-1]
    % 离散传函: H(z) = alpha / (1 - (1 - alpha) z^-1)
    evalin('base', 'alpha = Ts/(Tau + Ts);');
    evalin('base', 'plant_num = [alpha];');
    evalin('base', 'plant_den = [1, (alpha - 1)];');

    % =============== 模型参数（固定步长离散） ===============
    set_param(mdl, 'Solver', 'FixedStepDiscrete');
    set_param(mdl, 'FixedStep', 'Ts');
    set_param(mdl, 'StopTime', '0.02');

    % =============== 放置并配置模块 ===============
    % 参考源
    add_block('simulink/Sources/Constant', [mdl '/Vref'], 'Position', [30 50 90 80]);
    set_param([mdl '/Vref'], 'Value', 'V_ref_src');

    % 误差计算 e = V_ref - V_out
    add_block('simulink/Math Operations/Sum', [mdl '/Sum'], 'Position', [140 50 170 80]);
    set_param([mdl '/Sum'], 'Inputs', '+-');

    % 离散 PI 控制器
    add_block('simulink/Discrete/Discrete PID Controller', [mdl '/PI'], 'Position', [220 40 310 90]);
    set_param([mdl '/PI'], 'P', 'Kp_val');
    set_param([mdl '/PI'], 'I', 'Ki_val');
    set_param([mdl '/PI'], 'D', '0');
    set_param([mdl '/PI'], 'TimeDomain', 'Discrete-time');
    set_param([mdl '/PI'], 'SampleTime', 'Ts');

    % 被控对象（离散传函）
    add_block('simulink/Discrete/Discrete Transfer Fcn', [mdl '/Plant'], 'Position', [360 40 440 90]);
    set_param([mdl '/Plant'], 'Numerator', 'plant_num');
    set_param([mdl '/Plant'], 'Denominator', 'plant_den');
    set_param([mdl '/Plant'], 'SampleTime', 'Ts');

    % To Workspace: V_out, V_ref, error（Array 格式，便于 Python 读取）
    add_block('simulink/Sinks/To Workspace', [mdl '/ToWs_V_out'], 'Position', [500 40 580 80]);
    set_param([mdl '/ToWs_V_out'], 'VariableName', 'V_out', 'SaveFormat', 'Array');

    add_block('simulink/Sinks/To Workspace', [mdl '/ToWs_V_ref'], 'Position', [220 120 300 160]);
    set_param([mdl '/ToWs_V_ref'], 'VariableName', 'V_ref', 'SaveFormat', 'Array');

    add_block('simulink/Sinks/To Workspace', [mdl '/ToWs_error'], 'Position', [320 120 400 160]);
    set_param([mdl '/ToWs_error'], 'VariableName', 'error', 'SaveFormat', 'Array');

    % =============== 连接连线 ===============
    % Vref -> Sum(+)
    add_line(mdl, 'Vref/1', 'Sum/1', 'autorouting', 'on');
    % Plant 输出 -> Sum(-)
    add_line(mdl, 'Plant/1', 'Sum/2', 'autorouting', 'on');
    % Sum -> PI -> Plant
    add_line(mdl, 'Sum/1', 'PI/1', 'autorouting', 'on');
    add_line(mdl, 'PI/1', 'Plant/1', 'autorouting', 'on');
    % 观测写到工作区
    add_line(mdl, 'Plant/1', 'ToWs_V_out/1', 'autorouting', 'on');
    add_line(mdl, 'Vref/1', 'ToWs_V_ref/1', 'autorouting', 'on');
    add_line(mdl, 'Sum/1', 'ToWs_error/1', 'autorouting', 'on');

    % =============== 保存并关闭模型 ===============
    modelPath = fullfile(targetDir, [modelName '.slx']);
    save_system(mdl, modelPath);
    close_system(mdl, 0);
    fprintf('Model saved to: %s\n', modelPath);
end


