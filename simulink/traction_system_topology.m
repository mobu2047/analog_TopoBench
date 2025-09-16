%% traction_system_topology.m
% 双供电单元牵引拓扑（SPS）
% 结构：每单元 = 四象限 PWM 整流器 + DC 链路(预充/斩波/固定放电/接地检测) + VWF 逆变器(两台异步牵引电机)
% 传感器：LH1~LH8（电流），VH1~VH6（电压）
% 已修复：
%   - 本地函数统一移到文件末尾
%   - 三相电源参数名用稳健匹配
%   - 全部用 [] 进行字符串拼接
%   - 关键连线使用 safe_add_line，失败会给出清晰报错

%% 初始化
bdclose('all'); clc;
model = 'Traction_DualSupply_VWF';
new_system(model); open_system(model);

% 加载库
try
    load_system('powerlib');    % Specialized Power Systems
catch
    error('未找到 powerlib。请确认已安装 Simscape Electrical (Specialized Power Systems)。');
end

% 画布
set_param(model,'Location',[100 60 1650 980]);

% 库路径
lib.UniversalBridge = 'powerlib/Power Electronics/Universal Bridge';
lib.PWM2L            = 'powerlib/Power Electronics/PWM Generator (2-Level)';
lib.ThreePhaseSource = 'powerlib/Electrical Sources/Three-Phase Source';
lib.SeriesRLC        = 'powerlib/Elements/Series RLC Branch';
lib.MeasureI         = 'powerlib/Measurements/Current Measurement';
lib.MeasureV         = 'powerlib/Measurements/Voltage Measurement';
lib.Ground           = 'powerlib/Elements/Ground';
lib.Breaker          = 'powerlib/Power Electronics/Breaker';
lib.IGBT             = 'powerlib/Power Electronics/IGBT';
lib.Diode            = 'powerlib/Power Electronics/Diode';
lib.AsynMotor        = 'powerlib/Machines/Asynchronous Machine SI Units';
lib.Mux              = 'simulink/Signal Routing/Mux';
lib.Inport           = 'simulink/Ports & Subsystems/In1';
lib.Outport          = 'simulink/Ports & Subsystems/Out1';
lib.Goto             = 'simulink/Signal Routing/Goto';
lib.From             = 'simulink/Signal Routing/From';

%% 公共三相电网
add_block(lib.ThreePhaseSource,[model '/Grid'],'Position',[50 180 120 260]);
blk = [model '/Grid'];
set_param_robust(blk, {'Vrms','Vll','V_LL','V_RMS','V_rms','Voltage','V','Amplitude','Van_rms','Vphase'}, '6600'); % 6.6 kV LL
set_param_robust(blk, {'Frequency','Freq','frequency','f'}, '50');
set_param_robust(blk, {'PhaseAngle','Phase','Phase_deg','PhaseA','PhaseAdeg'}, '0');

%% 两套供电单元
unitA = create_supply_unit(model, lib, 'UnitA', [300 50 1380 540],   0, 'A');
unitB = create_supply_unit(model, lib, 'UnitB', [300 560 1380 1050], 20,'B');

% 电网接两套单元三相入口
for k = 1:2
    uname = {'UnitA','UnitB'};
    for ph = 1:3
        safe_add_line(model, ['Grid/' num2str(ph)], [uname{k} '/ACin' num2str(ph)], '连接电网到供电单元AC端');
    end
end

save_system(model);
disp(['模型已生成：' model '。如有端口号差异，请打开相应子系统微调 DC 端口。']);

%% =========================== 本地函数区（必须放在文件末尾） ===========================

function unitName = create_supply_unit(mdl, lib, name, posRect, pwmPhaseDeg, tagSuffix)
    % 创建子系统外框
    add_block('simulink/Subsystem',[mdl '/' name],'Position',posRect);
    sys = [mdl '/' name]; open_system(sys);

    % ---- 1) 三相 AC 输入端口 ----
    for ph = 1:3
        add_block(lib.Inport,[sys '/ACin' num2str(ph)],'Position',[30 60+60*(ph-1) 60 80+60*(ph-1)]);
    end

    % ---- 2) 四象限整流器 (Universal Bridge + PWM) ----
    add_block(lib.UniversalBridge,[sys '/Rectifier'],...
        'PowerElectronicDevice','IGBT/Diode','SnubberResistance','1e6','SnubberCapacitance','0',...
        'Position',[200 70 310 230]);

    % 在 ACin 与 Rectifier 之间串接 LH1~LH3
    for ph = 1:3
        blkLH = [sys '/LH' num2str(ph) '_' tagSuffix];
        add_block(lib.MeasureI, blkLH, 'Position',[140 60+60*(ph-1) 170 80+60*(ph-1)]);
        safe_add_line(sys, ['ACin' num2str(ph) '/1'], ['LH' num2str(ph) '_' tagSuffix '/1'], '整流输入电流计接线-前段');
        safe_add_line(sys, ['LH' num2str(ph) '_' tagSuffix '/1'], ['Rectifier/' num2str(ph)], '整流输入电流计接线-后段');
    end

    % 整流器 PWM 及相位
    add_block(lib.PWM2L,[sys '/PWM_Rect'], 'Position',[120 260 220 320], 'Carrierfrequency','2000');
    add_block('simulink/Sources/Constant',[sys '/PhaseShift_deg'], 'Value', num2str(pwmPhaseDeg), 'Position',[30 260 80 320]);
    safe_add_line(sys,'PhaseShift_deg/1','PWM_Rect/1','Rectifier PWM相位输入');

    % 用 3 路 PWM 输出循环驱动 6 个门极（占位）
    for g = 1:6
        pwmOut = num2str(mod(g-1,3)+1);
        safe_add_line(sys, ['PWM_Rect/' pwmOut], ['Rectifier/' num2str(g+3)], ['Rectifier门极连线 g=' num2str(g)]);
    end

    % ---- 3) DC 链路：上下两只电容 ----
    add_block(lib.SeriesRLC,[sys '/C_up'],'BranchType','C','Capacitance','5e-3','Position',[370 90 420 140]);
    add_block(lib.SeriesRLC,[sys '/C_dn'],'BranchType','C','Capacitance','5e-3','Position',[370 180 420 230]);

    % DC 节点标签
    add_block(lib.Goto,[sys '/DCp_Goto'], 'GotoTag', ['DCp_' tagSuffix], 'Position',[460 100 510 120]);
    add_block(lib.From,[sys '/DCp_From'], 'GotoTag', ['DCp_' tagSuffix], 'Position',[540 100 590 120]);
    add_block(lib.Goto,[sys '/DCm_Goto'], 'GotoTag', ['DCm_' tagSuffix], 'Position',[460 200 510 220]);
    add_block(lib.From,[sys '/DCm_From'], 'GotoTag', ['DCm_' tagSuffix], 'Position',[540 200 590 220]);
    add_block(lib.Goto,[sys '/DCmid_Goto'], 'GotoTag', ['DCmid_' tagSuffix], 'Position',[460 150 510 170]);
    add_block(lib.From,[sys '/DCmid_From'], 'GotoTag', ['DCmid_' tagSuffix], 'Position',[540 150 590 170]);

    % LH4：整流直流侧电流（串在 Rectifier DC+ 与 C_up 之间）
    add_block(lib.MeasureI,[sys '/LH4_' tagSuffix],'Position',[330 140 360 160]);

    % 某些版本 Rectifier DC 端口号可能不同；优先尝试 4=DC+、5=DC-
    tried = false;
    try
        safe_add_line(sys,'Rectifier/4',['LH4_' tagSuffix '/1'],'Rectifier DC+ -> LH4'); tried = true;
        safe_add_line(sys,['LH4_' tagSuffix '/1'],'C_up/1','LH4 -> C_{up}+');
        safe_add_line(sys,'Rectifier/5','C_dn/1','Rectifier DC- -> C_{dn}+');
    catch ME
        % 若失败，尝试 7/8（有些版本含测量端口导致序号偏移）
        if tried, rethrow(ME); end
        warning('Rectifier DC端口4/5连接失败，尝试备用端口7/8...');
        safe_add_line(sys,'Rectifier/7',['LH4_' tagSuffix '/1'],'Rectifier DC+ (alt) -> LH4');
        safe_add_line(sys,['LH4_' tagSuffix '/1'],'C_up/1','LH4 -> C_{up}+ (alt)');
        safe_add_line(sys,'Rectifier/8','C_dn/1','Rectifier DC- (alt) -> C_{dn}+');
    end

    % 上下电容另一端并联到 DCmid
    safe_add_line(sys,'C_up/2','DCmid_Goto/1','C_up 下端 -> DCmid');
    safe_add_line(sys,'C_dn/2','DCmid_Goto/1','C_dn 下端 -> DCmid');

    % DC+、DC- 导出
    safe_add_line(sys,'C_up/1','DCp_Goto/1','DC+标签');
    safe_add_line(sys,'C_dn/1','DCm_Goto/1','DC-标签');

    % ---- 4) 预充/旁路 ----
    add_block(lib.SeriesRLC,[sys '/Rpchg'],'BranchType','R','Resistance','5','Position',[600 80 650 130]);
    add_block(lib.Breaker,[sys '/Precharge_BRK'],'Externalcontrol','on','Position',[680 80 720 130]);
    add_block(lib.Breaker,[sys '/Bypass_BRK'],'Externalcontrol','on','Position',[600 150 650 200]);

    safe_add_line(sys,'DCp_From/1','Rpchg/1','预充R起点');
    safe_add_line(sys,'Rpchg/2','Precharge_BRK/1','预充开关前');
    safe_add_line(sys,'Precharge_BRK/2','DCmid_From/1','预充入DCmid');

    safe_add_line(sys,'DCp_From/1','Bypass_BRK/1','旁路起点');
    safe_add_line(sys,'Bypass_BRK/2','DCmid_From/1','旁路入DCmid');

    % ---- 5) 斩波回路：IGBT + Rch + 二极管 ----
    add_block(lib.IGBT,[sys '/Chopper_IGBT'],'Ron','1e-3','ForwardVolt','1.5','Position',[600 240 650 290]);
    add_block(lib.Diode,[sys '/Chopper_Diode'],'Position',[600 300 650 340]);
    add_block(lib.SeriesRLC,[sys ['/Rch_' tagSuffix]],'BranchType','R','Resistance','20','Position',[700 260 750 310]);
    add_block(lib.MeasureI,[sys ['/LH_chop_' tagSuffix]],'Position',[760 260 810 310]);

    safe_add_line(sys,'DCp_From/1','Chopper_IGBT/1','斩波IGBT阳极');
    safe_add_line(sys,'Chopper_IGBT/2',['Rch_' tagSuffix '/1'],'IGBT -> Rch');
    safe_add_line(sys,['Rch_' tagSuffix '/2'],['LH_chop_' tagSuffix '/1'],'Rch -> LH_chop');
    safe_add_line(sys,['LH_chop_' tagSuffix '/1'],'DCm_From/1','LH_chop -> DC-');

    % 续流二极管并接 IGBT 两端
    safe_add_line(sys,'Chopper_IGBT/2','Chopper_Diode/1','斩波续流-上');
    safe_add_line(sys,'Chopper_Diode/2','Chopper_IGBT/1','斩波续流-下');

    % ---- 6) 固定放电 & 接地检测 ----
    add_block(lib.SeriesRLC,[sys '/R3'],'BranchType','R','Resistance','500','Position',[600 360 650 410]);
    add_block(lib.SeriesRLC,[sys '/R4'],'BranchType','R','Resistance','500','Position',[680 360 730 410]);
    safe_add_line(sys,'DCp_From/1','R3/1','R3起点');
    safe_add_line(sys,'R3/2','R4/1','R3->R4');

    add_block(lib.Goto,[sys '/R34mid_Goto'],'GotoTag',['R34mid_' tagSuffix],'Position',[730 390 780 410]);
    add_block(lib.From,[sys '/R34mid_From'],'GotoTag',['R34mid_' tagSuffix],'Position',[860 390 910 410]);
    safe_add_line(sys,'R4/2','R34mid_Goto/1','R34中点到标签');

    add_block(lib.MeasureV,[sys ['/VH_gndDet_' tagSuffix]],'Position',[740 330 790 350]);
    add_block(lib.Ground,[sys '/GND_det'],'Position',[800 380 820 400]);
    safe_add_line(sys,'R34mid_From/1',['VH_gndDet_' tagSuffix '/1'],'接地检测+');
    safe_add_line(sys,['VH_gndDet_' tagSuffix '/2'],'GND_det/1','接地检测-接地');
    % 中点实际接地（保持直流中点参考）
    safe_add_line(sys,'R34mid_From/1','GND_det/1','R34中点接地(并联)');

    % 其余固定放电网络：R5~R10
    add_block(lib.SeriesRLC,[sys '/R5'],'BranchType','R','Resistance','1e3','Position',[600 430 650 480]);
    add_block(lib.SeriesRLC,[sys '/R6'],'BranchType','R','Resistance','1e3','Position',[680 430 730 480]);
    safe_add_line(sys,'DCmid_From/1','R5/1','R5起点'); safe_add_line(sys,'R5/2','R6/1','R5->R6'); safe_add_line(sys,'R6/2','DCm_From/1','R6->DC-');

    add_block(lib.SeriesRLC,[sys '/R7'],'BranchType','R','Resistance','2e3','Position',[600 500 650 550]);
    add_block(lib.SeriesRLC,[sys '/R8'],'BranchType','R','Resistance','2e3','Position',[680 500 730 550]);
    safe_add_line(sys,'DCp_From/1','R7/1','R7起点'); safe_add_line(sys,'R7/2','R8/1','R7->R8'); safe_add_line(sys,'R8/2','DCm_From/1','R8->DC-');

    add_block(lib.SeriesRLC,[sys '/R9'],'BranchType','R','Resistance','1.5e3','Position',[760 430 810 480]);
    add_block(lib.SeriesRLC,[sys '/R10'],'BranchType','R','Resistance','1.5e3','Position',[760 500 810 550]);
    safe_add_line(sys,'DCp_From/1','R9/1','R9起点');  safe_add_line(sys,'R9/2','DCmid_From/1','R9->DCmid');
    safe
end