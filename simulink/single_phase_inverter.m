function mdl = single_phase_inverter(cfg)
% 单相桥式逆变器（H-Bridge）仿真模型生成脚本（R2019a 兼容，无需额外工具箱）
% WHY: 用纯 Simulink 等效模型（不依赖 Simscape/SPS 库）稳健生成可运行的逆变器模型
% HOW: 单极性 PWM -> 两臂调制 gA/gB，构造 vo = Vdc*(gA - gB)；RL 负载用微分方程 di/dt=(vo-R*i)/L
%
% 备注：
% - 若你需要“器件级（IGBT/MOSFET/二极管）+ 物理电气端口”的模型，我可再给你 Simscape 版本（要求有 Simscape Electrical）。
% - 本脚本聚焦“为什么/怎么做”，尽量避免无用注释；保留对关键设计取舍的说明。

	% ========== 用户参数（可通过 cfg 覆盖） ==========
	p.Vdc   = 200;        % 直流母线电压 [V]
	p.fsw   = 10000;      % 开关频率（载波三角波）[Hz]
	p.fm    = 50;         % 基波频率（调制定弦波）[Hz]
	p.m     = 0.9;        % 调制度（0~1，单极性 PWM）
	p.R     = 10;         % 负载电阻 [Ohm]
	p.L     = 10e-3;      % 负载电感 [H]
	p.tstop = 0.1;        % 仿真时长 [s]
	p.maxStep = 1/(p.fsw*100); % WHY: 至少每周期100点，捕获PWM边沿与负载动态

	if nargin>=1 && isstruct(cfg)
		fn = fieldnames(cfg);
		for i = 1:numel(fn)
			if isfield(p, fn{i}), p.(fn{i}) = cfg.(fn{i}); end
		end
	end

	% ========== 创建/重建模型 ==========
	mdl = 'SinglePhaseHBridge_Unipolar';
	if bdIsLoaded(mdl), close_system(mdl,0); end
	if exist([mdl '.slx'],'file'), delete([mdl '.slx']); end % WHY: 保证从干净状态生成
	new_system(mdl);
	open_system(mdl);

	% 传参到基础工作区（供块参数表达式使用）
	assignin('base','fsw',p.fsw);
	assignin('base','fm', p.fm);
	assignin('base','Vdc',p.Vdc);
	assignin('base','Rload',p.R);
	assignin('base','Lload',p.L);

	% 仿真配置（变步长刚性较好的求解器）
	set_param(mdl, ...
		'StopTime', num2str(p.tstop), ...
		'Solver',   'ode23tb', ...
		'MaxStep',  num2str(p.maxStep), ...
		'StartTime','0');

	% 简化放置工具（少写坐标）
	function place(blk, pos), set_param(blk,'Position',pos); end 

	% ========== 信号源：调制定弦与三角载波 ==========
	% WHY: 单极性PWM需要两路调制：vma=+m*sin(2*pi*fm*t)、vmb=-vma；与同一三角载波比较
	a = add_block('simulink/Sources/Sine Wave',[mdl '/ModA'], 'MakeNameUnique','on');
	set_param(a, 'Amplitude', num2str(p.m), 'Frequency', '2*pi*fm', 'Bias','0','Phase','0','SampleTime','0');
	place(a,[60 60 120 90]);

	a_neg = add_block('simulink/Math Operations/Gain',[mdl '/Neg'], 'MakeNameUnique','on');
	set_param(a_neg,'Gain','-1');
	place(a_neg,[150 55 180 95]);

	% 三角载波用 Repeating Sequence 生成： [0, Tsw/2, Tsw/2, Tsw] => [1, -1, -1, 1]
	car = add_block('simulink/Sources/Repeating Sequence',[mdl '/Carrier'], 'MakeNameUnique','on');
	

	% ========== PWM 比较：得到两臂高桥臂导通指令（数值0/1） ==========
	% WHY: Relational Operator 输出逻辑量，后续转 double 参与算术
	cmpA = add_block('simulink/Logic and Bit Operations/Relational Operator',[mdl '/RelopA'], 'MakeNameUnique','on');
	set_param(cmpA, 'Operator','>');
	place(cmpA,[220 60 260 90]);

	cmpB = add_block('simulink/Logic and Bit Operations/Relational Operator',[mdl '/RelopB'], 'MakeNameUnique','on');
	set_param(cmpB, 'Operator','>');
	place(cmpB,[220 150 260 180]);

	% 比较输出转 double，便于后续 vo = Vdc*(gA - gB)
	dtcA = add_block('simulink/Signal Attributes/Data Type Conversion',[mdl '/ToDoubleA'], 'MakeNameUnique','on');
	set_param(dtcA,'OutDataTypeStr','double');
	place(dtcA,[280 60 320 90]);

	dtcB = add_block('simulink/Signal Attributes/Data Type Conversion',[mdl '/ToDoubleB'], 'MakeNameUnique','on');
	set_param(dtcB,'OutDataTypeStr','double');
	place(dtcB,[280 150 320 180]);

	% 接线：ModA -> Neg；ModA,Carrier -> RelopA；-ModA,Carrier -> RelopB
	add_line(mdl,'ModA/1','Neg/1','autorouting','on');
	add_line(mdl,'ModA/1','RelopA/1','autorouting','on');
	add_line(mdl,'Carrier/1','RelopA/2','autorouting','on');
	add_line(mdl,'Neg/1','RelopB/1','autorouting','on');
	add_line(mdl,'Carrier/1','RelopB/2','autorouting','on');
	add_line(mdl,'RelopA/1','ToDoubleA/1','autorouting','on');
	add_line(mdl,'RelopB/1','ToDoubleB/1','autorouting','on');

	% ========== H 桥等效输出：vo = Vdc*(gA - gB) ==========
	% WHY: 两个半桥节点分别为 {Vdc,0}，负载跨接两节点，电压等于差分（单极性PWM天然三电平）
	sumAB = add_block('simulink/Math Operations/Sum',[mdl '/gA_minus_gB'], 'MakeNameUnique','on');
	set_param(sumAB,'Inputs','|+-'); % 第一输入正、第二输入负
	place(sumAB,[360 100 380 140]);

	gainV = add_block('simulink/Math Operations/Gain',[mdl '/Gain_Vdc'], 'MakeNameUnique','on');
	set_param(gainV,'Gain','Vdc');
	place(gainV,[420 100 470 140]);

	add_line(mdl,'ToDoubleA/1','gA_minus_gB/1','autorouting','on');
	add_line(mdl,'ToDoubleB/1','gA_minus_gB/2','autorouting','on');
	add_line(mdl,'gA_minus_gB/1','Gain_Vdc/1','autorouting','on');

	% ========== RL 负载：di/dt = (vo - R*i)/L ==========
	% WHY: 用连续域微分方程刻画负载动态，不依赖电力电子专用库
	g_vo_over_L = add_block('simulink/Math Operations/Gain',[mdl '/Gain_1_over_L'], 'MakeNameUnique','on');
	set_param(g_vo_over_L,'Gain','1/Lload');
	place(g_vo_over_L,[500 100 560 140]);

	g_R_over_L = add_block('simulink/Math Operations/Gain',[mdl '/Gain_R_over_L'], 'MakeNameUnique','on');
	set_param(g_R_over_L,'Gain','Rload/Lload');
	place(g_R_over_L,[500 180 560 220]);

	sum_didt = add_block('simulink/Math Operations/Sum',[mdl '/Sum_didt'], 'MakeNameUnique','on');
	set_param(sum_didt,'Inputs','+-');
	place(sum_didt,[600 130 620 170]);

	int_i = add_block('simulink/Continuous/Integrator',[mdl '/Integrator_i'], 'MakeNameUnique','on');
	set_param(int_i,'InitialCondition','0');
	place(int_i,[660 130 680 170]);

	% 输出监测与记录
	scope = add_block('simulink/Sinks/Scope',[mdl '/Scope'], 'MakeNameUnique','on');
	set_param(scope,'NumInputPorts','2');
	place(scope,[820 110 870 190]);

	toWsV = add_block('simulink/Sinks/To Workspace',[mdl '/ToWs_vo'], 'MakeNameUnique','on');
	set_param(toWsV,'VariableName','vo','SaveFormat','Structure With Time');
	place(toWsV,[740 100 790 130]);

	toWsI = add_block('simulink/Sinks/To Workspace',[mdl '/ToWs_iL'], 'MakeNameUnique','on');
	set_param(toWsI,'VariableName','iL','SaveFormat','Structure With Time');
	place(toWsI,[740 160 790 190]);

	% 接线：vo/L、-R/L*i -> Sum；∫ -> i；反馈 R/L*i
	add_line(mdl,'Gain_Vdc/1','Gain_1_over_L/1','autorouting','on');
	add_line(mdl,'Gain_1_over_L/1','Sum_didt/1','autorouting','on');
	add_line(mdl,'Integrator_i/1','Gain_R_over_L/1','autorouting','on');
	add_line(mdl,'Gain_R_over_L/1','Sum_didt/2','autorouting','on');
	add_line(mdl,'Sum_didt/1','Integrator_i/1','autorouting','on');

	% 观测：vo 与 iL
	add_line(mdl,'Gain_Vdc/1','Scope/1','autorouting','on');
	add_line(mdl,'Integrator_i/1','Scope/2','autorouting','on');
	add_line(mdl,'Gain_Vdc/1','ToWs_vo/1','autorouting','on');
	add_line(mdl,'Integrator_i/1','ToWs_iL/1','autorouting','on');

	% ========== 保存并仿真（可选） ==========
	save_system(mdl);
	sim(mdl);

	% 让示波器窗口置前
	open_system(scope);

	% 方便上层脚本使用返回模型名
	if nargout==0, clear mdl; end
end