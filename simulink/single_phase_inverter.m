function mdl = single_phase_inverter(cfg)
% ������ʽ�������H-Bridge������ģ�����ɽű���R2019a ���ݣ�������⹤���䣩
% WHY: �ô� Simulink ��Чģ�ͣ������� Simscape/SPS �⣩�Ƚ����ɿ����е������ģ��
% HOW: ������ PWM -> ���۵��� gA/gB������ vo = Vdc*(gA - gB)��RL ������΢�ַ��� di/dt=(vo-R*i)/L
%
% ��ע��
% - ������Ҫ����������IGBT/MOSFET/�����ܣ�+ ��������˿ڡ���ģ�ͣ��ҿ��ٸ��� Simscape �汾��Ҫ���� Simscape Electrical����
% - ���ű��۽���Ϊʲô/��ô������������������ע�ͣ������Թؼ����ȡ���˵����

	% ========== �û���������ͨ�� cfg ���ǣ� ==========
	p.Vdc   = 200;        % ֱ��ĸ�ߵ�ѹ [V]
	p.fsw   = 10000;      % ����Ƶ�ʣ��ز����ǲ���[Hz]
	p.fm    = 50;         % ����Ƶ�ʣ����ƶ��Ҳ���[Hz]
	p.m     = 0.9;        % ���ƶȣ�0~1�������� PWM��
	p.R     = 10;         % ���ص��� [Ohm]
	p.L     = 10e-3;      % ���ص�� [H]
	p.tstop = 0.1;        % ����ʱ�� [s]
	p.maxStep = 1/(p.fsw*100); % WHY: ����ÿ����100�㣬����PWM�����븺�ض�̬

	if nargin>=1 && isstruct(cfg)
		fn = fieldnames(cfg);
		for i = 1:numel(fn)
			if isfield(p, fn{i}), p.(fn{i}) = cfg.(fn{i}); end
		end
	end

	% ========== ����/�ؽ�ģ�� ==========
	mdl = 'SinglePhaseHBridge_Unipolar';
	if bdIsLoaded(mdl), close_system(mdl,0); end
	if exist([mdl '.slx'],'file'), delete([mdl '.slx']); end % WHY: ��֤�Ӹɾ�״̬����
	new_system(mdl);
	open_system(mdl);

	% ���ε�����������������������ʽʹ�ã�
	assignin('base','fsw',p.fsw);
	assignin('base','fm', p.fm);
	assignin('base','Vdc',p.Vdc);
	assignin('base','Rload',p.R);
	assignin('base','Lload',p.L);

	% �������ã��䲽�����ԽϺõ��������
	set_param(mdl, ...
		'StopTime', num2str(p.tstop), ...
		'Solver',   'ode23tb', ...
		'MaxStep',  num2str(p.maxStep), ...
		'StartTime','0');

	% �򻯷��ù��ߣ���д���꣩
	function place(blk, pos), set_param(blk,'Position',pos); end 

	% ========== �ź�Դ�����ƶ����������ز� ==========
	% WHY: ������PWM��Ҫ��·���ƣ�vma=+m*sin(2*pi*fm*t)��vmb=-vma����ͬһ�����ز��Ƚ�
	a = add_block('simulink/Sources/Sine Wave',[mdl '/ModA'], 'MakeNameUnique','on');
	set_param(a, 'Amplitude', num2str(p.m), 'Frequency', '2*pi*fm', 'Bias','0','Phase','0','SampleTime','0');
	place(a,[60 60 120 90]);

	a_neg = add_block('simulink/Math Operations/Gain',[mdl '/Neg'], 'MakeNameUnique','on');
	set_param(a_neg,'Gain','-1');
	place(a_neg,[150 55 180 95]);

	% �����ز��� Repeating Sequence ���ɣ� [0, Tsw/2, Tsw/2, Tsw] => [1, -1, -1, 1]
	car = add_block('simulink/Sources/Repeating Sequence',[mdl '/Carrier'], 'MakeNameUnique','on');
	

	% ========== PWM �Ƚϣ��õ����۸��ű۵�ָͨ���ֵ0/1�� ==========
	% WHY: Relational Operator ����߼���������ת double ��������
	cmpA = add_block('simulink/Logic and Bit Operations/Relational Operator',[mdl '/RelopA'], 'MakeNameUnique','on');
	set_param(cmpA, 'Operator','>');
	place(cmpA,[220 60 260 90]);

	cmpB = add_block('simulink/Logic and Bit Operations/Relational Operator',[mdl '/RelopB'], 'MakeNameUnique','on');
	set_param(cmpB, 'Operator','>');
	place(cmpB,[220 150 260 180]);

	% �Ƚ����ת double�����ں��� vo = Vdc*(gA - gB)
	dtcA = add_block('simulink/Signal Attributes/Data Type Conversion',[mdl '/ToDoubleA'], 'MakeNameUnique','on');
	set_param(dtcA,'OutDataTypeStr','double');
	place(dtcA,[280 60 320 90]);

	dtcB = add_block('simulink/Signal Attributes/Data Type Conversion',[mdl '/ToDoubleB'], 'MakeNameUnique','on');
	set_param(dtcB,'OutDataTypeStr','double');
	place(dtcB,[280 150 320 180]);

	% ���ߣ�ModA -> Neg��ModA,Carrier -> RelopA��-ModA,Carrier -> RelopB
	add_line(mdl,'ModA/1','Neg/1','autorouting','on');
	add_line(mdl,'ModA/1','RelopA/1','autorouting','on');
	add_line(mdl,'Carrier/1','RelopA/2','autorouting','on');
	add_line(mdl,'Neg/1','RelopB/1','autorouting','on');
	add_line(mdl,'Carrier/1','RelopB/2','autorouting','on');
	add_line(mdl,'RelopA/1','ToDoubleA/1','autorouting','on');
	add_line(mdl,'RelopB/1','ToDoubleB/1','autorouting','on');

	% ========== H �ŵ�Ч�����vo = Vdc*(gA - gB) ==========
	% WHY: �������Žڵ�ֱ�Ϊ {Vdc,0}�����ؿ�����ڵ㣬��ѹ���ڲ�֣�������PWM��Ȼ����ƽ��
	sumAB = add_block('simulink/Math Operations/Sum',[mdl '/gA_minus_gB'], 'MakeNameUnique','on');
	set_param(sumAB,'Inputs','|+-'); % ��һ���������ڶ����븺
	place(sumAB,[360 100 380 140]);

	gainV = add_block('simulink/Math Operations/Gain',[mdl '/Gain_Vdc'], 'MakeNameUnique','on');
	set_param(gainV,'Gain','Vdc');
	place(gainV,[420 100 470 140]);

	add_line(mdl,'ToDoubleA/1','gA_minus_gB/1','autorouting','on');
	add_line(mdl,'ToDoubleB/1','gA_minus_gB/2','autorouting','on');
	add_line(mdl,'gA_minus_gB/1','Gain_Vdc/1','autorouting','on');

	% ========== RL ���أ�di/dt = (vo - R*i)/L ==========
	% WHY: ��������΢�ַ��̻̿����ض�̬����������������ר�ÿ�
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

	% ���������¼
	scope = add_block('simulink/Sinks/Scope',[mdl '/Scope'], 'MakeNameUnique','on');
	set_param(scope,'NumInputPorts','2');
	place(scope,[820 110 870 190]);

	toWsV = add_block('simulink/Sinks/To Workspace',[mdl '/ToWs_vo'], 'MakeNameUnique','on');
	set_param(toWsV,'VariableName','vo','SaveFormat','Structure With Time');
	place(toWsV,[740 100 790 130]);

	toWsI = add_block('simulink/Sinks/To Workspace',[mdl '/ToWs_iL'], 'MakeNameUnique','on');
	set_param(toWsI,'VariableName','iL','SaveFormat','Structure With Time');
	place(toWsI,[740 160 790 190]);

	% ���ߣ�vo/L��-R/L*i -> Sum���� -> i������ R/L*i
	add_line(mdl,'Gain_Vdc/1','Gain_1_over_L/1','autorouting','on');
	add_line(mdl,'Gain_1_over_L/1','Sum_didt/1','autorouting','on');
	add_line(mdl,'Integrator_i/1','Gain_R_over_L/1','autorouting','on');
	add_line(mdl,'Gain_R_over_L/1','Sum_didt/2','autorouting','on');
	add_line(mdl,'Sum_didt/1','Integrator_i/1','autorouting','on');

	% �۲⣺vo �� iL
	add_line(mdl,'Gain_Vdc/1','Scope/1','autorouting','on');
	add_line(mdl,'Integrator_i/1','Scope/2','autorouting','on');
	add_line(mdl,'Gain_Vdc/1','ToWs_vo/1','autorouting','on');
	add_line(mdl,'Integrator_i/1','ToWs_iL/1','autorouting','on');

	% ========== ���沢���棨��ѡ�� ==========
	save_system(mdl);
	sim(mdl);

	% ��ʾ����������ǰ
	open_system(scope);

	% �����ϲ�ű�ʹ�÷���ģ����
	if nargout==0, clear mdl; end
end