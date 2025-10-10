new_system('temp'); open_system('temp');
load_system('simulink');
load_system('MyLib/MyLib.slx')
add_block('simulink/Continuous/Derivative', 'temp/Derivative', 'MakeNameUnique','off');
add_block('MyLib/control', 'temp/control');
close_system('MyLib')

% ---------------- 参数回放（来自 model_params.json）演示 ----------------
% why: 将导出的模型/块参数自动回放到当前模型，保证“直接仿真”成为可能
% how: 优先从当前目录的 export_model_graph/model_params.json 读取；
%      如果存在 model 与 blocks 字段，分别回放模型级与块级参数；
%      块路径无法直接匹配时，回退用短名在模型内搜索首个同名块。
try
	jsonDir = fullfile(pwd, 'export_model_graph');
	params  = load_params_from_json(jsonDir);
	apply_parameters_to_model_llm('temp', params);
catch ME
	warning('参数回放跳过：%s', ME.message);
end

function create_block_in_model(row, isSubsystem)
	% 实际创建单个块，并设置位 ?/朝向（幂等）
	newPath = sanitize_path(char(row.NewPath));
	parent  = sanitize_path(char(row.ParentPath));
	btype   = char(row.BlockType);
    name    = char(row.ShortName);
	ori     = char(row.Orientation);
	pos     = [row.Left row.Top row.Right row.Bottom];
    lib     = char(row.LibraryLink);
    mir     = '';
    rot     = '';
    gtag    = '';
    ftag    = '';
    if ismember('Mirror', row.Properties.VariableNames),   mir  = char(row.Mirror);   end
    if ismember('Rotation', row.Properties.VariableNames), rot  = char(row.Rotation); end
    if ismember('GotoTag', row.Properties.VariableNames),  gtag = char(row.GotoTag);  end
    if ismember('FromTag', row.Properties.VariableNames),  ftag = char(row.FromTag);  end
    if ismember('GotoVisibility', row.Properties.VariableNames)
        gvis = char(row.GotoVisibility);
    else
        gvis = '';
    end

	ensure_system_exists(parent);

	if exist_block(newPath)
		try, set_param(newPath, 'Position', pos, 'Orientation', ori); end
		return;
	end

	try
		if isSubsystem
			if ~isempty(lib)
				add_block(lib, newPath, 'MakeNameUnique','off');
			else
				add_block('simulink/Ports & Subsystems/Subsystem', newPath, 'MakeNameUnique','off');
			end
		else
			if ~isempty(lib)
				add_block(lib, newPath, 'MakeNameUnique','off');
			else
				switch btype
					case 'Inport'
						add_block('simulink/Sources/In1', newPath, 'MakeNameUnique','off');
					case 'Outport'
						add_block('simulink/Sinks/Out1',  newPath, 'MakeNameUnique','off');
                    otherwise 
                        temp_block = find_system('simulink', 'BlockType', btype, 'Name', regexprep(name, '\d+$', ''));
                        add_block(char(temp_block(1,1)),newPath,'MakeNameUnique','off')
				end
			end
		end
        set_param(newPath, 'Position', pos, 'Orientation', ori);
        % 尝试回放镜像/旋转
        if ~isempty(mir)
            try, set_param(newPath,'BlockMirror',mir); end
        end
        if ~isempty(rot)
            try, set_param(newPath,'BlockRotation',rot); end
        end
        % 回放 Goto/From 标签
        if strcmpi(btype,'Goto')
            if ~isempty(gtag), try, set_param(newPath,'GotoTag',gtag); end, end
            if ~isempty(gvis), try, set_param(newPath,'TagVisibility',gvis); end, end
        elseif strcmpi(btype,'From') && ~isempty(ftag)
            try, set_param(newPath,'GotoTag',ftag); end
        end
	catch ME
		warning('创建块失败：%s（原因：%s ?', newPath, ME.message);
	end
end

function params = load_params_from_json(dirOrFile)
	% why: 读取与导出 CSV 同目录的 model_params.json；也支持直接传入 json 路径
	% how: 返回结构体，包含可选字段 model、blocks
	if nargin < 1 || isempty(dirOrFile)
		dirOrFile = fullfile(pwd, 'export_model_graph');
	end
	[dirp,~,ext] = fileparts(dirOrFile);
	if isempty(dirp) || (~isempty(ext) && ~strcmpi(ext,'.json'))
		% 输入为目录
		jsonPath = fullfile(dirOrFile, 'model_params.json');
	else
		% 输入为文件
		jsonPath = dirOrFile;
	end
	if exist(jsonPath, 'file') ~= 2
		error('找不到参数文件：%s', jsonPath);
	end
	txt = fileread(jsonPath);
	params = jsondecode(txt);
end

function apply_parameters_to_model_llm(modelName, params)
	% why: 将 params.model 和 params.blocks 回放到指定模型
	% how: 模型级直接 set_param；块级优先按 Path 精确匹配，否则用短名搜索
	if nargin < 2 || isempty(params), return; end
	try
		if isfield(params,'model') && ~isempty(params.model)
			apply_model_params_llm(modelName, params.model);
		end
	catch
	end
	try
		if isfield(params,'blocks') && ~isempty(params.blocks)
			apply_block_params_llm(modelName, params.blocks);
		end
	catch
	end
end

function apply_model_params_llm(modelName, modelParams)
	% 将模型级参数逐项 set_param
	fns = fieldnames(modelParams);
	for i = 1:numel(fns)
		pname = fns{i};
		try
			set_param(modelName, pname, char(modelParams.(pname)));
		catch
			% 某些版本/产品未安装时参数不可用，忽略
		end
	end
end

function apply_block_params_llm(modelName, blockParams)
	% 将块级参数逐项 set_param；无法直接按 Path 匹配时，用短名 find_system 回退
	for i = 1:numel(blockParams)
		try
			bp = blockParams(i);
			newPath = '';
			if isfield(bp,'Path') && ~isempty(bp.Path)
				% 尝试按绝对路径
				candidate = char(bp.Path);
				if startsWith(candidate, [modelName '/']) || strcmp(candidate, modelName)
					newPath = candidate;
				end
				% 路径不在当前模型：用短名回退
				if isempty(newPath)
					shortName = get_basename(candidate);
					lst = find_system(modelName, 'LookUnderMasks','all', 'SearchDepth', Inf, 'Name', shortName);
					if ~isempty(lst), newPath = lst{1}; end
				end
			end
			if isempty(newPath)
				continue;
			end

			if isfield(bp,'DialogParams') && ~isempty(bp.DialogParams)
				% WHY: 成组一次性 set_param，避免掩模块在中间态触发初始化校验失败（如 Repeating Sequence 时间/输出向量长度一致性）
				% HOW: 收集名值对，数值/数组使用 mat2str 序列化为字符串后再传入 set_param
				names = fieldnames(bp.DialogParams);
				nvPairs = {};
				for k = 1:numel(names)
					pname = names{k};
					% 跳过几何类参数（几何参数非运行期配置，不需要参与成组 set_param）
					if any(strcmpi(pname, {'Position','PortConnectivity','LineHandles'}))
						continue;
					end
					pval = bp.DialogParams.(pname);
					% 将非字符串参数序列化为可被 set_param 接受的字符串
					if ~(ischar(pval) || isstring(pval))
						pval = mat2str(pval);
					end
					nvPairs(end+1:end+2) = {pname, char(pval)}; %#ok<AGROW>
				end
				if ~isempty(nvPairs)
					try
						set_param(newPath, nvPairs{:});
					catch
						% 掩模/变体导致不可设置时跳过
					end
				end
			end
		catch
		end
	end
end

function b = get_basename(p)
	[~, b] = fileparts(char(p));
end

function p = sanitize_path(p)
	% 由于该脚本复用了 Rebuild 栈中的工具，这里补充 sanitize_path 以保证路径合法
	p = char(string(p));
	while ~isempty(p) && p(end) == '/', p = p(1:end-1); end
end