function newModel = rebuild_model_from_export(inputPath, newModelName)
% 重建 Simulink 模型（不需要 sigLines）
% inputPath     : 你前面导出的 MAT 或 JSON 文件路径（包含 elements/ports/connections）
% newModelName  : 新模型名称（缺省为 'recovered_model'）
%
% 还原内容：
% - 元件：名称、位置(Position)、朝向(Orientation)、库引用(LibraryLink)
% - 连接：基于 Source/Destination + 端口号，使用 autorouting 自动走线
%
% 限制与假设：
% - 若同一父系统中存在同名块，会尝试基于父系统匹配；仍歧义则取第一条并发出警告
% - 未保存 sigLines 时，拐点形状无法还原，仅能自动布线
% - 未保存块参数（除位置/朝向外），如需参数也可扩展从 JSON/MAT 中读取设置

	% -------- 1) 加载数据（MAT/JSON 自适应） --------
	if nargin < 1 || isempty(inputPath)
		error('请提供前面导出的 MAT 或 JSON 文件路径。');
	end
	if nargin < 2 || isempty(newModelName)
		newModelName = 'recovered_model';
	end

	data = load_or_decode_graph(inputPath);

	% 基本变量（若缺失则置空安全值）
	elements = safe_field(data, 'elements', struct('Path',{},'Name',{},'BlockType',{},'Orientation',{},'Position',{},'Center',{},'LibraryLink',{}));
	ports    = safe_field(data, 'ports',    struct('BlockPath',{},'PortNumber',{},'PortType',{},'Position',{})); %#ok<NASGU>
	conns    = safe_connections(data);

	assert(~isempty(elements), 'elements 为空，无法重建模型。');

	% 原模型根名（用于 path 重定位）；新模型名
	origRoot = get_root_from_elements(elements);
	newModel = char(newModelName);

	% -------- 2) 创建空白模型 --------
	if bdIsLoaded(newModel)
		close_system(newModel, 0);
	end
	new_system(newModel); open_system(newModel);

	% 预处理：计算新旧路径映射
    elemTable = preprocess_elements(elements, origRoot, newModel);

    % 过滤：剔除“库引用块”的内部内容（如 .../DC Voltage Source/Model）
    elemTable = filter_descendants_of_library_blocks(elemTable);
    
    % 先建 SubSystem，再建普通块
    subRows = elemTable(strcmp(elemTable.BlockType, 'SubSystem'), :);
    subRows = sortrows(subRows, 'Depth');
    for i = 1:height(subRows), create_block_in_model(subRows(i,:), true); end
    
    blkRows = elemTable(~strcmp(elemTable.BlockType, 'SubSystem'), :);
    blkRows = sortrows(blkRows, 'Depth');
    for i = 1:height(blkRows), create_block_in_model(blkRows(i,:), false); end

	% 3.1 先创建 SubSystem
	subRows = elemTable(strcmp(elemTable.BlockType, 'SubSystem'), :);
	subRows = sortrows(subRows, 'Depth');
	for i = 1:height(subRows)
		create_block_in_model(subRows(i,:), true);
	end

	% 3.2 再创建非 SubSystem 的普通块
	blkRows = elemTable(~strcmp(elemTable.BlockType, 'SubSystem'), :);
	blkRows = sortrows(blkRows, 'Depth');
	for i = 1:height(blkRows)
		create_block_in_model(blkRows(i,:), false);
	end

	% -------- 4) 建立连接（基于名字+端口号，结合路径推断父系统） --------
	% 构建“短名 -> 新路径列表”的索引，便于快速定位
	% 构建“短名 -> 新路径列表”的索引（取出-修改-写回）
    nameToPaths = containers.Map('KeyType','char','ValueType','any');
    for i = 1:height(elemTable)
        shortName = char(elemTable.ShortName{i});
        newPath   = char(elemTable.NewPath{i});
        if ~isKey(nameToPaths, shortName)
            nameToPaths(shortName) = {newPath};
        else
            paths = nameToPaths(shortName);
            paths{end+1} = newPath;
            nameToPaths(shortName) = paths;
        end
    end

	% 逐条连接
	for k = 1:numel(conns)
		S  = conns(k).Source;
		SP = conns(k).SourcePort;
		D  = conns(k).Destination;
		DP = conns(k).DestinationPort;

		% 跳过无效端口
		if ~isfinite(SP) || SP < 1 || ~isfinite(DP) || DP < 1
			warning('跳过无效连接：%s(%g) -> %s(%g)', S, SP, D, DP);
			continue;
		end

		% 候选路径
		if ~isKey(nameToPaths, S) || ~isKey(nameToPaths, D)
			warning('找不到源或目标块：%s 或 %s，跳过。', S, D);
			continue;
		end
		srcCandidates = nameToPaths(S);
		dstCandidates = nameToPaths(D);

		% 在候选中选出“同父系统”的一对
		[parentSys, srcPath, dstPath] = pick_pair_with_same_parent(srcCandidates, dstCandidates);
		if isempty(parentSys)
			% 兜底：取第一对并以其上层父系统作为 parent
			srcPath = srcCandidates{1};
			dstPath = dstCandidates{1};
			parentSys = common_parent_of_paths(srcPath, dstPath);
			warning('未找到同父系统的候选，兜底连接：%s -> %s（父系统：%s）', srcPath, dstPath, parentSys);
		end

		srcShort = basename(srcPath);
		dstShort = basename(dstPath);

		% 执行连接（autorouting）
		try
			add_line(parentSys, [srcShort '/' num2str(SP)], [dstShort '/' num2str(DP)], 'autorouting', 'on');
		catch ME
			warning('连接失败：%s(%g) -> %s(%g)。原因：%s', srcPath, SP, dstPath, DP, ME.message);
		end
	end

	% 最后整理
	set_param(newModel, 'SimulationCommand', 'update');
	disp(['模型已重建：' newModel]);
end

% ============================== 辅助函数 ==============================

function data = load_or_decode_graph(inputPath)
	% 加载 MAT 或 JSON，返回统一结构
	[~,~,ext] = fileparts(inputPath);
	switch lower(ext)
		case '.mat'
			S = load(inputPath);
			% 优先使用 graph，否则回退到散列变量
			if isfield(S, 'graph')
				data = S.graph;
			else
				data = struct();
				if isfield(S,'elements');     data.elements = S.elements;     end
				if isfield(S,'ports');        data.ports = S.ports;           end
				if isfield(S,'conn');         data.connections = S.conn;      end
				if isfield(S,'connectivity'); data.connectivity = S.connectivity; end
				if isfield(S,'linesOut');     data.lines = S.linesOut;        end %#ok<NASGU>
			end
		case '.json'
			txt  = fileread(inputPath);
			data = jsondecode(txt);
		otherwise
			error('不支持的文件类型：%s（请提供 .mat 或 .json）', ext);
	end
end

function v = safe_field(S, name, defaultV)
	% 安全取字段
	if isstruct(S) && isfield(S, name) && ~isempty(S.(name))
		v = S.(name);
	else
		v = defaultV;
	end
end

function conns = safe_connections(data)
	% 统一连接结构：支持 graph.connections 或 cell 形式的 connectivity
	if isfield(data, 'connections') && ~isempty(data.connections)
		conns = data.connections;
		return;
	end
	if isfield(data, 'connectivity') && ~isempty(data.connectivity)
		c = data.connectivity;
		if iscell(c)
			conns = [c{:}];
		else
			conns = c;
		end
	else
		conns = struct('Source',{},'SourcePort',{},'Destination',{},'DestinationPort',{});
	end
	% 端口 NaN -> -1（但连接阶段会再次过滤）
	for i = 1:numel(conns)
		if ~isfield(conns,'SourcePort') || isempty(conns(i).SourcePort) || ~isfinite(conns(i).SourcePort)
			conns(i).SourcePort = -1;
		end
		if ~isfield(conns,'DestinationPort') || isempty(conns(i).DestinationPort) || ~isfinite(conns(i).DestinationPort)
			conns(i).DestinationPort = -1;
		end
	end
end

function root = get_root_from_elements(elements)
	% 从 elements.Path 推断原根名
	p = elements(1).Path;
	slash = find(p=='/', 1, 'first');
	if isempty(slash)
		root = p;              % 已在根
	else
		root = p(1:slash-1);   % 根名
	end
end

function T = preprocess_elements(elements, origRoot, newModel)
    N = numel(elements);
    ShortName  = strings(N,1);
    OrigPath   = strings(N,1);
    NewPath    = strings(N,1);
    ParentPath = strings(N,1);
    BlockType  = strings(N,1);
    Orientation= strings(N,1);
    Left = zeros(N,1); Top = zeros(N,1); Right = zeros(N,1); Bottom = zeros(N,1);
    Depth = zeros(N,1);
    LibraryLink = strings(N,1);

    for i = 1:N
        OrigPath(i)   = string(elements(i).Path);
        ShortName(i)  = string(elements(i).Name);
        BlockType(i)  = string(def_str(elements(i), 'BlockType'));
        Orientation(i)= string(def_str(elements(i), 'Orientation', 'right'));
        pos = elements(i).Position; if isempty(pos), pos = [30 30 60 60]; end
        Left(i)=pos(1); Top(i)=pos(2); Right(i)=pos(3); Bottom(i)=pos(4);
        LibraryLink(i) = string(def_str(elements(i), 'LibraryLink'));

        np = sanitize_path(rebase_path(char(OrigPath(i)), origRoot, newModel));
        pp = sanitize_path(fileparts(np));
        NewPath(i)    = string(np);
        ParentPath(i) = string(pp);
        Depth(i)      = count(NewPath(i), "/");
    end

    T = table(ShortName, OrigPath, NewPath, ParentPath, BlockType, Orientation, ...
              Left, Top, Right, Bottom, LibraryLink, Depth);
end

function s = def_str(st, field, defaultV)
	% 结构取字符串字段，缺省为空/指定默认
	if nargin < 3, defaultV = ""; end
	if isfield(st, field) && ~isempty(st.(field))
		s = st.(field);
	else
		s = defaultV;
	end
end

function newPath = rebase_path(origPath, origRoot, newRoot)
	% 将原路径替换为新模型根路径
	origPath = char(origPath);
	origRoot = char(origRoot);
	newRoot  = char(newRoot);
	if startsWith(origPath, [origRoot '/'])
		suffix = origPath(length(origRoot)+2:end);
		if isempty(suffix)
			newPath = newRoot;
		else
			newPath = [newRoot '/' suffix];
		end
	elseif strcmp(origPath, origRoot)
		newPath = newRoot;
	else
		% 若无法识别，作为顶层块处理
		[newParent, base] = fileparts(origPath);
		if isempty(suffix)
			newPath = newRoot;
		else
			newPath = [newRoot '/' suffix];
		end
	end
end

function ensure_system_exists(sysPath)
	% 确保父系统链全部存在；逐级创建空 Subsystem
	if isempty(sysPath), return; end
	if bdIsRoot(sysPath)
		% 顶层模型天然存在
		return;
	end
	if exist_block(sysPath)
		return;
	end
	% 递归保证父存在
	parent = fileparts(sysPath);
	ensure_system_exists(parent);
	% 在 parent 下创建一个子系统，名称为当前末级名
	base = basename(sysPath);
	add_block('simulink/Ports & Subsystems/Subsystem', [parent '/' base], 'MakeNameUnique','off');
end

function tf = bdIsRoot(pathStr)
	% 判断是否是顶层模型名
	tf = isempty(fileparts(pathStr));
end

function tf = exist_block(pathStr)
	% 判断一个完整路径的块是否存在
	tf = false;
	try
		get_param(pathStr, 'Handle');
		tf = true;
	catch
		tf = false;
	end
end

function name = basename(pathStr)
	% 取最后一段名
	[~, name] = fileparts(pathStr);
end

function [parentSys, srcPath, dstPath] = pick_pair_with_same_parent(srcList, dstList)
	% 从候选路径中挑出“同父系统”的组合
	parentSys = '';
	srcPath = '';
	dstPath = '';
	for i = 1:numel(srcList)
		for j = 1:numel(dstList)
			p1 = fileparts(srcList{i});
			p2 = fileparts(dstList{j});
			if strcmp(p1, p2)
				parentSys = p1;
				srcPath = srcList{i};
				dstPath = dstList{j};
				return;
			end
		end
	end
end

function parent = common_parent_of_paths(p1, p2)
	% 求两条完整块路径的“最长共同父系统”
	s1 = split(string(p1), '/');
	s2 = split(string(p2), '/');
	n = min(numel(s1), numel(s2));
	idx = 0;
	for k = 1:n
		if s1(k) == s2(k), idx = k; else, break; end
	end
	if idx <= 1
		parent = char(s1(1)); % 顶层模型名
	else
		parent = strjoin(cellstr(s1(1:idx)),'/');
	end
end

newModel = rebuild_model_from_export(fullfile(pwd,'export_model_graph','model_graph.mat'), 'recovered_model');

function T = filter_descendants_of_library_blocks(T)
    % 找到所有库引用块的“根路径”
    libMask = T.LibraryLink ~= "";
    libRoots = cellfun(@char, T.NewPath(libMask), 'UniformOutput', false);
    if isempty(libRoots), return; end

    keep = true(height(T),1);
    for i = 1:height(T)
        np = char(T.NewPath{i});
        for r = 1:numel(libRoots)
            rootp = libRoots{r};
            if strncmp(np, [rootp '/'], length(rootp)+1)   % 位于库块内部
                keep(i) = false; 
                break;
            end
        end
    end
    T = T(keep,:);
end

function p = sanitize_path(p)
    % 去掉尾部斜杠，保持 Simulink 合法路径
    p = char(string(p));
    while ~isempty(p) && p(end) == '/', p = p(1:end-1); end
end

function create_block_in_model(row, isSubsystem)
    newPath = sanitize_path(char(row.NewPath));
    parent  = sanitize_path(char(row.ParentPath));
    btype   = char(row.BlockType);
    ori     = char(row.Orientation);
    pos     = [row.Left row.Top row.Right row.Bottom];
    lib     = char(row.LibraryLink);

    ensure_system_exists(parent);

    % 若已存在则仅更新位置/朝向
    if exist_block(newPath)
        try, set_param(newPath, 'Position', pos, 'Orientation', ori); end
        return;
    end

    try
        if isSubsystem
            if ~isempty(lib), add_block(lib, newPath, 'MakeNameUnique','off');
            else, add_block('simulink/Ports & Subsystems/Subsystem', newPath, 'MakeNameUnique','off'); end
        else
            if ~isempty(lib)
                add_block(lib, newPath, 'MakeNameUnique','off');
            else
                switch btype
                    case 'Inport',  add_block('simulink/Sources/In1', newPath, 'MakeNameUnique','off');
                    case 'Outport', add_block('simulink/Sinks/Out1',  newPath, 'MakeNameUnique','off');
                    otherwise
                        add_block('simulink/Ports & Subsystems/Subsystem', newPath, 'MakeNameUnique','off');
                        warning('块 %s 无库引用，使用 Subsystem 占位。', newPath);
                end
            end
        end
        set_param(newPath, 'Position', pos, 'Orientation', ori);
    catch ME
        warning('创建块失败：%s（原因：%s）', newPath, ME.message);
    end
end