function newModel = Copy_of_Rebuild_Model(inputPath, newModelName)
% 重建 Simulink 模型（不依赖 sigLines�?
% - 直接使用导出的完整路�? SourcePath/DestinationPath 精确定位�?
% - 端口号缺�?(-1/NaN)时，使用端口/块几何位置最近邻匹配到端口句�?
% - 自动创建父系统，过滤库块内部子块，所有连线使�? autorouting

	% -------- 1) 加载数据（MAT/JSON 自�?�应�? --------
	if nargin < 1 || isempty(inputPath)
		error('请提供前面导出的 MAT �? JSON 文件路径�?');
	end
	if nargin < 2 || isempty(newModelName)
		newModelName = 'recovered_model';
	end

	data = load_or_decode_graph(inputPath);

	% 基本变量（若缺失则置空安全�?�）
	elements = safe_field(data, 'elements', struct('Path',{},'Name',{},'BlockType',{},'Orientation',{},'Position',{},'Center',{},'LibraryLink',{}));
	ports    = safe_field(data, 'ports',    struct('BlockPath',{},'PortNumber',{},'PortType',{},'Position',{}));
	conns    = safe_connections(data);  % 现在包含 SourcePath/DestinationPath

	assert(~isempty(elements), 'elements 为空，无法重建模型�??');

	% 原模型根名（用于 path 重定位）；新模型�?
	origRoot = get_root_from_elements(elements);
	newModel = char(newModelName);

	% -------- 2) 创建空白模型 --------
	if bdIsLoaded(newModel)
		close_system(newModel, 0);
	end
	new_system(newModel); open_system(newModel);

	% -------- 3) 预处理块并创建（�?->子） --------
	elemTable = preprocess_elements(elements, origRoot, newModel);              % 计算新旧路径映射
	elemTable = filter_descendants_of_library_blocks(elemTable);                % 过滤库内部子�?

	% 先建 SubSystem
	subRows = elemTable(strcmp(elemTable.BlockType, 'SubSystem'), :);
	subRows = sortrows(subRows, 'Depth');
	for i = 1:height(subRows), create_block_in_model(subRows(i,:), true); end

	% 再建普�?�块
	blkRows = elemTable(~strcmp(elemTable.BlockType, 'SubSystem'), :);
	blkRows = sortrows(blkRows, 'Depth');
	for i = 1:height(blkRows), create_block_in_model(blkRows(i,:), false); end

	% -------- 4) 建立连接（优先用完整路径；端口句柄回�?几何匹配�? --------
	% 索引：原路径->新路径；新路�?->中心坐标；原路径->导出端口集合
	orig2new = containers.Map('KeyType','char','ValueType','char');
	centers  = containers.Map('KeyType','char','ValueType','any');
	for i = 1:height(elemTable)
		orig2new(char(elemTable.OrigPath{i})) = char(elemTable.NewPath{i});
		cx = (elemTable.Left(i)+elemTable.Right(i))/2;
		cy = (elemTable.Top(i)+elemTable.Bottom(i))/2;
		centers(char(elemTable.NewPath{i})) = [cx, cy];
	end
	portsByOrig = index_ports_by_blockpath(ports);  % origPath -> 端口结构数组

	for k = 1:numel(conns)
		% 读取连接的完整原路径；若缺失则回�?用名称匹配（极少出现�?
		srcOrigPath = char(getfield_or_default(conns(k), 'SourcePath', ''));
		dstOrigPath = char(getfield_or_default(conns(k), 'DestinationPath', ''));
		srcName     = char(getfield_or_default(conns(k), 'Source', ''));
		dstName     = char(getfield_or_default(conns(k), 'Destination', ''));
		SP          = getfield_or_default(conns(k), 'SourcePort', -1);
		DP          = getfield_or_default(conns(k), 'DestinationPort', -1);

		% 路径重映射到新模�?
		if ~isempty(srcOrigPath)
			srcNewPath = sanitize_path(rebase_path(srcOrigPath, origRoot, newModel));
		else
			% 回�??：按名称�? elemTable 中找第一条（不建议，但兼容旧数据�?
			idx = find(strcmp(elemTable.ShortName, string(srcName)), 1, 'first');
			if isempty(idx), warning('找不到源块：%s。跳过该连接�?', srcName); continue; end
			srcNewPath = char(elemTable.NewPath(idx));
			srcOrigPath = char(elemTable.OrigPath(idx));
		end

		if ~isempty(dstOrigPath)
			dstNewPath = sanitize_path(rebase_path(dstOrigPath, origRoot, newModel));
		else
			idx = find(strcmp(elemTable.ShortName, string(dstName)), 1, 'first');
			if isempty(idx), warning('找不到目标块�?%s。跳过该连接�?', dstName); continue; end
			dstNewPath = char(elemTable.NewPath(idx));
			dstOrigPath = char(elemTable.OrigPath(idx));
		end

		% 同父系统校验
		ps = fileparts(srcNewPath); pd = fileparts(dstNewPath);
		if ~strcmp(ps, pd)
			% �?般不应跨系统；此处保底：取共同父系统并继�?
			parentSys = common_parent_of_paths(srcNewPath, dstNewPath);
		else
			parentSys = ps;
		end

		% 计算中心，供几何回�??匹配使用
		if isKey(centers, dstNewPath), dstCenter = centers(dstNewPath); else, dstCenter = [inf inf]; end
		if isKey(centers, srcNewPath), srcCenter = centers(srcNewPath); else, srcCenter = [inf inf]; end

		% 解析�?/目标端口句柄：优先端口号；无效则�?近邻几何匹配（含保守端口�?
		try
			srcH = resolve_port_handle_by_geom(srcNewPath, srcOrigPath, portsByOrig, 'src', dstCenter, SP);
			dstH = resolve_port_handle_by_geom(dstNewPath, dstOrigPath, portsByOrig, 'dst', srcCenter, DP);
		catch ME
			warning('端口解析失败�?%s(%g) -> %s(%g)。原因：%s', srcNewPath, SP, dstNewPath, DP, ME.message);
			continue;
		end

		% 执行连接（autorouting�?
		try
			add_line(parentSys, srcH, dstH, 'autorouting','on');
		catch ME
			warning('连接失败�?%s(%g) -> %s(%g)。原因：%s', srcNewPath, SP, dstNewPath, DP, ME.message);
		end
	end

	% �?后整�?
	set_param(newModel, 'SimulationCommand', 'update');
	disp(['模型已重建：' newModel]);
end

% ============================== 辅助函数 ==============================

function data = load_or_decode_graph(inputPath)
	% 加载 MAT �? JSON，返回统�?结构
	[~,~,ext] = fileparts(inputPath);
	switch lower(ext)
		case '.mat'
			S = load(inputPath);
			if isfield(S, 'graph')
				data = S.graph;
			else
				data = struct();
				if isfield(S,'elements');     data.elements = S.elements;     end
				if isfield(S,'ports');        data.ports = S.ports;           end
				if isfield(S,'conn');         data.connections = S.conn;      end
				if isfield(S,'connectivity'); data.connectivity = S.connectivity; end
			end
		case '.json'
			txt  = fileread(inputPath);
			data = jsondecode(txt);
		otherwise
			error('不支持的文件类型�?%s（请提供 .mat �? .json�?', ext);
	end
end

function v = safe_field(S, name, defaultV)
	if isstruct(S) && isfield(S, name) && ~isempty(S.(name))
		v = S.(name);
	else
		v = defaultV;
	end
end

function conns = safe_connections(data)
	% 标准化连接结构，确保包含 SourcePath/DestinationPath
	if isfield(data, 'connections') && ~isempty(data.connections)
		conns = data.connections;
	elseif isfield(data, 'connectivity') && ~isempty(data.connectivity)
		c = data.connectivity;
		if iscell(c), conns = [c{:}]; else, conns = c; end
	else
		conns = struct('Source',{},'SourcePath',{},'SourcePort',{},'Destination',{},'DestinationPath',{},'DestinationPort',{});
	end
	% 字段补齐
	fieldsNeeded = {'Source','SourcePath','SourcePort','Destination','DestinationPath','DestinationPort'};
	for i = 1:numel(conns)
		for f = 1:numel(fieldsNeeded)
			fd = fieldsNeeded{f};
			if ~isfield(conns, fd) || isempty(conns(i).(fd))
				switch fd
					case {'Source','Destination','SourcePath','DestinationPath'}
						conns(i).(fd) = '';
					case {'SourcePort','DestinationPort'}
						conns(i).(fd) = -1;
				end
			end
		end
	end
end

function root = get_root_from_elements(elements)
	% �? elements.Path 推断原根�?
	p = elements(1).Path;
	slash = find(p=='/', 1, 'first');
	if isempty(slash)
		root = p;              % 已在�?
	else
		root = p(1:slash-1);   % 根名
	end
end

function T = preprocess_elements(elements, origRoot, newModel)
	% 生成用于创建的表格，并映射到新模型路�?
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
	if nargin < 3, defaultV = ""; end
	if isfield(st, field) && ~isempty(st.(field))
		s = st.(field);
	else
		s = defaultV;
	end
end

function newPath = rebase_path(origPath, origRoot, newRoot)
	% 将原路径替换为新模型根路径；无法精确映射时取�?后一级名挂到新根
	origPath = char(origPath);
	origRoot = char(origRoot);
	newRoot  = char(newRoot);
	if startsWith(origPath, [origRoot '/'])
		suffix = origPath(length(origRoot)+2:end);
		if isempty(suffix), newPath = newRoot; else, newPath = [newRoot '/' suffix]; end
	elseif strcmp(origPath, origRoot)
		newPath = newRoot;
	else
		[~, base] = fileparts(origPath);
		newPath = [newRoot '/' base];
	end
end

function T = filter_descendants_of_library_blocks(T)
	% 剔除库引用块内部子块（例�? ".../DC Voltage Source/Model"�?
	libMask = T.LibraryLink ~= "";
	libRoots = cellfun(@char, T.NewPath(libMask), 'UniformOutput', false);
	if isempty(libRoots), return; end

	keep = true(height(T),1);
	for i = 1:height(T)
		np = char(T.NewPath{i});
		for r = 1:numel(libRoots)
			rootp = libRoots{r};
			if strncmp(np, [rootp '/'], length(rootp)+1)
				keep(i) = false; break;
			end
		end
	end
	T = T(keep,:);
end

function p = sanitize_path(p)
	% 去掉尾部斜杠，保�? Simulink 合法路径
	p = char(string(p));
	while ~isempty(p) && p(end) == '/', p = p(1:end-1); end
end

function ensure_system_exists(sysPath)
	% 逐级创建父系�?
	sysPath = sanitize_path(sysPath);
	if isempty(sysPath) || bdIsRoot(sysPath), return; end
	if exist_block(sysPath), return; end
	parent = fileparts(sysPath);
	ensure_system_exists(parent);
	base = basename(sysPath);
	if ~exist_block([parent '/' base])
		add_block('simulink/Ports & Subsystems/Subsystem', [parent '/' base], 'MakeNameUnique','off');
	end
end

function tf = bdIsRoot(pathStr)
	tf = isempty(fileparts(pathStr));
end

function tf = exist_block(pathStr)
	tf = false;
	try, get_param(pathStr, 'Handle'); tf = true; catch, tf = false; end
end

function name = basename(pathStr)
	[~, name] = fileparts(pathStr);
end

function parent = common_parent_of_paths(p1, p2)
	s1 = split(string(p1), '/'); s2 = split(string(p2), '/');
	n = min(numel(s1), numel(s2)); idx = 1;
	for k = 1:n
		if s1(k) ~= s2(k), break; end
		idx = k;
	end
	parent = char(strjoin(cellstr(s1(1:idx)), '/'));
end

function create_block_in_model(row, isSubsystem)
	% 实际创建单个块，并设置位�?/朝向（幂等）
	newPath = sanitize_path(char(row.NewPath));
	parent  = sanitize_path(char(row.ParentPath));
	btype   = char(row.BlockType);
	ori     = char(row.Orientation);
	pos     = [row.Left row.Top row.Right row.Bottom];
	lib     = char(row.LibraryLink);

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
						add_block('simulink/Ports & Subsystems/Subsystem', newPath, 'MakeNameUnique','off');
						warning('�? %s 无库引用，使�? Subsystem 占位�?', newPath);
				end
			end
		end
		set_param(newPath, 'Position', pos, 'Orientation', ori);
	catch ME
		warning('创建块失败：%s（原因：%s�?', newPath, ME.message);
	end
end

% ---------- 连接阶段辅助 ----------

function portsIdx = index_ports_by_blockpath(ports)
	% 将导出的 ports �? BlockPath（原路径）分�?
	portsIdx = containers.Map('KeyType','char','ValueType','any');
	for i = 1:numel(ports)
		bp = char(ports(i).BlockPath);
		if ~isKey(portsIdx, bp)
			portsIdx(bp) = ports(i);
		else
			tmp = portsIdx(bp); tmp(end+1) = ports(i); portsIdx(bp) = tmp;
		end
	end
end

function h = resolve_port_handle_by_geom(newPath, origPath, portsIdx, role, otherCenter, preferredNum)
	% 基于“端口号优先 + 几何�?近邻回�??”解析端口句�?
	ph  = get_param(newPath, 'PortHandles');

	% 1) 若端口号有效，且 Inport/Outport 数量足够，直接取
	if isfinite(preferredNum) && preferredNum>=1
		try
			if strcmp(role,'src') && isfield(ph,'Outport') && numel(ph.Outport) >= preferredNum
				h = ph.Outport(preferredNum); return;
			end
			if strcmp(role,'dst') && isfield(ph,'Inport') && numel(ph.Inport) >= preferredNum
				h = ph.Inport(preferredNum);  return;
			end
		catch
		end
	end

	% 2) 收集�?有可用端口句柄及其坐标（含保守端口）
	allH = []; allP = []; allT = {};
	kinds = {'Outport','Inport','LConn','RConn','Conn'};
	for i = 1:numel(kinds)
		k = kinds{i};
		if isfield(ph, k) && ~isempty(ph.(k))
			hh = ph.(k)(:);
			pp = arrayfun(@(x) get_param(x,'Position'), hh, 'UniformOutput', false);
			pp = vertcat(pp{:});
			allH  = [allH; hh];
			allP  = [allP; pp];
			allT  = [allT; repmat({k}, numel(hh), 1)];
		end
	end
	assert(~isempty(allH), '�? %s 无任何端口�??', newPath);

	% 3) 候�?�集合：源优先出端口，目标优先入端口；物理端口两侧均允许
	if strcmp(role,'src')
		prefMask = ismember(allT, {'Outport','LConn','RConn','Conn'});
	else
		prefMask = ismember(allT, {'Inport','LConn','RConn','Conn'});
	end
	candH = allH(prefMask); candP = allP(prefMask, :);
	if isempty(candH), candH = allH; candP = allP; end

	% 4) 若有原端口坐标，选距离�?�对端中心�?�或该坐标最近的端口；否则用对端中心
	target = [];
	if isKey(portsIdx, origPath)
		P = portsIdx(origPath);
		if strcmp(role,'src')
			mask = strcmp({P.PortType}, 'outport') | strcmp({P.PortType}, 'port') | strcmp({P.PortType}, 'conserving');
		else
			mask = strcmp({P.PortType}, 'inport')  | strcmp({P.PortType}, 'port') | strcmp({P.PortType}, 'conserving');
		end
		P2 = P(mask); if isempty(P2), P2 = P; end
		pts = vertcat(P2.Position);
		d   = hypot(pts(:,1)-otherCenter(1), pts(:,2)-otherCenter(2));
		[~, idx] = min(d);
		target = P2(idx).Position;
	end
	if isempty(target), target = otherCenter; end

	% 5) �?近邻选择
	d = hypot(candP(:,1)-target(1), candP(:,2)-target(2));
	[~, iMin] = min(d);
	h = candH(iMin);
end

function val = getfield_or_default(S, field, def)
	if isfield(S, field) && ~isempty(S.(field))
		val = S.(field);
	else
		val = def;
	end
end
%newModel = rebuild_model_from_export(fullfile(pwd,'export_model_graph','model_graph.mat'), 'recovered_model');