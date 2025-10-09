function newModel = rebuild_model_from_export(inputPath, newModelName)
% 重建 Simulink 模型（不依赖 sigLines ?
% - 直接使用导出的完整路 ? SourcePath/DestinationPath 精确定位 ?
% - 端口号缺 ?(-1/NaN)时，使用端口/块几何位置最近邻匹配到端口句 ?
% - 自动创建父系统，过滤库块内部子块，所有连线使 ? autorouting

	% -------- 1) 加载数据（MAT/JSON 自 ? 应 ? --------
	if nargin < 1 || isempty(inputPath)
		error('请提供前面导出的 MAT  ? JSON 文件路径 ?');
	end
	if nargin < 2 || isempty(newModelName)
		newModelName = 'recovered_model';
	end

	data = load_or_decode_graph(inputPath);

	% 基本变量（若缺失则置空安全）
	elements = safe_field(data, 'elements', struct('Path',{},'Name',{},'BlockType',{},'Orientation',{},'Position',{},'Center',{},'LibraryLink',{},'Mirror',{},'Rotation',{},'GotoTag',{},'GotoVisibility',{},'FromTag',{}));
	ports    = safe_field(data, 'ports',    struct('BlockPath',{},'PortNumber',{},'PortType',{},'Position',{},'RelPos',{},'Side',{}));
	conns    = safe_connections(data);  % 现在包含 SourcePath/DestinationPath
    params   = safe_field(data, 'parameters', struct());  % 新增：模型与块参数（可选）
    % 侧文件回退：若模型级为空，则尝试加载同目录 params.json/csv
    params   = ensure_params_loaded(params, inputPath);

	assert(~isempty(elements), 'elements 为空，无法重建模型 ??');

    
    
	% 原模型根名（用于 path 重定位）；新模型 ?
	origRoot = get_root_from_elements(elements);
	newModel = char(newModelName);

	% -------- 2) 创建空白模型 --------
	if bdIsLoaded(newModel)
		close_system(newModel, 0);
	end
	new_system(newModel); open_system(newModel);
    load_system('simulink');

	% -------- 3) 预处理块并创建（ ?->子） --------
	elemTable = preprocess_elements(elements, origRoot, newModel);              % 计算新旧路径映射
	elemTable = filter_descendants_of_library_blocks(elemTable);                % 过滤库内部子 ?

	% 先建 SubSystem
	subRows = elemTable(strcmp(elemTable.BlockType, 'SubSystem'), :);
	subRows = sortrows(subRows, 'Depth');
	for i = 1:height(subRows), create_block_in_model(subRows(i,:), true); end

	% 再建普通块
	blkRows = elemTable(~strcmp(elemTable.BlockType, 'SubSystem'), :);
	blkRows = sortrows(blkRows, 'Depth');
	for i = 1:height(blkRows), create_block_in_model(blkRows(i,:), false); end

	% -------- 4) 建立连接（优先用完整路径；端口句柄回 ?几何匹配 ? --------
	% 索引：原路径->新路径；新路 ?->中心坐标；原路径->导出端口集合
	orig2new = containers.Map('KeyType','char','ValueType','char');
	centers  = containers.Map('KeyType','char','ValueType','any');
	for i = 1:height(elemTable)
		orig2new(char(elemTable.OrigPath{i})) = char(elemTable.NewPath{i});
		cx = (elemTable.Left(i)+elemTable.Right(i))/2;
		cy = (elemTable.Top(i)+elemTable.Bottom(i))/2;
		centers(char(elemTable.NewPath{i})) = [cx, cy];
	end
	portsByOrig = index_ports_by_blockpath(ports);  % origPath -> 端口结构数组

	% 允许的原路径集合（过滤库内部子块的连接）
	keepSet = containers.Map('KeyType','char','ValueType','logical');
	for i = 1:height(elemTable), keepSet(char(elemTable.OrigPath{i})) = true; end

	% 库根集合：用于识别并重定向库内部连线到库根端口
	libMask = elemTable.LibraryLink ~= "";
	libRootNewPaths  = cellfun(@char, elemTable.NewPath(libMask), 'UniformOutput', false);
	libRootOrigPaths = cellfun(@char, elemTable.OrigPath(libMask), 'UniformOutput', false);

	% -------- 4A) 预分类连接（信号/物理） --------
	tasksSignal   = struct('srcOrig',{},'dstOrig',{},'srcNew',{},'dstNew',{}, ...
		'SP',{},'DP',{},'SPK',{},'SPI',{},'DPK',{},'DPI',{}, ...
		'origin',{},'parentSys',{},'srcCenter',{},'dstCenter',{});
	tasksPhysical = tasksSignal;

	for k = 1:numel(conns)
		% 读取路径与端口号
		srcOrigPath = char(getfield_or_default(conns(k), 'SourcePath', ''));
		dstOrigPath = char(getfield_or_default(conns(k), 'DestinationPath', ''));
		srcName     = char(getfield_or_default(conns(k), 'Source', ''));
		dstName     = char(getfield_or_default(conns(k), 'Destination', ''));
		SP          = getfield_or_default(conns(k), 'SourcePort', -1);
		DP          = getfield_or_default(conns(k), 'DestinationPort', -1);
		% 可选：导出的端口种类与索引（若存在则用于精确匹配）
		SPK         = char(getfield_or_default(conns(k), 'SourcePortKind', ''));
		DPK         = char(getfield_or_default(conns(k), 'DestinationPortKind', ''));
		SPI         = getfield_or_default(conns(k), 'SourcePortIndex', -1);
		DPI         = getfield_or_default(conns(k), 'DestinationPortIndex', -1);
		originStr   = char(getfield_or_default(conns(k), 'Origin', ''));

		% 路径重映射
		if ~isempty(srcOrigPath)
			srcNewPath = sanitize_path(rebase_path(srcOrigPath, origRoot, newModel));
		else
			idx = find(strcmp(elemTable.ShortName, string(srcName)), 1, 'first');
			if isempty(idx), warning('找不到源块：%s。跳过该连接 ?', srcName); continue; end
			srcNewPath = char(elemTable.NewPath(idx));
			srcOrigPath = char(elemTable.OrigPath(idx));
		end
		if ~isempty(dstOrigPath)
			dstNewPath = sanitize_path(rebase_path(dstOrigPath, origRoot, newModel));
		else
			idx = find(strcmp(elemTable.ShortName, string(dstName)), 1, 'first');
			if isempty(idx), warning('找不到目标块 ?%s。跳过该连接 ?', dstName); continue; end
			dstNewPath = char(elemTable.NewPath(idx));
			dstOrigPath = char(elemTable.OrigPath(idx));
		end

		% 判断是否处于库内部：若一端在库内部且另一端在库外，则把库内部端重定向到库根块
		[srcNewPath, srcOrigPath, srcInLib, srcLibRootNew, srcLibRootOrig] = redirect_to_lib_root_if_inside(srcNewPath, srcOrigPath, libRootNewPaths, libRootOrigPaths);
		[dstNewPath, dstOrigPath, dstInLib, dstLibRootNew, dstLibRootOrig] = redirect_to_lib_root_if_inside(dstNewPath, dstOrigPath, libRootNewPaths, libRootOrigPaths);

		% 如果两端都在同一个库内部，则跳过（库内部自连线）
		if srcInLib && dstInLib && strcmp(srcLibRootNew, dstLibRootNew)
			continue;
		end

		% 过滤库内部连接（严格依赖创建的元素集合）
		if ~(isKey(keepSet, srcOrigPath) && isKey(keepSet, dstOrigPath))
			continue; 
		end

		% 父系统
		ps = fileparts(srcNewPath); pd = fileparts(dstNewPath);
		if ~strcmp(ps, pd)
			parentSys = common_parent_of_paths(srcNewPath, dstNewPath);
		else
			parentSys = ps;
		end

		% 中心
		if isKey(centers, dstNewPath), dstCenter = centers(dstNewPath); else, dstCenter = [inf inf]; end
		if isKey(centers, srcNewPath), srcCenter = centers(srcNewPath); else, srcCenter = [inf inf]; end

		% 判定域
		[srcType, srcKnown] = get_export_port_type(portsByOrig, srcOrigPath, SP, 'src');
		[dstType, dstKnown] = get_export_port_type(portsByOrig, dstOrigPath, DP, 'dst');
		hasConsSrc = block_has_conserving_port(portsByOrig, srcOrigPath);
		hasConsDst = block_has_conserving_port(portsByOrig, dstOrigPath);
		newHasConsSrc = block_has_physical_ports_new(srcNewPath);
		newHasConsDst = block_has_physical_ports_new(dstNewPath);
		% 仅当来源为 PortConnectivity 且“源/目标两端都具备物理端子”时，判定为物理连接
		isPhysical = false;
		if strcmpi(originStr,'pc')
			physSrc = strcmpi(srcType,'conserving') || hasConsSrc || newHasConsSrc;
			physDst = strcmpi(dstType,'conserving') || hasConsDst || newHasConsDst;
			isPhysical = physSrc && physDst;
		else
			isPhysical = false;  % 普通 line 一律视为信号域
		end
		% 仅保留 PC 边且被判为物理的精简日志
		if strcmpi(originStr,'pc') && isPhysical
			fprintf('[pc] %s -> %s | phys=1 | srcT=%s dstT=%s | newSrc=%d newDst=%d\n', ...
				srcOrigPath, dstOrigPath, string(srcType), string(dstType), newHasConsSrc, newHasConsDst);
		end

		task = struct('srcOrig',srcOrigPath,'dstOrig',dstOrigPath,'srcNew',srcNewPath,'dstNew',dstNewPath, ...
			'SP',SP,'DP',DP,'SPK',SPK,'SPI',SPI,'DPK',DPK,'DPI',DPI, ...
			'origin',originStr,'parentSys',parentSys,'srcCenter',srcCenter,'dstCenter',dstCenter);
		if isPhysical
			tasksPhysical(end+1) = task; %#ok<AGROW>
		elseif strcmpi(originStr,'line')
			% 非物理且来自普通线对象：作为信号线处理
			tasksSignal(end+1) = task;   %#ok<AGROW>
		else
			% 非物理且来自 PC：跳过，避免与 line 重复
			continue;
		end
	end

	% -------- 4A.1) 利用正反向物理边互补 Kind/Index（用反向的 SPK/SPI 作为本边的 DPK/DPI） --------
	% why: 之前以“块对”为键仅保存一条记录，若同一对块存在多条物理边，会被最后一条覆盖，
	%      导致还原时所有 DPI 被错误地统一到同一个索引（常见为 3）。
	% how: 为每个块对维护一个按出现顺序的端口信息队列，并在互补时逐条消费，确保一一对应。
	pair2srcKind = containers.Map('KeyType','char','ValueType','any');
	for i = 1:numel(tasksPhysical)
		SPK_i = tasksPhysical(i).SPK; SPI_i = tasksPhysical(i).SPI;
		if ~isempty(SPK_i) && ~(isstring(SPK_i) && strlength(SPK_i)==0) && SPI_i>=1
			k = [char(tasksPhysical(i).srcNew) '->' char(tasksPhysical(i).dstNew)];
			% 初始化为结构体数组队列，避免后续被覆盖
			if ~isKey(pair2srcKind, k)
				pair2srcKind(k) = struct('k',{},'i',{});
			end
			lst = pair2srcKind(k);
			lst(end+1) = struct('k',SPK_i,'i',SPI_i); %#ok<AGROW>
			pair2srcKind(k) = lst;
		end
	end
	% 按反向键消费队列，实现一一匹配，避免 DPI 被同值覆盖
	for i = 1:numel(tasksPhysical)
		needDP = (isempty(tasksPhysical(i).DPK) || (isstring(tasksPhysical(i).DPK) && strlength(tasksPhysical(i).DPK)==0) || tasksPhysical(i).DPI<1);
		if needDP
			rk = [char(tasksPhysical(i).dstNew) '->' char(tasksPhysical(i).srcNew)];
			if isKey(pair2srcKind, rk)
				lst = pair2srcKind(rk);
				if ~isempty(lst)
					info = lst(1);
					% 互补：反向边的源端口即为本边的目标端口
					tasksPhysical(i).DPK = info.k;
					tasksPhysical(i).DPI = info.i;
					% 消费已使用项，保持与多边一一对应
					lst = lst(2:end);
					pair2srcKind(rk) = lst;
				end
			end
		end
	end

	% -------- 4A.2) 物理边去重（同一对端口仅保留一条） --------
	tasksPhysical = dedup_physical_tasks(tasksPhysical);

	% -------- 4B) 先连接“信号线” --------
	for t = 1:numel(tasksSignal)
		S = tasksSignal(t);
		try
			srcH = resolve_port_handle_by_kind_index(S.srcNew, S.SPK, S.SPI);
			dstH = resolve_port_handle_by_kind_index(S.dstNew, S.DPK, S.DPI);
			%debug_print_addline(S.parentSys, srcH, dstH, 'signal');
			if ~port_has_existing_line(dstH)
				%fprintf('[signal:add] parent=%s srcH=%d dstH=%d\n', S.parentSys, srcH, dstH);
				add_line(S.parentSys, srcH, dstH, 'autorouting','on');
			end
		catch ME
			warning('信号连线失败 ?%s(%g) -> %s(%g)。原因：%s', S.srcNew, S.SP, S.dstNew, S.DP, ME.message);
		end
	end

	% -------- 4C) 再连接“物理线” --------
	for t = 1:numel(tasksPhysical)
		P = tasksPhysical(t);
		try
			srcH = resolve_port_handle_by_kind_index(P.srcNew, P.SPK, P.SPI);
			dstH = resolve_port_handle_by_kind_index(P.dstNew, P.DPK, P.DPI);
			
			% 以句柄所在父系统求公共父系统，避免系统不匹配
			psrc = get_param(srcH,'Parent'); pdst = get_param(dstH,'Parent');
			parentUse = common_parent_of_paths(psrc, pdst);
			if port_has_existing_line(dstH)
				L = get_param(dstH,'Line');
				try, add_line(parentUse, srcH, L, 'autorouting','on'); catch, add_line(parentUse, L, srcH, 'autorouting','on'); end
			else
				add_line(parentUse, srcH, dstH, 'autorouting','on');
			end
		catch ME
			warning('物理连线失败 ?%s(%g) -> %s(%g)。原因：%s', P.srcNew, P.SP, P.dstNew, P.DP, ME.message);
		end
	end

	% -------- 5) 回放参数（模型与块） --------
	try
		apply_parameters_to_model(newModel, elemTable, params);
	catch ME
		warning('参数回放失败，将以默认参数运行：%s', ME.message);
	end

	%set_param(newModel, 'SimulationCommand', 'update');
	disp(['模型已重建：' newModel]);
end
% ============================== 辅助函数 ==============================


function h = resolve_port_handle_by_kind_index(blockPath, kind, idx)
	% 若导出的端口种类/索引存在，直接返回对应句柄；否则返回 []
	h = [];
	try
		if isempty(kind) || idx < 1
			return;
		end
		ph = get_param(blockPath,'PortHandles');
		map = struct('outport','Outport','inport','Inport','lconn','LConn','rconn','RConn','conn','Conn');
		if isfield(map, lower(kind))
			field = map.(lower(kind));
		else
			return;
		end
		if isfield(ph, field) && numel(ph.(field)) >= idx
			h = ph.(field)(idx);
		end
    catch

		h = [];
	end
end

function [newPath,outOrig,inLib,libRootNew,libRootOrig] = redirect_to_lib_root_if_inside(newPath, origPath, libRootsNew, libRootsOrig)
    % 若 newPath 落在某个库根块下，则重定向到库根并返回库标志
    inLib = false; libRootNew = ''; libRootOrig = '';
    try
        for i = 1:numel(libRootsNew)
            rootp = libRootsNew{i};
            if strncmp(newPath, [rootp '/'], length(rootp)+1)
                inLib = true; libRootNew = rootp; libRootOrig = libRootsOrig{i};
                newPath = rootp; origPath = libRootsOrig{i};
                break;
            end
        end
    catch
    end
    outOrig = origPath;
end

function tasks = dedup_physical_tasks(tasks)
	% 按 srcNew/dstNew/DPK/DPI 维度去重，若 DPK/DPI 缺失则仅用 srcNew/dstNew 键
	if isempty(tasks), return; 
    
    end
	seen = containers.Map('KeyType','char','ValueType','logical');
	keep = true(1,numel(tasks));
	for i = 1:numel(tasks)
		k = build_phys_key(tasks(i));
		if isKey(seen,k)
			keep(i) = false;
		else
			seen(k) = true;
		end
	end
	tasks = tasks(keep);
end

function k = build_phys_key(t)
	try
		dpk = char(t.DPK);
	catch
		dpk = '';

    end
     
	try



		dpi = t.DPI;
	catch
		dpi = -1;
	end
	if isempty(dpk) || dpi<1
		k = sprintf('%s=>%s', char(t.srcNew), char(t.dstNew));
	else
		k = sprintf('%s=>%s|%s#%d', char(t.srcNew), char(t.dstNew), dpk, dpi);
	end
end

function data = load_or_decode_graph(inputPath)
	% 加载 MAT  ? JSON，返回统 ?结构
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
			error('不支持的文件类型 ?%s（请提供 .mat  ? .json ?', ext);
	end
end

function v = safe_field(S, name, defaultV)
	if isstruct(S) && isfield(S, name) && ~isempty(S.(name))
		v = S.(name);
	else
		v = defaultV;
	end
end

function params = ensure_params_loaded(params, inputPath)
    % 若 params.model 为空 struct，则尝试从边车文件加载（与导出侧保持解耦）
    try
        needModel = true;
        try, needModel = isempty(fieldnames(params)) || ~isfield(params,'model') || isempty(fieldnames(params.model)); catch, needModel = true; end
        if ~needModel, return; end

        [pdir, pbase, ~] = fileparts(inputPath);
        % 查找 *_params.json（优先）
        jsonPath = fullfile(pdir, sprintf('%s_params.json', pbase));
        if exist(jsonPath, 'file')
            try
                txt = fileread(jsonPath); J = jsondecode(txt);
                if isstruct(J)
                    if ~isfield(params,'blocks') && isfield(J,'blocks'), params.blocks = J.blocks; end
                    if (~isfield(params,'model') || isempty(fieldnames(params.model))) && isfield(J,'model')
                        params.model = J.model;
                    end
                    return;
                end
            catch
            end
        end

        % 次选：model/block params CSV（仅在 json 不存在时尝试）
        mcsv = fullfile(pdir, sprintf('%s_model_params.csv', pbase));
        if exist(mcsv,'file')
            try
                Tm = readtable(mcsv);
                pm = struct();
                for i = 1:height(Tm)
                    pm.(char(Tm.Param{i})) = char(string(Tm.Value{i}));
                end
                params.model = pm;
            catch
            end
        end

        bcsv = fullfile(pdir, sprintf('%s_block_params.csv', pbase));
        if exist(bcsv,'file') && (~isfield(params,'blocks') || isempty(params.blocks))
            try
                Tb = readtable(bcsv);
                params.blocks = rebuild_blocks_from_long_table(Tb);
            catch
            end
        end
    catch
    end
end

function blocks = rebuild_blocks_from_long_table(Tb)
    blocks = struct('Path',{},'BlockType',{},'MaskType',{},'DialogParams',{});
    try
        if isempty(Tb), return; end
        % 聚合：每个 Path 聚成一条记录
        [G, keys] = findgroups(Tb.Path);
        for k = 1:max(G)
            idx = find(G==k);
            if isempty(idx), continue; end
            path = char(keys{k});
            btype = '';
            try, btype = char(Tb.BlockType{idx(1)}); catch, btype = ''; end
            dp = struct();
            for j = idx(:)'
                try
                    dp.(char(Tb.Param{j})) = char(string(Tb.Value{j}));
                catch
                end
            end
            blocks(end+1) = struct('Path',path,'BlockType',btype,'MaskType','', 'DialogParams',dp); %#ok<AGROW>
        end
    catch
    end
end
function s = def_or_empty_struct(S, field)
    % 工具：如果字段存在则返回之；否则返回空 struct
    if isstruct(S) && isfield(S, field) && ~isempty(S.(field))
        s = S.(field);
    else
        s = struct();
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
	%  ? elements.Path 推断原根 ?
	p = elements(1).Path;
	slash = find(p=='/', 1, 'first');
	if isempty(slash)
		root = p;              % 已在 ?
	else
		root = p(1:slash-1);   % 根名
	end
end

function T = preprocess_elements(elements, origRoot, newModel)
	% 生成用于创建的表格，并映射到新模型路 ?
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
    Mirror      = strings(N,1);
    Rotation    = strings(N,1);
    GotoTag     = strings(N,1);
    GotoVisibility = strings(N,1);
    FromTag     = strings(N,1);

	for i = 1:N
		OrigPath(i)   = string(elements(i).Path);
		ShortName(i)  = string(elements(i).Name);
		BlockType(i)  = string(def_str(elements(i), 'BlockType'));
		Orientation(i)= string(def_str(elements(i), 'Orientation', 'right'));
		pos = elements(i).Position; if isempty(pos), pos = [30 30 60 60]; end
		Left(i)=pos(1); Top(i)=pos(2); Right(i)=pos(3); Bottom(i)=pos(4);
        LibraryLink(i) = string(def_str(elements(i), 'LibraryLink'));
        Mirror(i)      = string(def_str(elements(i), 'Mirror'));
        Rotation(i)    = string(def_str(elements(i), 'Rotation'));
        GotoTag(i)     = string(def_str(elements(i), 'GotoTag'));
        GotoVisibility(i) = string(def_str(elements(i), 'GotoVisibility'));
        FromTag(i)     = string(def_str(elements(i), 'FromTag'));

		np = sanitize_path(rebase_path(char(OrigPath(i)), origRoot, newModel));
		pp = sanitize_path(fileparts(np));
		NewPath(i)    = string(np);
		ParentPath(i) = string(pp);
		Depth(i)      = count(NewPath(i), "/");
	end

    T = table(ShortName, OrigPath, NewPath, ParentPath, BlockType, Orientation, ...
              Left, Top, Right, Bottom, LibraryLink, Mirror, Rotation, GotoTag, GotoVisibility, FromTag, Depth);
end

% ========================= 参数回放相关函数 =========================
function apply_parameters_to_model(newModel, elemTable, params)
    % why: 复原模型求解器/时间步等全局参数，以及各块的对话参数，使“直接仿真”成为可能
    % how: 仅在导出时存在参数时才尝试回放；set_param 全部使用字符串值

    if nargin < 3 || isempty(params), return; end
    try
        if isfield(params,'model') && ~isempty(params.blocks)
            apply_model_params(newModel, params.blocks);
        end
    catch
    end

    try
        if isfield(params,'blocks') && ~isempty(params.blocks)
            apply_block_params(elemTable, params.blocks);
        end
    catch
    end
end

function apply_model_params(newModel, modelParams)
    fns = fieldnames(modelParams);
    for i = 1:numel(fns)
        pname = fns{i};
        try
            set_param(newModel, pname, char(modelParams.(pname)));
        catch
            % 某些版本/产品未安装时参数不可用，忽略
        end
    end
end

function apply_block_params(elemTable, blockParams)
    % 构建“原路径->新路径”查找，便于使用导出 Path 进行回放
    mapOrigToNew = containers.Map('KeyType','char','ValueType','char');
    for i = 1:height(elemTable)
        mapOrigToNew(char(elemTable.OrigPath{i})) = char(elemTable.NewPath{i});
    end

    for i = 1:numel(blockParams)
        try
            bp = blockParams(i);
            origPath = char(bp.Path);
            if isKey(mapOrigToNew, origPath)
                newPath = mapOrigToNew(origPath);
            else
                % 回退：用短名匹配首个同名块
                idx = find(strcmp(elemTable.ShortName, string(get_basename(origPath))), 1, 'first');
                if isempty(idx), continue; end
                newPath = char(elemTable.NewPath(idx));
            end

            % 逐参数 set_param
            if isfield(bp,'DialogParams') && ~isempty(bp.DialogParams)
                names = fieldnames(bp.DialogParams);
                for k = 1:numel(names)
                    pname = names{k};
                    % 跳过几何类参数
                    if any(strcmpi(pname, {'Position','PortConnectivity','LineHandles'}))
                        continue;
                    end
                    pval  = bp.DialogParams.(pname);
                    try
                        set_param(newPath, pname, char(pval));
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

%=========================参数回放函数结束=========================

function s = def_str(st, field, defaultV)
	if nargin < 3, defaultV = ""; end
	if isfield(st, field) && ~isempty(st.(field))
		s = st.(field);
	else
		s = defaultV;
	end
end

function newPath = rebase_path(origPath, origRoot, newRoot)
	% 将原路径替换为新模型根路径；无法精确映射时取 ?后一级名挂到新根
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
	% 剔除库引用块内部子块（例 ? ".../DC Voltage Source/Model" ?
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
	% 去掉尾部斜杠，保 ? Simulink 合法路径
	p = char(string(p));
	while ~isempty(p) && p(end) == '/', p = p(1:end-1); end
end

function ensure_system_exists(sysPath)
	% 逐级创建父系 ?
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
                        
                    %otherwise
						%add_block('simulink/Ports & Subsystems/Subsystem', newPath, 'MakeNameUnique','off');
						%warning(' ? %s 无库引用，使 ? Subsystem 占位 ?', newPath);
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


function portsIdx = index_ports_by_blockpath(ports)
	% 将导出的 ports  ? BlockPath（原路径）分 ?
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

function tf = block_has_conserving_port(portsIdx, origPath)
	% 判断该原路径的端口是否包含物理保守端口（Side/Type 提示）
	tf = false;
	try
		if isKey(portsIdx, origPath)
			P = portsIdx(origPath);
			if isstruct(P)
				types = {P.PortType};
				% 物理端口判定：Simscape conserving 或 Specialized Power Systems 的 LConn/RConn/Conn
				tf = any(strcmpi(types,'conserving')) || any(strcmpi(types,'LConn')) || any(strcmpi(types,'RConn')) || any(strcmpi(types,'Conn'));
			else
				for i = 1:numel(P)
					typ = '';
					try, typ = P(i).PortType; end
					if strcmpi(typ,'conserving') || strcmpi(typ,'LConn') || strcmpi(typ,'RConn') || strcmpi(typ,'Conn')
						tf = true; break; 
					end
				end
			end
		end
	catch
		tf = false;
	end
end

function tf = port_has_existing_line(portHandle)
	% 检查端口是否已有连接（避免重复 add_line）
	tf = false;
	try
		l = get_param(portHandle,'Line');
		if l ~= -1
			% 目标或源端口已有线
			tf = true;
		end
	catch
		tf = false;
	end
end

function tf = block_has_physical_ports_new(blockPath)
	% 直接在“新模型的块”上探测是否存在物理端口（LConn/RConn/Conn）
	tf = false;
	try
		ph = get_param(blockPath,'PortHandles');
		if (isfield(ph,'LConn') && ~isempty(ph.LConn)) || (isfield(ph,'RConn') && ~isempty(ph.RConn)) || (isfield(ph,'Conn') && ~isempty(ph.Conn))
			tf = true;
		end
	catch
		tf = false;
	end
end

function [ptype, known] = get_export_port_type(portsIdx, origPath, pnum, role)
	% 从导出信息推断端口类型（inport/outport/conserving/port），若可识别返回 known=true
	ptype = '';

    
	known = false;
	try
		if isKey(portsIdx, origPath)
			P = portsIdx(origPath);
			% 优先使用端口号匹配一条，如果无端口号就看集合里是否有明确的类型
			candidate = [];
			if isfinite(pnum) && pnum>=1 && pnum<=numel(P)
				candidate = P(pnum);
			end
			if ~isempty(candidate)
				if isfield(candidate,'PortType') && ~isempty(candidate.PortType)
					ptype = candidate.PortType; known = true; return; 
				end
			end
			% 汇总端口类型
			types = {};
			for i = 1:numel(P)
				try, types{end+1} = P(i).PortType; catch, end %#ok<AGROW>
			end
            % 物理端子：conserving 或 LConn/RConn/Conn
            if any(strcmpi(types,'conserving')) || any(strcmpi(types,'LConn')) || any(strcmpi(types,'RConn')) || any(strcmpi(types,'Conn'))
                ptype = 'conserving'; known = true; return;
            end
            if strcmpi(role,'src') && any(strcmpi(types,'outport'))
				ptype = 'outport'; known = true; return;
			end
            if strcmpi(role,'dst') && any(strcmpi(types,'inport'))
				ptype = 'inport'; known = true; return;
			end
		end
	catch
		ptype = '';
		known = false;
	end
end


function val = getfield_or_default(S, field, def)
	if isfield(S, field) && ~isempty(S.(field))
		val = S.(field);
	else
		val = def;
	end
end
newModel = rebuild_model_from_export(fullfile(pwd,'export_model_graph','model_graph.mat'), 'recovered_model');