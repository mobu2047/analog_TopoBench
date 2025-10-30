newModel = rebuild_model_from_export(fullfile(pwd,'export_model_graph','model_graph.mat'), 'recovered_model', 1);
function newModel = rebuild_model_from_export(inputPath, newModelName, useCsv)
% 复建 Simulink 模型的主流程（支持信号线与物理线）
% - 通过导出的绝对路径 SourcePath/DestinationPath 精确定位块
% - 当端口号缺失(-1/NaN)时，优先用端口种类/索引进行匹配
% - 对库块内部连接进行规避处理，连线时尽量开启 autorouting

	% -------- 1) 读取图数据（CSV/MAT，默认 CSV） --------
	% why: 默认用导出目录下的 CSV；与 MAT/JSON 的结构保持一致
	% how: 入参 useCsv 可为 true/false 或 'csv'/'mat'，CSV 缺失时回退 MAT/JSON
	if nargin < 1 || isempty(inputPath)
		% 指向导出目录下的 model_graph.mat；同目录存在 CSV 时优先读 CSV
		inputPath = fullfile(pwd,'export_model_graph','model_graph.mat');
	end
	if nargin < 2 || isempty(newModelName)
		newModelName = 'recovered_model';
	end
	if nargin < 3 || isempty(useCsv)
		useCsv = true;  % 默认按 CSV 读取
	end
	if isstring(useCsv) || ischar(useCsv)
		sw = lower(char(useCsv));
		if strcmp(sw,'csv')
			useCsv = true;
		elseif any(strcmp(sw,{'mat','json'}))
			useCsv = false;
		else
			try, useCsv = logical(str2double(sw)); catch, useCsv = true; end
		end
	end

	try
		if useCsv
			data = load_graph_from_csv_dir(inputPath);  % 读取同目录 CSV -> 统一结构
		else
			data = load_or_decode_graph(inputPath);     % 从 MAT/JSON 读取
		end
	except ME
		% CSV 读取失败，自动回退到 MAT/JSON
		warning('CSV 读取失败，回退 MAT/JSON：%s', ME.message);
		data = load_or_decode_graph(inputPath);
	end

	% 兜底与默认结构，填充空表结构
	elements = safe_field(data, 'elements', struct('Path',{},'Name',{},'BlockType',{},'Orientation',{},'Position',{},'Center',{},'LibraryLink',{},'Mirror',{},'Rotation',{},'GotoTag',{},'GotoVisibility',{},'FromTag',{}));
	ports    = safe_field(data, 'ports',    struct('BlockPath',{},'PortNumber',{},'PortType',{},'Position',{},'RelPos',{},'Side',{}));
	conns    = data.connections;  % 基于 SourcePath/DestinationPath
	params   = safe_field(data, 'parameters', struct());  % 可选模型/块参数
	assert(~isempty(elements), 'elements 为空，无法复建模型');
    

    
    
	% 从 elements 推断共同根（路径首段），用于路径重映射
	origRoot = get_root_from_elements(elements);
	newModel = char(newModelName);

	% -------- 2) 创建空模型 --------
	if bdIsLoaded(newModel)
		close_system(newModel, 0);
	end
	new_system(newModel); open_system(newModel);
    load_system('simulink');

	% -------- 3) 预处理 elements 并创建块 --------
	elemTable = preprocess_elements(elements, origRoot, newModel);              % 计算新旧路径映射
	elemTable = filter_descendants_of_library_blocks(elemTable);                % 过滤库内部子层级

	% 先创建 SubSystem
	subRows = elemTable(strcmp(elemTable.BlockType, 'SubSystem'), :);
	subRows = sortrows(subRows, 'Depth');
	for i = 1:height(subRows), create_block_in_model(subRows(i,:), true); end

	% 再创建普通块
	blkRows = elemTable(~strcmp(elemTable.BlockType, 'SubSystem'), :);
	blkRows = sortrows(blkRows, 'Depth');
	for i = 1:height(blkRows), create_block_in_model(blkRows(i,:), false); end

	% -------- 4) 基于导出连接关系生成连线 --------
	% 原路径->新路径；定位父系统->选择合适父系统；端口类型与索引匹配
	orig2new = containers.Map('KeyType','char','ValueType','char');
	centers  = containers.Map('KeyType','char','ValueType','any');
	for i = 1:height(elemTable)
		orig2new(char(elemTable.OrigPath{i})) = char(elemTable.NewPath{i});
		cx = (elemTable.Left(i)+elemTable.Right(i))/2;
		cy = (elemTable.Top(i)+elemTable.Bottom(i))/2;
		centers(char(elemTable.NewPath{i})) = [cx, cy];
	end
	portsByOrig = index_ports_by_blockpath(ports);  % origPath -> 端口结构索引

	% 保留需要参与连线的原路径集合
	keepSet = containers.Map('KeyType','char','ValueType','logical');
	for i = 1:height(elemTable), keepSet(char(elemTable.OrigPath{i})) = true; end

	% 标记库块根路径，连线时用于库内重定向处理
	libMask = elemTable.LibraryLink ~= "";
	libRootNewPaths  = cellfun(@char, elemTable.NewPath(libMask), 'UniformOutput', false);
	libRootOrigPaths = cellfun(@char, elemTable.OrigPath(libMask), 'UniformOutput', false);

	% -------- 4A) 生成候选任务并分类（信号/物理） --------
	tasksSignal   = struct('srcOrig',{},'dstOrig',{},'srcNew',{},'dstNew',{}, ...
		'SP',{},'DP',{},'SPK',{},'SPI',{},'DPK',{},'DPI',{}, ...
		'origin',{},'parentSys',{},'srcCenter',{},'dstCenter',{});
	tasksPhysical = tasksSignal;

	for k = 1:numel(conns)
		% 读取路径与端口索引
		srcOrigPath = char(getfield_or_default(conns(k), 'SourcePath', ''));
		dstOrigPath = char(getfield_or_default(conns(k), 'DestinationPath', ''));
		srcName     = char(getfield_or_default(conns(k), 'Source', ''));
		dstName     = char(getfield_or_default(conns(k), 'Destination', ''));
		SP          = getfield_or_default(conns(k), 'SourcePort', -1);
		DP          = getfield_or_default(conns(k), 'DestinationPort', -1);
		% 可选端口种类/索引信息（用于更精确匹配）
		SPK         = char(getfield_or_default(conns(k), 'SourcePortKind', ''));
		DPK         = char(getfield_or_default(conns(k), 'DestinationPortKind', ''));
		SPI         = getfield_or_default(conns(k), 'SourcePortIndex', -1);
		DPI         = getfield_or_default(conns(k), 'DestinationPortIndex', -1);
		originStr   = char(getfield_or_default(conns(k), 'Origin', ''));

		% 路径映射
		if ~isempty(srcOrigPath)
			srcNewPath = sanitize_path(rebase_path(srcOrigPath, origRoot, newModel));
		else
			idx = find(strcmp(elemTable.ShortName, string(srcName)), 1, 'first');
			if isempty(idx), warning('未找到源块：%s，跳过该连线条目', srcName); continue; end
			srcNewPath = char(elemTable.NewPath(idx));
			srcOrigPath = char(elemTable.OrigPath(idx));
		end
		if ~isempty(dstOrigPath)
			dstNewPath = sanitize_path(rebase_path(dstOrigPath, origRoot, newModel));
		else
			idx = find(strcmp(elemTable.ShortName, string(dstName)), 1, 'first');
			if isempty(idx), warning('未找到目标块：%s，跳过该连线条目', dstName); continue; end
			dstNewPath = char(elemTable.NewPath(idx));
			dstOrigPath = char(elemTable.OrigPath(idx));
		end

		% 若在库块内部，重定向到各自库块根，避免在库内部拉线
		[srcNewPath, srcOrigPath, srcInLib, srcLibRootNew, srcLibRootOrig] = redirect_to_lib_root_if_inside(srcNewPath, srcOrigPath, libRootNewPaths, libRootOrigPaths);
		[dstNewPath, dstOrigPath, dstInLib, dstLibRootNew, dstLibRootOrig] = redirect_to_lib_root_if_inside(dstNewPath, dstOrigPath, libRootNewPaths, libRootOrigPaths);

		% 若源与目标同属同一库块根，则忽略该库内连接
		if srcInLib && dstInLib && strcmp(srcLibRootNew, dstLibRootNew)
			continue;
		end

		% 仅对需要参与连线的元素进行处理
		if ~(isKey(keepSet, srcOrigPath) && isKey(keepSet, dstOrigPath))
			continue; 
		end

		% 选择父系统
		ps = fileparts(srcNewPath); pd = fileparts(dstNewPath);
		if ~strcmp(ps, pd)
			parentSys = common_parent_of_paths(srcNewPath, dstNewPath);
		else
			parentSys = ps;
		end

		% 端点几何中心（可用于后续布线策略）
		if isKey(centers, dstNewPath), dstCenter = centers(dstNewPath); else, dstCenter = [inf inf]; end
		if isKey(centers, srcNewPath), srcCenter = centers(srcNewPath); else, srcCenter = [inf inf]; end

		% 判别端口类型
		[srcType, srcKnown] = get_export_port_type(portsByOrig, srcOrigPath, SP, 'src');
		[dstType, dstKnown] = get_export_port_type(portsByOrig, dstOrigPath, DP, 'dst');
		hasConsSrc = block_has_conserving_port(portsByOrig, srcOrigPath);
		hasConsDst = block_has_conserving_port(portsByOrig, dstOrigPath);
		newHasConsSrc = block_has_physical_ports_new(srcNewPath);
		newHasConsDst = block_has_physical_ports_new(dstNewPath);
		% 若来源为 PortConnectivity，且两端均为物理端口，则认为是物理线
		isPhysical = false;
		if strcmpi(originStr,'pc')
			physSrc = strcmpi(srcType,'conserving') || hasConsSrc || newHasConsSrc;
			physDst = strcmpi(dstType,'conserving') || hasConsDst || newHasConsDst;
			isPhysical = physSrc && physDst;
		else
			isPhysical = false;  % 普通 line 视为信号线
		end
		% 打印物理连线调试信息（可选）
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
			% 普通导出连线作为信号线处理
			tasksSignal(end+1) = task;   %#ok<AGROW>
		else
			% ???????????? PC?????????????? line ???
			continue;
		end
	end

	% -------- 4A.1) 物理连线补全目标端口的 Kind/Index --------
	% why: 经常只有源端带 Kind/Index，目标端缺失，需要在配对连线中补齐
	% how: 以“dst->src”为键保存源端 Kind/Index 列表，逐条匹配并消费
	pair2srcKind = containers.Map('KeyType','char','ValueType','any');
	for i = 1:numel(tasksPhysical)
		SPK_i = tasksPhysical(i).SPK; SPI_i = tasksPhysical(i).SPI;
		if ~isempty(SPK_i) && ~(isstring(SPK_i) && strlength(SPK_i)==0) && SPI_i>=1
			k = [char(tasksPhysical(i).srcNew) '->' char(tasksPhysical(i).dstNew)];
				% 初始化为结构体数组，随后依次追加记录
			if ~isKey(pair2srcKind, k)
				pair2srcKind(k) = struct('k',{},'i',{});
			end
			lst = pair2srcKind(k);
			lst(end+1) = struct('k',SPK_i,'i',SPI_i); %#ok<AGROW>
			pair2srcKind(k) = lst;
		end
	end
	% 反向匹配：为缺失 DPK/DPI 的目标端补全 Kind/Index
	for i = 1:numel(tasksPhysical)
		needDP = (isempty(tasksPhysical(i).DPK) || (isstring(tasksPhysical(i).DPK) && strlength(tasksPhysical(i).DPK)==0) || tasksPhysical(i).DPI<1);
		if needDP
			rk = [char(tasksPhysical(i).dstNew) '->' char(tasksPhysical(i).srcNew)];
			if isKey(pair2srcKind, rk)
				lst = pair2srcKind(rk);
				if ~isempty(lst)
					info = lst(1);
					% 为目标端补全端口种类/索引（使用配对源端的首条记录）
					tasksPhysical(i).DPK = info.k;
					tasksPhysical(i).DPI = info.i;
					% 消费已使用的首条记录
					lst = lst(2:end);
					pair2srcKind(rk) = lst;
				end
			end
		end
	end

	% -------- 4A.2) 物理连线去重 --------
	tasksPhysical = dedup_physical_tasks(tasksPhysical);

	% -------- 4B) 添加信号线 --------
	for t = 1:numel(tasksSignal)
		S = tasksSignal(t);
		try
			srcH = resolve_port_handle_by_kind_index(S.srcNew, S.SPK, S.SPI);
			dstH = resolve_port_handle_by_kind_index(S.dstNew, S.DPK, S.DPI);
			% debug_print_addline(S.parentSys, srcH, dstH, 'signal');
			if ~port_has_existing_line(dstH)
				%fprintf('[signal:add] parent=%s srcH=%d dstH=%d\n', S.parentSys, srcH, dstH);
				add_line(S.parentSys, srcH, dstH, 'autorouting','on');
			end
		catch ME
			warning('添加信号线失败：%s(%g) -> %s(%g)，原因：%s', S.srcNew, S.SP, S.dstNew, S.DP, ME.message);
		end
	end

	% -------- 4C) 添加物理线 --------
	for t = 1:numel(tasksPhysical)
		P = tasksPhysical(t);
		try
			srcH = resolve_port_handle_by_kind_index(P.srcNew, P.SPK, P.SPI);
			dstH = resolve_port_handle_by_kind_index(P.dstNew, P.DPK, P.DPI);
			
			% 在较高层父系统处布线，避免跨系统 add_line 失败
			psrc = get_param(srcH,'Parent'); pdst = get_param(dstH,'Parent');
			parentUse = common_parent_of_paths(psrc, pdst);
			if port_has_existing_line(dstH)
				L = get_param(dstH,'Line');
				try, add_line(parentUse, srcH, L, 'autorouting','on'); catch, add_line(parentUse, L, srcH, 'autorouting','on'); end
			else
				add_line(parentUse, srcH, dstH, 'autorouting','on');
			end
		catch ME
			warning('添加物理线失败：%s(%g) -> %s(%g)，原因：%s', P.srcNew, P.SP, P.dstNew, P.DP, ME.message);
		end
	end

	% -------- 5) 回填参数并更新模型 --------
	try
		apply_parameters_to_model(newModel, elemTable, params);
	catch ME
		warning('回填参数失败，按默认/已有配置继续：%s', ME.message);
	end

	set_param(newModel, 'SimulationCommand', 'update');
	disp(['模型复建完成：' newModel]);
    save_system(newModel)
end



function h = resolve_port_handle_by_kind_index(blockPath, kind, idx)
	% 根据端口种类(kind)与索引(idx)解析端口句柄；失败返回 []
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
    % 若 newPath 位于某库块内部，则重定向到库块根，避免跨库内部连线
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
	% 按 srcNew/dstNew/DPK/DPI 维度去重；若 DPK/DPI 缺失则按 srcNew/dstNew 去重
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
	% 从 MAT 或 JSON 文件读取图数据，统一为 data 结构
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
			error('不支持的文件扩展名：%s（仅支持 .mat 或 .json）', ext);
	end
end

function data = load_graph_from_csv_dir(inputPath)
	% why: 从目录读取 model_elements/ports/connections 三个 CSV
	%      并封装为与 MAT 读取一致的 data 结构，便于统一处理
	% how: 若传入 .mat 路径，则使用同目录；否则视为目录路径
	[dirp,~,ext] = fileparts(inputPath);
	if isempty(dirp) || (~isempty(ext) && ~strcmpi(ext,'.mat') && ~strcmpi(ext,'.json'))
		% 传入的是目录（或非常规扩展名），按目录处理
		dirp = inputPath;
	end
	if isempty(dirp)
		dirp = pwd;
	end

	% 目标文件位于同一目录
	fe = fullfile(dirp,'model_elements.csv');
	fp = fullfile(dirp,'model_ports.csv');
	fc = fullfile(dirp,'model_connections.csv');
	fj = fullfile(dirp,'model_params.json');

	% 读取 CSV
	assert(exist(fe,'file')==2 && exist(fp,'file')==2 && exist(fc,'file')==2, '缺少 CSV 文件：需包含 elements/ports/connections');
	Te = readcell(fe, 'TextType','string');
	Tp = readcell(fp, 'TextType','string');
	Tc = readcell(fc, 'TextType','string');
    varNames = cellfun(@char, Te(1, :), 'UniformOutput', false);
    Te = cell2table(Te(2:end, :), 'VariableNames', varNames);
    varNames = cellfun(@char, Tp(1, :), 'UniformOutput', false);
    Tp = cell2table(Tp(2:end, :), 'VariableNames', varNames);
    varNames = cellfun(@char, Tc(1, :), 'UniformOutput', false);
    Tc = cell2table(Tc(2:end, :), 'VariableNames', varNames);
	% elements: struct 结构体数组
    
elements = table_elements_to_structs(Te);
% ports: 端口表转结构体
ports    = table_ports_to_structs(Tp);
% connections: 连接表转结构体
connections = table_conns_to_structs(Tc);

	% 读取可选参数 JSON（存在则解析）
	params = struct();
	try
		if exist(fj,'file')==2
			txt = fileread(fj);
			params = jsondecode(txt);
		end
	catch
		params = struct();
	end

	data = struct('elements',elements,'ports',ports,'connections',connections,'parameters',params);
end

function elements = table_elements_to_structs(T)
	% 将 elements 表转换为结构体数组，字段与 MAT 导出格式一致
	n = height(T); elements = repmat(struct('Path','', 'Name','', 'BlockType','', 'Orientation','', 'Position',[], 'Center',[], 'LibraryLink','', 'Mirror','', 'Rotation','', 'GotoTag','', 'GotoVisibility','', 'FromTag',''), 0, 1);
	for i = 1:n
		try
			pos = [num_or(T.Position_1(i),30) num_or(T.Position_2(i),30) num_or(T.Position_3(i),60) num_or(T.Position_4(i),60)];
			ctr = [num_or(T.Center_1(i), (pos(1)+pos(3))/2) num_or(T.Center_2(i), (pos(2)+pos(4))/2)];
			E = struct();
			E.Path          = char(T.Path(i));
			E.Name          = char(T.Name(i));
			E.BlockType     = char(T.BlockType(i));
			E.Orientation   = char(T.Orientation(i));
			E.Position      = pos;
			E.Center        = ctr;
			E.LibraryLink   = char(def_str_in_table(T,'LibraryLink',i,""));
			E.Mirror        = char(def_str_in_table(T,'Mirror',i,""));
			E.Rotation      = char(num_or_str(T,'Rotation',i,'0'));
			E.GotoTag       = char(def_str_in_table(T,'GotoTag',i,""));
			E.GotoVisibility= char(def_str_in_table(T,'GotoVisibility',i,""));
			E.FromTag       = char(def_str_in_table(T,'FromTag',i,""));
			elements(end+1) = E; %#ok<AGROW>
		catch
		end
	end
end

function s = num_or_str(T, col, i, def)
	% 将表格单元安全转换为字符串：支持数值/字符串/缺失，统一输出字符串
	if nargin < 4, def = '0'; end
	try
		if any(strcmp(T.Properties.VariableNames, col))
			val = T.(col)(i);
			if ismissing(val)
				s = def; return;
			end
			if isstring(val)
				if strlength(val)==0
					s = def; else, s = char(val); end
				return;
			end
			try
				nv = num_or(val, NaN);
				if isnan(nv)
					s = def;
				else
					s = char(string(nv));
				end
			catch
				s = def;
			end
			return;
		end
	catch
	end
	s = def;
end

function ports = table_ports_to_structs(T)
	% 将 ports 表转换为结构体数组，包含端口类型、位置与相对位置
	n = height(T); ports = repmat(struct('BlockPath','', 'PortNumber',[], 'PortType','', 'Position',[], 'RelPos',[], 'Side',''), 0, 1);
	for i = 1:n
		try
			pos   = [num_or(T.Position_1(i),NaN) num_or(T.Position_2(i),NaN)];
			rel   = [num_or(T.RelPos_1(i),NaN) num_or(T.RelPos_2(i),NaN)];
			pnum  = num_or(T.PortNumber(i), NaN);
			P = struct();
			P.BlockPath = char(T.BlockPath(i));
			P.PortNumber= pnum;
			P.PortType  = char(T.PortType(i));
			P.Position  = pos;
			P.RelPos    = rel;
			P.Side      = char(def_str_in_table(T,'Side',i,""));
			ports(end+1) = P; %#ok<AGROW>
		catch
		end
	end
end

function conns = table_conns_to_structs(T)
	% 将 connections 表转换为结构体数组，含端口编号/种类/索引与来源
	n = height(T); conns = repmat(struct('Source','', 'SourcePath','', 'SourcePort',-1, 'SourcePortKind','', 'SourcePortIndex',-1, 'Destination','', 'DestinationPath','', 'DestinationPort',-1, 'DestinationPortKind','', 'DestinationPortIndex',-1, 'Origin',''), 0, 1);
	for i = 1:n
		try
			C = struct();
			C.Source                    = char(T.Source(i));
			C.SourcePath                = char(T.SourcePath(i));
			C.SourcePort                = num_or(T.SourcePort(i), -1);
			C.SourcePortKind            = char(def_str_in_table(T,'SourcePortKind',i,""));
			C.SourcePortIndex           = num_or(def_num_in_table(T,'SourcePortIndex',i), -1);
			C.Destination               = char(T.Destination(i));
			C.DestinationPath           = char(T.DestinationPath(i));
			C.DestinationPort           = num_or(T.DestinationPort(i), -1);
			C.DestinationPortKind       = char(def_str_in_table(T,'DestinationPortKind',i,""));
			C.DestinationPortIndex      = num_or(def_num_in_table(T,'DestinationPortIndex',i), -1);
			C.Origin                    = char(def_str_in_table(T,'Origin',i,""));
			conns(end+1) = C; %#ok<AGROW>
		catch
		end
	end
end

function v = num_or(x, def)
	% 宽容转换：将字符串/缺失转为数值；失败返回默认值
	try
		if ismissing(x) || (isstring(x) && strlength(x)==0)
			v = def; return;
		end
		vx = double(x);
		if ~isnan(vx)
			v = vx; return;
		end
	catch
	end
	try, v = str2double(x); catch, v = def; end
	if isnan(v), v = def; end
end

function s = def_str_in_table(T, col, i, def)
	% 从表格按列名/行号取字符串；缺失或列不存在时返回默认值
	if nargin < 4, def = ""; end
	try
		if any(strcmp(T.Properties.VariableNames, col))
			s = T.(col)(i);
			if ismissing(s) || (isstring(s) && strlength(s)==0)
				s = def;
			end
			return;
		end
	catch
	end
	s = def;
end

function v = def_num_in_table(T, col, i)
	% 从表格按列名/行号取数值；缺失或列不存在返回 NaN
	try
		if any(strcmp(T.Properties.VariableNames, col))
			v = num_or(T.(col)(i), NaN);
			return;
		end
	catch
	end
	v = NaN;
end

function v = safe_field(S, name, defaultV)
	if isstruct(S) && isfield(S, name) && ~isempty(S.(name))
		v = S.(name);
	else
		v = defaultV;
	end
end

function blocks = rebuild_blocks_from_long_table(Tb)
    % 将“长表”形式的块参数聚合成 Path 维度的结构体（可选使用）
    blocks = struct('Path',{},'BlockType',{},'MaskType',{},'DialogParams',{});
    try
        if isempty(Tb), return; end
        % 聚合：每个 Path 归并为一条记录
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


function root = get_root_from_elements(elements)
	% 计算 elements.Path 的共同根（首段）；若不存在统一根，返回空串
	try
		if isempty(elements)
			root = '';
			return;
		end
		firstTokens = strings(numel(elements),1);
		for i = 1:numel(elements)
			p = string(elements(i).Path);
			parts = split(p, '/');
			if isempty(parts)
				firstTokens(i) = p; %#ok<*AGROW>
			else
				firstTokens(i) = parts(1);
			end
		end
		uniq = unique(firstTokens);
		if numel(uniq) == 1
			root = char(uniq(1));
		else
			root = '';
		end
	catch
		root = '';
	end
end

function T = preprocess_elements(elements, origRoot, newModel)
	% 预处理 elements：生成新旧路径、父路径、深度、朝向与位置等表
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

% ========================= 回填模型/块参数 =========================
function apply_parameters_to_model(newModel, elemTable, params)
    % why: 回填模型/块参数，尽量还原原模型配置
    % how: 运行期避免对动态属性回填；set_param 统一使用字符串值

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
    % 设置模型级参数（版本/产品缺失容错）
    fns = fieldnames(modelParams);
    for i = 1:numel(fns)
        pname = fns{i};
        try
            set_param(newModel, pname, char(modelParams.(pname)));
		catch
			% 不同版本/未安装相关产品时可能失败，忽略即可
        end
    end
end

function apply_block_params(elemTable, blockParams)
    % 按 OrigPath->NewPath 映射定位块，过滤不应回填的动态属性
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
				% 模糊匹配：按块短名回退（同名取第一条）
                idx = find(strcmp(elemTable.ShortName, string(get_basename(origPath))), 1, 'first');
                if isempty(idx), continue; end
                newPath = char(elemTable.NewPath(idx));
            end

			% ????????? set_param??????A?????????????????????????????????????????
			if isfield(bp,'DialogParams') && ~isempty(bp.DialogParams)
				names = fieldnames(bp.DialogParams);
				nvPairs = {};
				for k = 1:numel(names)
					pname = names{k};
					% 排除不应回填的动态属性
					if any(strcmpi(pname, {'Position','PortConnectivity','LineHandles'}))
						continue;
					end
					pval  = bp.DialogParams.(pname);
					% 非字符串用 mat2str 序列化，确保 set_param 可接受
					if ~(ischar(pval) || isstring(pval))
						pval = mat2str(pval);
					end
					nvPairs(end+1:end+2) = {pname, char(pval)}; %#ok<AGROW>
				end
				if ~isempty(nvPairs)
					try
						set_param(newPath, nvPairs{:});
					catch
						% 设置参数失败，忽略即可
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

%=========================安全获取默认值=========================

function s = def_str(st, field, defaultV)
	if nargin < 3, defaultV = ""; end
	if isfield(st, field) && ~isempty(st.(field))
		s = st.(field);
	else
		s = defaultV;
	end
end

function newPath = rebase_path(origPath, origRoot, newRoot)
	% 将原路径重定址到新模型根下：
	% - 有共同根：替换前缀 origRoot -> newRoot
	% - 无共同根：整体挂到 newRoot 下，保留原有层级
	origPath = char(origPath);
	origRoot = char(origRoot);
	newRoot  = char(newRoot);
	if isempty(origRoot)
		% 无统一根：整体挂接到新根
		if isempty(origPath)
			newPath = newRoot;
		else
			newPath = [newRoot '/' origPath];
		end
		return;
	end
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
	% 过滤库块的内部层级（如 ".../DC Voltage Source/Model"），仅保留库块根
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
	% 清理路径字符串，去除尾部斜杠，保证 Simulink 合法路径
	p = char(string(p));
	while ~isempty(p) && p(end) == '/', p = p(1:end-1); end
end

function ensure_system_exists(sysPath)
	% 逐级确保父系统存在；不存在则创建 Subsystem 占位
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
	% 在模型中创建块：按来源库/类型、位置、朝向与标签设置
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
					%warning('占位：%s 无合适库，使用 Subsystem 占位', newPath);
				end
			end
		end
        set_param(newPath, 'Position', pos, 'Orientation', ori);
        % 镜像/旋转
        if ~isempty(mir)
            try, set_param(newPath,'BlockMirror',mir); end
        end
        if ~isempty(rot)
            try, set_param(newPath,'BlockRotation',rot); end
        end
        % 回填 Goto/From 标签
        if strcmpi(btype,'Goto')
            if ~isempty(gtag), try, set_param(newPath,'GotoTag',gtag); end, end
            if ~isempty(gvis), try, set_param(newPath,'TagVisibility',gvis); end, end
        elseif strcmpi(btype,'From') && ~isempty(ftag)
            try, set_param(newPath,'GotoTag',ftag); end
        end
		catch ME
			warning('创建块失败：%s，原因：%s', newPath, ME.message);
	end
end


function portsIdx = index_ports_by_blockpath(ports)
	% 按 BlockPath 聚合端口，便于后续按原路径查询端口信息
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
	% 判断原路径对应块是否包含物理端口（conserving/LConn/RConn/Conn）
	tf = false;
	try
		if isKey(portsIdx, origPath)
			P = portsIdx(origPath);
			if isstruct(P)
				types = {P.PortType};
				% 含有 Simscape conserving 或 Specialized Power Systems 的 LConn/RConn/Conn 端口
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
	% 判断端口是否已存在连线，避免重复 add_line
	tf = false;
	try
		l = get_param(portHandle,'Line');
		if l ~= -1
			% 目标端口已有连线
			tf = true;
		end
	catch
		tf = false;
	end
end

function tf = block_has_physical_ports_new(blockPath)
	% 在新建模型中检测块是否具备物理端口（LConn/RConn/Conn）
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
	% 从导出 ports 推断端口类型（inport/outport/conserving），known=true 表示确定
	ptype = '';

    
	known = false;
	try
		if isKey(portsIdx, origPath)
			P = portsIdx(origPath);
			% 若 pnum 合法则优先按编号取候选端口
			candidate = [];
			if isfinite(pnum) && pnum>=1 && pnum<=numel(P)
				candidate = P(pnum);
			end
			if ~isempty(candidate)
				if isfield(candidate,'PortType') && ~isempty(candidate.PortType)
					ptype = candidate.PortType; known = true; return; 
				end
			end
			% 统计所有端口类型
			types = {};
			for i = 1:numel(P)
				try, types{end+1} = P(i).PortType; catch, end %#ok<AGROW>
			end
			% 若存在物理端口（conserving/LConn/RConn/Conn）则视为物理
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