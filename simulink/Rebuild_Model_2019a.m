newModel = rebuild_model_from_export(fullfile(pwd,'export_model_graph','model_graph.mat'), 'recovered_model');
function newModel = rebuild_model_from_export(inputPath, newModelName)
% �ؽ� Simulink ģ�ͣ������� sigLines ?
% - ֱ��ʹ�õ���������· ? SourcePath/DestinationPath ��ȷ��λ ?
% - �˿ں�ȱ ?(-1/NaN)ʱ��ʹ�ö˿�/�鼸��λ�������ƥ�䵽�˿ھ� ?
% - �Զ�������ϵͳ�����˿���ڲ��ӿ飬��������ʹ ? autorouting

	% -------- 1) �������ݣ�MAT/JSON �� ? Ӧ ? --------
	if nargin < 1 || isempty(inputPath)
		error('���ṩǰ�浼���� MAT  ? JSON �ļ�·�� ?');
	end
	if nargin < 2 || isempty(newModelName)
		newModelName = 'recovered_model';
	end

	data = load_or_decode_graph(inputPath);

	% ������������ȱʧ���ÿհ�ȫ��
	elements = safe_field(data, 'elements', struct('Path',{},'Name',{},'BlockType',{},'Orientation',{},'Position',{},'Center',{},'LibraryLink',{},'Mirror',{},'Rotation',{},'GotoTag',{},'GotoVisibility',{},'FromTag',{}));
	ports    = safe_field(data, 'ports',    struct('BlockPath',{},'PortNumber',{},'PortType',{},'Position',{},'RelPos',{},'Side',{}));
	conns    = safe_connections(data);  % ���ڰ��� SourcePath/DestinationPath
    params   = safe_field(data, 'parameters', struct());  % ������ģ������������ѡ��
    % ���ļ����ˣ���ģ�ͼ�Ϊ�գ����Լ���ͬĿ¼ params.json/csv
    params   = ensure_params_loaded(params, inputPath);

	assert(~isempty(elements), 'elements Ϊ�գ��޷��ؽ�ģ�� ??');

    
    
	% ԭģ�͸��������� path �ض�λ������ģ�� ?
	origRoot = get_root_from_elements(elements);
	newModel = char(newModelName);

	% -------- 2) �����հ�ģ�� --------
	if bdIsLoaded(newModel)
		close_system(newModel, 0);
	end
	new_system(newModel); open_system(newModel);

	% -------- 3) Ԥ�����鲢������ ?->�ӣ� --------
	elemTable = preprocess_elements(elements, origRoot, newModel);              % �����¾�·��ӳ��
	elemTable = filter_descendants_of_library_blocks(elemTable);                % ���˿��ڲ��� ?

	% �Ƚ� SubSystem
	subRows = elemTable(strcmp(elemTable.BlockType, 'SubSystem'), :);
	subRows = sortrows(subRows, 'Depth');
	for i = 1:height(subRows), create_block_in_model(subRows(i,:), true); end

	% �ٽ���ͨ��
	blkRows = elemTable(~strcmp(elemTable.BlockType, 'SubSystem'), :);
	blkRows = sortrows(blkRows, 'Depth');
	for i = 1:height(blkRows), create_block_in_model(blkRows(i,:), false); end

	% -------- 4) �������ӣ�����������·�����˿ھ���� ?����ƥ�� ? --------
	% ������ԭ·��->��·������· ?->�������ꣻԭ·��->�����˿ڼ���
	orig2new = containers.Map('KeyType','char','ValueType','char');
	centers  = containers.Map('KeyType','char','ValueType','any');
	for i = 1:height(elemTable)
		orig2new(char(elemTable.OrigPath{i})) = char(elemTable.NewPath{i});
		cx = (elemTable.Left(i)+elemTable.Right(i))/2;
		cy = (elemTable.Top(i)+elemTable.Bottom(i))/2;
		centers(char(elemTable.NewPath{i})) = [cx, cy];
	end
	portsByOrig = index_ports_by_blockpath(ports);  % origPath -> �˿ڽṹ����

	% ������ԭ·�����ϣ����˿��ڲ��ӿ�����ӣ�
	keepSet = containers.Map('KeyType','char','ValueType','logical');
	for i = 1:height(elemTable), keepSet(char(elemTable.OrigPath{i})) = true; end

	% ������ϣ�����ʶ���ض�����ڲ����ߵ�����˿�
	libMask = elemTable.LibraryLink ~= "";
	libRootNewPaths  = cellfun(@char, elemTable.NewPath(libMask), 'UniformOutput', false);
	libRootOrigPaths = cellfun(@char, elemTable.OrigPath(libMask), 'UniformOutput', false);

	% -------- 4A) Ԥ�������ӣ��ź�/������ --------
	tasksSignal   = struct('srcOrig',{},'dstOrig',{},'srcNew',{},'dstNew',{}, ...
		'SP',{},'DP',{},'SPK',{},'SPI',{},'DPK',{},'DPI',{}, ...
		'origin',{},'parentSys',{},'srcCenter',{},'dstCenter',{});
	tasksPhysical = tasksSignal;

	for k = 1:numel(conns)
		% ��ȡ·����˿ں�
		srcOrigPath = char(getfield_or_default(conns(k), 'SourcePath', ''));
		dstOrigPath = char(getfield_or_default(conns(k), 'DestinationPath', ''));
		srcName     = char(getfield_or_default(conns(k), 'Source', ''));
		dstName     = char(getfield_or_default(conns(k), 'Destination', ''));
		SP          = getfield_or_default(conns(k), 'SourcePort', -1);
		DP          = getfield_or_default(conns(k), 'DestinationPort', -1);
		% ��ѡ�������Ķ˿������������������������ھ�ȷƥ�䣩
		SPK         = char(getfield_or_default(conns(k), 'SourcePortKind', ''));
		DPK         = char(getfield_or_default(conns(k), 'DestinationPortKind', ''));
		SPI         = getfield_or_default(conns(k), 'SourcePortIndex', -1);
		DPI         = getfield_or_default(conns(k), 'DestinationPortIndex', -1);
		originStr   = char(getfield_or_default(conns(k), 'Origin', ''));

		% ·����ӳ��
		if ~isempty(srcOrigPath)
			srcNewPath = sanitize_path(rebase_path(srcOrigPath, origRoot, newModel));
		else
			idx = find(strcmp(elemTable.ShortName, string(srcName)), 1, 'first');
			if isempty(idx), warning('�Ҳ���Դ�飺%s������������ ?', srcName); continue; end
			srcNewPath = char(elemTable.NewPath(idx));
			srcOrigPath = char(elemTable.OrigPath(idx));
		end
		if ~isempty(dstOrigPath)
			dstNewPath = sanitize_path(rebase_path(dstOrigPath, origRoot, newModel));
		else
			idx = find(strcmp(elemTable.ShortName, string(dstName)), 1, 'first');
			if isempty(idx), warning('�Ҳ���Ŀ��� ?%s������������ ?', dstName); continue; end
			dstNewPath = char(elemTable.NewPath(idx));
			dstOrigPath = char(elemTable.OrigPath(idx));
		end

		% �ж��Ƿ��ڿ��ڲ�����һ���ڿ��ڲ�����һ���ڿ��⣬��ѿ��ڲ����ض��򵽿����
		[srcNewPath, srcOrigPath, srcInLib, srcLibRootNew, srcLibRootOrig] = redirect_to_lib_root_if_inside(srcNewPath, srcOrigPath, libRootNewPaths, libRootOrigPaths);
		[dstNewPath, dstOrigPath, dstInLib, dstLibRootNew, dstLibRootOrig] = redirect_to_lib_root_if_inside(dstNewPath, dstOrigPath, libRootNewPaths, libRootOrigPaths);

		% ������˶���ͬһ�����ڲ��������������ڲ������ߣ�
		if srcInLib && dstInLib && strcmp(srcLibRootNew, dstLibRootNew)
			continue;
		end

		% ���˿��ڲ����ӣ��ϸ�����������Ԫ�ؼ��ϣ�
		if ~(isKey(keepSet, srcOrigPath) && isKey(keepSet, dstOrigPath))
			continue; 
		end

		% ��ϵͳ
		ps = fileparts(srcNewPath); pd = fileparts(dstNewPath);
		if ~strcmp(ps, pd)
			parentSys = common_parent_of_paths(srcNewPath, dstNewPath);
		else
			parentSys = ps;
		end

		% ����
		if isKey(centers, dstNewPath), dstCenter = centers(dstNewPath); else, dstCenter = [inf inf]; end
		if isKey(centers, srcNewPath), srcCenter = centers(srcNewPath); else, srcCenter = [inf inf]; end

		% �ж���
		[srcType, srcKnown] = get_export_port_type(portsByOrig, srcOrigPath, SP, 'src');
		[dstType, dstKnown] = get_export_port_type(portsByOrig, dstOrigPath, DP, 'dst');
		hasConsSrc = block_has_conserving_port(portsByOrig, srcOrigPath);
		hasConsDst = block_has_conserving_port(portsByOrig, dstOrigPath);
		newHasConsSrc = block_has_physical_ports_new(srcNewPath);
		newHasConsDst = block_has_physical_ports_new(dstNewPath);
		% ������ԴΪ PortConnectivity �ҡ�Դ/Ŀ�����˶��߱��������ӡ�ʱ���ж�Ϊ��������
		isPhysical = false;
		if strcmpi(originStr,'pc')
			physSrc = strcmpi(srcType,'conserving') || hasConsSrc || newHasConsSrc;
			physDst = strcmpi(dstType,'conserving') || hasConsDst || newHasConsDst;
			isPhysical = physSrc && physDst;
		else
			isPhysical = false;  % ��ͨ line һ����Ϊ�ź���
		end
		% ������ PC ���ұ���Ϊ�����ľ�����־
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
			% ��������������ͨ�߶�����Ϊ�ź��ߴ���
			tasksSignal(end+1) = task;   %#ok<AGROW>
		else
			% ������������ PC�������������� line �ظ�
			continue;
		end
	end

	% -------- 4A.1) ���������������߻��� Kind/Index���÷���� SPK/SPI ��Ϊ���ߵ� DPK/DPI�� --------
	% why: ֮ǰ�ԡ���ԡ�Ϊ��������һ����¼����ͬһ�Կ���ڶ��������ߣ��ᱻ���һ�����ǣ�
	%      ���»�ԭʱ���� DPI �������ͳһ��ͬһ������������Ϊ 3����
	% how: Ϊÿ�����ά��һ��������˳��Ķ˿���Ϣ���У����ڻ���ʱ�������ѣ�ȷ��һһ��Ӧ��
	pair2srcKind = containers.Map('KeyType','char','ValueType','any');
	for i = 1:numel(tasksPhysical)
		SPK_i = tasksPhysical(i).SPK; SPI_i = tasksPhysical(i).SPI;
		if ~isempty(SPK_i) && ~(isstring(SPK_i) && strlength(SPK_i)==0) && SPI_i>=1
			k = [char(tasksPhysical(i).srcNew) '->' char(tasksPhysical(i).dstNew)];
			% ��ʼ��Ϊ�ṹ��������У��������������
			if ~isKey(pair2srcKind, k)
				pair2srcKind(k) = struct('k',{},'i',{});
			end
			lst = pair2srcKind(k);
			lst(end+1) = struct('k',SPK_i,'i',SPI_i); %#ok<AGROW>
			pair2srcKind(k) = lst;
		end
	end
	% ����������Ѷ��У�ʵ��һһƥ�䣬���� DPI ��ֵͬ����
	for i = 1:numel(tasksPhysical)
		needDP = (isempty(tasksPhysical(i).DPK) || (isstring(tasksPhysical(i).DPK) && strlength(tasksPhysical(i).DPK)==0) || tasksPhysical(i).DPI<1);
		if needDP
			rk = [char(tasksPhysical(i).dstNew) '->' char(tasksPhysical(i).srcNew)];
			if isKey(pair2srcKind, rk)
				lst = pair2srcKind(rk);
				if ~isempty(lst)
					info = lst(1);
					% ����������ߵ�Դ�˿ڼ�Ϊ���ߵ�Ŀ��˿�
					tasksPhysical(i).DPK = info.k;
					tasksPhysical(i).DPI = info.i;
					% ������ʹ�����������һһ��Ӧ
					lst = lst(2:end);
					pair2srcKind(rk) = lst;
				end
			end
		end
	end

	% -------- 4A.2) ������ȥ�أ�ͬһ�Զ˿ڽ�����һ���� --------
	tasksPhysical = dedup_physical_tasks(tasksPhysical);

	% -------- 4B) �����ӡ��ź��ߡ� --------
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
			warning('�ź�����ʧ�� ?%s(%g) -> %s(%g)��ԭ��%s', S.srcNew, S.SP, S.dstNew, S.DP, ME.message);
		end
	end

	% -------- 4C) �����ӡ������ߡ� --------
	for t = 1:numel(tasksPhysical)
		P = tasksPhysical(t);
		try
			srcH = resolve_port_handle_by_kind_index(P.srcNew, P.SPK, P.SPI);
			dstH = resolve_port_handle_by_kind_index(P.dstNew, P.DPK, P.DPI);
			
			% �Ծ�����ڸ�ϵͳ�󹫹���ϵͳ������ϵͳ��ƥ��
			psrc = get_param(srcH,'Parent'); pdst = get_param(dstH,'Parent');
			parentUse = common_parent_of_paths(psrc, pdst);
			if port_has_existing_line(dstH)
				L = get_param(dstH,'Line');
				try, add_line(parentUse, srcH, L, 'autorouting','on'); catch, add_line(parentUse, L, srcH, 'autorouting','on'); end
			else
				add_line(parentUse, srcH, dstH, 'autorouting','on');
			end
		catch ME
			warning('��������ʧ�� ?%s(%g) -> %s(%g)��ԭ��%s', P.srcNew, P.SP, P.dstNew, P.DP, ME.message);
		end
	end

	% -------- 5) �طŲ�����ģ����飩 --------
	try
		apply_parameters_to_model(newModel, elemTable, params);
	catch ME
		warning('�����ط�ʧ�ܣ�����Ĭ�ϲ������У�%s', ME.message);
	end

	set_param(newModel, 'SimulationCommand', 'update');
	disp(['ģ�����ؽ���' newModel]);
end
% ============================== �������� ==============================


function h = resolve_port_handle_by_kind_index(blockPath, kind, idx)
	% �������Ķ˿�����/�������ڣ�ֱ�ӷ��ض�Ӧ��������򷵻� []
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
    % �� newPath ����ĳ��������£����ض��򵽿�������ؿ��־
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
	% �� srcNew/dstNew/DPK/DPI ά��ȥ�أ��� DPK/DPI ȱʧ����� srcNew/dstNew ��
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
	% ���� MAT  ? JSON������ͳ ?�ṹ
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
			error('��֧�ֵ��ļ����� ?%s�����ṩ .mat  ? .json ?', ext);
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
    % �� params.model Ϊ�� struct�����Դӱ߳��ļ����أ��뵼���ౣ�ֽ��
    try
        needModel = true;
        try, needModel = isempty(fieldnames(params)) || ~isfield(params,'model') || isempty(fieldnames(params.model)); catch, needModel = true; end
        if ~needModel, return; end

        [pdir, pbase, ~] = fileparts(inputPath);
        % ���� *_params.json�����ȣ�
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

        % ��ѡ��model/block params CSV������ json ������ʱ���ԣ�
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
        % �ۺϣ�ÿ�� Path �۳�һ����¼
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
    % ���ߣ�����ֶδ����򷵻�֮�����򷵻ؿ� struct
    if isstruct(S) && isfield(S, field) && ~isempty(S.(field))
        s = S.(field);
    else
        s = struct();
    end
end

function conns = safe_connections(data)
	% ��׼�����ӽṹ��ȷ������ SourcePath/DestinationPath
	if isfield(data, 'connections') && ~isempty(data.connections)
		conns = data.connections;
	elseif isfield(data, 'connectivity') && ~isempty(data.connectivity)
		c = data.connectivity;
		if iscell(c), conns = [c{:}]; else, conns = c; end
	else
		conns = struct('Source',{},'SourcePath',{},'SourcePort',{},'Destination',{},'DestinationPath',{},'DestinationPort',{});
	end
	% �ֶβ���
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
	%  ? elements.Path �ƶ�ԭ�� ?
	p = elements(1).Path;
	slash = find(p=='/', 1, 'first');
	if isempty(slash)
		root = p;              % ���� ?
	else
		root = p(1:slash-1);   % ����
	end
end

function T = preprocess_elements(elements, origRoot, newModel)
	% �������ڴ����ı��񣬲�ӳ�䵽��ģ��· ?
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

% ========================= �����ط���غ��� =========================
function apply_parameters_to_model(newModel, elemTable, params)
    % why: ��ԭģ�������/ʱ�䲽��ȫ�ֲ������Լ�����ĶԻ�������ʹ��ֱ�ӷ��桱��Ϊ����
    % how: ���ڵ���ʱ���ڲ���ʱ�ų��Իطţ�set_param ȫ��ʹ���ַ���ֵ

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
            % ĳЩ�汾/��Ʒδ��װʱ���������ã�����
        end
    end
end

function apply_block_params(elemTable, blockParams)
    % ������ԭ·��->��·�������ң�����ʹ�õ��� Path ���лط�
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
                % ���ˣ��ö���ƥ���׸�ͬ����
                idx = find(strcmp(elemTable.ShortName, string(get_basename(origPath))), 1, 'first');
                if isempty(idx), continue; end
                newPath = char(elemTable.NewPath(idx));
            end

			% 成组一次性 set_param（方案A）：避免掩模块在参数不一致中间态触发初始化报错
			if isfield(bp,'DialogParams') && ~isempty(bp.DialogParams)
				names = fieldnames(bp.DialogParams);
				nvPairs = {};
				for k = 1:numel(names)
					pname = names{k};
					% 跳过几何类参数
					if any(strcmpi(pname, {'Position','PortConnectivity','LineHandles'}))
						continue;
					end
					pval  = bp.DialogParams.(pname);
					% 非字符串一律用 mat2str 序列化，确保 set_param 可接受
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

%=========================�����طź�������=========================

function s = def_str(st, field, defaultV)
	if nargin < 3, defaultV = ""; end
	if isfield(st, field) && ~isempty(st.(field))
		s = st.(field);
	else
		s = defaultV;
	end
end

function newPath = rebase_path(origPath, origRoot, newRoot)
	% ��ԭ·���滻Ϊ��ģ�͸�·�����޷���ȷӳ��ʱȡ ?��һ�����ҵ��¸�
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
	% �޳������ÿ��ڲ��ӿ飨�� ? ".../DC Voltage Source/Model" ?
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
	% ȥ��β��б�ܣ��� ? Simulink �Ϸ�·��
	p = char(string(p));
	while ~isempty(p) && p(end) == '/', p = p(1:end-1); end
end

function ensure_system_exists(sysPath)
	% �𼶴�����ϵ ?
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
	% ʵ�ʴ��������飬������λ ?/�����ݵȣ�
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
						%warning(' ? %s �޿����ã�ʹ ? Subsystem ռλ ?', newPath);
				end
			end
		end
        set_param(newPath, 'Position', pos, 'Orientation', ori);
        % ���Իطž���/��ת
        if ~isempty(mir)
            try, set_param(newPath,'BlockMirror',mir); end
        end
        if ~isempty(rot)
            try, set_param(newPath,'BlockRotation',rot); end
        end
        % �ط� Goto/From ��ǩ
        if strcmpi(btype,'Goto')
            if ~isempty(gtag), try, set_param(newPath,'GotoTag',gtag); end, end
            if ~isempty(gvis), try, set_param(newPath,'TagVisibility',gvis); end, end
        elseif strcmpi(btype,'From') && ~isempty(ftag)
            try, set_param(newPath,'GotoTag',ftag); end
        end
	catch ME
		warning('������ʧ�ܣ�%s��ԭ��%s ?', newPath, ME.message);
	end
end


function portsIdx = index_ports_by_blockpath(ports)
	% �������� ports  ? BlockPath��ԭ·������ ?
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
	% �жϸ�ԭ·���Ķ˿��Ƿ�����������ض˿ڣ�Side/Type ��ʾ��
	tf = false;
	try
		if isKey(portsIdx, origPath)
			P = portsIdx(origPath);
			if isstruct(P)
				types = {P.PortType};
				% �����˿��ж���Simscape conserving �� Specialized Power Systems �� LConn/RConn/Conn
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
	% ���˿��Ƿ��������ӣ������ظ� add_line��
	tf = false;
	try
		l = get_param(portHandle,'Line');
		if l ~= -1
			% Ŀ���Դ�˿�������
			tf = true;
		end
	catch
		tf = false;
	end
end

function tf = block_has_physical_ports_new(blockPath)
	% ֱ���ڡ���ģ�͵Ŀ顱��̽���Ƿ���������˿ڣ�LConn/RConn/Conn��
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
	% �ӵ�����Ϣ�ƶ϶˿����ͣ�inport/outport/conserving/port��������ʶ�𷵻� known=true
	ptype = '';

    
	known = false;
	try
		if isKey(portsIdx, origPath)
			P = portsIdx(origPath);
			% ����ʹ�ö˿ں�ƥ��һ��������޶˿ںžͿ��������Ƿ�����ȷ������
			candidate = [];
			if isfinite(pnum) && pnum>=1 && pnum<=numel(P)
				candidate = P(pnum);
			end
			if ~isempty(candidate)
				if isfield(candidate,'PortType') && ~isempty(candidate.PortType)
					ptype = candidate.PortType; known = true; return; 
				end
			end
			% ���ܶ˿�����
			types = {};
			for i = 1:numel(P)
				try, types{end+1} = P(i).PortType; catch, end %#ok<AGROW>
			end
            % �������ӣ�conserving �� LConn/RConn/Conn
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
%newModel = rebuild_model_from_export(fullfile(pwd,'export_model_graph','model_graph.mat'), 'recovered_model');