function csv_to_mat(csv_dir, model_tag, out_mat_path)
% CSV → MAT 还原脚本
% why:
% - 允许在仅有 CSV（三张表 + 可选参数表）时重建 `model_graph.mat`，供重建模型或进一步分析
% how:
% - 读取 `*_elements.csv`、`*_ports.csv`、`*_connections.csv`，重组为结构数组
% - 若存在 `*_model_params.csv` 与 `*_block_params.csv`，同时组装 parameters 字段
% - 最终保存为 `graph/elements/ports/conn/connectivity` 到指定 MAT 文件

    % -------- 参数与默认值 --------
    if nargin < 1 || isempty(csv_dir)
        csv_dir = fullfile(pwd, 'export_model_graph');
    end
    if nargin < 2 || isempty(model_tag)
        % 尝试从目录中推断一个前缀（取 elements.csv 的前缀）
        model_tag = infer_model_tag(csv_dir);
    end
    if nargin < 3 || isempty(out_mat_path)
        out_mat_path = fullfile(csv_dir, sprintf('%s_graph.mat', model_tag));
    end

    % -------- 读取三张 CSV --------
    ele_csv = fullfile(csv_dir, sprintf('%s_elements.csv', model_tag));
    por_csv = fullfile(csv_dir, sprintf('%s_ports.csv', model_tag));
    con_csv = fullfile(csv_dir, sprintf('%s_connections.csv', model_tag));

    T_e = read_if_exists(ele_csv);
    T_p = read_if_exists(por_csv);
    T_c = read_if_exists(con_csv);

    % -------- 表 -> 结构数组 --------
    elements = table_to_elements(T_e);
    ports    = table_to_ports(T_p);
    conn     = table_to_connections(T_c);

    % -------- 组织 graph 对象 --------
    graph = struct();
    graph.model       = char(model_tag);
    graph.timestamp   = char(datetime('now','Format','yyyy-MM-dd HH:mm:ss'));
    graph.counts      = struct('elements',numel(elements),'ports',numel(ports),'connections',numel(conn),'lines',0);
    graph.elements    = elements;
    graph.ports       = ports;
    graph.connections = conn;
    graph.lines       = struct('Handle',{},'IsRoot',{},'SrcPath',{},'DstPaths',{},'Points',{}); % CSV 暂不含 lines

    % -------- 可选参数 CSV -> parameters --------
    params = struct();
    mp_csv = fullfile(csv_dir, sprintf('%s_model_params.csv', model_tag));
    bp_csv = fullfile(csv_dir, sprintf('%s_block_params.csv', model_tag));
    if exist(mp_csv,'file')
        try
            Tm = readtable(mp_csv);
            pm = struct();
            for i = 1:height(Tm)
                pm.(char(Tm.Param{i})) = char(string(Tm.Value{i}));
            end
            params.model = pm;
        catch
        end
    end
    if exist(bp_csv,'file')
        try
            Tb = readtable(bp_csv);
            params.blocks = rebuild_blocks_from_long_table(Tb);
        catch
        end
    end
    if ~isempty(fieldnames_safe(params))
        graph.parameters = params;
    end

    % -------- 落盘：与导出侧保持兼容的变量集合 --------
    connectivity = {}; %#ok<NASGU> % 历史变量名，保持
    save(out_mat_path, 'graph','elements','ports','conn','connectivity','-v7.3');
    fprintf('CSV 已还原为 MAT：%s\n', out_mat_path);
end

function T = read_if_exists(path)
    % 读取 CSV（若不存在则返回空 table）
    if exist(path,'file')
        T = readtable(path);
    else
        T = table();
    end
end

function elements = table_to_elements(T)
    % 将 elements 宽表还原为结构数组
    elements = struct('Path',{},'Name',{},'BlockType',{},'Orientation',{},'Position',{},'Center',{},'LibraryLink',{},'Mirror',{},'Rotation',{},'GotoTag',{},'GotoVisibility',{},'FromTag',{});
    if isempty(T), return; end
    n = height(T);
    for i = 1:n
        pos = [safe_num(T,'Left',i,0), safe_num(T,'Top',i,0), safe_num(T,'Right',i,0), safe_num(T,'Bottom',i,0)];
        ctr = [safe_num(T,'CenterX',i,0), safe_num(T,'CenterY',i,0)];
        elements(end+1) = struct( ... %#ok<AGROW>
            'Path',        safe_str(T,'Path',i,''), ...
            'Name',        safe_str(T,'Name',i,''), ...
            'BlockType',   safe_str(T,'BlockType',i,''), ...
            'Orientation', safe_str(T,'Orientation',i,''), ...
            'Position',    pos, ...
            'Center',      ctr, ...
            'LibraryLink', safe_str(T,'LibraryLink',i,''), ...
            'Mirror',      safe_str(T,'Mirror',i,''), ...
            'Rotation',    safe_str(T,'Rotation',i,''), ...
            'GotoTag',     safe_str(T,'GotoTag',i,''), ...
            'GotoVisibility', safe_str(T,'GotoVisibility',i,''), ...
            'FromTag',     safe_str(T,'FromTag',i,'') ...
        );
    end
end

function ports = table_to_ports(T)
    % 将 ports 宽表还原为结构数组
    ports = struct('BlockPath',{},'PortNumber',{},'PortType',{},'Position',{},'RelPos',{},'Side',{});
    if isempty(T), return; end
    n = height(T);
    for i = 1:n
        ports(end+1) = struct( ... %#ok<AGROW>
            'BlockPath', safe_str(T,'BlockPath',i,''), ...
            'PortNumber', safe_num(T,'PortNumber',i,-1), ...
            'PortType',   safe_str(T,'PortType',i,''), ...
            'Position',   [safe_num(T,'X',i,NaN), safe_num(T,'Y',i,NaN)], ...
            'RelPos',     [safe_num(T,'RelX',i,NaN), safe_num(T,'RelY',i,NaN)], ...
            'Side',       safe_str(T,'Side',i,'') ...
        );
    end
end

function conn = table_to_connections(T)
    % 将 connections 宽表还原为结构数组
    conn = struct('Source',{},'SourcePath',{},'SourcePort',{},'SourcePortKind',{},'SourcePortIndex',{},'Destination',{},'DestinationPath',{},'DestinationPort',{},'DestinationPortKind',{},'DestinationPortIndex',{},'Origin',{});
    if isempty(T), return; end
    n = height(T);
    for i = 1:n
        conn(end+1) = struct( ... %#ok<AGROW>
            'Source',      safe_str(T,'Source',i,''), ...
            'SourcePath',  safe_str(T,'SourcePath',i,''), ...
            'SourcePort',  safe_num(T,'SourcePort',i,-1), ...
            'SourcePortKind',  safe_str(T,'SourcePortKind',i,''), ...
            'SourcePortIndex', safe_num(T,'SourcePortIndex',i,-1), ...
            'Destination',      safe_str(T,'Destination',i,''), ...
            'DestinationPath',  safe_str(T,'DestinationPath',i,''), ...
            'DestinationPort',  safe_num(T,'DestinationPort',i,-1), ...
            'DestinationPortKind',  safe_str(T,'DestinationPortKind',i,''), ...
            'DestinationPortIndex', safe_num(T,'DestinationPortIndex',i,-1), ...
            'Origin',       safe_str(T,'Origin',i,'') ...
        );
    end
end

function blocks = rebuild_blocks_from_long_table(Tb)
    % 由长表（Path, BlockType, Param, Value）重建 blocks 参数结构
    blocks = struct('Path',{},'BlockType',{},'MaskType',{},'DialogParams',{});
    try
        if isempty(Tb), return; end
        [G, keys] = findgroups(Tb.Path);
        for k = 1:max(G)
            idx = find(G==k);
            if isempty(idx), continue; end
            path = char(Tb.Path{idx(1)});
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
        blocks = struct('Path',{},'BlockType',{},'MaskType',{},'DialogParams',{});
    end
end

function s = safe_str(T, field, i, def)
    % 读字符串型列，容错返回 char
    try
        if ismember(field, T.Properties.VariableNames)
            v = T.(field)(i);
            s = char(string(v));
            if ~isempty(s), return; end
        end
    catch
    end
    s = char(def);
end

function v = safe_num(T, field, i, def)
    % 读数值型列，容错返回 double
    try
        if ismember(field, T.Properties.VariableNames)
            v = double(T.(field)(i));
            if ~isnan(v), return; end
        end
    catch
    end
    v = double(def);
end

function tag = infer_model_tag(dirp)
    % 从 elements.csv 推断前缀
    d = dir(fullfile(dirp,'*_elements.csv'));
    if ~isempty(d)
        name = d(1).name;
        tag = erase(name, '_elements.csv');
    else
        tag = 'model';
    end
end

function n = fieldnames_safe(S)
    try
        n = fieldnames(S);
    catch
        n = {};
    end
end


