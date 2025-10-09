
    % 打开文件准备写入代码
 model_name = 'untitled1';
 open_system(model_name);
  
    
    % 获取模型中的所有块
 blocks = find_system(model_name, 'SearchDepth', 1);
    
    % 跳过模型本身
 blocks = blocks(2:end);
    for i = 1:length(blocks)
        disp(getfullname(blocks(i,1)));
    end
% 获取模型中所有“线”对象（信号线）
% 说明：这一步覆盖普通 Simulink 信号线；但拿不到物理网络线（如 RLC 两端子线）
all_blocks = find_system(model_name, 'SearchDepth',1,'FindAll','on', ...
    'LookUnderMasks','on', 'FollowLinks','on', 'type','line');

% 创建一个空的结构体或单元格数组来存储连接关系
connectivity = {};
% 去重：源块全名|源端口 => 目标块全名|目标端口
conn_keys = containers.Map('KeyType','char','ValueType','logical');

% 遍历每条“信号线”，并递归遍历分支（避免漏掉 LineChildren）
for i = 1:length(all_blocks)
    block = all_blocks(i,1);               % 实际是 line 句柄，沿用你的变量名

    src_port = get_param(block, 'SrcPortHandle');
    dst_ports = get_param(block, 'DstPortHandle'); % 保留你的原变量（后续我们用递归统一收集）
    if src_port ~= -1
        src_block = get_param(src_port, 'Parent');
        src_block_name = get_param(src_block, 'Name');
        src_block_full = getfullname(src_block);
        src_port_num = get_param(src_port, 'PortNumber');

        % 递归获取该线及全部分支的目标端口
        visited_lines = [];
        [dst_blk_handles, dst_port_handles, visited_lines] = collect_all_dsts(block, visited_lines);

        % 处理所有目标
        for j = 1:length(dst_port_handles)
            if dst_port_handles(j) ~= -1
                dst_block = dst_blk_handles(j);
                dst_block_name = get_param(dst_block, 'Name');
                dst_block_full = getfullname(dst_block);
                dst_port_num = get_param(dst_port_handles(j), 'PortNumber');

                % 新增：记录端口种类与“在各自类型数组中的索引”（稳定且可复现）
                [src_kind, src_index] = kind_and_index_by_handle(src_block, src_port);
                [dst_kind, dst_index] = kind_and_index_by_handle(dst_block, dst_port_handles(j));

                key = sprintf('%s|%d=>%s|%d', src_block_full, src_port_num, dst_block_full, dst_port_num);
                if ~isKey(conn_keys, key)
                    conn_keys(key) = true;
                    connectivity{end+1} = struct( ...
                        'Source',           src_block_name, ...
                        'SourcePath',       src_block_full, ...   % 新增：源块完整路径
                        'SourcePort',       src_port_num, ...
                        'SourcePortKind',   src_kind, ...        % 新增：源端口种类（Outport/Inport/...）
                        'SourcePortIndex',  src_index, ...       % 新增：在该种类数组中的索引
                        'Destination',      dst_block_name, ...
                        'DestinationPath',  dst_block_full, ...   % 新增：目标块完整路径
                        'DestinationPort',  dst_port_num, ...
                        'DestinationPortKind',  dst_kind, ...    % 新增
                        'DestinationPortIndex', dst_index, ...   % 新增
                        'Origin',           'line' ...            % 可选：来源标记（普通信号线）
                    );
                    disp(['Line from ', src_block_name, '(', num2str(src_port_num), ') to ', ...
                                      dst_block_name, '(', num2str(dst_port_num), ')']);
                end
            end
        end
    end
end

% ========= 新增：基于 PortConnectivity 的扫描 =========
all_blocks_pc = find_system(model_name, 'FindAll','on','LookUnderMasks','all','FollowLinks','on','Type','block');  % 不限制层级，确保掩模/链接下的端口也统计

for i = 1:length(all_blocks_pc)
    bh = all_blocks_pc(i);
    pc = get_param(bh, 'PortConnectivity');  % 每个端口的上下游连接信息（同时支持物理连接与信号线）

    % 遍历该块的每个端口连接
    for p = 1:numel(pc)
        % 1) 作为“输出端口”一侧：本块 -> 下游块（包含信号与物理端口）
        if isfield(pc(p),'DstBlock') && ~isempty(pc(p).DstBlock) && all(pc(p).DstBlock ~= -1)
            src_block_full = getfullname(bh);
            src_block_name = get_param(bh, 'Name');
            src_port_num   = get_port_num(pc(p));   % 当前端口号（本块一侧）
            % 新增：端口种类与在该种类数组的索引
            [src_kind2, src_index2] = kind_and_index_by_pc(bh, pc(p));

            for d = 1:numel(pc(p).DstBlock)
                dst_bh         = pc(p).DstBlock(d);
                dst_block_full = getfullname(dst_bh);
                dst_block_name = get_param(dst_bh, 'Name');
                % 目标端口号在物理域常无意义，这里改为“在目标块PC中反查 SrcBlock==bh 的条目”
                dst_pc_entry   = find_pc_entry_by_srcblock(dst_bh, bh);
                dst_port_num   = get_port_num(dst_pc_entry);  % 若取得为空则返回 NaN→后续统一为 -1
                [dst_kind2, dst_index2] = kind_and_index_by_pc(dst_bh, dst_pc_entry);

                key = sprintf('%s|%d=>%s|%d', src_block_full, src_port_num, dst_block_full, dst_port_num);
                if ~isKey(conn_keys, key)
                    conn_keys(key) = true;
                    connectivity{end+1} = struct( ...
                        'Source',           src_block_name, ...
                        'SourcePath',       src_block_full, ...   % 新增
                        'SourcePort',       src_port_num, ...
                        'SourcePortKind',   src_kind2, ...
                        'SourcePortIndex',  src_index2, ...
                        'Destination',      dst_block_name, ...
                        'DestinationPath',  dst_block_full, ...   % 新增
                        'DestinationPort',  dst_port_num, ...
                        'DestinationPortKind',  dst_kind2, ...
                        'DestinationPortIndex', dst_index2, ...
                        'Origin',           'pc' ...              % 可选：来源标记（PortConnectivity）
                    );
                    disp(['Line from ', src_block_name, '(', num2str(src_port_num), ') to ', ...
                                      dst_block_name, '(', num2str(dst_port_num), ') [PC]']);
                end
            end
        end

        % 2) 作为“输入端口”一侧：上游块 -> 本块（物理端口常为双向，这里也补齐）
        if isfield(pc(p),'SrcBlock') && ~isempty(pc(p).SrcBlock) && pc(p).SrcBlock ~= -1
            src_bh          = pc(p).SrcBlock;
            src_block_full  = getfullname(src_bh);
            src_block_name  = get_param(src_bh, 'Name');
            % 上游块的端口号应取 SrcPort；没有则置为 -1 以便后续识别
            if isfield(pc(p),'SrcPort') && ~isempty(pc(p).SrcPort)
                src_port_num = pc(p).SrcPort;
            else
                src_port_num = -1;
            end
            [src_kind3, src_index3] = kind_and_index_by_pc(src_bh, pc(p));

            dst_block_full  = getfullname(bh);
            dst_block_name  = get_param(bh, 'Name');
            dst_port_num    = get_port_num(pc(p));  % 本块一侧端口号
            [dst_kind3, dst_index3] = kind_and_index_by_pc(bh, pc(p));

            key = sprintf('%s|%d=>%s|%d', src_block_full, src_port_num, dst_block_full, dst_port_num);
            if ~isKey(conn_keys, key)
                conn_keys(key) = true;
                connectivity{end+1} = struct( ...
                    'Source',           src_block_name, ...
                    'SourcePath',       src_block_full, ...      % 新增
                    'SourcePort',       src_port_num, ...
                    'SourcePortKind',   src_kind3, ...
                    'SourcePortIndex',  src_index3, ...
                    'Destination',      dst_block_name, ...
                    'DestinationPath',  dst_block_full, ...      % 新增
                    'DestinationPort',  dst_port_num, ...
                    'DestinationPortKind',  dst_kind3, ...
                    'DestinationPortIndex', dst_index3, ...
                    'Origin',           'pc' ...                 % 可选：来源标记（PortConnectivity）
                );
                disp(['Line from ', src_block_name, '(', num2str(src_port_num), ') to ', ...
                                  dst_block_name, '(', num2str(dst_port_num), ') [PC]']);
            end
        end
    end
end
% 现在 connectivity 单元格数组包含了所有的源-目标连接对（信号线 + 物理网络线）
for i = 1:length(connectivity)
    disp(connectivity{1,i});
end
% ============================================================
% 采集所有元件（block）的位置信息（像素坐标，原点为画布左上）
% why: 后续可视化/导出需要几何信息；保持与连接表同一根模型
% ============================================================



% 拿到所有 block 句柄（包含掩模/链接/变体）
all_blocks_for_pos = all_blocks_pc;

% 结果容器：元件、端口（可选）、连线（可选）
elements = struct('Path',{},'Name',{},'BlockType',{},'Orientation',{}, ...
                  'Position',{},'Center',{},'LibraryLink',{}, ...
                  'Mirror',{},'Rotation',{}, ...
                  'GotoTag',{},'GotoVisibility',{},'FromTag',{});  % 新增：几何镜像/旋转与Goto/From标签/可见性
ports    = struct('BlockPath',{},'PortNumber',{},'PortType',{},'Position',{}, ...
                  'RelPos',{},'Side',{});      % 新增：端口相对坐标与侧别
sigLines = struct('Handle',{},'IsRoot',{},'SrcPath',{},'DstPaths',{},'Points',{});

% ========== A) 元件位置 ==========
for i = 1:numel(all_blocks_for_pos)
    bh = all_blocks_for_pos(i);

    % 元件基础信息
    path  = getfullname(bh);
    name  = get_param(bh,'Name');
    btype = get_param(bh,'BlockType');              % 某些掩模块可能为空字符串
    ori   = get_param(bh,'Orientation');            % right/left/up/down
    pos   = get_param(bh,'Position');               % [L T R B]
    ctr   = [(pos(1)+pos(3))/2, (pos(2)+pos(4))/2]; % 中心点
    lib   = '';
    % 新增：尽可能读取镜像/旋转信息（并非所有块都有这些属性）
    mir   = '';
    rot   = '';
    try, mir = get_param(bh,'BlockMirror'); end
    try, rot = get_param(bh,'BlockRotation'); end
    % 新增：若为 Goto/From，记录其标签，便于重建时消除默认标签冲突
    gtag = '';
    gvis = '';
    ftag = '';
    if strcmpi(btype,'Goto')
        try, gtag = get_param(bh,'GotoTag'); end
        try, gvis = get_param(bh,'TagVisibility'); catch, gvis = ''; end
    elseif strcmpi(btype,'From')
        try, ftag = get_param(bh,'GotoTag'); end % From 块同样使用 GotoTag 字段
    end
    if strcmp(get_param(bh,'LinkStatus'),'resolved')
        lib = get_param(bh,'ReferenceBlock');       % 来自库的引用路径
    end

    elements(end+1) = struct( ...                   %#ok<AGROW>
        'Path',        path, ...
        'Name',        name, ...
        'BlockType',   btype, ...
        'Orientation', ori, ...
        'Position',    pos, ...
        'Center',      ctr, ...
        'LibraryLink', lib, ...
        'Mirror',      mir, ...
        'Rotation',    rot, ...
        'GotoTag',     gtag, ...
        'GotoVisibility', gvis, ...
        'FromTag',     ftag ...
    );
end

% ========== B) 端口位置（可选，但强烈建议一起存） ==========
% why: 即使某些“物理网络线”没有常规 line 对象，也能用端口坐标重建边
for i = 1:numel(all_blocks_for_pos)
    bh = all_blocks_for_pos(i);
    path = getfullname(bh);

    pc = get_param(bh,'PortConnectivity');  % 端口连接与几何信息
    for p = 1:numel(pc)
        pnum = get_port_num(pc(p));         % 兼容 PortNumber/Port
        ppos = get_port_position(pc(p), bh);% 优先用 pc(p).Position，必要时回退到句柄
        ptyp = '';
        if isfield(pc(p),'Type') && ~isempty(pc(p).Type)
            ptyp = pc(p).Type;              % inport/outport/conserving 等
        else
            if ~isempty(pc(p).DstBlock) && all(pc(p).DstBlock~=-1)
                ptyp = 'outport';
            elseif ~isempty(pc(p).SrcBlock) && pc(p).SrcBlock~=-1
                ptyp = 'inport';
            else
                ptyp = 'port';
            end
        end
        % 进一步：用 PortHandles 精确识别 LConn/RConn/Conn（某些库下 pc(p).Type 为空）
        try
            ph = get_param(bh,'PortHandles');
            typeNames = {'LConn','RConn','Conn','Inport','Outport'};  % 物理在前
            candPos = []; candType = {};
            for tt = 1:numel(typeNames)
                nm = typeNames{tt};
                if isfield(ph, nm) && ~isempty(ph.(nm))
                    hh = ph.(nm)(:);
                    pp = arrayfun(@(x) get_param(x,'Position'), hh, 'UniformOutput', false);
                    if ~isempty(pp)
                        candPos = [candPos; vertcat(pp{:})]; %#ok<AGROW>
                        candType = [candType; repmat({lower(nm)}, numel(pp), 1)]; %#ok<AGROW>
                    end
                end
            end
            if ~any(isnan(ppos)) && ~isempty(candPos)
                d = hypot(candPos(:,1)-ppos(1), candPos(:,2)-ppos(2));
                [~, kmin] = min(d);
                detected = candType{kmin};
                % 若原类型为空/port/inout，则用检测结果覆盖
                if isempty(ptyp) || strcmpi(ptyp,'port') || strcmpi(ptyp,'inout')
                    ptyp = detected;
                end
            end
        catch
        end
        % 新增：端口相对块的归一化坐标与侧别，提升重建时物理端口匹配的稳健性
        rel = [NaN NaN];
        side = '';
        try
            if ~any(isnan(ppos))
                w = max(1, (pos(3)-pos(1))); h = max(1, (pos(4)-pos(2)));
                rel = [(ppos(1)-pos(1))/w, (ppos(2)-pos(2))/h];
                % 根据与边界的距离判断侧别
                dL = abs(ppos(1)-pos(1)); dR = abs(ppos(1)-pos(3));
                dT = abs(ppos(2)-pos(2)); dB = abs(ppos(2)-pos(4));
                [~,iSide] = min([dL,dR,dT,dB]);
                side = {'left','right','top','bottom'}
                side = side{iSide};
            end
        catch
        end

        ports(end+1) = struct( ...          %#ok<AGROW>
            'BlockPath',  path, ...
            'PortNumber', pnum, ...
            'PortType',   ptyp, ...
            'Position',   ppos, ...
            'RelPos',     rel, ...
            'Side',       side ...
        );
    end
end

% ========== C) 常规信号线的折线坐标（可选） ==========
% why: 普通 Simulink 信号线有 line 对象，能直接拿 polyline 点；物理线若没有，则用 B) 的端点重建
all_lines = all_blocks;
for i = 1:numel(all_lines)
    lh = all_lines(i);
    srcH = get_param(lh,'SrcPortHandle');
    isRoot = srcH ~= -1;

    % 源与目标（注意：branch line 也有自己的 Dst 集合）
    srcPath = '';
    if isRoot
        srcPath = getfullname(get_param(srcH,'Parent'));
    end
    dstHs = get_param(lh,'DstBlockHandle');
    if isequal(dstHs,-1) || isempty(dstHs)
        dstPaths = {};
    else
        dstPaths = arrayfun(@(h)getfullname(h), dstHs(:),'UniformOutput',false);
    end

    pts = get_param(lh,'Points'); % Nx2 折线坐标
    sigLines(end+1) = struct( ... %#ok<AGROW>
        'Handle',  lh, ...
        'IsRoot',  isRoot, ...
        'SrcPath', srcPath, ...
        'DstPaths',{dstPaths}, ...
        'Points',  pts ...
    );
end

% 你现在拥有：
% elements  -> 所有元件的几何信息（位置/中心/朝向/库链接）
% ports     -> 所有端口的像素坐标（可用于重建物理连接）
% sigLines  -> 所有常规“信号线”的折线点集
% 可按需保存为 MAT/JSON，或与 connectivity 表进行 join
% =========
%% ========= 导出模型几何与连接信息（JSON / MAT / CSV，含完整路径） =========
% why:
% - JSON：跨语言可读，包含完整嵌套结构（含完整路径，便于复原无歧义）
% - MAT：保留原始变量，便于 MATLAB 二次分析
% - CSV：便于快速查看与表格处理（元素/端口/连线三张表）

% 1) 基本输出目录与文件名（对模型名做清洗，确保可作为文件名）
if ~exist('model_root','var') || isempty(model_root), model_root = 'model'; end
model_tag = regexprep(char(model_root),'[^\w]','_');
out_dir   = fullfile(pwd, 'export_model_graph');
if ~exist(out_dir, 'dir'); mkdir(out_dir); end

% 2) 兜底：若变量缺失则给空结构，避免导出时报错
if ~exist('elements','var') || isempty(elements)
    elements = struct('Path',{},'Name',{},'BlockType',{},'Orientation',{},'Position',{},'Center',{},'LibraryLink',{});
end
if ~exist('ports','var') || isempty(ports)
    ports = struct('BlockPath',{},'PortNumber',{},'PortType',{},'Position',{});
end
if ~exist('sigLines','var') || isempty(sigLines)
    sigLines = struct('Handle',{},'IsRoot',{},'SrcPath',{},'DstPaths',{},'Points',{});
end
if ~exist('connectivity','var') || isempty(connectivity)
    connectivity = {};
end

% 连接从 cell 转成 struct 数组，统一字段以“包含完整路径”为准
if iscell(connectivity)
    if isempty(connectivity)
        conn = struct('Source',{},'SourcePath',{},'SourcePort',{}, ...
                      'Destination',{},'DestinationPath',{},'DestinationPort',{}, ...
                      'Origin',{});
    else
        conn = [connectivity{:}];
    end
else
    conn = connectivity;
end

% 3) 规范化字段与数值（补全缺失字段；端口号 NaN -> -1）
wantedFields = {'Source','SourcePath','SourcePort','Destination','DestinationPath','DestinationPort','Origin'};
if isempty(conn)
    % 空连接表也需要确保字段存在，便于 JSON/CSV 写出
    conn = cell2struct(cell(size(wantedFields)), wantedFields, 2);
    conn(1) = []; % 置空但保留字段定义
else
    % 为所有连接补齐字段，保持一致性
    for i = 1:numel(conn)
        % 字段缺失则补空
        for f = 1:numel(wantedFields)
            fld = wantedFields{f};
            if ~isfield(conn, fld) || isempty(conn(i).(fld))
                switch fld
                    case {'Source','Destination','SourcePath','DestinationPath','Origin'}
                        conn(i).(fld) = '';
                    otherwise
                        conn(i).(fld) = -1;
                end
            end
        end
        % 端口号标准化
        if ~isfield(conn,'SourcePort') || isempty(conn(i).SourcePort) || isnan(conn(i).SourcePort)
            conn(i).SourcePort = -1;
        end
        if ~isfield(conn,'DestinationPort') || isempty(conn(i).DestinationPort) || isnan(conn(i).DestinationPort)
            conn(i).DestinationPort = -1;
        end
        % 确保字符串为 char（避免 string 编码差异）
        conn(i).Source          = char(conn(i).Source);
        conn(i).SourcePath      = char(conn(i).SourcePath);
        conn(i).Destination     = char(conn(i).Destination);
        conn(i).DestinationPath = char(conn(i).DestinationPath);
        if ~isempty(conn(i).Origin)
            conn(i).Origin = char(conn(i).Origin);
        else
            conn(i).Origin = '';
        end
    end
end

% 4) 组织总对象并写 JSON（完整信息，包含完整路径）
graph = struct();
graph.model       = char(model_root);
graph.timestamp   = char(datetime('now','Format','yyyy-MM-dd HH:mm:ss'));
graph.counts      = struct('elements',numel(elements),'ports',numel(ports), ...
                           'connections',numel(conn),'lines',numel(sigLines));
graph.elements    = elements;
graph.ports       = ports;
graph.connections = conn;
graph.lines       = sigLines;

json_path = fullfile(out_dir, sprintf('%s_graph.json', model_tag));
txt = jsonencode(graph, 'ConvertInfAndNaN', true);   % MATLAB 会将 NaN 编码为 null
fid = fopen(json_path,'w');
assert(fid~=-1, '无法创建文件: %s', json_path);
fwrite(fid, txt, 'char');
fclose(fid);

% 5) 另存 MAT（保留原始结构，便于后续 MATLAB 直接加载）
mat_path = fullfile(out_dir, sprintf('%s_graph.mat', model_tag));
save(mat_path, 'graph','elements','ports','sigLines','conn','connectivity','-v7.3');

% 6) 再导出三份 CSV（展开关键几何字段，便于快速查看）
% 6.1 元件 CSV：展开 Position/Center
if ~isempty(elements)
    names  = {elements.Name}';
    paths  = {elements.Path}';
    types  = {elements.BlockType}';
    oris   = {elements.Orientation}';
    libs   = {elements.LibraryLink}';
    mirs   = {elements.Mirror}';
    rots   = {elements.Rotation}';
    gtags  = {elements.GotoTag}';
    gvises = {elements.GotoVisibility}';
    ftags  = {elements.FromTag}';
    posMat = vertcat(elements.Position);     % Nx4: [L T R B]
    ctrMat = vertcat(elements.Center);       % Nx2: [Cx Cy]
    T_e = table(names, paths, types, oris, libs, mirs, rots, gtags, gvises, ftags, ...
        posMat(:,1), posMat(:,2), posMat(:,3), posMat(:,4), ...
        ctrMat(:,1), ctrMat(:,2), ...
        'VariableNames', {'Name','Path','BlockType','Orientation','LibraryLink','Mirror','Rotation','GotoTag','GotoVisibility','FromTag', ...
                          'Left','Top','Right','Bottom','CenterX','CenterY'});
    writetable(T_e, fullfile(out_dir, sprintf('%s_elements.csv', model_tag)));
end

% 6.2 端口 CSV：展开 Position
if ~isempty(ports)
    bpaths = {ports.BlockPath}';
    pnums  = [ports.PortNumber]';
    ptypes = {ports.PortType}';
    ppos   = vertcat(ports.Position);        % Nx2: [x y]
    prel   = vertcat(ports.RelPos);          % Nx2: [rx ry]
    sides  = {ports.Side}';
    T_p = table(bpaths, pnums, ptypes, ppos(:,1), ppos(:,2), prel(:,1), prel(:,2), sides, ...
        'VariableNames', {'BlockPath','PortNumber','PortType','X','Y','RelX','RelY','Side'});
    writetable(T_p, fullfile(out_dir, sprintf('%s_ports.csv', model_tag)));
end

% 6.3 连线 CSV：源/目标 + 完整路径 + 端口号 + 来源
if ~isempty(conn)
    srcs    = {conn.Source}';
    srcps   = {conn.SourcePath}';
    sps     = [conn.SourcePort]';
    spk     = getfield_or_default_cell(conn,'SourcePortKind');
    spi     = getfield_or_default_num(conn,'SourcePortIndex');
    dsts    = {conn.Destination}';
    dstps   = {conn.DestinationPath}';
    dps     = [conn.DestinationPort]';
    dpk     = getfield_or_default_cell(conn,'DestinationPortKind');
    dpi     = getfield_or_default_num(conn,'DestinationPortIndex');
    origins = {conn.Origin}';
    T_c = table(srcs, srcps, sps, spk, spi, dsts, dstps, dps, dpk, dpi, origins, ...
        'VariableNames', {'Source','SourcePath','SourcePort','SourcePortKind','SourcePortIndex', ...
                          'Destination','DestinationPath','DestinationPort','DestinationPortKind','DestinationPortIndex','Origin'});
    writetable(T_c, fullfile(out_dir, sprintf('%s_connections.csv', model_tag)));
end

fprintf('导出完成：\n JSON  -> %s\n MAT   -> %s\n CSVs  -> %s\n', json_path, mat_path, out_dir);
close_system(model_name);
% =========
% 工具函数：递归收集“某条线及其所有分支”的目标端口（仅对普通信号线有效）
% =========
function [dstBlks, dstPorts, visited_lines] = collect_all_dsts(line_h, visited_lines)
    if any(visited_lines == line_h)
        dstBlks = [];
        dstPorts = [];
        return;
    end
    visited_lines(end+1) = line_h; %#ok<AGROW>

    dstBlks  = get_param(line_h,'DstBlockHandle');
    dstPorts = get_param(line_h,'DstPortHandle');
    if isequal(dstBlks, -1) || isempty(dstBlks)
        dstBlks = [];
        dstPorts = [];
    end

    children = get_param(line_h,'LineChildren');
    if ~isequal(children, -1) && ~isempty(children)
        for t = 1:numel(children)
            [cBlks, cPorts, visited_lines] = collect_all_dsts(children(t), visited_lines);
            if ~isempty(cBlks)
                dstBlks  = [dstBlks(:);  cBlks(:) ]; %#ok<AGROW>
                dstPorts = [dstPorts(:); cPorts(:)]; %#ok<AGROW>
            end
        end
    end
end

function n = get_port_num(pcEntry)
    % 兼容不同版本的 PortConnectivity 字段差异
    if isfield(pcEntry, 'PortNumber') && ~isempty(pcEntry.PortNumber)
        n = pcEntry.PortNumber;
    elseif isfield(pcEntry, 'Port') && ~isempty(pcEntry.Port)
        n = pcEntry.Port;
    else
        % 某些特殊端口（如物理端口）可能没有编号，这里返回 NaN 以示区分
        n = NaN;
    end
end

function [kind, idx] = kind_and_index_by_handle(blockH, portH)
    % 通过 PortHandles 反查端口属于哪一类及在该类数组的索引
    kind = '';
    idx  = -1;
    try
        ph = get_param(blockH,'PortHandles');
        fields = {'Outport','Inport','LConn','RConn','Conn'};
        for i = 1:numel(fields)
            f = fields{i};
            if isfield(ph,f) && ~isempty(ph.(f))
                k = find(ph.(f) == portH, 1);
                if ~isempty(k)
                    kind = lower(f);
                    idx  = k;
                    return;
                end
            end
        end
    catch
    end
end

function [kind, idx] = kind_and_index_by_pc(blockH, pcEntry)
    % 通过 pcEntry 和 PortHandles 推断端口种类与索引（用于 pc 来源）
    kind = '';
    idx  = -1;
    try
        ph = get_param(blockH,'PortHandles');
        % 优先使用 Type
        if isfield(pcEntry,'Type') && ~isempty(pcEntry.Type)
            switch lower(pcEntry.Type)
                case {'inport','outport','lconn','rconn','conn','conserving'}
                    f = map_type_to_field(lower(pcEntry.Type));
                    if isfield(ph,f) && ~isempty(ph.(f))
                        kind = lower(f); idx = 1; % 无法精确时返回 1 占位
                        return;
                    end
            end
        end
        % 回退：根据坐标匹配最近端口句柄，并返回其种类与索引
        if isfield(pcEntry,'Position') && ~isempty(pcEntry.Position)
            candH = []; candP = []; candF = {};
            fields = {'Outport','Inport','LConn','RConn','Conn'};
            for i = 1:numel(fields)
                f = fields{i};
                if isfield(ph,f) && ~isempty(ph.(f))
                    hh = ph.(f)(:);
                    pp = arrayfun(@(x) get_param(x,'Position'), hh, 'UniformOutput', false);
                    candH = [candH; hh]; %#ok<AGROW>
                    candP = [candP; vertcat(pp{:})]; %#ok<AGROW>
                    candF = [candF; repmat({f}, numel(hh),1)]; %#ok<AGROW>
                end
            end
            if ~isempty(candH)
                d = hypot(candP(:,1)-pcEntry.Position(1), candP(:,2)-pcEntry.Position(2));
                [~,k] = min(d);
                f = candF{k};
                kind = lower(f);
                % 原错误代码：
                % idx = find(get_param(blockH,'PortHandles').(f) == candH(k), 1);

                % 修复后的代码：
                portHandles = get_param(blockH, 'PortHandles');
                idx = find(portHandles.(f) == candH(k), 1);
            end
        end
    catch
    end
end

function f = map_type_to_field(tp)
    switch tp
        case {'lconn','rconn','conn'}
            f = upper(tp);
        case 'conserving'
            f = 'Conn';
        case 'inport'
            f = 'Inport';
        case 'outport'
            f = 'Outport';
        otherwise
            f = 'Conn';
    end
end


function pcEntry = find_pc_entry_by_srcblock(dstBlockH, srcBlockH)
    % 在目标块的 PortConnectivity 中找到“上游块==srcBlockH”的条目
    pcEntry = struct();
    try
        pcAll = get_param(dstBlockH,'PortConnectivity');
        for k = 1:numel(pcAll)
            if isfield(pcAll(k),'SrcBlock') && ~isempty(pcAll(k).SrcBlock) && pcAll(k).SrcBlock ~= -1
                if pcAll(k).SrcBlock == srcBlockH
                    pcEntry = pcAll(k); return;
                end
            end
        end
    catch
        pcEntry = struct();
    end
end

function c = getfield_or_default_cell(S, field)
    if isstruct(S)
        try
            c = {S.(field)}';
        catch
            n = numel(S); c = repmat({''}, n, 1);
        end
    else
        c = {};
    end
end

function v = getfield_or_default_num(S, field)
    if isstruct(S)
        try
            v = [S.(field)]';
        catch
            n = numel(S); v = -1*ones(n,1);
        end
    else
        v = [];
    end
end

function pos = get_port_position(pcEntry, blockHandle)
    % why: 优先用 PortConnectivity 自带的屏幕坐标；若无则回退到端口句柄
    if isfield(pcEntry,'Position') && ~isempty(pcEntry.Position)
        pos = pcEntry.Position; % [x y]
        return;
    end
    % 回退：通过端口句柄拿位置（兼容 Inport/Outport/LConn/RConn 等）
    pos = [NaN NaN];
    try
        pnum = get_port_num(pcEntry);
        ph = get_param(blockHandle,'PortHandles');
        candidates = {};
        if isfield(ph,'Inport');   candidates{end+1} = ph.Inport;   end 
        if isfield(ph,'Outport');  candidates{end+1} = ph.Outport;  end 
        if isfield(ph,'LConn');    candidates{end+1} = ph.LConn;    end 
        if isfield(ph,'RConn');    candidates{end+1} = ph.RConn;    end 
        if isfield(ph,'Conn');     candidates{end+1} = ph.Conn;     end 
        for c = 1:numel(candidates)
            arr = candidates{c};
            if isempty(arr); continue; end
            if ~isnan(pnum) && pnum>=1 && pnum<=numel(arr)
                pos = get_param(arr(pnum),'Position');
                return;
            end
        end
    catch
        % 忽略个别不兼容端口；保持 [NaN NaN]
    end
end