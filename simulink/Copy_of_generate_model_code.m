
    model_name = 'SinglePhaseHBridge_Unipolar';
    
    % 获取模型中的 ??有块
    blocks = find_system(model_name, 'SearchDepth', 1);
    
    % 跳过模型本身
    blocks = blocks(2:end);
    for i = 1:length(blocks)
        disp(getfullname(blocks(i,1)));
    end
% 获取模型中所有 ? 线”对象（信号线）
% 说明：这 ??步覆盖普 ?? Simulink 信号线；但拿不到物理网络线（ ?? RLC 两端子线 ??
all_blocks = find_system(model_name, 'SearchDepth',1,'FindAll','on', ...
    'LookUnderMasks','on', 'FollowLinks','on', 'type','line');

% 创建 ??个空的结构体或单元格数组来存储连接关 ??
connectivity = {};
% 去重：源块全名|源端 ?? => 目标块全名|目标端口
conn_keys = containers.Map('KeyType','char','ValueType','logical');

% 遍历每条“信号线”，并 ? 归遍历分支（避免漏 ?? LineChildren ??
for i = 1:length(all_blocks)
    block = all_blocks(i,1);               % 实际 ?? line 句柄，沿用你的变量名

    src_port = get_param(block, 'SrcPortHandle');
    dst_ports = get_param(block, 'DstPortHandle'); % 保留你的原变量（后续我们用 ? 归统一收集 ??
    if src_port ~= -1
        src_block = get_param(src_port, 'Parent');
        src_block_name = get_param(src_block, 'Name');
        src_block_full = getfullname(src_block);
        src_port_num = get_param(src_port, 'PortNumber');

        % 递归获取该线及全部分支的目标端口
        visited_lines = [];
        [dst_blk_handles, dst_port_handles, visited_lines] = collect_all_dsts(block, visited_lines);

        % 处理 ??有目 ??
        for j = 1:length(dst_port_handles)
            if dst_port_handles(j) ~= -1
                dst_block = dst_blk_handles(j);
                dst_block_name = get_param(dst_block, 'Name');
                dst_block_full = getfullname(dst_block);
                dst_port_num = get_param(dst_port_handles(j), 'PortNumber');

                key = sprintf('%s|%d=>%s|%d', src_block_full, src_port_num, dst_block_full, dst_port_num);
                if ~isKey(conn_keys, key)
                    conn_keys(key) = true;
                    connectivity{end+1} = struct( ...
                        'Source',           src_block_name, ...
                        'SourcePath',       src_block_full, ...   % 新增：源块完整路 ??
                        'SourcePort',       src_port_num, ...
                        'Destination',      dst_block_name, ...
                        'DestinationPath',  dst_block_full, ...   % 新增：目标块完整路径
                        'DestinationPort',  dst_port_num, ...
                        'Origin',           'line' ...            % 可 ? ：来源标记（普通信号线 ??
                    );
                    disp(['Line from ', src_block_name, '(', num2str(src_port_num), ') to ', ...
                                      dst_block_name, '(', num2str(dst_port_num), ')']);
                end
            end
        end
    end
end

% ========= 新增：基 ?? PortConnectivity 的扫 ?? =========
all_blocks_pc = find_system(model_name, 'FindAll','on','LookUnderMasks','all','FollowLinks','on','Type','block');  % 不限制层级，确保掩模/链接下的端口也统 ??

for i = 1:length(all_blocks_pc)
    bh = all_blocks_pc(i);
    pc = get_param(bh, 'PortConnectivity');  % 每个端口的上下游连接信息（同时支持物理连接与信号线）

    % 遍历该块的每个端口连 ??
    for p = 1:numel(pc)
        % 1) 作为“输出端口 ? 一侧：本块 -> 下游块（包含信号与物理端口）
        if isfield(pc(p),'DstBlock') && ~isempty(pc(p).DstBlock) && all(pc(p).DstBlock ~= -1)
            src_block_full = getfullname(bh);
            src_block_name = get_param(bh, 'Name');
            src_port_num   = get_port_num(pc(p));   % 当前端口号（本块 ??侧）

            for d = 1:numel(pc(p).DstBlock)
                dst_bh         = pc(p).DstBlock(d);
                dst_block_full = getfullname(dst_bh);
                dst_block_name = get_param(dst_bh, 'Name');
                dst_port_num   = pc(p).DstPort(d);

                key = sprintf('%s|%d=>%s|%d', src_block_full, src_port_num, dst_block_full, dst_port_num);
                if ~isKey(conn_keys, key)
                    conn_keys(key) = true;
                    connectivity{end+1} = struct( ...
                        'Source',           src_block_name, ...
                        'SourcePath',       src_block_full, ...   % 新增
                        'SourcePort',       src_port_num, ...
                        'Destination',      dst_block_name, ...
                        'DestinationPath',  dst_block_full, ...   % 新增
                        'DestinationPort',  dst_port_num, ...
                        'Origin',           'pc' ...              % 可 ? ：来源标记（PortConnectivity ??
                    );
                    disp(['Line from ', src_block_name, '(', num2str(src_port_num), ') to ', ...
                                      dst_block_name, '(', num2str(dst_port_num), ') [PC]']);
                end
            end
        end

        % 2) 作为“输入端口 ? 一侧：上游 ?? -> 本块（物理端口常为双向，这里也补齐）
        if isfield(pc(p),'SrcBlock') && ~isempty(pc(p).SrcBlock) && pc(p).SrcBlock ~= -1
            src_bh          = pc(p).SrcBlock;
            src_block_full  = getfullname(src_bh);
            src_block_name  = get_param(src_bh, 'Name');
            % 上游块的端口号应 ?? SrcPort；没有则置为 -1 以便后续识别
            if isfield(pc(p),'SrcPort') && ~isempty(pc(p).SrcPort)
                src_port_num = pc(p).SrcPort;
            else
                src_port_num = -1;
            end

            dst_block_full  = getfullname(bh);
            dst_block_name  = get_param(bh, 'Name');
            dst_port_num    = get_port_num(pc(p));  % 本块 ??侧端口号

            key = sprintf('%s|%d=>%s|%d', src_block_full, src_port_num, dst_block_full, dst_port_num);
            if ~isKey(conn_keys, key)
                conn_keys(key) = true;
                connectivity{end+1} = struct( ...
                    'Source',           src_block_name, ...
                    'SourcePath',       src_block_full, ...      % 新增
                    'SourcePort',       src_port_num, ...
                    'Destination',      dst_block_name, ...
                    'DestinationPath',  dst_block_full, ...      % 新增
                    'DestinationPort',  dst_port_num, ...
                    'Origin',           'pc' ...                 % 可 ? ：来源标记（PortConnectivity ??
                );
                disp(['Line from ', src_block_name, '(', num2str(src_port_num), ') to ', ...
                                  dst_block_name, '(', num2str(dst_port_num), ') [PC]']);
            end
        end
    end
end
% 现在 connectivity 单元格数组包含了 ??有的 ??-目标连接对（信号 ?? + 物理网络线）
for i = 1:length(connectivity)
    disp(connectivity{1,i});
end
% ============================================================
% 采集 ??有元件（block）的位置信息（像素坐标，原点为画布左上）
% why: 后续可视 ??/导出 ??要几何信息；保持与连接表同一根模 ??
% ============================================================



% 拿到 ?? ?? block 句柄（包含掩 ??/链接/变体 ??
all_blocks_for_pos = all_blocks_pc;

% 结果容器：元件 ? 端口（可 ? ）、连线（可 ? ）
elements = struct('Path',{},'Name',{},'BlockType',{},'Orientation',{}, ...
                  'Position',{},'Center',{},'LibraryLink',{});
ports    = struct('BlockPath',{},'PortNumber',{},'PortType',{},'Position',{});
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
    ctr   = [(pos(1)+pos(3))/2, (pos(2)+pos(4))/2]; % 中心 ??
    lib   = '';
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
        'LibraryLink', lib ...
    );
end

% ========== B) 端口位置（可选，但强烈建议一起存 ?? ==========
% why: 即使某些“物理网络线”没有常 ?? line 对象，也能用端口坐标重建 ??
for i = 1:numel(all_blocks_for_pos)
    bh = all_blocks_for_pos(i);
    path = getfullname(bh);

    pc = get_param(bh,'PortConnectivity');  % 端口连接与几何信 ??
    for p = 1:numel(pc)
        pnum = get_port_num(pc(p));         % 兼容 PortNumber/Port
        ppos = get_port_position(pc(p), bh);% 优先 ?? pc(p).Position，必要时回 ??到句 ??
        ptyp = '';
        if isfield(pc(p),'Type') && ~isempty(pc(p).Type)
            ptyp = pc(p).Type;              % inport/outport/conserving等；若为空再粗略判断
        else
            if ~isempty(pc(p).DstBlock) && all(pc(p).DstBlock~=-1)
                ptyp = 'outport';
            elseif ~isempty(pc(p).SrcBlock) && pc(p).SrcBlock~=-1
                ptyp = 'inport';
            else
                ptyp = 'port';
            end
        end

        ports(end+1) = struct( ...          %#ok<AGROW>
            'BlockPath',  path, ...
            'PortNumber', pnum, ...
            'PortType',   ptyp, ...
            'Position',   ppos ...
        );
    end
end

% ========== C) 常规信号线的折线坐标（可选） ==========
% why: 普 ?? Simulink 信号线有 line 对象，能直接 ?? polyline 点；物理线若没有，则 ?? B) 的端点重 ??
all_lines = all_blocks;
for i = 1:numel(all_lines)
    lh = all_lines(i);
    srcH = get_param(lh,'SrcPortHandle');
    isRoot = srcH ~= -1;

    % 源与目标（注意：branch line 也有自己 ?? Dst 集合 ??
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
% elements  ->  ??有元件的几何信息（位 ??/中心/朝向/库链接）
% ports     ->  ??有端口的像素坐标（可用于重建物理连接 ??
% sigLines  ->  ??有常规 ? 信号线”的折线点集
% 可按 ??保存 ?? MAT/JSON，或 ?? connectivity 表进 ?? join
% =========
%% ========= 导出模型几何与连接信息（JSON / MAT / CSV，含完整路径 ?? =========
% why:
% - JSON：跨语言可读，包含完整嵌套结构（含完整路径，便于复原无歧义）
% - MAT：保留原始变量，便于 MATLAB 二次分析
% - CSV：便于快速查看与表格处理（元 ??/端口/连线三张表）

% 1) 基本输出目录与文件名（对模型名做清洗，确保可作为文件名）
if ~exist('model_root','var') || isempty(model_root), model_root = 'model'; end
model_tag = regexprep(char(model_root),'[^\w]','_');
out_dir   = fullfile(pwd, 'export_model_graph');
if ~exist(out_dir, 'dir'); mkdir(out_dir); end

% 2) 兜底：若变量缺失则给空结构，避免导出时报 ??
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

% 连接 ?? cell 转成 struct 数组，统 ??字段以 ? 包含完整路径 ? 为 ??
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

% 3) 规范化字段与数 ? （补全缺失字段；端口号 NaN -> -1 ??
wantedFields = {'Source','SourcePath','SourcePort','Destination','DestinationPath','DestinationPort','Origin'};
if isempty(conn)
    % 空连接表也需要确保字段存在，便于 JSON/CSV 写出
    conn = cell2struct(cell(size(wantedFields)), wantedFields, 2);
    conn(1) = []; % 置空但保留字段定 ??
else
    % 为所有连接补齐字段，保持 ??致 ??
    for i = 1:numel(conn)
        % 字段缺失则补 ??
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
        % 确保字符串为 char（避 ?? string 编码差异 ??
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

% 4) 组织总对象并 ?? JSON（完整信息，包含完整路径 ??
%graph = struct();
%graph.model       = char(model_root);
%graph.timestamp   = char(datetime('now','Format','yyyy-MM-dd HH:mm:ss'));
%graph.counts      = struct('elements',numel(elements),'ports',numel(ports), ...
                          % 'connections',numel(conn),'lines',numel(sigLines));
%graph.elements    = elements;
%graph.ports       = ports;
%graph.connections = conn;
%graph.lines       = sigLines;

%json_path = fullfile(out_dir, sprintf('%s_graph.json', model_tag));
%txt = jsonencode(graph, 'PrettyPrint', true);   % MATLAB 会将 NaN 编码 ?? null
%fid = fopen(json_path,'w');
%assert(fid~=-1, '无法创建文件: %s', json_path);
%fwrite(fid, txt, 'char');
%fclose(fid);

% 5) 另存 MAT（保留原始结构，便于后续 MATLAB 直接加载 ??
mat_path = fullfile(out_dir, sprintf('%s_graph.mat', model_tag));
save(mat_path, 'graph','elements','ports','sigLines','conn','connectivity','-v7.3');

% 6) 再导出三 ?? CSV（展 ??关键几何字段，便于快速查看）
% 6.1 元件 CSV：展 ?? Position/Center
if ~isempty(elements)
    names  = {elements.Name}';
    paths  = {elements.Path}';
    types  = {elements.BlockType}';
    oris   = {elements.Orientation}';
    libs   = {elements.LibraryLink}';
    posMat = vertcat(elements.Position);     % Nx4: [L T R B]
    ctrMat = vertcat(elements.Center);       % Nx2: [Cx Cy]
    T_e = table(names, paths, types, oris, libs, ...
        posMat(:,1), posMat(:,2), posMat(:,3), posMat(:,4), ...
        ctrMat(:,1), ctrMat(:,2), ...
        'VariableNames', {'Name','Path','BlockType','Orientation','LibraryLink', ...
                          'Left','Top','Right','Bottom','CenterX','CenterY'});
    writetable(T_e, fullfile(out_dir, sprintf('%s_elements.csv', model_tag)));
end

% 6.2 端口 CSV：展 ?? Position
if ~isempty(ports)
    bpaths = {ports.BlockPath}';
    pnums  = [ports.PortNumber]';
    ptypes = {ports.PortType}';
    ppos   = vertcat(ports.Position);        % Nx2: [x y]
    T_p = table(bpaths, pnums, ptypes, ppos(:,1), ppos(:,2), ...
        'VariableNames', {'BlockPath','PortNumber','PortType','X','Y'});
    writetable(T_p, fullfile(out_dir, sprintf('%s_ports.csv', model_tag)));
end

% 6.3 连线 CSV：源/目标 + 完整路径 + 端口 ?? + 来源
if ~isempty(conn)
    srcs    = {conn.Source}';
    srcps   = {conn.SourcePath}';
    sps     = [conn.SourcePort]';
    dsts    = {conn.Destination}';
    dstps   = {conn.DestinationPath}';
    dps     = [conn.DestinationPort]';
    origins = {conn.Origin}';
    T_c = table(srcs, srcps, sps, dsts, dstps, dps, origins, ...
        'VariableNames', {'Source','SourcePath','SourcePort','Destination','DestinationPath','DestinationPort','Origin'});
    writetable(T_c, fullfile(out_dir, sprintf('%s_connections.csv', model_tag)));
end

fprintf('导出完成：\n JSON  -> %s\n MAT   -> %s\n CSVs  -> %s\n', json_path, mat_path, out_dir);

% =========
% 工具函数： ? 归收集“某条线及其 ??有分支 ? 的目标端口（仅对普通信号线有效 ??
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
                dstBlks  = [dstBlks(:);  cBlks(:) ]; 
                dstPorts = [dstPorts(:); cPorts(:)]; 
            end
        end
    end
end

function n = get_port_num(pcEntry)
    % 兼容不同版本 ?? PortConnectivity 字段差异
    if isfield(pcEntry, 'PortNumber') && ~isempty(pcEntry.PortNumber)
        n = pcEntry.PortNumber;
    elseif isfield(pcEntry, 'Port') && ~isempty(pcEntry.Port)
        n = pcEntry.Port;
    else
        % 某些特殊端口（如物理端口）可能没有编号，这里返回 NaN 以示区分
        n = NaN;
    end
end

function pos = get_port_position(pcEntry, blockHandle)
    % why: 优先 ?? PortConnectivity 自带的屏幕坐标；若无则回 ??到端口句 ??
    if isfield(pcEntry,'Position') && ~isempty(pcEntry.Position)
        pos = pcEntry.Position; % [x y]
        return;
    end
    % 回 ??： ? 过端口句柄拿位置（兼容 Inport/Outport/LConn/RConn 等）
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