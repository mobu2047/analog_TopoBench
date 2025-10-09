
    % ���ļ�׼��д�����
 model_name = 'untitled1';
 open_system(model_name);
  
    
    % ��ȡģ���е����п�
 blocks = find_system(model_name, 'SearchDepth', 1);
    
    % ����ģ�ͱ���
 blocks = blocks(2:end);
    for i = 1:length(blocks)
        disp(getfullname(blocks(i,1)));
    end
% ��ȡģ�������С��ߡ������ź��ߣ�
% ˵������һ��������ͨ Simulink �ź��ߣ����ò������������ߣ��� RLC �������ߣ�
all_blocks = find_system(model_name, 'SearchDepth',1,'FindAll','on', ...
    'LookUnderMasks','on', 'FollowLinks','on', 'type','line');

% ����һ���յĽṹ���Ԫ���������洢���ӹ�ϵ
connectivity = {};
% ȥ�أ�Դ��ȫ��|Դ�˿� => Ŀ���ȫ��|Ŀ��˿�
conn_keys = containers.Map('KeyType','char','ValueType','logical');

% ����ÿ�����ź��ߡ������ݹ������֧������©�� LineChildren��
for i = 1:length(all_blocks)
    block = all_blocks(i,1);               % ʵ���� line �����������ı�����

    src_port = get_param(block, 'SrcPortHandle');
    dst_ports = get_param(block, 'DstPortHandle'); % �������ԭ���������������õݹ�ͳһ�ռ���
    if src_port ~= -1
        src_block = get_param(src_port, 'Parent');
        src_block_name = get_param(src_block, 'Name');
        src_block_full = getfullname(src_block);
        src_port_num = get_param(src_port, 'PortNumber');

        % �ݹ��ȡ���߼�ȫ����֧��Ŀ��˿�
        visited_lines = [];
        [dst_blk_handles, dst_port_handles, visited_lines] = collect_all_dsts(block, visited_lines);

        % ��������Ŀ��
        for j = 1:length(dst_port_handles)
            if dst_port_handles(j) ~= -1
                dst_block = dst_blk_handles(j);
                dst_block_name = get_param(dst_block, 'Name');
                dst_block_full = getfullname(dst_block);
                dst_port_num = get_param(dst_port_handles(j), 'PortNumber');

                % ��������¼�˿������롰�ڸ������������е����������ȶ��ҿɸ��֣�
                [src_kind, src_index] = kind_and_index_by_handle(src_block, src_port);
                [dst_kind, dst_index] = kind_and_index_by_handle(dst_block, dst_port_handles(j));

                key = sprintf('%s|%d=>%s|%d', src_block_full, src_port_num, dst_block_full, dst_port_num);
                if ~isKey(conn_keys, key)
                    conn_keys(key) = true;
                    connectivity{end+1} = struct( ...
                        'Source',           src_block_name, ...
                        'SourcePath',       src_block_full, ...   % ������Դ������·��
                        'SourcePort',       src_port_num, ...
                        'SourcePortKind',   src_kind, ...        % ������Դ�˿����ࣨOutport/Inport/...��
                        'SourcePortIndex',  src_index, ...       % �������ڸ����������е�����
                        'Destination',      dst_block_name, ...
                        'DestinationPath',  dst_block_full, ...   % ������Ŀ�������·��
                        'DestinationPort',  dst_port_num, ...
                        'DestinationPortKind',  dst_kind, ...    % ����
                        'DestinationPortIndex', dst_index, ...   % ����
                        'Origin',           'line' ...            % ��ѡ����Դ��ǣ���ͨ�ź��ߣ�
                    );
                    disp(['Line from ', src_block_name, '(', num2str(src_port_num), ') to ', ...
                                      dst_block_name, '(', num2str(dst_port_num), ')']);
                end
            end
        end
    end
end

% ========= ���������� PortConnectivity ��ɨ�� =========
all_blocks_pc = find_system(model_name, 'FindAll','on','LookUnderMasks','all','FollowLinks','on','Type','block');  % �����Ʋ㼶��ȷ����ģ/�����µĶ˿�Ҳͳ��

for i = 1:length(all_blocks_pc)
    bh = all_blocks_pc(i);
    pc = get_param(bh, 'PortConnectivity');  % ÿ���˿ڵ�������������Ϣ��ͬʱ֧�������������ź��ߣ�

    % �����ÿ��ÿ���˿�����
    for p = 1:numel(pc)
        % 1) ��Ϊ������˿ڡ�һ�ࣺ���� -> ���ο飨�����ź�������˿ڣ�
        if isfield(pc(p),'DstBlock') && ~isempty(pc(p).DstBlock) && all(pc(p).DstBlock ~= -1)
            src_block_full = getfullname(bh);
            src_block_name = get_param(bh, 'Name');
            src_port_num   = get_port_num(pc(p));   % ��ǰ�˿ںţ�����һ�ࣩ
            % �������˿��������ڸ��������������
            [src_kind2, src_index2] = kind_and_index_by_pc(bh, pc(p));

            for d = 1:numel(pc(p).DstBlock)
                dst_bh         = pc(p).DstBlock(d);
                dst_block_full = getfullname(dst_bh);
                dst_block_name = get_param(dst_bh, 'Name');
                % Ŀ��˿ں��������������壬�����Ϊ����Ŀ���PC�з��� SrcBlock==bh ����Ŀ��
                dst_pc_entry   = find_pc_entry_by_srcblock(dst_bh, bh);
                dst_port_num   = get_port_num(dst_pc_entry);  % ��ȡ��Ϊ���򷵻� NaN������ͳһΪ -1
                [dst_kind2, dst_index2] = kind_and_index_by_pc(dst_bh, dst_pc_entry);

                key = sprintf('%s|%d=>%s|%d', src_block_full, src_port_num, dst_block_full, dst_port_num);
                if ~isKey(conn_keys, key)
                    conn_keys(key) = true;
                    connectivity{end+1} = struct( ...
                        'Source',           src_block_name, ...
                        'SourcePath',       src_block_full, ...   % ����
                        'SourcePort',       src_port_num, ...
                        'SourcePortKind',   src_kind2, ...
                        'SourcePortIndex',  src_index2, ...
                        'Destination',      dst_block_name, ...
                        'DestinationPath',  dst_block_full, ...   % ����
                        'DestinationPort',  dst_port_num, ...
                        'DestinationPortKind',  dst_kind2, ...
                        'DestinationPortIndex', dst_index2, ...
                        'Origin',           'pc' ...              % ��ѡ����Դ��ǣ�PortConnectivity��
                    );
                    disp(['Line from ', src_block_name, '(', num2str(src_port_num), ') to ', ...
                                      dst_block_name, '(', num2str(dst_port_num), ') [PC]']);
                end
            end
        end

        % 2) ��Ϊ������˿ڡ�һ�ࣺ���ο� -> ���飨����˿ڳ�Ϊ˫������Ҳ���룩
        if isfield(pc(p),'SrcBlock') && ~isempty(pc(p).SrcBlock) && pc(p).SrcBlock ~= -1
            src_bh          = pc(p).SrcBlock;
            src_block_full  = getfullname(src_bh);
            src_block_name  = get_param(src_bh, 'Name');
            % ���ο�Ķ˿ں�Ӧȡ SrcPort��û������Ϊ -1 �Ա����ʶ��
            if isfield(pc(p),'SrcPort') && ~isempty(pc(p).SrcPort)
                src_port_num = pc(p).SrcPort;
            else
                src_port_num = -1;
            end
            [src_kind3, src_index3] = kind_and_index_by_pc(src_bh, pc(p));

            dst_block_full  = getfullname(bh);
            dst_block_name  = get_param(bh, 'Name');
            dst_port_num    = get_port_num(pc(p));  % ����һ��˿ں�
            [dst_kind3, dst_index3] = kind_and_index_by_pc(bh, pc(p));

            key = sprintf('%s|%d=>%s|%d', src_block_full, src_port_num, dst_block_full, dst_port_num);
            if ~isKey(conn_keys, key)
                conn_keys(key) = true;
                connectivity{end+1} = struct( ...
                    'Source',           src_block_name, ...
                    'SourcePath',       src_block_full, ...      % ����
                    'SourcePort',       src_port_num, ...
                    'SourcePortKind',   src_kind3, ...
                    'SourcePortIndex',  src_index3, ...
                    'Destination',      dst_block_name, ...
                    'DestinationPath',  dst_block_full, ...      % ����
                    'DestinationPort',  dst_port_num, ...
                    'DestinationPortKind',  dst_kind3, ...
                    'DestinationPortIndex', dst_index3, ...
                    'Origin',           'pc' ...                 % ��ѡ����Դ��ǣ�PortConnectivity��
                );
                disp(['Line from ', src_block_name, '(', num2str(src_port_num), ') to ', ...
                                  dst_block_name, '(', num2str(dst_port_num), ') [PC]']);
            end
        end
    end
end
% ���� connectivity ��Ԫ��������������е�Դ-Ŀ�����Ӷԣ��ź��� + ���������ߣ�
for i = 1:length(connectivity)
    disp(connectivity{1,i});
end
% ============================================================
% �ɼ�����Ԫ����block����λ����Ϣ���������꣬ԭ��Ϊ�������ϣ�
% why: �������ӻ�/������Ҫ������Ϣ�����������ӱ�ͬһ��ģ��
% ============================================================



% �õ����� block �����������ģ/����/���壩
all_blocks_for_pos = all_blocks_pc;

% ���������Ԫ�����˿ڣ���ѡ�������ߣ���ѡ��
elements = struct('Path',{},'Name',{},'BlockType',{},'Orientation',{}, ...
                  'Position',{},'Center',{},'LibraryLink',{}, ...
                  'Mirror',{},'Rotation',{}, ...
                  'GotoTag',{},'GotoVisibility',{},'FromTag',{});  % ���������ξ���/��ת��Goto/From��ǩ/�ɼ���
ports    = struct('BlockPath',{},'PortNumber',{},'PortType',{},'Position',{}, ...
                  'RelPos',{},'Side',{});      % �������˿������������
sigLines = struct('Handle',{},'IsRoot',{},'SrcPath',{},'DstPaths',{},'Points',{});

% ========== A) Ԫ��λ�� ==========
for i = 1:numel(all_blocks_for_pos)
    bh = all_blocks_for_pos(i);

    % Ԫ��������Ϣ
    path  = getfullname(bh);
    name  = get_param(bh,'Name');
    btype = get_param(bh,'BlockType');              % ĳЩ��ģ�����Ϊ���ַ���
    ori   = get_param(bh,'Orientation');            % right/left/up/down
    pos   = get_param(bh,'Position');               % [L T R B]
    ctr   = [(pos(1)+pos(3))/2, (pos(2)+pos(4))/2]; % ���ĵ�
    lib   = '';
    % �����������ܶ�ȡ����/��ת��Ϣ���������п鶼����Щ���ԣ�
    mir   = '';
    rot   = '';
    try, mir = get_param(bh,'BlockMirror'); end
    try, rot = get_param(bh,'BlockRotation'); end
    % ��������Ϊ Goto/From����¼���ǩ�������ؽ�ʱ����Ĭ�ϱ�ǩ��ͻ
    gtag = '';
    gvis = '';
    ftag = '';
    if strcmpi(btype,'Goto')
        try, gtag = get_param(bh,'GotoTag'); end
        try, gvis = get_param(bh,'TagVisibility'); catch, gvis = ''; end
    elseif strcmpi(btype,'From')
        try, ftag = get_param(bh,'GotoTag'); end % From ��ͬ��ʹ�� GotoTag �ֶ�
    end
    if strcmp(get_param(bh,'LinkStatus'),'resolved')
        lib = get_param(bh,'ReferenceBlock');       % ���Կ������·��
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

% ========== B) �˿�λ�ã���ѡ����ǿ�ҽ���һ��棩 ==========
% why: ��ʹĳЩ�����������ߡ�û�г��� line ����Ҳ���ö˿������ؽ���
for i = 1:numel(all_blocks_for_pos)
    bh = all_blocks_for_pos(i);
    path = getfullname(bh);

    pc = get_param(bh,'PortConnectivity');  % �˿������뼸����Ϣ
    for p = 1:numel(pc)
        pnum = get_port_num(pc(p));         % ���� PortNumber/Port
        ppos = get_port_position(pc(p), bh);% ������ pc(p).Position����Ҫʱ���˵����
        ptyp = '';
        if isfield(pc(p),'Type') && ~isempty(pc(p).Type)
            ptyp = pc(p).Type;              % inport/outport/conserving ��
        else
            if ~isempty(pc(p).DstBlock) && all(pc(p).DstBlock~=-1)
                ptyp = 'outport';
            elseif ~isempty(pc(p).SrcBlock) && pc(p).SrcBlock~=-1
                ptyp = 'inport';
            else
                ptyp = 'port';
            end
        end
        % ��һ������ PortHandles ��ȷʶ�� LConn/RConn/Conn��ĳЩ���� pc(p).Type Ϊ�գ�
        try
            ph = get_param(bh,'PortHandles');
            typeNames = {'LConn','RConn','Conn','Inport','Outport'};  % ������ǰ
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
                % ��ԭ����Ϊ��/port/inout�����ü��������
                if isempty(ptyp) || strcmpi(ptyp,'port') || strcmpi(ptyp,'inout')
                    ptyp = detected;
                end
            end
        catch
        end
        % �������˿���Կ�Ĺ�һ���������������ؽ�ʱ����˿�ƥ����Ƚ���
        rel = [NaN NaN];
        side = '';
        try
            if ~any(isnan(ppos))
                w = max(1, (pos(3)-pos(1))); h = max(1, (pos(4)-pos(2)));
                rel = [(ppos(1)-pos(1))/w, (ppos(2)-pos(2))/h];
                % ������߽�ľ����жϲ��
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

% ========== C) �����ź��ߵ��������꣨��ѡ�� ==========
% why: ��ͨ Simulink �ź����� line ������ֱ���� polyline �㣻��������û�У����� B) �Ķ˵��ؽ�
all_lines = all_blocks;
for i = 1:numel(all_lines)
    lh = all_lines(i);
    srcH = get_param(lh,'SrcPortHandle');
    isRoot = srcH ~= -1;

    % Դ��Ŀ�꣨ע�⣺branch line Ҳ���Լ��� Dst ���ϣ�
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

    pts = get_param(lh,'Points'); % Nx2 ��������
    sigLines(end+1) = struct( ... %#ok<AGROW>
        'Handle',  lh, ...
        'IsRoot',  isRoot, ...
        'SrcPath', srcPath, ...
        'DstPaths',{dstPaths}, ...
        'Points',  pts ...
    );
end

% ������ӵ�У�
% elements  -> ����Ԫ���ļ�����Ϣ��λ��/����/����/�����ӣ�
% ports     -> ���ж˿ڵ��������꣨�������ؽ��������ӣ�
% sigLines  -> ���г��桰�ź��ߡ������ߵ㼯
% �ɰ��豣��Ϊ MAT/JSON������ connectivity ����� join
% =========
%% ========= ����ģ�ͼ�����������Ϣ��JSON / MAT / CSV��������·���� =========
% why:
% - JSON�������Կɶ�����������Ƕ�׽ṹ��������·�������ڸ�ԭ�����壩
% - MAT������ԭʼ���������� MATLAB ���η���
% - CSV�����ڿ��ٲ鿴������Ԫ��/�˿�/�������ű�

% 1) �������Ŀ¼���ļ�������ģ��������ϴ��ȷ������Ϊ�ļ�����
if ~exist('model_root','var') || isempty(model_root), model_root = 'model'; end
model_tag = regexprep(char(model_root),'[^\w]','_');
out_dir   = fullfile(pwd, 'export_model_graph');
if ~exist(out_dir, 'dir'); mkdir(out_dir); end

% 2) ���ף�������ȱʧ����սṹ�����⵼��ʱ����
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

% ���Ӵ� cell ת�� struct ���飬ͳһ�ֶ��ԡ���������·����Ϊ׼
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

% 3) �淶���ֶ�����ֵ����ȫȱʧ�ֶΣ��˿ں� NaN -> -1��
wantedFields = {'Source','SourcePath','SourcePort','Destination','DestinationPath','DestinationPort','Origin'};
if isempty(conn)
    % �����ӱ�Ҳ��Ҫȷ���ֶδ��ڣ����� JSON/CSV д��
    conn = cell2struct(cell(size(wantedFields)), wantedFields, 2);
    conn(1) = []; % �ÿյ������ֶζ���
else
    % Ϊ�������Ӳ����ֶΣ�����һ����
    for i = 1:numel(conn)
        % �ֶ�ȱʧ�򲹿�
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
        % �˿ںű�׼��
        if ~isfield(conn,'SourcePort') || isempty(conn(i).SourcePort) || isnan(conn(i).SourcePort)
            conn(i).SourcePort = -1;
        end
        if ~isfield(conn,'DestinationPort') || isempty(conn(i).DestinationPort) || isnan(conn(i).DestinationPort)
            conn(i).DestinationPort = -1;
        end
        % ȷ���ַ���Ϊ char������ string ������죩
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

% 4) ��֯�ܶ���д JSON��������Ϣ����������·����
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
txt = jsonencode(graph, 'ConvertInfAndNaN', true);   % MATLAB �Ὣ NaN ����Ϊ null
fid = fopen(json_path,'w');
assert(fid~=-1, '�޷������ļ�: %s', json_path);
fwrite(fid, txt, 'char');
fclose(fid);

% 5) ��� MAT������ԭʼ�ṹ�����ں��� MATLAB ֱ�Ӽ��أ�
mat_path = fullfile(out_dir, sprintf('%s_graph.mat', model_tag));
save(mat_path, 'graph','elements','ports','sigLines','conn','connectivity','-v7.3');

% 6) �ٵ������� CSV��չ���ؼ������ֶΣ����ڿ��ٲ鿴��
% 6.1 Ԫ�� CSV��չ�� Position/Center
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

% 6.2 �˿� CSV��չ�� Position
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

% 6.3 ���� CSV��Դ/Ŀ�� + ����·�� + �˿ں� + ��Դ
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

fprintf('������ɣ�\n JSON  -> %s\n MAT   -> %s\n CSVs  -> %s\n', json_path, mat_path, out_dir);
close_system(model_name);
% =========
% ���ߺ������ݹ��ռ���ĳ���߼������з�֧����Ŀ��˿ڣ�������ͨ�ź�����Ч��
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
    % ���ݲ�ͬ�汾�� PortConnectivity �ֶβ���
    if isfield(pcEntry, 'PortNumber') && ~isempty(pcEntry.PortNumber)
        n = pcEntry.PortNumber;
    elseif isfield(pcEntry, 'Port') && ~isempty(pcEntry.Port)
        n = pcEntry.Port;
    else
        % ĳЩ����˿ڣ�������˿ڣ�����û�б�ţ����ﷵ�� NaN ��ʾ����
        n = NaN;
    end
end

function [kind, idx] = kind_and_index_by_handle(blockH, portH)
    % ͨ�� PortHandles ����˿�������һ�༰�ڸ������������
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
    % ͨ�� pcEntry �� PortHandles �ƶ϶˿����������������� pc ��Դ��
    kind = '';
    idx  = -1;
    try
        ph = get_param(blockH,'PortHandles');
        % ����ʹ�� Type
        if isfield(pcEntry,'Type') && ~isempty(pcEntry.Type)
            switch lower(pcEntry.Type)
                case {'inport','outport','lconn','rconn','conn','conserving'}
                    f = map_type_to_field(lower(pcEntry.Type));
                    if isfield(ph,f) && ~isempty(ph.(f))
                        kind = lower(f); idx = 1; % �޷���ȷʱ���� 1 ռλ
                        return;
                    end
            end
        end
        % ���ˣ���������ƥ������˿ھ����������������������
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
                % ԭ������룺
                % idx = find(get_param(blockH,'PortHandles').(f) == candH(k), 1);

                % �޸���Ĵ��룺
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
    % ��Ŀ���� PortConnectivity ���ҵ������ο�==srcBlockH������Ŀ
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
    % why: ������ PortConnectivity �Դ�����Ļ���ꣻ��������˵��˿ھ��
    if isfield(pcEntry,'Position') && ~isempty(pcEntry.Position)
        pos = pcEntry.Position; % [x y]
        return;
    end
    % ���ˣ�ͨ���˿ھ����λ�ã����� Inport/Outport/LConn/RConn �ȣ�
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
        % ���Ը��𲻼��ݶ˿ڣ����� [NaN NaN]
    end
end