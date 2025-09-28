
    % ���ļ�׼��д�����
    model_name = untitled1;

  
    
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
     'FollowLinks','on', 'type','line');

% ����һ���յĽṹ���Ԫ���������洢���ӹ�ϵ
connectivity = {};
% ȥ�أ�Դ��ȫ��|Դ�˿� => Ŀ���ȫ��|Ŀ��˿�
%conn_keys = containers.Map('KeyType','char','ValueType','logical');

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
                %if ~isKey(conn_keys, key)
                    %conn_keys(key) = true;
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
               %end
            end
        end
    end
end

% ========= ���������� PortConnectivity ��ɨ�� =========
all_blocks_pc = find_system(model_name, 'FindAll','on','FollowLinks','on','Type','block');  % �����Ʋ㼶��ȷ����ģ/�����µĶ˿�Ҳͳ��

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
                %if ~isKey(conn_keys, key)
                    %conn_keys(key) = true;
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
                %end
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
            %if ~isKey(conn_keys, key)
                %conn_keys(key) = true;
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
            %end
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

% ==================����ѡ������ģ�����������ã������ڼ���/����==================
% why:
% - �����������߲����ԡ�ֱ�ӷ��桱������ģ������������Ի�����
% - ���������е����߼�����ö������ռ����������迪�أ������½��ű�
% how:
% - ģ�Ͳ����������÷���/������ؼ����������ץȡ��
% - �������ʹ�� DialogParameters �б����� get_param����ܱ���̬/������ֶ�
% - ֵһ�����л�Ϊ�ַ��������� set_param �ط�
ENABLE_PARAM_EXPORT = true;  % ��Ϊ false ����ȫ����������������Ӱ����������
% ����ʽ�����������ã��������/������ + ���Թ���
PARAM_FILTER = get_param_filter_config();
params = struct('model', struct(), 'blocks', struct('Path',{},'BlockType',{},'MaskType',{},'DialogParams',{}));
if ENABLE_PARAM_EXPORT
    try
        % ʶ����ʵģ�͸�����������ʹ��ռλ model_root ���� get_param ʧ��
        % why: bdroot ���ص�ǰģ��������Ĭ�� model_root ��ʵ�ʲ�һ��ʱ�ᵼ���ղ���
        root_for_params = '';
        try
            if exist('all_blocks_pc','var') && ~isempty(all_blocks_pc)
                root_for_params = char(bdroot(all_blocks_pc(1)));
            end
        catch
        end
        if isempty(root_for_params)
            try, root_for_params = char(bdroot); catch, root_for_params = char(model_root); end
        end

        % �ռ�ģ�ͼ������������/ʱ�䲽�ȣ����ԡ����ڼ���¼��Ϊ׼
        params.model = collect_model_params(root_for_params, PARAM_FILTER);

        % �ռ��鼶�Ի�����
        params.blocks = collect_block_params(all_blocks_pc, PARAM_FILTER);

        % ע�뵽 graph�����ڵ��ļ���ԭ���������� graph.model Ϊ��ʵ����
        graph.parameters = params;
        graph.model = char(root_for_params);
    catch ME
        warning('���������������⣬����������ע�룺%s', ME.message);
    end
end

json_path = fullfile(out_dir, sprintf('%s_graph.json', model_tag));
txt = jsonencode(graph, 'PrettyPrint', true);   % MATLAB �Ὣ NaN ����Ϊ null
fid = fopen(json_path,'w');
assert(fid~=-1, '�޷������ļ�: %s', json_path);
fwrite(fid, txt, 'char');
fclose(fid);

% 5) ��� MAT������ԭʼ�ṹ�����ں��� MATLAB ֱ�Ӽ��أ�
mat_path = fullfile(out_dir, sprintf('%s_graph.mat', model_tag));
save(mat_path, 'graph','elements','ports','sigLines','conn','connectivity','-v7.3');

% �������˲���������ͬʱ�������̣����ڵ������/�ȶ�/�汾����
if ENABLE_PARAM_EXPORT
    try
        params_json_path = fullfile(out_dir, sprintf('%s_params.json', model_tag));
        ptxt = jsonencode(params, 'PrettyPrint', true);
        fidp = fopen(params_json_path,'w');
        assert(fidp~=-1, '�޷������ļ�: %s', params_json_path);
        fwrite(fidp, ptxt, 'char'); fclose(fidp);
    catch ME
        warning('������������ʧ�ܣ�%s', ME.message);
    end
end

% 6) �� MAT ������ CSV���ϸ��� MAT �е�����Ϊ׼��ȷ�� CSV �� MAT һ�£�
% why:
% - ���⡰�ڴ��б����������Ķ������� CSV �� MAT ��һ��
% - ͳһ��ڣ����ȴӸոձ���� mat_path ���� graph/elements/ports/conn
try
    S_csv = load(mat_path); % ֻ������������������ͬ����������֤һ����
catch ME
    warning('�� MAT ���ص����ṹʧ�ܣ������� CSV ���ɣ�%s', ME.message);
    S_csv = struct();
end

% ���������� CSV ���������
try
    if isfield(S_csv,'graph') && isstruct(S_csv.graph)
        conn = S_csv.conn;
        T = struct2table(conn);
        writetable(T, 'export_model_graph/model_connections.csv', 'WriteVariableNames', true, 'QuoteStrings', true,'Encoding', 'UTF-8');      
        elements = S_csv.elements;
        T = struct2table(elements);
        writetable(T, 'export_model_graph/model_elements.csv', 'WriteVariableNames', true, 'QuoteStrings', true,'Encoding', 'UTF-8'); 
        ports = S_csv.ports;
        T = struct2table(ports);
        writetable(T, 'export_model_graph/model_ports.csv', 'WriteVariableNames', true, 'QuoteStrings', true,'Encoding', 'UTF-8'); 
        siglines = S_csv.sigLines;
        T = struct2table(siglines);
        writetable(T, 'export_model_graph/model_siglines.csv', 'WriteVariableNames', true, 'QuoteStrings', true,'Encoding', 'UTF-8'); 
     end
  end






fprintf('������ɣ�\n JSON  -> %s\n MAT   -> %s\n CSVs  -> %s\n', json_path, mat_path, out_dir);

if ENABLE_PARAM_EXPORT
    fprintf('�����ѵ�������%s\n', out_dir);
end

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
                idx  = find(get_param(blockH,'PortHandles').(f) == candH(k), 1);
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

function pcEntry = get_pc_entry_for_port(blockH, portNum)
    % ��Ŀ���� PortConnectivity ���ҵ���˿ں�ƥ�����Ŀ
    pcEntry = struct();
    try
        pcAll = get_param(blockH,'PortConnectivity');
        for k = 1:numel(pcAll)
            if isfield(pcAll(k),'PortNumber') && isequal(pcAll(k).PortNumber, portNum)
                pcEntry = pcAll(k); return;
            elseif isfield(pcAll(k),'Port') && isequal(pcAll(k).Port, portNum)
                pcEntry = pcAll(k); return;
            end
        end
    catch
        pcEntry = struct();
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

% =========
% ���ߺ����������ɼ������̣�
% =========
function m = collect_model_params(model_root, PARAM_FILTER)
    % why: �ɼ����������/ʱ��/���ݵ��뵼���ȹؼ��������©ֱ�ӷ�����������
    % how: ���һ���ѡ�����������ڼ���¼��ֵͳһתΪ�ַ��������� set_param �ط�
    % ����ͨ������׷��/�ü�
    defaultCand = {'StartTime','StopTime','SolverType','Solver','FixedStep','AbsTol','RelTol', ...
            'StrictBusMsg','MinStepSize','MaxStepSize','MaxConsecutiveZCs','SignalLogging','ReturnWorkspaceOutputs', ...
            'DataTypeOverride','MinMaxOverflowLogging','AlgebraicLoopMsg','SimCompilerOptimization','InlineParams'};
    cand = merge_name_list(defaultCand, PARAM_FILTER.model.white, PARAM_FILTER.model.black);
    m = struct();
    try
        root = char(model_root);
        % model ��������ռ�
        for i = 1:numel(cand)
            pname = cand{i};
            try
                val = get_param(root, pname);
                if param_allowed_by_attr([] , pname, val, PARAM_FILTER.model.attr_black)
                    m.(pname) = param_value_to_string(val);
                end
            catch
            end
        end
    catch
    end
end

function blocks = collect_block_params(all_blocks_pc, PARAM_FILTER)
    % why: ʹ�� DialogParameters �ܸ��Ǵ������������� Mask ������������ץȡֻ��/����ֶ�
    blocks = struct('Path',{},'BlockType',{},'MaskType',{},'DialogParams',{});
    for i = 1:numel(all_blocks_pc)
        bh = all_blocks_pc(i);
        try
            path  = getfullname(bh);
            btype = get_param(bh,'BlockType');
            mtype = '';
            try, mtype = get_param(bh,'MaskType'); catch, mtype = ''; end
            dp = struct();
            % DialogParameters: struct�������ֶ���Ϊ�����õĶԻ������
            dlg = struct();
            try, dlg = get_param(bh,'DialogParameters'); catch, dlg = struct(); end
            if ~isempty(dlg)
                names = fieldnames(dlg);
                for k = 1:numel(names)
                    pname = names{k};
                    % ��������/���ߵȼ����ظ���
                    if any(strcmpi(pname, {'Position','PortConnectivity','LineHandles'}))
                        continue;
                    end
                    % ���Ƽ����ˣ���/��������
                    if ~name_allowed(pname, PARAM_FILTER.block.white, PARAM_FILTER.block.black)
                        continue;
                    end
                    try
                        % ���Լ����ˣ�ObjectParameters ���ṩ���ԣ�read-only/no-set/deprecated �ȣ�
                        if param_allowed_on_block(bh, pname, PARAM_FILTER.block.attr_black)
                            val = get_param(bh, pname);
                            dp.(pname) = param_value_to_string(val);
                        end
                    catch
                        % ��Щ���ӿ��������ֻ�����ܱ�����ƣ����Լ���
                    end
                end
            end

            % ���䣺Mask ֵ���������� DialogParameters δ���ǣ�
            try
                mnames = get_param(bh,'MaskNames');
                mvals  = get_param(bh,'MaskValues');
                if iscell(mnames) && iscell(mvals) && numel(mnames)==numel(mvals)
                    for t = 1:numel(mnames)
                        key = char(mnames{t});
                        % ���Ƽ�����
                        if ~isfield(dp, key) && name_allowed(key, PARAM_FILTER.mask.white, PARAM_FILTER.mask.black)
                            dp.(key) = param_value_to_string(mvals{t});
                        end
                    end
                end
            catch
            end

            blocks(end+1) = struct('Path',path,'BlockType',btype,'MaskType',mtype,'DialogParams',dp); %#ok<AGROW>
        catch
        end
    end
end

function s = param_value_to_string(v)
    % why: set_param ͳһ���ַ������ã���ͬ����ͳһ���л�Ϊ�ȶ��ַ���
    try
        if isstring(v)
            s = char(v);
        elseif ischar(v)
            s = v;
        elseif isnumeric(v)
            s = mat2str(v);
        elseif islogical(v)
            s = char(string(v));  % 'true'/'false'�������Ի�����Ҳ���� 'on'/'off' �ַ�
        elseif iscell(v)
            % �� cell �ڲ�����ת�ַ����� JSON�������˶���ط�
            s = char(jsonencode(v));
        elseif isstruct(v)
            s = char(jsonencode(v));
        else
            % Fallback��ʹ�� disp ���Ϊ�ַ���
            s = strtrim(evalc('disp(v)'));
        end
    catch
        s = '';
    end
end

% =========
% �����������ж����������ã����ֽ��
% =========
function CFG = get_param_filter_config()
    % why: ������Щ������Ҫ/����Ҫ���������ã�����ɢ���ڵ����߼���
    % �û����ڴ˰�/��������չ�����败���ɼ�������
    CFG = struct();

    % ���ư����������ȱ��������������ǿ�ƺ��ԣ�
    CFG.block.white = {'SampleTime','InitialCondition','Gain','Numerator','Denominator','UpperSaturationLimit','LowerSaturationLimit','OutDataTypeStr'};
    CFG.block.black = {'Position','Orientation','LineHandles','PortConnectivity','ForegroundColor','BackgroundColor','ShowName','NamePlacement','Priority','Tag'};

    CFG.mask.white  = {};  % Ĭ��ȫ�������������к�����
    CFG.mask.black  = {'ForegroundColor','BackgroundColor'}; % ��ģUI��

    CFG.model.white = {};  % ʹ�� defaultCand �Ѹ��ǳ�����
    CFG.model.black = {};  % ��Ҫ���ڴ�����

    % ���Ժ�������������Щ���ԵĲ���������
    % ������read-only, no-set, no-query, deprecated, simulated-only, run-time-only ��
    CFG.block.attr_black = {'read-only','no-set','deprecated'};
    CFG.model.attr_black = {'read-only','no-set','deprecated'};
end

function allowed = name_allowed(name, whites, blacks)
    nm = char(name);
    if any(strcmpi(nm, blacks)), allowed = false; return; end
    if isempty(whites), allowed = true; return; end
    allowed = any(strcmpi(nm, whites));
end

function ok = param_allowed_on_block(blockH, pname, attr_black)
    % ͨ�� ObjectParameters ���Թ��ˣ����ɲ�ѯ��
    ok = true;
    try
        op = get_param(blockH, 'ObjectParameters');
        if isfield(op, pname) && isfield(op.(pname), 'Attributes')
            attrs = lower(string(op.(pname).Attributes));
            for i = 1:numel(attr_black)
                if any(attrs == lower(string(attr_black{i})))
                    ok = false; return;
                end
            end
        end
    catch
    end
end

function ok = param_allowed_by_attr(~, pname, ~, attr_black)
    % ģ�ͼ����� block �����������չ�㱣��ͬ���ӿ�
    ok = true; %#ok<INUSD>
    try
        a = lower(string(attr_black)); %#ok<NASGU>
    catch
    end
end

function merged = merge_name_list(defaults, whites, blacks)
    % ���Ĭ�Ϻ�ѡ + ��������Ȼ��ȥ��������
    lst = defaults;
    if ~isempty(whites)
        for i = 1:numel(whites)
            if ~any(strcmpi(lst, whites{i}))
                lst{end+1} = whites{i}; %#ok<AGROW>
            end
        end
    end
    keep = true(1, numel(lst));
    for i = 1:numel(lst)
        if any(strcmpi(lst{i}, blacks))
            keep(i) = false;
        end
    end
    merged = lst(keep);
end

