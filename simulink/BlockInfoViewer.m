classdef BlockInfoViewer < handle
    properties
        fig
        blockHandleEdit
        resultTable
        infoText
        currentHandle
    end
    
    methods
        function obj = BlockInfoViewer()
            % 创建主窗口
            obj.fig = figure('Name', 'Simulink 模块信息查看器', ...
                'Position', [100, 100, 900, 600], ...
                'MenuBar', 'none', ...
                'ToolBar', 'none', ...
                'NumberTitle', 'off');
            
            % 创建输入区域
            uicontrol('Style', 'text', ...
                'Position', [20, 560, 150, 25], ...
                'String', '输入模块句柄:', ...
                'HorizontalAlignment', 'left');
            
            obj.blockHandleEdit = uicontrol('Style', 'edit', ...
                'Position', [160, 560, 200, 25], ...
                'String', '3.900004882812500e+02', ...
                'Callback', @(src,event)obj.updateInfo());
            
            uicontrol('Style', 'pushbutton', ...
                'Position', [380, 560, 100, 25], ...
                'String', '更新信息', ...
                'Callback', @(src,event)obj.updateInfo());
            
            % 创建结果显示表格
            columnNames = {'参数名', '参数值'};
            columnWidths = {200, 600};
            obj.resultTable = uitable('Parent', obj.fig, ...
                'Position', [20, 100, 860, 440], ...
                'ColumnName', columnNames, ...
                'ColumnWidth', columnWidths, ...
                'RowName', []);
            
            % 创建信息提示区域
            obj.infoText = uicontrol('Style', 'text', ...
                'Position', [20, 60, 860, 30], ...
                'String', '请输入有效的模块句柄', ...
                'HorizontalAlignment', 'left', ...
                'BackgroundColor', [0.9, 0.9, 0.9]);
            
            % 初始更新
            obj.updateInfo();
        end
        
        function updateInfo(obj)
            % 获取输入的句柄
            handleStr = get(obj.blockHandleEdit, 'String');
            
            try
                % 尝试将输入转换为句柄
                handle = str2double(handleStr);
                
                if isnan(handle) || ~ishandle(handle)
                    error('无效的句柄');
                end
                
                obj.currentHandle = handle;
                
                % 获取模块基本信息
                blockName = getfullname(handle);
                %blockType = get_param(handle, 'BlockType');
                
                % 获取所有可用参数
                allParams = get_param(handle, 'ObjectParameters');
                paramNames = fieldnames(allParams);
                
                % 准备表格数据
                tableData = cell(length(paramNames) + 4, 2);
                
                % 添加基本信息
                tableData{1,1} = '模块句柄';
                tableData{1,2} = handleStr;
                
                tableData{2,1} = '完整路径';
                tableData{2,2} = blockName;
                
                tableData{3,1} = '模块类型';
                %tableData{3,2} = blockType;
                
                tableData{4,1} = '父系统';
                tableData{4,2} = get_param(handle, 'Parent');
                
                % 获取端口信息
                try
                    portHandles = get_param(handle, 'PortHandles');
                    tableData{5,1} = '输入端口数量';
                    tableData{5,2} = num2str(length(portHandles.Inport));
                    
                    tableData{6,1} = '输出端口数量';
                    tableData{6,2} = num2str(length(portHandles.Outport));
                    
                    startIdx = 7;
                catch
                    startIdx = 5;
                end
                
                % 获取其他参数
                for i = 1:length(paramNames)
                    try
                        paramValue = get_param(handle, paramNames{i});
                        if ischar(paramValue)
                            tableData{i+startIdx-1,1} = paramNames{i};
                            tableData{i+startIdx-1,2} = paramValue;
                        elseif isnumeric(paramValue)
                            tableData{i+startIdx-1,1} = paramNames{i};
                            tableData{i+startIdx-1,2} = num2str(paramValue);
                        else
                            tableData{i+startIdx-1,1} = paramNames{i};
                            tableData{i+startIdx-1,2} = '[非文本/数值数据]';
                        end
                    catch
                        tableData{i+startIdx-1,1} = paramNames{i};
                        tableData{i+startIdx-1,2} = '[无法获取]';
                    end
                end
                
                % 更新表格
                set(obj.resultTable, 'Data', tableData);
                
                % 更新状态信息
                %set(obj.infoText, 'String', ...
                    %sprintf('成功获取模块信息: %s (%s)', blockName, blockType), ...
                    %'BackgroundColor', [0.7, 0.9, 0.7]);
                
            catch ME
                % 显示错误信息
                set(obj.infoText, 'String', ...
                    ['错误: ' ME.message], ...
                    'BackgroundColor', [0.9, 0.7, 0.7]);
                
                % 清空表格
                set(obj.resultTable, 'Data', {});
            end
        end
    end
end

% 创建查看器实例
%viewer = BlockInfoViewer();