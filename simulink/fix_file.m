function fix_mojibake_file(infile, outfile, mode)
% 尝试从错误编码转换造成的乱码中救回中文
% mode:
%  - 'utf8_as_gbk' : 原本UTF-8被当作GBK读/存后乱码（常见：出现“浣犳...”）
%  - 'gbk_as_utf8' : 原本GBK被当作UTF-8读/存后乱码（常见：出现许多�或拼音符号）

    raw = fileread(infile);  % 读入当前“乱码文本”
    switch mode
        case 'utf8_as_gbk'
            % 先把“错误显示的Unicode”当成GBK去取字节，再按UTF-8解回
            bytes = unicode2native(raw, 'GBK');
            fixed = native2unicode(bytes, 'UTF-8');
        case 'gbk_as_utf8'
            % 先把“错误显示的Unicode”当成UTF-8去取字节，再按GBK解回
            bytes = unicode2native(raw, 'UTF-8');
            fixed = native2unicode(bytes, 'GBK');
        otherwise
            error('mode 取值: utf8_as_gbk 或 gbk_as_utf8');
    end

    % 以 UTF-8(with BOM) 写出（R2019 识别更稳）
    fid = fopen(outfile, 'w', 'n', 'UTF-8');
    assert(fid ~= -1, '无法写出到: %s', outfile);
    fwrite(fid, fixed, 'char'); fclose(fid);
end

fix_mojibake_file('Rebuild_Model.m','Rebuild_Model.m','utf8_as_gbk');