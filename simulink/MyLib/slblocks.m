function blkStruct = slblocks
% 指定库在浏览器中的显示信息

Browser.Library = 'MyLib';  % 您的库文件名（不含扩展名）
Browser.Name    = 'My Library';  % 在浏览器中显示的名称
Browser.IsFlat  = 0;  % 0表示使用层次结构，1表示扁平结构

blkStruct.Browser = Browser;