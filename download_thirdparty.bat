@echo off
set downloadurl=https://s3-ap-southeast-1.amazonaws.com/zephyr-thirdparty/thirdparty/pointcompresser_thirdparty.zip
set downloadpath=%~dp0thirdparty.zip
set directory=%~dp0thirdparty
if not exist %directory% mkdir %directory%
%WINDIR%\System32\WindowsPowerShell\v1.0\powershell.exe -Command "& {Import-Module BitsTransfer;Start-BitsTransfer '%downloadurl%' '%downloadpath%'; $shell = new-object -com shell.application; $zip = $shell.NameSpace('%downloadpath%'); foreach($item in $zip.items()) { $shell.Namespace('%directory%').copyhere($item);}; remove-item '%downloadpath%';}"
echo download complete and extracted to the directory.
pause