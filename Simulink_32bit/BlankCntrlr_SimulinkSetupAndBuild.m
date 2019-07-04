
close all, clear, clc; 



% This simple script aligns the path to the model ... and then
% generates code from the Simulink model and builds the DLL or SO.

SmlkMdl = 'BlankCntrlr_model';


Ts = 0.01; % Sec

open(SmlkMdl)

% Building the DLL (Windows) or SO (Linux)
% ==================================================
if ispc % WINDOWS DLL Build (32-bit DLL from 32 or 64 bit Matlab)
    % Copy the discon.tmf file from DISCONtmf_PC and put it in your build 
    %     directory (with discon_main.c, discon.tlc, etc.). This will 
    %     provide the details needed to build a DLL (rather than an SO).
    if strcmp(computer('arch'),'win64') % 64-bit Matlab installation.
        % Build DLL in two-step process that supports creating 32-bit DLL creation
        % from a 64-bit installation of Matlab (R2017a).
        % Note: Using 'Microsoft Windows SDK 7.1 (C)' for C language compilation.
        %       Set compiler with 'mex -setup'
        % Two-step process:
        %   1) Generate code-only from Simulink model
        %   2) Replace resulting R2017a(64bit) setup_mssdk71.bat with R2013b(32bit)
        %   setup_mssdk71.bat file and finish building the DLL.
        sprintf('32-bit DLL build with 64-bit Matlab installation.../n')
        codegenfolder = sprintf('%s_discon_rtw',SmlkMdl);
        R2013bSDKSetup = fullfile(pwd,'setup_mssdk71.bat'); % Path to the R2013b setup_mssdk71.bat (saved in git).
        load_system(SmlkMdl);
        set_param(SmlkMdl, 'GenCodeOnly', true);
        slbuild(SmlkMdl);
        cd(codegenfolder);
        delete('setup_mssdk71.bat');
        copyfile(R2013bSDKSetup);
        system([SmlkMdl '.bat']);
        cd('../');
    else % 32-bit Matlab installation.
        % If using 32-bit installation of Matlab, can simply generate code and
        % build DLL directly from the Simulink model (Ctrl-B) in the same way that
        % a *.so is built in Linux. Or with the following commands...
        sprintf('32-bit DLL build with 32-bit Matlab installation.../n')
        load_system(SmlkMdl);
        slbuild(SmlkMdl);
        codegenfolder = sprintf('%s_discon_rtw',SmlkMdl);
        cd(codegenfolder);
        system([SmlkMdl '.bat']);
        cd('../');
    end
else % LINUX SO Build (64-bit DLL from 64-bit Matlab)
    % Copy the discon.tmf file from DISCONtmf_LINUX and put it in your build 
    %      directory (with discon_main.c, discon.tlc, etc.). This will provide 
    %      the details needed to build an SO (rather than a DLL).
    % Open the model and uncheck "generate code only" in the "code generation 
    %      tab" of "model configuration parameters". I'm still looking for a way 
    %      to automate this in the same way as the WINDOWS version.
    % If using 64-bit installation of Matlab, can simply generate code and
    % build SO directly from the Simulink model (Ctrl-B),
    % or with the following commands...
    sprintf('64-bit SO build with 64-bit Matlab installation.../n')
    load_system(SmlkMdl);
    slbuild(SmlkMdl);
end



