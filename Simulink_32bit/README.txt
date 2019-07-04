August 10, 2017 -- J. Butterworth
Creating "Bladed-Style" DLLs (Windows) and SOs (Linux) from Matlab R2017a


Files and Directories:
======================
* File: README.txt  -- This readme file
* File: BlankCntrlr_SimulinkSetupAndBuild.m  -- Matlab m-file that walks the user through the build process
* File: BlankCntrlr_model.mdl -- A Simulink model set up to be a empty ("dummy") controller. It is configured to be built into a DLL or SO in this process
* File: discon_main.c -- DISCON main file
* File: discon.tlc -- DISCON target file
* File: setup_mssdk71.bat -- Bat file that allows the build of a 32-bit DLL from a 64bit installation of Matlab (in this case, R2017a)
* Directory: DISCONtmf_LINUX -- Directory that contains discon.tmf, a template makefile configured for LINUX
* Directory: DISCONtmf_PC  -- Directory that contains discon.tmf, a template makefile configured for WINDOWS

Setup:
======================
As before, the sampling time Ts must be defined in the MATLAB workspace, as it is needed for compilation.
For WINDOWS, use the Windows SDK 7.1 compiler and configure Matlab to use it....
You also need to install the Windows SDK 7.1, and after installation enable it with the mex –setup command in MATLAB, see:
https://nl.mathworks.com/matlabcentral/answers/101105-how-do-i-install-microsoft-windows-sdk-7-1
https://www.microsoft.com/en-us/download/details.aspx?id=8279
For LINUX, use the gcc compiler and and configure Matlab to use it. 



Building the DLL (Windows -- 32-bit DLL from 32-bit or 64-bit Matlab):
======================
Copy the discon.tmf file from DISCONtmf_PC and put it in your build directory (with discon_main.c, discon.tlc, etc.). This will provide the details needed to build a DLL (rather than an SO).

Run the script BlankCntrlr_SimulinkSetupAndBuild.m. The script will determine if Matlab is a 32-bit of 64-bit installation and build the DLL appropriately.

For a 64-bit Matlab installation:
The DLL is created in two-step process that supports creating 32-bit DLL creation from a 64-bit installation of Matlab (R2017a)
        1) Generate code-only from Simulink model
        2) Replace resulting R2017a(64bit) setup_mssdk71.bat with R2013b(32bit) setup_mssdk71.bat file and finish building the DLL.

For a 32-bit Matlab installation:
One can simply generate code and build the DLL directly from the Simulink model {Ctrl-B} (but one MUST uncheck "generate code only" in the "code generation tab" of "model configuration parameters" in the Simulink model). Also, the script BlankCntrlr_SimulinkSetupAndBuild.m can build the model regardless of the "generate code only" setting.



Building the SO (LINUX -- 64-bit SO from 64-bit Matlab):
======================
Copy the discon.tmf file from DISCONtmf_LINUX and put it in your build directory (with discon_main.c, discon.tlc, etc.). This will provide the details needed to build an SO (rather than a DLL).

Open the model and uncheck "generate code only" in the "code generation tab" of "model configuration parameters". I'm still looking for a way to automate this in the same way as the WINDOWS version.

One can simply generate code and build the DLL directly from the Simulink model {Ctrl-B}. Also, the script BlankCntrlr_SimulinkSetupAndBuild.m can build the model regardless of the "generate code only" setting.


