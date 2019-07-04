External Controller design for GH Bladed and FASTv8 (Bladed interface).

Contents:
NREL_5MW_CPC.prj/prx        GH Bladed project of a 5MW turbine
wind012.wnd                 Sample 12 m/s turbulent wind file (Bladed)

DISCON_Empty.slx            Empty controller file, including all available inputs and outputs
DISCON_NREL5MW.slx          Baseline NREL5MW controller for use with Bladed or FASTv8
Parameters_NREL5MW.mat      Parameters needed for compilation of the controller
DISCON_Simulink64.dll       Compiled baseline controller (64-bit). Only works with openfast_x64.exe (see below).
discon.c                    C file (needed for the generation of DISCON.DLL from a Simulink model)
discon.tlc                  TLC file (needed for the generation of DISCON.DLL from a Simulink model)
discon_vc.tmf               TMF file (needed for the generation of DISCON.DLL from a Simulink model)

Requirements:
1. GH Bladed
2. Matlab 2014a/2014b/2015a/2015b (32-bit version!) 
3. Microsoft Visual Studio 2005/2008/2010

Compiling DISCON DLL 32/64-bit:
1a. For 32-bit DLL compilation, download MATLAB 2015b (32-bit) and install Windows Windows SDK 7.1. See this topic for installation guidance:
https://nl.mathworks.com/matlabcentral/answers/101105-how-do-i-install-microsoft-windows-sdk-7-1 
1b. For 64-bit compilation, download the latest 64-bit MATLAB version and install the MinGW compiler:
https://nl.mathworks.com/support/compilers.html

2. Open MATLAB and load the Parameters.mat in your current workspace
3. Change your current folder to the .\Simulink_files folder
4. Open DISCON_xxx.slx in Simulink
5. Press CTRL-B to build a DLL

Note: When you use a 32-bit version of MATLAB, you can use the compiled DLL with Bladed and a 32-bit version of FASTv8 (fast_win32.exe). When you use a 64-bit version of MATLAB, the compiled DLL will also be 64-bit, which is incompatible with Bladed, but compatible with 64-bit FASTv8 (openfast_x64.exe).

Comments:
1. Do not change the names of the inputs and outputs blocks in DISCON_xxx.slx
2. Do not change the name DISCON_xxx.slx to DISCON.slx (all other names are allowed)

Sebastiaan Mulders
S.P.Mulders@tudelft.nl
Delft Center of Systems and Control
Delft University of Technology

Credits to:
Ivo Houtzager
Delft Center of Systems and Control
Delft University of Technology