# External Controller design for GH Bladed and OpenFAST (Bladed interface)

## Contents

### Folder -- Bladed_NREL5MW_model:
- NREL_5MW_CPC.prj/prx        GH Bladed project of a 5MW turbine
- wind012.wnd                 Sample 12 m/s turbulent wind file (Bladed)

### Folder -- Simulink_32bit:
Refer to README.txt file in the folder (Credits and thanks to J. Butterworth, Envision Energy)

### Folder -- Simulink_64bit:
- DISCON_Empty.slx            Empty controller file, including all available inputs and outputs
- DISCON_NREL5MW.slx          Baseline NREL5MW controller for use with Bladed or OpenFAST
- Parameters_NREL5MW.mat      Parameters needed for compilation of the controller
- discon.c                    C file (needed for the generation of DISCON.DLL from a Simulink model)
- discon.tlc                  TLC file (needed for the generation of DISCON.DLL from a Simulink model)
- discon_vc.tmf               TMF file (needed for the generation of DISCON.DLL from a Simulink model)

## Requirements (for 64-bit DLL/SO compilation):
1. GH Bladed
2. MATLAB / Simulink, tested with 2018b, but might also work with other versions
3. Microsoft Visual Studio 2005/2008/2010

## Compiling DISCON DLL 32-bit:
1a. For 32-bit DLL compilation, install Windows Windows SDK 7.1, and see further instructions in the "Simulink_32bit\README.txt" file:
https://nl.mathworks.com/matlabcentral/answers/101105-how-do-i-install-microsoft-windows-sdk-7-1 

## Compiling DISCON DLL 64-bit:
1. For 64-bit compilation, download the latest 64-bit MATLAB version (tested with 2018b) and install the MinGW compiler:
https://nl.mathworks.com/support/compilers.html
2. Open MATLAB and load the Parameters_NREL5MW.mat in your current workspace
3. Change your current folder to the .\Simulink_files folder
4. Open DISCON_xxx.slx in Simulink
5. Press CTRL-B to build a DLL

Note: When compiling a 32-bit DISCON DLL, you can use the DLL with Bladed and a 32-bit version of OpenFAST (fast_win32.exe). When you use a 64-bit version of MATLAB, the compiled DLL will also be 64-bit, which is incompatible with Bladed, but compatible with 64-bit OpenFAST (openfast_x64.exe).

Comments:
1. Do not change the names of the inputs and outputs blocks in DISCON_xxx.slx
2. Do not change the name DISCON_xxx.slx to DISCON.slx (all other names are allowed)

## Referencing
When you use DISCON_Simulink in any publication, please cite the following paper:
* Mulders, S.P. and Zaaijer, M.B. and Bos, R. and van Wingerden, J.W. "Wind turbine control: open-source software for control education, standardization and compilation". Journal of Physics: Conference Series. Vol. 1452. No. 1. IOP Publishing, 2020. [Link to the paper](https://iopscience.iop.org/article/10.1088/1742-6596/1452/1/012010)

## References 
Sebastiaan Mulders
S.P.Mulders@tudelft.nl
Delft Center of Systems and Control
Delft University of Technology

Credits to:

Ivo Houtzager
Delft Center of Systems and Control
Delft University of Technology

Jeffrey Butterworth, Ph.D. (Support in making 32 bit compilation possible with a 64 bit MATLAB)
Envision Energy
1919 14th Street, Suite 800
Boulder, CO 80302
