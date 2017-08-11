External Controller design for GH Bladed.

Contents:
NREL_5MW_CPC.prj		GH Bladed project of a 5MW turbine
DISCON.DLL			Generated External Controller
discon.tlc			TLC file (needed for the generation of DISCON.DLL from a Simulink model)
discon_vc.tmf			TMF file (needed for the generation of DISCON.DLL from a Simulink model)
DISCONdesign.m			Matlab script file which designs the pitch and torque controllers
DISCON_Bladed_NREL5MW.slx	Simulink model for generating the external controller DISCON.DLL
linmod.mat			Contains the linearized models (v = 1..25m/s) of the basic wind turbine 

Requirements:
1. GH Bladed
2. Matlab 2014a/2014b/2015a/2015b (32-bit version!) 
3. Microsoft Visual Studio 2005/2008/2010

Getting started:
1. Open NREL_5MW.prj in GH Bladed
or (when you apply changes to the controllers)
1. Apply changes and run DISCONdesign.m in MATLAB
2. Open DISCON_Bladed_NREL5MW.mdl, apply the changes and press CTRL-B to compile DISCON.DLL
3. Reload DISCON.DLL in NREL_5MW.prj

Comments:
1. Do not change the names of the inputs and outputs blocks in DISCON_Bladed_NREL5MW.mdl
2. Do not change the name DISCON_Bladed_NREL5MW.mdl to DISCON.mdl (all other names are allowed)

Sebastiaan Mulders
S.P.Mulders@tudelft.nl
Delft Center of Systems and Control
Delft University of Technology

Credits to:
Ivo Houtzager
Delft Center of Systems and Control
Delft University of Technology