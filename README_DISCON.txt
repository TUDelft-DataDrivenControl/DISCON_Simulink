External Controller design for GH Bladed.

Contents:
NREL_5MW_CPC.prj/prx		GH Bladed project of a 5MW turbine
wind012.wnd			Sample 12 m/s turbulent wind file (Bladed)

DISCON_Empty.slx		Empty controller file, including all available inputs and outputs
DISCON_NREL5MW.slx		Baseline NREL5MW controller for use with Bladed or FASTv8
DISCON.DLL			Compiled baseline controller

discon.c			C file (needed for the generation of DISCON.DLL from a Simulink model)
discon.tlc			TLC file (needed for the generation of DISCON.DLL from a Simulink model)
discon_vc.tmf			TMF file (needed for the generation of DISCON.DLL from a Simulink model)

Requirements:
1. GH Bladed
2. Matlab 2014a/2014b/2015a/2015b (32-bit version!) 
3. Microsoft Visual Studio 2005/2008/2010

Comments:
1. Do not change the names of the inputs and outputs blocks in DISCON_xxx.slx
2. Do not change the name DISCON_xxx.mdl to DISCON.mdl (all other names are allowed)

Sebastiaan Mulders
S.P.Mulders@tudelft.nl
Delft Center of Systems and Control
Delft University of Technology

Credits to:
Ivo Houtzager
Delft Center of Systems and Control
Delft University of Technology