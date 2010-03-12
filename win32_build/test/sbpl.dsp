# Microsoft Developer Studio Project File - Name="sbpl" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Console Application" 0x0103

CFG=sbpl - Win32 Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "sbpl.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "sbpl.mak" CFG="sbpl - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "sbpl - Win32 Release" (based on "Win32 (x86) Console Application")
!MESSAGE "sbpl - Win32 Debug" (based on "Win32 (x86) Console Application")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
RSC=rc.exe

!IF  "$(CFG)" == "sbpl - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "Release"
# PROP BASE Intermediate_Dir "Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "Release"
# PROP Intermediate_Dir "Release"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_CONSOLE" /D "_MBCS" /YX /FD /c
# ADD CPP /nologo /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_CONSOLE" /D "_MBCS" /FR /YX /FD /c
# ADD BASE RSC /l 0x409 /d "NDEBUG"
# ADD RSC /l 0x409 /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:console /machine:I386
# ADD LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:console /machine:I386

!ELSEIF  "$(CFG)" == "sbpl - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "Debug"
# PROP BASE Intermediate_Dir "Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "Debug"
# PROP Intermediate_Dir "Debug"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /Gm /GX /ZI /Od /D "WIN32" /D "_DEBUG" /D "_CONSOLE" /D "_MBCS" /YX /FD /GZ /c
# ADD CPP /nologo /W3 /Gm /GX /ZI /Od /D "WIN32" /D "_DEBUG" /D "_CONSOLE" /D "_MBCS" /FR /YX /FD /GZ /c
# ADD BASE RSC /l 0x409 /d "_DEBUG"
# ADD RSC /l 0x409 /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:console /debug /machine:I386 /pdbtype:sept
# ADD LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:console /debug /machine:I386 /pdbtype:sept

!ENDIF 

# Begin Target

# Name "sbpl - Win32 Release"
# Name "sbpl - Win32 Debug"
# Begin Group "Source Files"

# PROP Default_Filter "cpp;c;cxx;rc;def;r;odl;idl;hpj;bat"
# Begin Source File

SOURCE=..\..\src\utils\2Dgridsearch.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\planners\ADStar\adplanner.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\planners\ARAStar\araplanner.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\discrete_space_information\nav2d\environment_nav2D.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\discrete_space_information\nav2d_uu\environment_nav2duu.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\discrete_space_information\navxythetalat\environment_navxythetalat.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\discrete_space_information\robarm\environment_robarm.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\discrete_space_information\template\environment_XXX.cpp
# PROP Exclude_From_Scan -1
# PROP BASE Exclude_From_Build 1
# End Source File
# Begin Source File

SOURCE=..\..\src\utils\heap.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\test\main.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\utils\mdp.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\planners\PPCP\ppcpplanner.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\planners\RStar\rstarplanner.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\utils\utils.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\planners\VI\viplanner.cpp
# End Source File
# End Group
# Begin Group "Header Files"

# PROP Default_Filter "h;hpp;hxx;hm;inl"
# Begin Source File

SOURCE=..\..\src\utils\2Dgridsearch.h
# End Source File
# Begin Source File

SOURCE=..\..\src\planners\ADStar\adplanner.h
# End Source File
# Begin Source File

SOURCE=..\..\src\planners\ARAStar\araplanner.h
# End Source File
# Begin Source File

SOURCE=..\..\src\sbpl\config.h
# End Source File
# Begin Source File

SOURCE=..\..\src\discrete_space_information\environment.h
# End Source File
# Begin Source File

SOURCE=..\..\src\discrete_space_information\nav2d\environment_nav2D.h
# End Source File
# Begin Source File

SOURCE=..\..\src\discrete_space_information\nav2d_uu\environment_nav2duu.h
# End Source File
# Begin Source File

SOURCE=..\..\src\discrete_space_information\navxythetalat\environment_navxythetalat.h
# End Source File
# Begin Source File

SOURCE=..\..\src\discrete_space_information\robarm\environment_robarm.h
# End Source File
# Begin Source File

SOURCE=..\..\src\discrete_space_information\template\environment_XXX.h
# PROP Exclude_From_Scan -1

!IF  "$(CFG)" == "sbpl - Win32 Release"

# PROP BASE Exclude_From_Build 1

!ELSEIF  "$(CFG)" == "sbpl - Win32 Debug"

# PROP BASE Exclude_From_Build 1
# PROP Exclude_From_Build 1

!ENDIF 

# End Source File
# Begin Source File

SOURCE=..\..\src\sbpl\headers.h
# End Source File
# Begin Source File

SOURCE=..\..\src\utils\heap.h
# End Source File
# Begin Source File

SOURCE=..\..\src\utils\key.h
# End Source File
# Begin Source File

SOURCE=..\..\src\utils\list.h
# End Source File
# Begin Source File

SOURCE=..\..\src\utils\mdp.h
# End Source File
# Begin Source File

SOURCE=..\..\src\utils\mdpconfig.h
# End Source File
# Begin Source File

SOURCE=..\..\src\planners\planner.h
# End Source File
# Begin Source File

SOURCE=..\..\src\planners\PPCP\ppcpplanner.h
# End Source File
# Begin Source File

SOURCE=..\..\src\planners\RStar\rstarplanner.h
# End Source File
# Begin Source File

SOURCE=..\..\src\utils\utils.h
# End Source File
# Begin Source File

SOURCE=..\..\src\planners\VI\viplanner.h
# End Source File
# End Group
# Begin Group "Resource Files"

# PROP Default_Filter "ico;cur;bmp;dlg;rc2;rct;bin;rgs;gif;jpg;jpeg;jpe"
# End Group
# End Target
# End Project
