!include x64.nsh
!define APPNAME "Piezo_Board_Interface"
!define COMPANYNAME "Casamia"
# define installer name
OutFile "Piezo_Board_Installer.exe"

#Icon "Ocarina_MP3.ico"
 
# set desktop as install directory
#InstallDir $DESKTOP
InstallDir "C:\${COMPANYNAME}\${APPNAME}"
 
# default section start
Section
 
# specify file to go in output path

MessageBox MB_OK "Piezo_Board_Interface installation in progress!"

SetOutPath "$INSTDIR\Driver"
File /nonfatal /a /r "Driver\" #note back slash at the end

ExecWait $INSTDIR\Driver\40002537_J.exe # define output path

SetOutPath "$INSTDIR\Framework"
File /nonfatal /a /r "Framework\" #note back slash at the end

ExecWait $INSTDIR\Framework\dotNetFx45_Full_setup.exe # define output path

SetOutPath "$INSTDIR\Piezo_Board_App"
File /nonfatal /a /r "Piezo_Board_App\" #note back slash at the end

SetOutPath $INSTDIR
# define uninstaller name
WriteUninstaller $INSTDIR\uninstaller.exe

MessageBox MB_OK "Piezo_Board_Interface installation finish!"
#-------
# default section end
SectionEnd

# create a section to define what the uninstaller does.
# the section will always be named "Uninstall"
Section "Uninstall"

MessageBox MB_OK "Piezo_Board_Interface Uninstaller run!"

RMDIR /r $INSTDIR
RMDIR $INSTDIR

MessageBox MB_OK "Piezo_Board_Interface Uninstaller finish!"
 
SectionEnd
