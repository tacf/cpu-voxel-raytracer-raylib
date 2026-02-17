@echo off
setlocal

set "SCRIPT_DIR=%~dp0"
set "BUILD_DIR=%SCRIPT_DIR%build"
if not "%BUILD_DIR:~-1%"=="\" set "BUILD_DIR=%BUILD_DIR%\"

if defined CONFIG (
  set "BUILD_CONFIG=%CONFIG%"
) else (
  set "BUILD_CONFIG=Release"
)

if defined RAYLIB_FETCHCONTENT (
  set "FETCH=%RAYLIB_FETCHCONTENT%"
) else (
  set "FETCH=ON"
)

if defined CMAKE_POLICY_VERSION_MINIMUM (
  set "POLICY_MIN=%CMAKE_POLICY_VERSION_MINIMUM%"
) else (
  set "POLICY_MIN=3.5"
)

cmake -S "%SCRIPT_DIR%" -B "%BUILD_DIR%" -DRAYLIB_FETCHCONTENT=%FETCH% -DCMAKE_POLICY_VERSION_MINIMUM=%POLICY_MIN%
if errorlevel 1 exit /b 1

cmake --build "%BUILD_DIR%" --config "%BUILD_CONFIG%"
if errorlevel 1 exit /b 1

set "EXE=%BUILD_DIR%%BUILD_CONFIG%\voxel_dda_raylib.exe"
if exist "%EXE%" goto run

set "EXE=%BUILD_DIR%voxel_dda_raylib.exe"
if exist "%EXE%" goto run

set "EXE=%BUILD_DIR%Release\voxel_dda_raylib.exe"
if exist "%EXE%" goto run

set "EXE=%BUILD_DIR%Debug\voxel_dda_raylib.exe"
if exist "%EXE%" goto run

set "EXE=%BUILD_DIR%RelWithDebInfo\voxel_dda_raylib.exe"
if exist "%EXE%" goto run

echo error: could not find built executable voxel_dda_raylib under "%BUILD_DIR%"
exit /b 1

:run
"%EXE%"
endlocal
