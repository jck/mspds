@echo off

set HIDAPI=hidapi-0.7.0
set MSVC_Ver=msvc

set HIDAPIDOWNLOAD=https://github.com/signal11/hidapi/downloads

set LIBDIR=lib
set INCLUDEDIR=include

echo After installation there will be two further folders, "lib" and "include".
echo Unzipped files will be deleted in order to have a clean environment!

:: creating lib directory
if not exist %LIBDIR% (
	md %LIBDIR%
)

:: creating include directory
if not exist %INCLUDEDIR% (
	md %INCLUDEDIR%
)

:: installing hidapi
echo.
echo Installing hidapi!
echo.

if not exist %INCLUDEDIR% (
	goto error_no_include
)

if not exist %INCLUDEDIR%\hidapi.h (
    if not exist %HIDAPI%\hidapi\hidapi.h (
        goto error_no_hidapiheader
    ) else (
        move %HIDAPI%\hidapi\hidapi.h %INCLUDEDIR%
    )
)

goto finished

:error_no_include
echo.
echo *** Path %INCLUDEDIR% not found! ***
echo Script possibly broken. 
echo This should not happen. 
echo Check paths and configuartion of %0!
goto end

:error_no_hidapiheader
echo.
if exist %HIDAPI%.zip (
	echo *** You have to unzip %HIDAPI%.zip! ***
) else (
	echo *** Please download %HIDAPI%.zip from "%HIDAPIDOWNLOAD%" 
	echo and/or change settings of %0! ***
)
goto finsished_error

:finsished_error
echo.
echo *** Finished with errors! ***
goto end

:finished
echo.
echo Finished!
goto end

:end
Pause





