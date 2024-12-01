@echo off
setlocal enabledelayedexpansion

:: Recursively search through all subdirectories for .png and .jpg files
for /r %%f in (*.png *.jpg) do (
    echo Converting "%%f" to BMP...
    magick "%%f" "%%~dpnf.bmp"

    :: Delete the original .png or .jpg file after conversion
    del "%%f"
)

echo Conversion complete and original files deleted.
pause