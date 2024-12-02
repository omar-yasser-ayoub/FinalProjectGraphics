@echo off
setlocal enabledelayedexpansion

:: Recursively search through all subdirectories for .png and .jpg files
for /r %%f in (*.png *.jpg) do (
    echo Converting "%%f" to 24-bit BMP3 and resizing to 512x512...

    :: Convert to 24-bit BMP3 and resize the image while maintaining the aspect ratio
    magick "%%f" -resize 512x512 -depth 24 BMP3:"%%~dpnf.bmp"

    :: Delete the original .png or .jpg file after conversion
    del "%%f"
)

echo Conversion complete, and original files deleted.
pause