# Capturing the ONX LCD over serial

The temporary screenshot build mirrors ST7789 pixels in RAM and emits one
RGB565 screenshot ten seconds after the LCD initializes. It freezes only LCD
rendering while transmitting; CAN polling continues.

Create a separate build without changing the production `sdkconfig`:

```powershell
idf.py -B build-screenshot -D SDKCONFIG=sdkconfig.screenshot `
  -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.screenshot.defaults" set-target esp32s3
idf.py -B build-screenshot -D SDKCONFIG=sdkconfig.screenshot `
  -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.screenshot.defaults" build
```

Flash it to the LCD-equipped board's actual serial port, then start the
receiver **before resetting the board** so no screenshot rows are missed:

```powershell
python tools\capture_onx_screenshot.py COM13 -o onx_screenshot.bmp
```

Use the board's actual COM port if it differs. The receiver saves a standard
24-bit BMP that can be opened in Windows Photos or uploaded for review. The
temporary build uses an additional 150 KiB framebuffer and should be replaced
with the normal firmware after capture.
