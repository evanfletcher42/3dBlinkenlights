# 3dBlinkenlights
3D Scanning & Animation for Addressable Christmas Tree lights

[Project page](evanfletcher42.com/2022/12/17/a-3d-christmas-tree-sparse-bundle-adjustment-of-addressable-leds/)

![A video of RGB ligths on a Christmas tree, showing 3D animations](/images/sweeps_gif.gif)

This repo contains firmware and code for:
 - **3D reconstructing LEDs** on a Christmas tree, using arbitrary video of LEDs blinking out coded patterns
 - **True volumetric 3D animations** based on these scans
  
The reconstruction is approached as a monocular structure-from-motion / sparse bundle adjustment problem.  Please see the [project post](evanfletcher42.com/2022/12/17/a-3d-christmas-tree-sparse-bundle-adjustment-of-addressable-leds/) for technical details.
  
## Contents
 - `ledreconstruct`: The reconstruction code, as a VS2019 C++ project.  
 - `PythonScripts`: A collection of scripts for camera calibration, and for generating firmware LED-coordinate headers from reconstruction output.
 - `LedCodesFW`: Firmware that blinks out a coded pattern mapping to each LED's address.  Video of this pattern is the input for reconstruction.
 - `AnimationFW`: Code that runs the animation seen above, using 3D-reconstructed points for each LED.
