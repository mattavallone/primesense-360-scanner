# 3D Scan to STL

Windows C++ application for capturing a PrimeSense/OpenNI depth frame and
writing a triangle mesh to STL.

The PrimeSense WinUSB transport and live viewer are maintained as separate
repositories and included here as Git submodules:

- `components/primesense-openni2-winusb`
- `components/primesense-viewer`

After cloning this project, initialize the components with:

```powershell
git submodule update --init --recursive
```

Configure and build from PowerShell:

```powershell
cmake -S . -B build
cmake --build build --config Debug
cmake --build build --config Debug --target build_winusb_openni
```

The build expects OpenNI2 in `C:\Program Files\OpenNI2` and OpenCV in
`C:\opencv\build`; change `OpenNI_DIR` or `OpenCV_DIR` when configuring CMake
if they are installed elsewhere. See the submodule READMEs for standalone
viewer and runtime details.
