<p align="center"><img src="https://github.com/user-attachments/assets/a6b54019-04b2-4804-949c-32f24023128b" /></p>

RenderDoc is a frame-capture based graphics debugger, currently available for Vulkan, D3D11, D3D12, OpenGL, and OpenGL ES development on Windows, Linux, Android, and Nintendo Switch&trade;. It is completely open-source under the MIT license.

## RenderBlox

RenderBlox is a fork of RenderDoc that will capture the AvatarExporter Roblox scene, and create a fully built FBX of your Roblox Avatar from it.
This project follows after the Mario Kart Arcade GP DX exporter tool I created, which has been huge for my learning to make this possible.

## Usage

1. Open qrenderdoc.exe
2. From the program, launch Roblox Studio
3. Load AvatarExporter.rbxl and press F5 to play
4. Enter any username, once its ready to capture make sure EVERYTHING is on screen, then press F10
5. Open capture, expand the first Scene, expand 'Id_Opaque' and right click -> Set Reference on the event containing '(6, 1)'
6. Right click on Id_OpaqueCasters -> Export FBX and save your new .fbx

**Demonstration**

https://github.com/user-attachments/assets/e82c1b6e-51a5-4b9b-9375-b861174159de
