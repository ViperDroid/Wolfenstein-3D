# Wolfenstein-3D
How to Compile and Run
The process is identical to the Snake game.
Get Tools: Make sure you have DOSBox and NASM.
Set up Workspace: Create a folder (e.g., C:\dos_projects\) and put nasm.exe in it. Save the code above as raycast.asm in the same folder.
Compile:
Start DOSBox.
mount c C:\dos_projects
c:
nasm raycast.asm -f bin -o raycast.com
Run:
raycast.com
You should see a window looking into a 3D maze. Use the arrow keys to navigate. The illusion is powerful, and it's all running on pure, classic x86 assembly code
