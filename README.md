# 2025-Robot-Codebase
Full codebase for Nanuet Knightronz's FRC robotics team: 2025 Competition Robot

### **SUB-CODEBASES**

> This repo is structured into a main and dev branch, the latter for a testbed robot. In addition, there are seperate repos which are merged with this to ensure redundacy:
> 
>  - SwerveDrive
>  - Limelight
>  - Trajectory/PathPlanning
> 
> These codebases are subject to change from constant development.
>

### **THIRD PARTY LIBRARIES**

> YAGSL
>  - Used by SwerveDrive for easier implementation
>  - Multiple Dependent Libraries

> KAUAI LABS
>  - NavX IMU
>  - Component of SwerveDrive

> REVLIB
>  - NEO Motors
>  - SPARK's and SPARK Max
>  - REV ION System

> PHOTONLIB
>  - Photonvision Implementation
>  - Bot Alignment???

**Full Changelog**: https://github.com/Nanuet-Knightronz/SwerveDrive/commits/beta_v1.0.0

**To build the drivetrain code, import all files from "master" branch in this repo to WPILIB's VS Code
 and select "Deploy Robot Code" from the command palette "Ctrl + Shift + P" RioLog should then appear and give a "build successful" message.**

**See for more details**: https://docs.wpilib.org/en/stable/docs/software/vscode-overview/deploying-robot-code.html

Programmed in Intellij IDEA and WPILib VS Code with Temurin openJDK 21

