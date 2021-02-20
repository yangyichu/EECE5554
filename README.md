### **This repository contains project files of Yichu Yang For EECE554 Robot sensing and navigation.**


# **1.Final Project Dir Tree**
```bash
+---Final Project
|   +---Data                    #Data for testing, contains first 30s from total 602s data  
|   |   +---pcd1
|   |   \---pcd2
\---Src
    +---C++
    |   |   main.cpp
    |   |   pcl_common.dll
    |   |   pcl_commond.dll
    |   |   Project1.vcxproj
    |   |   Project1.vcxproj.user
    |   |   
    |   +---Setup
    |   |   \---Setup1
    |   |       |   Final Project Yichu Yang.msi
    |   |       |   setup.exe
    |   |       |   Setup1.vdproj
    |   |       +---Debug
    |   |       |       setup.exe
    |   |       |       Setup1.msi
    |   |       \---Release
    |   \---x64
    |       +---Debug
    |       \---Release
    |                   
    \---MATLAB
            deg2utm.m
            final_proj.asv
            final_proj.m
            plotminbox.m
```

# **2.How to RUN the code?**
## For MATLAB Project
**To run sample code, run final_proj.m under MATLAB directory.**
**MATLAB demonstration prerequisites list:**
```bash
-Name-                      -Tested Version-
MATLAB                          -9.7
Image Processing Toolbox        -11.0
Computer Vision Toolbox         -9.1
Navigation Toolbox              -1.0
Robotics System Toolbox         -3.0
ROS Toolbox                     -1.0
```
- **If it pops up and says toolbox not found, just install it using MATLAB app installer, no custom toolbox needed.**
- **If it works properly you shoud see a figure window with four subplots:**
    - **subplot1 is current lidar view** 
    - **subplot2 is mapping result** 
    - **subplot3 is yaw angle calculated from filter and raw input from IMU**
    - **subplot4 is Localization trajectory and GPS trajectory** 
    - ![image](https://gitlab.com/yang.yich/eece5554_roboticssensing/raw/master/MATLAB%20user%20interface.jpg)

## For C++ Project
- **For C++ project, you can open main.cpp and read the codes. If you want to see the result, you need to install the project.**
- **After installation you will see a file called Final Project.exe under the installation directory**
- (C:\Program Files (x86)\Final Project Yichu Yang by default)
- **The C++ project prerequisites: PCL 1.10.1 ,OPENNI2.**
- **Run Final Project Yichu Yang.msi to start installation**
- **IMPORTANT! Put the Final Project folder under D: directly (otherwise the programme cannot find the data needed), it should look like this:**
```bash
 \---D:
    +---Final Project
```
- **If it is installed correctly, you will see two output windows, left one is current lidar view, right one is current mapping results.**
![image](https://gitlab.com/yang.yich/eece5554_roboticssensing/raw/master/C++%20user%20interface.jpg)

