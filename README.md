# color_checker_detection
It’s a color checker detection utility for each pattern, and environment only requires OpenCV.  
![image](https://github.com/silicon-optronics-inc/color_checker_detection/blob/master/doc/demo.gif)  
**※1** This image [source](https://imgur.com/VUiuRTq)


## 1 Features
* The algorithm is compatible with various color checker.
* With the ability to locate/detect multi-color checker in an image.
* With the ability to analyze color checker from special light source.
* ONLY need to install OpenCV 3.X or 4.X.


## 2 Environment
* Language: C++
* IDE: Visual Studio 2017; However, this project uses standard C++ library only, so it could be coded & compiled on other C++ IDE
* Third party library: OpenCV 3.X or 4.X


## 3 Usage
Step1. Set the relative path for OpenCV library in this project’s properties page and then compile this project.

Step2. Create “image” directory inside the output directory.

Step3. Put an image into the “image” directory just created.

Step4. Run color_checker_detection.exe, and find the inferenced result inside the “image” directory.


## 4 Algorithm Concept
Step1. AI training model  
Train the different color checker through OpenCV, and produce training model: cascade.xml  
PS. If you want to train your own model, please reference [OpenCV official documents](https://docs.opencv.org/master/dc/d88/tutorial_traincascade.html).

Step2. Image processing  
Crop the pure color checker image which is inferenced from training model. Deal with the tilt angle calibration and analyze the centroid for each small grid.  
![image](https://github.com/silicon-optronics-inc/color_checker_detection/blob/master/doc/process.gif)


## 5 About
This project is developed by [SOI](http://www.soinc.com.tw/en/).  
[SOI](http://www.soinc.com.tw/en/)(Silicon Optronics, Inc.) is a CMOS image sensor design company founded in May 2004 and listed in the Taiwan Emerging Market under ticker number 3530. The management team is comprised of seasoned executives with extensive experience in pixel design, image processing algorithms, analog circuits, optoelectronic systems, supply chain management, and sales.

