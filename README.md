# color_checker_detection
It’s a color checker detection utility for each pattern, and it only needs to install OpenCV.


## 1 Features
* The algorithm is compatible for various color checker.
* With the ability to analyze color checker from special light source.
* With the ability to analyze multi-color checker in an image.
* ONLY need to install OpenCV 3.X or 4.X.


## 2 Environment
* Language: C++
* IDE: I use Visual Studio 2017; However, this project uses standard C++ library, so it could be coded & compiled on other C++ IDE
* Third party library: OpenCV 3.X or 4.X


## 3 Usage
Step1. Set the relative path for OpenCV library in this project’s properties page and then compile this project.

Step2. Create “in” & “out” directory beside the color_checker_detection.exe.

Step3. Put an image into the “in” directory you just created.

Step4. Change the g_input_img string for the input image in color_checker_detection.cpp.
```c++
string g_input_img = "image.jpg";
```

Step5. Run color_checker_detection.exe, and you could find the analyzed result in the “out” directory.


## 4 Algorithm Concept
Step1. AI training model
Through OpenCV - cascade classifier to training the different color checker, and produce training model: cascade.xml
PS. If you want to create the model yourself, please reference [OpenCV official documents](https://docs.opencv.org/master/dc/d88/tutorial_traincascade.html).

Step2. Image processing
Take out the pure color checker image which is analyzed from training model. Deal with the tilt angle calibration and analyze the centroid for each small grid.

## 5 About
This project is developed by [SOI](http://www.soinc.com.tw/en/). 
[SOI](http://www.soinc.com.tw/en/)(Silicon Optronics, Inc.) is a CMOS image sensor design company founded in May 2004 and listed in the Taiwan Emerging Market under ticker number 3530. The management team is comprised of seasoned executives with extensive experience in pixel design, image processing algorithms, analog circuits, optoelectronic systems, supply chain management, and sales.

