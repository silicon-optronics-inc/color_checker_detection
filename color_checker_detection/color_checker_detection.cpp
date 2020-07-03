#include "pch.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include "afxcoll.h" //for CString & CStringArray
#include "color_grids_detection.h"

using namespace std;
using namespace cv;

//if not define, use opencv3.x format
#define OPENCV_4

int g_detection_count = 0;
string g_input_path = "in\\";
string g_input_img = "image.jpg";
string g_output_path = "out\\";
string g_classifier_path = "classifier\\cascade.xml";
vector<Rect> g_rets;

// Detect and draw detected object boxes on image
void detectAndDraw(
    cv::Mat& img,                               // input image
    cv::Ptr<cv::CascadeClassifier> classifier  // preloaded classifier
)
{
    // Just some pretty colors to draw with
    enum { BLUE, AQUA, CYAN, GREEN };
    static cv::Scalar colors[] =
    {
        cv::Scalar(0, 0, 255),
        cv::Scalar(0, 128, 255),
        cv::Scalar(0, 255, 255),
        cv::Scalar(0, 255, 0)
    };
    // Image preparation:
    cv::Mat gray(img.size(), CV_8UC1);
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    cv::equalizeHist(gray, gray);

    // Detect objects if any
    vector<cv::Rect> objects;
    classifier->detectMultiScale(
        gray, // input image
        objects, // place for the results
        1.1, // scale factor
        6,
#ifdef OPENCV_4
        1, // (old format cascades only)
#else
        CV_HAAR_DO_CANNY_PRUNING,
#endif // OPENCV_4
        cv::Size(129, 90)); // throw away detections smaller than this

    printf("object number:%d\r\n", (int)objects.size());
    if (objects.size() > 0)
    {
        g_detection_count++;
    }

    // Loop through to found objects and draw boxes around them
    int i = 0;
    for (vector<cv::Rect>::iterator r = objects.begin(); r != objects.end(); r++, ++i)
    {
        cv::Rect r_ = (*r);
        cv::rectangle(img, r_, colors[i % 4]);

        g_rets.push_back(r_);
    }
}

int main()
{
    Mat img;
    Mat crop_img;
    cv::Ptr<cv::CascadeClassifier> cascade(new cv::CascadeClassifier(g_classifier_path));
    Point color_checker_top_left;

#ifdef OPENCV_4
    img = cv::imread(g_input_path + g_input_img, IMREAD_COLOR);
#else
    img = cv::imread(g_input_path + g_input_img, CV_LOAD_IMAGE_COLOR);
#endif // OPENCV_4
    if (img.empty())
        return 0;

    detectAndDraw(img, cascade);

    bool ret = false;

    for (int i = 0; i < g_rets.size(); i++)
    {
        crop_img = img(g_rets[i]);

        color_checker_top_left = g_rets[i].tl();

        class ColorGridsDetection color_grids_detection;

        Point grids_coordinate[TOTAL_ROWS][TOTAL_COLUMNS];
        Point grids_coordinate_on_input_img[TOTAL_ROWS][TOTAL_COLUMNS];

        ret = color_grids_detection.DetectColorGrids(crop_img, grids_coordinate);

        cout << endl << "***Final result***" << endl;

        if (ret != true)
        {
            cout << "not a color checker..." << endl;
        }
        else
        {
            for (int row = 0; row < TOTAL_ROWS; row++)
            {
                for (int col = 0; col < TOTAL_COLUMNS; col++)
                {
                    grids_coordinate_on_input_img[row][col].x = grids_coordinate[row][col].x + color_checker_top_left.x;
                    grids_coordinate_on_input_img[row][col].y = grids_coordinate[row][col].y + color_checker_top_left.y;

                    circle(img, grids_coordinate_on_input_img[row][col], 7, Scalar(255, 255, 255), 2);
                }
            }

            cout << "found it!" << endl;
            imwrite(g_output_path + g_input_img + "-grid.jpg", img);
        }
    }

    g_rets.clear();

    cout << "valid object number by image: " << g_detection_count << endl;

    system("pause");
    return 0;
}
