#pragma once
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#define TOTAL_COLUMNS 6
#define TOTAL_ROWS 4
#define TOTAL_GRIDS (TOTAL_COLUMNS * TOTAL_ROWS)
#define SMALL_GRID_RATE_MAX 0.9
#define SMALL_GRID_RATE_MIN 0.35
#define SQUARE_CORRELATION_ENDURANCE 0.2
#define PI 3.14159
#define MIN_COLOR_GRID_NUM 8
#define ANGLE_SUBTRACT_MAX_ENDURANCE 1.6

typedef struct Orientation
{
    Point left_top;
    Point right_bottom;
}Orientation;

typedef struct SmallGridSizeRange
{
    int max_size;
    int min_size;
}SmallGridSizeRange;

struct less_than_key
{
    inline bool operator() (const Point2f& struct1, const Point2f& struct2)
    {
        return (struct1.y < struct2.y);
    }
};

class ColorGridsDetection
{
public:
    bool DetectColorGrids(Mat Color_checker_img, Point grids_position[TOTAL_ROWS][TOTAL_COLUMNS]);

private:
    void FilterColorGrid(vector<vector<cv::Point>>& Contours, SmallGridSizeRange Grid_size_range, vector<double>& Match_value);
    void FilterOutOverlapPoint(vector<vector<cv::Point>> Contours, vector<Point2f>& centers_position);
    void GetRotationPoint(Point Org_point, Point* Rotation_point, int Img_width, int Img_height, float Angle);
    void FindContourCenter(vector<vector<cv::Point>> Contours, vector<Point2f>& centers_position);
    void GetSmallGridSizeRange(Mat Color_checker_image, SmallGridSizeRange* Small_grid_size_range);
};