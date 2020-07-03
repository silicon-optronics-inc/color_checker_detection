#include <string> //for string
#include "pch.h"
#include <cmath>
#include "color_grids_detection.h"

vector<Point> g_square_contour{ Point(0,0),Point(1,0),Point(1,1),Point(0,1) };

Mat g_input_img;

void ColorGridsDetection::FilterColorGrid(vector<vector<cv::Point>>& Contours, SmallGridSizeRange Grid_size_range, vector<double>& Match_value)
{
    double area;
    double match_value;

    for (vector<vector<cv::Point>>::iterator it_contour = Contours.begin(); it_contour != Contours.end();)
    {
        area = contourArea(*it_contour);

        match_value = matchShapes(*it_contour, g_square_contour, cv::CONTOURS_MATCH_I2, 0.0);

        if (area > Grid_size_range.min_size && area < Grid_size_range.max_size && match_value < SQUARE_CORRELATION_ENDURANCE)
        {
            it_contour++;
            Match_value.push_back(match_value);
        }
        else
        {
            it_contour = Contours.erase(it_contour);
        }
    }
}

void ColorGridsDetection::FilterOutOverlapPoint(vector<vector<cv::Point>> Contours, vector<Point2f>& centers_position)
{
    unsigned char inside_contour_count = 0;
    unsigned char ret = 0;
    Point2f temp_center(0.0, 0.0);

    for (vector<vector<cv::Point>>::iterator it_contour = Contours.begin(); it_contour != Contours.end();)
    {
        inside_contour_count = 0;

        for (vector<Point2f>::iterator it_center = centers_position.begin(); it_center != centers_position.end();)
        {
            temp_center = *it_center;

            ret = (int)pointPolygonTest(*it_contour, *it_center, false);
            if (ret == 1 || ret == 0) //1:inside contour¡A0:outside contour
            {
                if (inside_contour_count >= 1)
                {
                    it_center = centers_position.erase(it_center);
                }
                else
                {
                    it_center++;
                }

                inside_contour_count++;
            }
            else
            {
                it_center++;
            }
        }

        it_contour++;
    }
}

void ColorGridsDetection::GetRotationPoint(Point Org_point, Point* Rotation_point, int Img_width, int Img_height, float Angle)
{
    Point2f org_point_new_coor;
    Point2f rotation_point_new_coor;
    float half_width, half_height;
    double rad = 0;
    half_width = (float)Img_width / 2;
    half_height = (float)Img_height / 2;
    rad = (Angle * PI) / 180;

    //Original image coordinate system is defined in the left-top corner of image,
    //but we wanna rotate it from the image center.
    //Therefore, we need to change the coordinate system to the image center.
    org_point_new_coor.x = Org_point.x - half_width;
    if (Org_point.y <= half_height)
    {
        org_point_new_coor.y = half_height - Org_point.y;
    }
    else
    {
        org_point_new_coor.y = -1 * (Org_point.y - half_height);
    }

    //rotation matrix
    rotation_point_new_coor.x = org_point_new_coor.x*((float)cos(rad)) - org_point_new_coor.y*((float)sin(rad));
    rotation_point_new_coor.y = org_point_new_coor.x*((float)sin(rad)) + org_point_new_coor.y*((float)cos(rad));

    //After rotation, need to change the coordinate system back to left-top corner
    Rotation_point->x = (int)(rotation_point_new_coor.x + half_width);
    if (rotation_point_new_coor.y >= 0)
    {
        Rotation_point->y = (int)(half_height - rotation_point_new_coor.y);
    }
    else
    {
        Rotation_point->y = (int)(-1 * (rotation_point_new_coor.y) + half_height);
    }
}

void ColorGridsDetection::FindContourCenter(vector<vector<cv::Point>> Contours, vector<Point2f>& centers_position)
{
    Moments my_moment;
    for (int i = 0; i < Contours.size(); i++)
    {
        my_moment = moments(Contours[i], false);
        centers_position[i] = Point2f((float)(my_moment.m10 / my_moment.m00), (float)(my_moment.m01 / my_moment.m00));
    }
}

void ColorGridsDetection::GetSmallGridSizeRange(Mat Color_checker_image, SmallGridSizeRange* Small_grid_size_range)
{
    int base_small_grid_size;
    base_small_grid_size = (int)(Color_checker_image.cols*Color_checker_image.rows / TOTAL_GRIDS);
    Small_grid_size_range->max_size = (int)(base_small_grid_size * SMALL_GRID_RATE_MAX);
    Small_grid_size_range->min_size = (int)(base_small_grid_size * SMALL_GRID_RATE_MIN);
}

bool ColorGridsDetection::DetectColorGrids(Mat Color_checker_img, Point grids_position[TOTAL_ROWS][TOTAL_COLUMNS])
{
    Color_checker_img.copyTo(g_input_img);

    Mat gray_img(Color_checker_img.size(), CV_8UC1);
    cvtColor(Color_checker_img, gray_img, cv::COLOR_RGB2GRAY);

    Mat denoise_img;
    fastNlMeansDenoising(gray_img, denoise_img, 10, 7, 21); //book P313

    Mat threshold_img;
    adaptiveThreshold(denoise_img, threshold_img, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 21, 3);

    Mat element(3, 3, CV_8U, Scalar(1));

    Mat eroded_img;
    erode(threshold_img, eroded_img, element);

    SmallGridSizeRange small_grid_size_range;
    GetSmallGridSizeRange(Color_checker_img, &small_grid_size_range);

    //Find all contours
    vector<vector<cv::Point>> contours;
    findContours(eroded_img, contours, cv::noArray(), cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

    vector<double> contours_match_value;

    FilterColorGrid(contours, small_grid_size_range, contours_match_value);

    if (contours.size() < MIN_COLOR_GRID_NUM)
    {
        return false;
    }

    vector<Point2f> center_positions(contours.size());

    FindContourCenter(contours, center_positions);

    FilterOutOverlapPoint(contours, center_positions);

    sort(center_positions.begin(), center_positions.end(), less_than_key());

    int center_position_size = (int)center_positions.size();

    //judge rotation
    //Compare the tilt angle of topped 2 centroids with the tilt angle of bottomed 2 centroids,
    //if the result is similar, then calibrate the tilt angle to reduce the probability of fail judgement
    float rad_top, rad_bottom;
    float angle_top, angle_bottom, angle_avg;
    float angle_subtract;
    rad_top = atan((center_positions[1].y - center_positions[0].y) / (center_positions[1].x - center_positions[0].x));
    angle_top = (float)(rad_top * 180 / PI);

    rad_bottom = atan((center_positions[center_position_size - 1].y - center_positions[center_position_size - 2].y)
        / (center_positions[center_position_size - 1].x - center_positions[center_position_size - 2].x));
    angle_bottom = (float)(rad_bottom * 180 / PI);

    angle_subtract = (float)fabs(angle_top - angle_bottom);

    if (angle_subtract >= ANGLE_SUBTRACT_MAX_ENDURANCE)
    {
        //cout << "***NO Rotation***" << endl;

        //Choice any one of centroids as base
        Orientation orientation_rotation;
        orientation_rotation.left_top = (Point)center_positions[0];
        orientation_rotation.right_bottom = (Point)center_positions[0];

        //Analyze the left-top and right-bottom centroids
        for (int i = 0; i < center_positions.size(); i++)
        {
            if (center_positions[i].x < orientation_rotation.left_top.x)
            {
                orientation_rotation.left_top.x = (int)center_positions[i].x;
            }

            if (center_positions[i].y < orientation_rotation.left_top.y)
            {
                orientation_rotation.left_top.y = (int)center_positions[i].y;
            }

            if (center_positions[i].x > orientation_rotation.right_bottom.x)
            {
                orientation_rotation.right_bottom.x = (int)center_positions[i].x;
            }

            if (center_positions[i].y > orientation_rotation.right_bottom.y)
            {
                orientation_rotation.right_bottom.y = (int)center_positions[i].y;
            }
        }

        //Calculate the x & y translation for each grid
        Point2f translation;
        translation.x = (float)(orientation_rotation.right_bottom.x - orientation_rotation.left_top.x) / (TOTAL_COLUMNS - 1);
        translation.y = (float)(orientation_rotation.right_bottom.y - orientation_rotation.left_top.y) / (TOTAL_ROWS - 1);
        
        //Inference the remainder grids centroid coordinate from the left-top centroid coordinate and x & y translation
        Point grid_coordinate[TOTAL_ROWS][TOTAL_COLUMNS];

        for (int row = 0; row < TOTAL_ROWS; row++)
        {
            for (int col = 0; col < TOTAL_COLUMNS; col++)
            {
                grid_coordinate[row][col] = Point((int)(orientation_rotation.left_top.x + translation.x*col),
                    (int)(orientation_rotation.left_top.y + translation.y*row));

                cout << "grid[" + to_string(row) + "][" + to_string(col) + "]= "
                    + to_string(grid_coordinate[row][col].x) + ", "
                    + to_string(grid_coordinate[row][col].y) << "\t";

                grids_position[row][col] = grid_coordinate[row][col];
            }
            cout << endl;
        }
    }
    else
    {
        //cout << "***DO Rotation***" << endl;

        angle_avg = (angle_top + angle_bottom) / 2;

        Mat rotation_img(Color_checker_img.size(), CV_8UC1);
        Mat rotation_mat;
        Point2f center((float)(Color_checker_img.cols / 2), (float)(Color_checker_img.rows / 2));

        rotation_mat = getRotationMatrix2D(center, angle_avg*(1), 1.0);
        warpAffine(eroded_img, rotation_img, rotation_mat, Color_checker_img.size());

        //Find all tontours
        vector<vector<cv::Point>> contours_rotation;
        findContours(rotation_img, contours_rotation, cv::noArray(), cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

        vector<double> contours_rotation_match_value;

        FilterColorGrid(contours_rotation, small_grid_size_range, contours_rotation_match_value);

        //cout << endl << endl << "*****filter*****" << endl;

        vector<Point2f> center_position_rotation(contours_rotation.size());

        FindContourCenter(contours_rotation, center_position_rotation);

        //Choice any one of centroids as base
        Orientation orientation_rotation;
        orientation_rotation.left_top = (Point)center_position_rotation[0];
        orientation_rotation.right_bottom = (Point)center_position_rotation[0];

        for (int i = 0; i < contours_rotation.size(); i++)
        {
            //Analyze the left-top and right-bottom centroids
            if (center_position_rotation[i].x < orientation_rotation.left_top.x)
            {
                orientation_rotation.left_top.x = (int)center_position_rotation[i].x;
            }

            if (center_position_rotation[i].y < orientation_rotation.left_top.y)
            {
                orientation_rotation.left_top.y = (int)center_position_rotation[i].y;
            }

            if (center_position_rotation[i].x > orientation_rotation.right_bottom.x)
            {
                orientation_rotation.right_bottom.x = (int)center_position_rotation[i].x;
            }

            if (center_position_rotation[i].y > orientation_rotation.right_bottom.y)
            {
                orientation_rotation.right_bottom.y = (int)center_position_rotation[i].y;
            }
        }

        //Calculate the x & y translation for each grid
        Point2f translation;
        translation.x = (float)(orientation_rotation.right_bottom.x - orientation_rotation.left_top.x) / (TOTAL_COLUMNS - 1);
        translation.y = (float)(orientation_rotation.right_bottom.y - orientation_rotation.left_top.y) / (TOTAL_ROWS - 1);

        //Inference the remainder grids centroid coordinate from the left-top centroid coordinate and x & y translation
        Point grid_coordinate[TOTAL_ROWS][TOTAL_COLUMNS];

        for (int row = 0; row < TOTAL_ROWS; row++)
        {
            for (int col = 0; col < TOTAL_COLUMNS; col++)
            {
                grid_coordinate[row][col] = Point((int)(orientation_rotation.left_top.x + translation.x*col),
                    (int)(orientation_rotation.left_top.y + translation.y*row));
            }
        }

        //cout << endl << endl << "***transform back to org coordinate***" << endl;

        Point grid_coordinate_org[TOTAL_ROWS][TOTAL_COLUMNS];
        for (int row = 0; row < TOTAL_ROWS; row++)
        {
            for (int col = 0; col < TOTAL_COLUMNS; col++)
            {
                //Mapping the centroid from rotation calibration to original image
                GetRotationPoint(grid_coordinate[row][col], &grids_position[row][col], Color_checker_img.cols, Color_checker_img.rows, -1 * angle_avg);
            }
        }
    }

    return true;
}