#pragma once
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#define PI 3.1415926
namespace ld{
    using namespace std;
    using namespace cv;
    Mat imageCopy(Mat &image)
{
    Mat result;
    image.copyTo(result);
    return result;
}
void convertColor(Mat &image, Mat &result, int flag)
{
    cvtColor(image, result, flag);
    return;
}
void addImage(Mat &image1, Mat &image2, Mat &result) 
{
    add(image1, image2, result);
}

void imageMorphologyKernel(Mat &result, int shape, int size) 
{
    result = getStructuringElement(shape, Size(size, size));
    return;
}
void imageMorphologyEx(Mat &image, Mat &result, int op, Mat &kernel, int iterations) 
{
    morphologyEx(image, result, op, kernel, Point(), iterations);
    return;
}

Mat makeBlackImage(Mat &image, bool color)
{
    if(color)
        return Mat::zeros(image.size(), CV_8UC3);
    else
        return Mat::zeros(image.size(), image.type());
}
Mat fillPolyROI(Mat &image, vector<Point> points)
{
    Mat result = makeBlackImage(image, false);
    vector<vector<Point> > fillContAll;
    fillContAll.push_back(points);
    if(image.channels()==1)
        fillPoly(result, fillContAll, Scalar(255));
    else 
        fillPoly(result, fillContAll, Scalar(255, 255, 255));
    return result;
}
void polyROI(Mat &image, Mat &result, vector<Point> points) 
{
    result = fillPolyROI(image, points);
    bitwise_and(result, image, result);
    return;
}
void imageHoughLinesP(Mat &image, vector<Vec4i> &lines, double rho, double theta, int threshold, double minLineLength, double maxLineGap) 
{
    lines.clear();
    HoughLinesP(image,lines,rho,theta,threshold, minLineLength, maxLineGap);
    return;
}

void splitTwoSideLines(vector<Vec4i> &lines, vector<vector<float>> &lefts, vector<vector<float>> &rights, float slope_threshold)
{
    int i;
    lefts.clear();
    rights.clear();
    vector<float> temp;
    for( i = 0 ; i < lines.size() ; i++ )
    {
        temp.clear();
        Vec4i line = lines[i];
        int x1, y1, x2, y2;
        x1 = line[0];
        y1 = line[1];
        x2 = line[2];
        y2 = line[3];
        if (x1 - x2 == 0)
            continue;
        float slope = (float)(y2-y1)/(float)(x2-x1);
        if (abs(slope) < slope_threshold)
            continue;
        if( slope <= 0)
        {
            temp.push_back(slope);
            temp.push_back(x1);
            temp.push_back(y1);
            temp.push_back(x2);
            temp.push_back(y2);
            lefts.push_back(temp);
        }
        else
        {
            temp.push_back(slope);
            temp.push_back(x1);
            temp.push_back(y1);
            temp.push_back(x2);
            temp.push_back(y2);
            rights.push_back(temp);
        }
    }
    return;
}
void splitOneSideLines(vector<Vec4i> &lines, vector<vector<float>> &arranged_lines, float slope_threshold)
{
    int i;
    arranged_lines.clear();
    vector<float> temp;
    for( i = 0 ; i < lines.size() ; i++ )
    {
        temp.clear();
        Vec4i line = lines[i];
        int x1, y1, x2, y2;
        x1 = line[0];
        y1 = line[1];
        x2 = line[2];
        y2 = line[3];
        if (x1 - x2 == 0)
            continue;
        float slope = (float)(y2-y1)/(float)(x2-x1);
        if (abs(slope) < slope_threshold)
            continue;
        temp.push_back(slope);
        temp.push_back(x1);
        temp.push_back(y1);
        temp.push_back(x2);
        temp.push_back(y2);
        arranged_lines.push_back(temp);
    }
    return;
}
bool comp(vector<float> a, vector<float> b)
{
    return (a[0] > b[0]);
}
void medianPoint(vector<vector<float>> &lines, vector<float> &line)
{
    line.clear();
    size_t size = lines.size();
    if (size == 0)
        return;
    sort(lines.begin(), lines.end(), comp);
    line = lines[(int)(size/2.0)];
    return;
}
int interpolate(int x1, int y1, int x2, int y2, int y)
{
    return int(float(y - y1) * float(x2-x1) / float(y2-y1) + x1);
}
void lineFittingOneSide(Mat &image, Mat &result, vector<Vec4i> &lines, Scalar color, int thickness, float slope_threshold)
{
    result = imageCopy(image);
    int height = image.rows;
    vector<vector<float>> arrangedLines;
    splitOneSideLines(lines, arrangedLines, slope_threshold);
    vector<float> medianLine;
    medianPoint(arrangedLines, medianLine);
    int min_y = int(height * 0.6);
    int max_y = height;
    int min_x = interpolate(medianLine[1], medianLine[2], medianLine[3], medianLine[4], min_y);
    int max_x = interpolate(medianLine[1], medianLine[2], medianLine[3], medianLine[4], max_y);
    line(result, Point(min_x, min_y), Point(max_x, max_y), color, thickness);
    result;
}
void lineFitting(Mat &image, Mat &result, vector<Vec4i> &lines, Scalar color, int thickness, float slope_threshold)
{
    result = imageCopy(image);
    int height = image.rows;
    vector<vector<float>> lefts, rights;
    splitTwoSideLines(lines, lefts, rights, slope_threshold);
    vector<float> left, right;
    medianPoint(lefts, left);
    medianPoint(rights, right);
    int min_y = int(height * 0.6);
    int max_y = height;
    if( !left.empty()) 
    {
        int min_x_left = interpolate(left[1], left[2], left[3], left[4], min_y);
        int max_x_left = interpolate(left[1], left[2], left[3], left[4], max_y);
        line(result, Point(min_x_left, min_y), Point(max_x_left, max_y), color, thickness);
    }
    if( !right.empty())
    {
        int min_x_right = interpolate(right[1], right[2], right[3], right[4], min_y);
        int max_x_right = interpolate(right[1], right[2], right[3], right[4], max_y);
        line(result, Point(min_x_right, min_y), Point(max_x_right, max_y), color, thickness);
    }
    return;
}
    void imageProcessing(Mat &image, Mat &result)
{
    //result = imageCopy(image)
    result = imageCopy(image);
    //HLS = convertColor(result, cv2.COLOR_BGR2HLS)
    Mat HLS;
    convertColor(result, HLS, COLOR_BGR2HLS);
    //Y_lower = np.array([15, 52, 75])
    Scalar Y_lower = Scalar(15, 52, 75);
    //Y_upper = np.array([30, 190, 255])
    Scalar Y_upper = Scalar(30, 190, 255);
    //Y_BIN = rangeColor(HLS, Y_lower, Y_upper)
    Mat Y_BIN;
    inRange(HLS, Y_lower, Y_upper, Y_BIN);
    //W_lower = np.array([0, 200, 0])
    Scalar W_lower = Scalar(0, 200, 0);
    //W_upper = np.array([180, 255, 255])
    Scalar W_upper = Scalar(180, 255, 255);
    //W_BIN = rangeColor(HLS, W_lower, W_upper)
    Mat W_BIN;
    inRange(HLS, W_lower, W_upper, W_BIN);
    //result = addImage(Y_BIN, W_BIN)
    addImage(Y_BIN, W_BIN, result);
    //MORPH_ELLIPSE = imageMorphologyKernel(cv2.MORPH_ELLIPSE, 7)
    Mat KERNEL_ELLIPSE;
    imageMorphologyKernel(KERNEL_ELLIPSE, MORPH_ELLIPSE, 7); 
    //result = imageMorphologyEx(result, cv2.MORPH_CLOSE , MORPH_ELLIPSE)
    imageMorphologyEx(result, result, MORPH_CLOSE, KERNEL_ELLIPSE,1); 
    //MORPH_CROSS = imageMorphologyKernel(cv2.MORPH_CROSS, 3)
    Mat KERNEL_CROSS;
    imageMorphologyKernel(KERNEL_CROSS, MORPH_CROSS, 3); 
    //result = imageMorphologyEx(result, cv2.MORPH_OPEN , MORPH_CROSS)
    imageMorphologyEx(result, result, MORPH_OPEN, KERNEL_CROSS,1); 
    //result_line = imageMorphologyEx(result, cv2.MORPH_GRADIENT , MORPH_CROSS)
    Mat result_line;
    imageMorphologyEx(result, result_line, MORPH_GRADIENT, KERNEL_CROSS,1); 
    //height, width = image.shape[:2]
    int width = image.cols;
    int height = image.rows;
    //src_pt1 = [int(width*0.4), int(height*0.65)]
    //src_pt2 = [int(width*0.6), int(height*0.65)]
    //src_pt3 = [int(width*0.9), int(height*0.9)]
    //src_pt4 = [int(width*0.1), int(height*0.9)]
    //roi_poly_02 = np.array([[tuple(src_pt1), tuple(src_pt2), tuple(src_pt3), tuple(src_pt4)]], dtype=np.int32)
    vector<Point> points;
    points.push_back(Point(int(width*0.4), int(height*0.65)));
    points.push_back(Point(int(width*0.6), int(height*0.65)));
    points.push_back(Point(int(width*0.9), int(height*0.9)));
    points.push_back(Point(int(width*0.1), int(height*0.9)));
    //line_roi = polyROI(result_line, roi_poly_02)
    Mat line_roi;
    polyROI(result_line, line_roi, points); 
    //lines = houghLinesP(line_roi, 1, np.pi/180, 10, 5, 10)
    vector<Vec4i> lines;
    imageHoughLinesP(line_roi, lines, 1, 3.141592/180., 10, 5, 10); 
    //result = lineFitting(image, lines, (0, 0, 255), 5, 5. * np.pi / 180.)
    lineFitting(image, result, lines, Scalar(0, 0, 255), 5, 3.141592 * 5. / 180.);
    return;
}
}
