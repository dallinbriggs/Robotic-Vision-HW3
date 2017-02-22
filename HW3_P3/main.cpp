#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/videoio.hpp"
#include <fstream>
#include <string>

using namespace cv;
using namespace std;

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if  ( event == EVENT_LBUTTONDOWN )
    {
        cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
        vector<Point2f> *pts = (vector<Point2f>*)userdata;
        pts->push_back(Point2f(x,y));
    }
    else if  ( event == EVENT_RBUTTONDOWN )
    {
        cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
    }
    else if  ( event == EVENT_MBUTTONDOWN )
    {
        cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
    }

}


int main(int argc, char *argv[])
{
    Mat image_left;
    Mat image_color_left;
    Mat image_right;
    Mat image_color_right;
    Mat image_undistort_left;
    Mat image_undistort_right;
    Size patternsize;
    string filename_left;
    string filename_right;
    string header_left;
    string header_right;
    string tail;
    Size imageSize;
    Mat distCoeffs_left;
    Mat distCoeffs_right;
    Mat cameraMatrix_left;
    Mat cameraMatrix_right;
    Mat fundamental_matrix;
    vector<Vec3f> lines_left;
    vector<Vec3f> lines_right;

    vector<Point2f> point_left;
    vector<Point2f> point_right;

    Mat R, T, E, F;

    cameraMatrix_left = Mat::eye(3, 3, CV_64F);
    cameraMatrix_right = Mat::eye(3, 3, CV_64F);


    vector<vector<Point2f>> imagePoints_left;

    vector<vector<Point2f>> imagePoints_right;
    distCoeffs_left = Mat::zeros(8, 1, CV_64F);
    distCoeffs_right = Mat::zeros(8, 1, CV_64F);
    imagePoints_left.clear();
    imagePoints_right.clear();
    int numCornersHor = 10;
    int numCornersVer = 7;
    int numSquares = numCornersHor * numCornersVer;

    vector<Point3f> obj;
    for(int j=0;j<numSquares;j++)
        obj.push_back(Point3f(j/numCornersHor*3.88636, j%numCornersHor*3.88636, 0.0f));

    header_left = "/home/dallin/robotic_vision/HW3/Images/stereoL1";
    header_right = "/home/dallin/robotic_vision/HW3/Images/stereoR1";
    tail = ".bmp";
    patternsize = cvSize(10,7);
    imageSize = cvSize(640,480);


    filename_left = header_left + tail;
    filename_right = header_right + tail;

    image_left = imread(filename_left,CV_LOAD_IMAGE_GRAYSCALE);
    image_color_left = imread(filename_left,CV_LOAD_IMAGE_COLOR);

    image_right = imread(filename_right,CV_LOAD_IMAGE_GRAYSCALE);
    image_color_right = imread(filename_right,CV_LOAD_IMAGE_COLOR);

    FileStorage fs_left("Intrinsic_calibration_baseball_left.xml", FileStorage::READ);

    fs_left["CameraMatrix"] >> cameraMatrix_left;
    fs_left["DistortionCoefficients"] >> distCoeffs_left;
    FileStorage fs_right("Intrinsic_calibration_baseball_right.xml", FileStorage::READ);
    fs_right["CameraMatrix"] >> cameraMatrix_right;
    fs_right["DistortionCoefficients"] >> distCoeffs_right;

    undistort(image_color_left,image_undistort_left,cameraMatrix_left,distCoeffs_left,noArray());
    undistort(image_color_right,image_undistort_right,cameraMatrix_right,distCoeffs_right,noArray());

    imshow("Left", image_undistort_left);
    imshow("right", image_undistort_right);
    moveWindow("right",650,30);


    while(point_left.size() < 3)
    {
        setMouseCallback("Left", CallBackFunc, &point_left);
        waitKey(1);
    }

    while(point_right.size() < 3)
    {
        setMouseCallback("right", CallBackFunc, &point_right);
        waitKey(1);
    }

    cout << point_left << endl;
    cout << point_right << endl;

    for(int i=0; i < 3; i++)
    {
        circle(image_undistort_left,point_left[i],1,Scalar(0,255,0),7,LINE_8,0);
    }
    //    imshow("Left", image_undistort_left);

    for(int i=0; i < 3; i++)
    {
        circle(image_undistort_right,point_right[i],1,Scalar(0,255,0),7,LINE_8,0);
    }
    //    imshow("right", image_undistort_right);

    FileStorage fs_stereo("Extrinsic_calibration_baseball.xml", FileStorage::READ);
    fs_stereo["F"] >> fundamental_matrix;

    computeCorrespondEpilines(point_left,1,fundamental_matrix,lines_left);
    computeCorrespondEpilines(point_right,2,fundamental_matrix,lines_right);

    for (std::vector<cv::Vec3f>::const_iterator it = lines_left.begin(); it!=lines_left.end(); ++it)
    {
        // Draw the line between first and last column
        cv::line(image_undistort_right,
                 cv::Point(0,-(*it)[2]/(*it)[1]),
                cv::Point(image_undistort_right.cols,-((*it)[2]+
                          (*it)[0]*image_undistort_right.cols)/(*it)[1]),
                cv::Scalar(0,0,255));
    }
    for (std::vector<cv::Vec3f>::const_iterator it = lines_right.begin(); it!=lines_right.end(); ++it)
    {
        // Draw the line between first and last column
        cv::line(image_undistort_left,
                 cv::Point(0,-(*it)[2]/(*it)[1]),
                cv::Point(image_undistort_left.cols,-((*it)[2]+
                          (*it)[0]*image_undistort_left.cols)/(*it)[1]),
                cv::Scalar(0,0,255));
    }

    imshow("Left", image_undistort_left);
    imshow("right", image_undistort_right);

    waitKey(0);

    return 0;
}

