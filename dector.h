#ifndef DECTOR_H
#define DECTOR_H
#define M_PI 3.14159265358979323846
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <algorithm>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

using namespace cv;
using namespace std;

class Dector {
private:
    bool debug = false;
    int CAR_CENTRE_COL = 0;
    int CAR_CENTRE_ROW = 0;
    int LINE_WIDTH_MAX = 0;
    uchar LINE_WIDTH_MIN = 0;
    uchar TWOTIMES_WHITE_LINE_WIDTH_MAX = 0;
    uchar ONE_TIME_THRESHOLD = 0;
    uchar MEAN_CENTRE_GAP = 0;
    uchar EDGE_CENTRE_ROW_GAP_1 = 0;
    uchar EDGE_CENTRE_COL_GAP_1 = 0;
    uchar EDGE_CENTRE_ROW_GAP_2 = 0;
    uchar EDGE_CENTRE_COL_GAP_2 = 0;
    uchar EDGE_CENTRE_ROW_GAP_3 = 0;
    uchar EDGE_CENTRE_COL_GAP_3 = 0;
    uchar KERNELS_COUNT = 0;
    uchar MARKS_GAP = 0;
    uchar SAME_LINE_ANGLE = 0;
    uchar SAME_LINE_GAP_1 = 0;
    uchar SAME_LINE_GAP_2 = 0;
    uchar AREA_1 = 0;
    uchar AREA_2 = 0;
    uchar AREA_3 = 0;
    enum Direction {BOTTOM_LEFT, BOTTOM_RIGHT, TOP_RIGHT, TOP_LEFT,NO};
    struct Line{
        int row;
        int startCol;
        int endCol;
        int width;
        bool isBlack;
    };
    struct Kernel {
        int row;
        int col;
        int type;//1：定位标识；２：导航标识
    };
    int clnt_sock;

public:
    volatile double position_err = 0;
    volatile double angle_err = 0;
    volatile int decode_value = 0;
    int last_decode_value = 0;
    volatile int centre_y = 0;
    volatile int centre_x = 320;
    volatile int routeNodes[30];
    volatile int command[30];
    volatile int nodeIndex = 0;
    volatile int stopNum = 0;
    volatile bool readyToTurn = false;
    volatile bool stopDecode = false;
    volatile bool commandStop = false;
    int last_roi_x = 320;
    int imageCols = 0;
    int imageRows = 0;
    int roiCols = 640;
    int roiRows = 720;
    Dector();
    void cameraTest(int clnt);
    void videoTest(string, int clnt);
    void mediaStream(VideoCapture capture, int delay);
    void imageTest(string);
    void imageProcess(Mat &frame, Mat &thresholded);
    vector<Point> scanImageEfficiet(Mat&);
    void analyseLines(vector<Line>&lines, vector<Kernel>&, uchar*);
    vector<Point> analyseKernels(vector<Kernel> &);
    void analyseLocationKernels(vector<Kernel>&, vector<Kernel>&, vector<Kernel>&, vector<Point>&);
    void analyseNavigationKernels(vector<Kernel>&, vector<Point>&);
    Point denoiseAndCentralization(vector<Kernel>&);
    void findKernelsCentre(int*, int*, vector<Kernel> &);
    Point calculateMeanPointAndPush(uchar&, int&, int&, vector<Point> &kernelPoints);
    bool isValidPoint(Point &);
    vector<int> calculateEdges(vector<Point>&);
    int decodeImage(Mat&, vector<Point> &, vector<Point>&);
    Direction calculDirection(int, int, int, int , vector<int> edges);
    void errorMeasure(vector<Point>&, Mat&);
    void removeRepeat(vector<Point> &, int, int);
    void myPutText(string text, Mat src, int x, int y);
    int decode(vector<Point> &pt_local,vector<Point> &pt_encode);
    CvPoint transformPoint(const CvPoint pointToTransform, const Mat matrix);
};

#endif // DECTOR_H
