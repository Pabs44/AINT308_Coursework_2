//James Rogers Mar 2022 (c) Plymouth University
#include<iostream>
#include<fstream>
#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;

//a drawing function that can draw a line based on rho and theta values.
//useful for drawing lines from the hough line detector.
void lineRT(Mat &Src, Vec2f L, Vec3b color, int thickness){
    Point pt1, pt2;
    double a = cos(static_cast<double>(L[1]));
    double b = sin(static_cast<double>(L[1]));
    double x0 = a*static_cast<double>(L[0]);
    double y0 = b*static_cast<double>(L[0]);
    pt1.x = cvRound(x0 + 10000*(-b));
    pt1.y = cvRound(y0 + 10000*(a));
    pt2.x = cvRound(x0 - 10000*(-b));
    pt2.y = cvRound(y0 - 10000*(a));
    line(Src, pt1, pt2, color, thickness, LINE_AA);
}

int main(){

    //Open video file
    VideoCapture CarVideo("../Task5/DashCam.mp4");
    if(!CarVideo.isOpened()){
        cout<<"Error opening video"<<endl;
        return -1;
    }

    //main program loop
    while(true){

        //open the next frame from the video file, or exit the loop if its the end
        Mat Frame;
        CarVideo.read(Frame);
        if(Frame.empty()) break;

        //==========================Your code goes here==========================
        Mat outputFrame;
        Canny(Frame, outputFrame, 50, 100);

        int rhoRes = 1;
        int Threshold = 270;
        vector<Vec2f> lines;
        HoughLines(outputFrame, lines, rhoRes, M_PI/180, Threshold, 0, 0);

        Vec3b color = cv::Vec3b(250,0,40);
        for(int i = 0; i < (int)lines.size(); i++){
            if(lines[i](0) < 900){
                //left lane
                if(lines[i](0) > 0.1 && lines[i](1) > 0.1 && lines[i](1) < 1){
                    lineRT(Frame, lines[i], color, 2);
                    cout<<"new line "<<i<<endl;
                    cout<<lines[i](0)<<endl<<lines[i](1)<<endl;
                //right lane
                }else if(lines[i](0) < -150 && lines[i](0) > -300 && lines[i](1) >= 2.3 && lines[i](1) < 3){
                    lineRT(Frame, lines[i], color, 2);
                    cout<<"new line "<<i<<endl;
                    cout<<lines[i](0)<<endl<<lines[i](1)<<endl;
                }
            }
        }

        //display frame
        imshow("Video", Frame);
        imshow("Canny", outputFrame);
        waitKey(10);
    }
}



