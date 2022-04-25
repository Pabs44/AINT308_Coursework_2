//James Rogers Nov 2020 (c) Plymouth University
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <stdio.h>

using namespace cv;
using namespace std;

int main()
{
    //Calibration file paths (you need to make these)
    string intrinsic_filename = "../intrinsics.xml";
    string extrinsic_filename = "../extrinsics.xml";

    //================================================Load Calibration Files===============================================
    //This code loads in the intrinsics.xml and extrinsics.xml calibration files, and creates: map11, map12, map21, map22.
    //These four matrices are used to distort the camera images to apply the lense correction.
    Rect roi1, roi2;
    Mat Q;
    Size img_size = {640,480};

    FileStorage fs(intrinsic_filename, FileStorage::READ);
    if(!fs.isOpened()){
        printf("Failed to open file %s\n", intrinsic_filename.c_str());
        return -1;
    }
    Mat M1, D1, M2, D2;
    fs["M1"] >> M1;
    fs["D1"] >> D1;
    fs["M2"] >> M2;
    fs["D2"] >> D2;

    fs.open(extrinsic_filename, FileStorage::READ);
    if(!fs.isOpened()){
        printf("Failed to open file %s\n", extrinsic_filename.c_str());
        return -1;
    }
    Mat R, T, R1, P1, R2, P2;
    fs["R"] >> R;
    fs["T"] >> T;

    stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );

    Mat map11, map12, map21, map22;
    initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
    initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

    //===============================================Stereo SGBM Settings==================================================
    //This sets up the block matcher, which is used to create the disparity map. The various settings can be changed to
    //obtain different results. Note that some settings will crash the program.

    int SADWindowSize=5;            //must be an odd number >=3
    int numberOfDisparities=256;    //must be divisable by 16

    Ptr<StereoSGBM> sgbm = StereoSGBM::create(0,16,3);
    sgbm->setBlockSize(SADWindowSize);
    sgbm->setPreFilterCap(63);
    sgbm->setP1(8*3*SADWindowSize*SADWindowSize);
    sgbm->setP2(32*3*SADWindowSize*SADWindowSize);
    sgbm->setMinDisparity(0);
    sgbm->setNumDisparities(numberOfDisparities);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(32);
    sgbm->setDisp12MaxDiff(1);
    sgbm->setMode(StereoSGBM::MODE_SGBM);

    //==================================================Main Program Loop================================================
    int ImageNum=0; //current image index
    while (1){
        //Load images from file (needs changing for known distance targets)
        Mat Left =imread("../Task4/Unknown Targets/left" +to_string(ImageNum)+".jpg");
        Mat Right=imread("../Task4/Unknown Targets/right"+to_string(ImageNum)+".jpg");
        cout<<"Loaded image: "<<ImageNum<<endl;

        //Distort image to correct for lens/positional distortion
        remap(Left, Left, map11, map12, INTER_LINEAR);
        remap(Right, Right, map21, map22, INTER_LINEAR);

        //Match left and right images to create disparity image
        Mat disp16bit, disp8bit;
        sgbm->compute(Left, Right, disp16bit);                               //compute 16-bit greyscalse image with the stereo block matcher
        disp16bit.convertTo(disp8bit, CV_8U, 255/(numberOfDisparities*16.)); //Convert disparity map to an 8-bit greyscale image so it can be displayed (Only for imshow, do not use for disparity calculations)

        //==================================Your code goes here===============================
        ushort pixel_disp_print = disp16bit.at<ushort>(240,380);
        int pixel_dist_print = 62500/pixel_disp_print;
        cout<<pixel_disp_print<<endl;
        cout<<pixel_dist_print<<" cm"<<endl;

        //Sweep whole image for disparity of pixels and place in matrix
        int y = 480, x = 640;
        Mat distanceMap(y, x, CV_16U);
        ushort pixel_dist = 0, old_pixel_dist = 0;
        for(int ydx=0; ydx<y; ydx++){
            for(int xdx=0; xdx<x; xdx++){
                ushort pixel_disp = disp16bit.at<ushort>(ydx,xdx); //check current pixel disparity
                if(pixel_disp != 0){
                    old_pixel_dist = pixel_dist;
                    pixel_dist = 62500/pixel_disp;
                }else pixel_dist = old_pixel_dist;
                if(pixel_dist > 20) pixel_dist = pixel_dist-10;
                distanceMap.at<ushort>(ydx,xdx) = pixel_dist;
            }
        }
        /*pixel_dist = 0, old_pixel_dist = 0;
        for(int ydx=200; ydx<270; ydx++){
            for(int xdx=310; xdx<480; xdx++){
                ushort pixel_disp = disp16bit.at<ushort>(ydx,xdx); //check current pixel disparity
                if(pixel_disp != 0){
                    old_pixel_dist = pixel_dist;
                    pixel_dist = 62500/pixel_disp;
                }else{
                    pixel_dist = old_pixel_dist;
                }
                pixelDistance.at<ushort>(ydx,xdx) += pixel_dist+1000;
            }
        }*/

        cout<<distanceMap.size<<endl;
        //display images until x is pressed
        int key=0;
        while(waitKey(10)!='x'){
            imshow("left", Left);
            imshow("right", Right);
            imshow("disparity", disp8bit);
            imshow("distance", distanceMap);
        }

        //move to next image
        ImageNum++;
        if(ImageNum>7){
            ImageNum=0;
        }
    }

    return 0;
}





