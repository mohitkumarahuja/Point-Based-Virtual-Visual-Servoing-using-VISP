#include "MyFreenectDevice.hpp"
#include <iostream>

#include <visp/vpImageIo.h>
#include <visp/vpIoTools.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDot2.h>
#include <visp/vpPoint.h>
#include <visp/vpPose.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpRobotCamera.h>
#include <visp/vpServo.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpFeatureBuilder.h>




using namespace cv;
using namespace std;

int main(int argc, char **argv) {
    vpImage<unsigned char> I(480,640);
    vpDisplayX d(I, 50, 50, "output image");
    Freenect::Freenect freenect;
    MyFreenectDevice& device = freenect.createDevice<MyFreenectDevice>(0);
    device.startVideo();

    //*******************************************
    //*************TODO by STUDENTS**************
    //*******************************************



    // PART 1
    // get the kienct streaming
    while(1){
        device.getVideo(I);
        vpDisplay::display(I);
        vpDisplay::flush(I);
        if (vpDisplay::getClick(I, false))
            break;
    }

    //track dot
    vpDot2 detectDot;
    detectDot.setGraphics(true);
    detectDot.initTracking(I); //initilaise dot

    while(1){
        device.getVideo(I);
        vpDisplay::display(I);

        detectDot.track(I);
        vpDisplay::flush(I);

        if (vpDisplay::getClick(I, false))
            break;
    }

    //PART 2
    //set intrinsic parameters
    double au = 535;
    double av = 535;
    double u0 = 320;
    double v0 = 240;
    vpCameraParameters camParam(au, av, u0, v0);

    //define four world points
    std::vector<vpPoint> pt(4);
    pt[0].setWorldCoordinates(-0.10, 0.10, 0.0);
    pt[1].setWorldCoordinates(-0.10, -0.10, 0.0);
    pt[2].setWorldCoordinates(0.10, -0.10, 0.0);
    pt[3].setWorldCoordinates(0.10, 0.10, 0.0);

    //four dots for tracking
    std::vector<vpDot2> dot(4);
    dot[0].initTracking(I);
    dot[1].initTracking(I);
    dot[2].initTracking(I);
    dot[3].initTracking(I);

    vpHomogeneousMatrix cMo; //object in camera frame

    while(1){

        vpDisplay::display(I);
        //track four dots
        for (unsigned int i=0; i < dot.size(); i ++) {
            dot[i].setGraphics(true);
            dot[i].track(I);
        }

        //convert meters to pixels using intrinsics
        vpPose pose;     double x=0, y=0;
        for (unsigned int i=0; i < pt.size(); i ++) {
            vpPixelMeterConversion::convertPoint(camParam, dot[i].getCog(), x, y);
            pt[i].set_x(x);
            pt[i].set_y(y);
            pose.addPoint(pt[i]);
        }
        pose.computePose(vpPose::LAGRANGE, cMo); //compute pose of object in camera frame

        // show the intial frame of object in camera frame
        device.getVideo(I);
        //vpDisplay::displayFrame(I, cMo, camParam, 0.05, vpColor::red);
        detectDot.track(I);
        vpDisplay::flush(I);

        std::cout << "Part2";

        //PART 3

        // virtual free flying robot
        vpRobotCamera roboCam;
        //Seting initial pose
        roboCam.setPosition(cMo);

        vpServo vpSer;
        vpSer.setServo(vpServo::EYEINHAND_CAMERA);
        vpSer.setInteractionMatrixType(vpServo::CURRENT, vpServo::PSEUDO_INVERSE);

        //desired pose
        vpHomogeneousMatrix cdMo(0, 0, 0.75, 0, 0, 0);

        vpFeaturePoint pd[4], p[4];
        for (int i=0; i<4; i++){
            pt[i].track(cdMo);
            vpFeatureBuilder::create(pd[i], pt[i]);

        }

        std::cout << "Part21";

        for (unsigned int iter=0; iter < 150; iter ++) {
            for (int i=0; i<4; i++){
            pt[i].changeFrame(cMo);
            pt[i].projection();

            pt[i].track(cMo);
            vpFeatureBuilder::create(p[i], pt[i]);
            vpSer.addFeature(p[i], pd[i]);
        }
        vpColVector v = vpSer.computeControlLaw();
        roboCam.setVelocity(vpRobot::CAMERA_FRAME, v);
        roboCam.getPosition(cMo);
std::cout << "Part22";

        device.getVideo(I);
        vpDisplay::displayFrame(I, cMo, camParam, 0.05, vpColor::blue);
        vpDisplay::displayFrame(I, cdMo, camParam, 0.05, vpColor::red);
        vpDisplay::flush(I);
        }
        std::cout << cMo[0] << " " << cMo[1] << " " << cMo[2] << " " << cMo[3] << " " << cMo[4] << " " << cMo[5] ;
        if (vpDisplay::getClick(I, false))
            break;
    }

    //*******************************************
    //*******************************************
    //*******************************************


    device.stopVideo();

    return 0;
}
