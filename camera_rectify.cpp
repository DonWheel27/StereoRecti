//读取原始图像，进行图像矫正（去畸变，进行极线矫正）
//储存画出极线的矫正后图片与未画出极线的矫正后的图片
//输出矫正后的左相机内参矩阵，原左相机坐标系到校正后相机坐标系的旋转矩阵等数据

#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;

void img_rectify_save(std::string p, std::vector<cv::String> images, cv::Mat frame, cv::Mat img_fixed, cv::Mat mapx, cv::Mat mapy, cv::Rect validROI)
{
    //std::string folderpath1 = "D:\\C++Project\\VS Project\\StereoRecti\\Rectified\\" + p;
    //std::string folderpath2 = "D:\\C++Project\\VS Project\\StereoRecti\\Epipolar_line\\" + p;

    //for verification
    //std::string folderpath1 = "D:\\C++Project\\VS Project\\StereoRecti\\verification_data\\Rectified\\" + p;
   // std::string folderpath2 = "D:\\C++Project\\VS Project\\StereoRecti\\verification_data\\Epipolar_line\\" + p;
    //637-double
    std::string folderpath1 = "D:\\C++Project\\VS Project\\StereoRecti\\637-double\\Rectified\\" + p;
    std::string folderpath2 = "D:\\C++Project\\VS Project\\StereoRecti\\637-double\\Epipolar_line\\" + p;
    for (int i = 0; i < images.size(); i++)
    {
        frame = cv::imread(images[i]);
        cv::Size size = frame.size();
        cv::remap(frame, img_fixed, mapx, mapy, cv::INTER_LINEAR);
        // std::cout<< folderpath + "\\rectified_"+ p + std::to_string(i+1) + ".bmp" << endl;
        cv::imwrite(folderpath1 + "\\rectified_" + p + std::to_string(i + 1) + ".bmp", img_fixed);

        for (int i = 1, iend = 20; i < iend; i++)
        {
            int h = img_fixed.rows / iend * i;
            if (p == "L") {
                cv::line(img_fixed, Point2i(0, h), Point2i(img_fixed.cols, h), Scalar(0, 0, 255), 3);
                cv::rectangle(img_fixed, validROI, cv::Scalar(0, 255, 0), 6);
            }
            else {
                cv::line(img_fixed, Point2i(0, h), Point2i(img_fixed.cols, h), Scalar(0, 255, 0), 3);
                cv::rectangle(img_fixed, validROI, cv::Scalar(0, 0, 255), 6);
            }

        }

        cv::imwrite(folderpath2 + "\\Epipolar_line_" + p + std::to_string(i + 1) + ".bmp", img_fixed);

        //cv::resize(img_fixed, img_fixed, cv::Size(img_fixed.cols /2, img_fixed.rows /2));
        //cv::imshow(p + std::to_string(i+1) + "_after_rectify", img_fixed);
        //cv::waitKey(0);
    }

}




int main() {

    double L_cameraM[3][3] = { 4.415583390204002e+03, 0, 1.018290501188417e+03,
                                   0, 4.414605221314402e+03, 7.318330112995719e+02,
                                   0, 0, 1 };
    cv::Mat L_cameraMatrix = cv::Mat(3, 3, cv::DataType<double>::type, L_cameraM);

    double L_distCo[1][5] = { -0.033660669512576, 0.458309867067065, 3.918431140134439e-04, 4.666233834787876e-05, -3.202272054753638 };
    cv::Mat L_distCoeffs = cv::Mat(1, 5, cv::DataType<double>::type, L_distCo);

    double R_cameraM[3][3] = { 4.412466490709043e+03, 0, 1.018614330010497e+03,
                                   0, 4.411877082176653e+03, 7.769501716573342e+02,
                                   0, 0, 1 };
    cv::Mat R_cameraMatrix = cv::Mat(3, 3, cv::DataType<double>::type, R_cameraM);

    double R_distCo[1][5] = { -0.018498880079274, 0.263926594075295, 2.817189203019057e-04, 8.039769482077263e-04, -3.270709149566655 };
    cv::Mat R_distCoeffs = cv::Mat(1, 5, cv::DataType<double>::type, R_distCo);

    //R和T是stereocalibrate计算出来的，Cam1坐标到Cam2坐标的旋转矩阵与平移向量
    double R_stereo[3][3] = { 0.951262423824121, 0.001131643109195, 0.308380479933689,
                               -7.379359412671605e-04, 0.999998757061960, -0.001393314423171,
                              -0.308381673370526, 0.001097842615607, 0.951262076543991 };
    cv::Mat R = cv::Mat(3, 3, cv::DataType<double>::type, R_stereo);

    double T_stereo[3][1] = { -1.596194776835307e+02, 0.232442979012112, 24.576936372234500 };
    cv::Mat T = cv::Mat(3, 1, cv::DataType<double>::type, T_stereo);


    //std::string L_path = "D:\\C++Project\\learning_opencv\\calibration_test\\LR_images\\Left\\*.bmp";
    //std::string R_path = "D:\\C++Project\\learning_opencv\\calibration_test\\LR_images\\Right\\*.bmp";

    //for verification
    //std::string L_path = "D:\\C++Project\\VS Project\\StereoRecti\\verification_data\\L\\637\\*.bmp";
    //std::string R_path = "D:\\C++Project\\VS Project\\StereoRecti\\verification_data\\R\\637\\*.bmp";

    //static table
    //std::string L_path = "D:\\C++Project\\VS Project\\StereoRecti\\637-static-table\\637-static-table\\L\\*.bmp";
    //std::string R_path = "D:\\C++Project\\VS Project\\StereoRecti\\637-static-table\\637-static-table\\R\\*.bmp";
    //rotating table
    //std::string L_path = "D:\\C++Project\\VS Project\\StereoRecti\\637-rotating-table\\637-rotating-table\\L\\*.bmp";
    //std::string R_path = "D:\\C++Project\\VS Project\\StereoRecti\\637-rotating-table\\637-rotating-table\\R\\*.bmp";
    //vertical-slider
    std::string L_path = "D:\\C++Project\\VS Project\\StereoRecti\\637-double\\637-double\\L\\*.bmp";
    std::string R_path = "D:\\C++Project\\VS Project\\StereoRecti\\637-double\\637-double\\R\\*.bmp";


    cv::Mat testing = cv::imread("D:\\C++Project\\VS Project\\StereoRecti\\637-double\\637-double\\L\\L01.bmp");

    cv::Size size = testing.size();

    std::vector<cv::String> L_images, R_images;

    cv::glob(L_path, L_images);
    cv::glob(R_path, R_images);

    std::cout << "********************************************************" << '\n' << endl;
    std::cout << "Stereo camera params: " << endl;
    std::cout << "Left_cameraMatrix : " << L_cameraMatrix << endl;
    std::cout << "Left_cam_distCoeffs : " << L_distCoeffs << endl;
    std::cout << "Right_cameraMatrix : " << R_cameraMatrix << endl;
    std::cout << "Right_cam_distCoeffs : " << R_distCoeffs << endl;
    std::cout << "rotaion matrix from right Cam to left Cam is: " << R << endl;
    std::cout << "tramsition vector from right Cam to left Cam is: " << T << endl;
    std::cout << "All params loaded successfully!" << '\n' << endl;
    std::cout << "********************************************************" << '\n' << endl;
    //定义相机坐标系矫正矩阵，把相机坐标系进行旋转实现极线矫正
    //R1，R2把矫正前相机坐标系下的点坐标转换到矫正后的相机坐标系中
    //P1把校正后Cam1坐标系的坐标投影到Cam1像素坐标，P2把矫正后Cam1坐标系的坐标投影到Cam2像素坐标
    //P1是Cam1矫正后的新内参矩阵
    cv::Mat R1, R2, P1, P2, Q;
    cv::Rect L_validROI, R_validROI;

    cv::stereoRectify(L_cameraMatrix, L_distCoeffs, R_cameraMatrix, R_distCoeffs,
                      size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, size, &L_validROI, &R_validROI);


    std::cout << "********************************************************" << '\n' << endl;
    std::cout << "Left rectified rotation matrix R1: " << R1 << endl;
    std::cout << "Left new camera matrix P1: " << P1 << endl;
    std::cout << "Right rectified rotation matrix R2: " << R2 << endl;
    std::cout << "Right new camera matrix P2: " << P2 << endl;
    std::cout << "disparity-to-depth 4x4 mapping matrix Q: " << Q << endl;
    std::cout << "Left image valid ROI: " << L_validROI << endl;
    std::cout << "Right image valid ROI: " << R_validROI << endl;
    std::cout << "********************************************************" << '\n' << endl;

    std::ofstream outputFile("Rectify_data.txt");
    outputFile << "Left rectified rotation matrix R1: " << '\n' << R1 << '\n' << endl;
    outputFile << "Left new camera matrix P1: " << '\n' << P1 << '\n' << endl;
    outputFile << "Right rectified rotation matrix R2: " << '\n' << R2 << '\n' << endl;
    outputFile << "Right new camera matrix P2: " << '\n' << P2 << '\n' << endl;
    outputFile << "disparity-to-depth 4x4 mapping matrix Q: " << '\n' << Q << '\n' << endl;
    outputFile << "Left image valid ROI: " << '\n' << L_validROI << '\n' << endl;
    outputFile << "Right image valid ROI: " << '\n' << R_validROI << '\n' << endl;
    outputFile.close();

    //定义像素坐标变换矩阵mapx，mapy
    cv::Mat frame, frame0;
    cv::Mat L_mapx, L_mapy, R_mapx, R_mapy;
    cv::Mat L_img_fixed, R_img_fixed;
    std::string p1 = "L";
    std::string p2 = "R";

    cv::Rect new_cameraMatrix_roi(0, 0, 3, 3);
    P1 = P1(new_cameraMatrix_roi);
    P2 = P2(new_cameraMatrix_roi);

    //得到映射关系图
    cv::initUndistortRectifyMap(L_cameraMatrix, L_distCoeffs, R1, P1, size, CV_32F, L_mapx, L_mapy);
    cv::initUndistortRectifyMap(R_cameraMatrix, R_distCoeffs, R2, P2, size, CV_32F, R_mapx, R_mapy);

    img_rectify_save(p1, L_images, frame, L_img_fixed, L_mapx, L_mapy, L_validROI);
    img_rectify_save(p2, R_images, frame, R_img_fixed, R_mapx, R_mapy, R_validROI);

    return 0;
}