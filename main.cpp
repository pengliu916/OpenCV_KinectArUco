#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "RGBDStreamer/IRGBDStreamer.h"

#pragma warning(disable:4290)
#include <aruco.h>
#include <cvdrawingutils.h>

using namespace cv;
int main()
{
    IRGBDStreamer* pKinect2 =
        StreamFactory::createFromKinect2(true, true, true);
    pKinect2->StartStream();
    FrameData frames[IRGBDStreamer::kNumBufferTypes];
    uint16_t ColorWidth, ColorHeight, InfraredWidth, InfraredHeight;
    do
    {
        pKinect2->GetColorReso(ColorWidth, ColorHeight);
    } while (ColorWidth == 0 || ColorHeight == 0);

    aruco::MarkerDetector markerDetector;
    cv::Mat infMat, infDisplayMat, colMat, colDetectMat, colDisplayMat;

    float cameraParams[] = {
        1.0500229488335692e+03f, 0.f, 9.7136112531037020e+02f,
        0.f, 1.0497918256490759e+03f, 5.4918051628416595e+02f,
        0.f, 0.f, 1.f};
    float distorsionParams[] = {
        2.9149065825195806e-02f, -2.9654300241936486e-02f,
        -8.0282793629378685e-04f, -1.0993038057764051e-03f};
    cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32F, cameraParams);
    cv::Mat distorsionMatrix = cv::Mat(4, 1, CV_32F, distorsionParams);
    aruco::CameraParameters
        cp(cameraMatrix, distorsionMatrix, cv::Size(1920,1080));
    //Code for Visualize live video streams
    bool newData = false;
    while (true)
    {
        do {
            newData = pKinect2->GetNewFrames(
                frames[IRGBDStreamer::kColor],
                frames[IRGBDStreamer::kDepth],
                frames[IRGBDStreamer::kInfrared]);
        } while (!newData);

        if (frames[IRGBDStreamer::kColor].pData == nullptr) continue;
        colMat = cv::Mat(ColorHeight, ColorWidth, CV_8UC4,
            frames[IRGBDStreamer::kColor].pData);
        cvtColor(colMat, colMat, CV_BGRA2RGBA);
        cvtColor(colMat, colDetectMat, CV_BGRA2GRAY);

        float markerSize = 0.2f;
        vector<aruco::Marker> vMarkers =
            markerDetector.detect(colDetectMat, cp, markerSize);
        for (unsigned int i = 0; i < vMarkers.size(); i++) {
            cout << vMarkers[i] << endl;
            vMarkers[i].draw(colDetectMat, Scalar(0, 0, 255), 2);
        }
        for (unsigned int i = 0; i < vMarkers.size(); i++) {
            aruco::CvDrawingUtils::draw3dCube(colMat, vMarkers[i], cp);
        }
        imshow("Sensor0", colMat);
        // waitKey is essential for imshow to work properly in loop
        waitKey(10);
    }
}