#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "RGBDStreamer/IRGBDStreamer.h"

#include "IObjectTracking.h"
#pragma comment(lib, "TrackingDLL.lib")

#pragma warning(disable:4290)
#include <aruco.h>
#include <cvdrawingutils.h>

#define MARKER_MAP 1
#define USE_KINECT 1

double cameraParams[] = {
    1.0500229488335692e+03f, 0.f, 9.7136112531037020e+02f,
    0.f, 1.0497918256490759e+03f, 5.4918051628416595e+02f,
    0.f, 0.f, 1.f };
double distorsionParams[] = {
    2.9149065825195806e-02f, -2.9654300241936486e-02f,
    -8.0282793629378685e-04f, -1.0993038057764051e-03f };

// For \Vicon
Tracking::IObjectTracking* pViconTracker;
std::string camName = "TestVirtualCamera";
std::string boardName = "RedPlane";

// For Kinect
IRGBDStreamer* pKinect2;
FrameData frames[IRGBDStreamer::kNumBufferTypes];
uint16_t ColorWidth, ColorHeight;

// For aruco
float markerSize = 0.038f;
// Read marker map
aruco::MarkerMap TheMarkerMapConfig;
aruco::MarkerDetector markerDetector;
std::vector<aruco::Marker> vMarkers;

cv::Mat inputColMat, outputColMat, colDetectMat, tmp;

char userInput;

void ChangeUpVector(double pData[9])
{
    double tmp = pData[3];
    pData[3] = pData[6];
    pData[6] = tmp;
    tmp = pData[4];
    pData[4] = pData[7];
    pData[7] = tmp;
    tmp = pData[5];
    pData[5] = pData[8];
    pData[8] = tmp;
    pData[6] *= -1.f;
    pData[7] *= -1.f;
    pData[8] *= -1.f;

    tmp = pData[1];
    pData[1] = pData[2];
    pData[2] = -tmp;
    tmp = pData[4];
    pData[4] = pData[5];
    pData[5] = -tmp;
    tmp = pData[7];
    pData[7] = pData[8];
    pData[8] = -tmp;
}

void getPosistion(const std::string& name, cv::Mat& result)
{
    Tracking::Pos pos;
    pViconTracker->GetPosition(name, name, pos);
    // Y up to Z up
    result.at<double>(0, 0) = pos.X;
    result.at<double>(0, 1) = pos.Z;
    result.at<double>(0, 2) = -pos.Y;
}

void getRotationMat(const std::string& name, cv::Mat& R)
{
    static double pMat[9];
    pViconTracker->GetRotationMat(name, name, pMat);
    ChangeUpVector(pMat);
    R = (cv::Mat_<double>(3, 3) << pMat[0], pMat[1], pMat[2], pMat[3],
        pMat[4], pMat[5], pMat[6], pMat[7], pMat[8]);
}

int main()
{
    outputColMat = cv::Mat(1080, 1920, CV_8UC4);
    colDetectMat = cv::Mat(1080, 1920, CV_8UC1);
    tmp = cv::Mat(1080, 1920, CV_8UC4);
#if USE_KINECT
    // Prepare Kinect
    pKinect2 = StreamFactory::createFromKinect2(true, true, true);
    pKinect2->StartStream();
    pKinect2->GetColorReso(ColorWidth, ColorHeight);
#endif
    // Prepare params for sensor
    cv::Mat cameraMatrix = cv::Mat(3, 3, CV_64F, cameraParams);
    cv::Mat distorsionMatrix = cv::Mat(4, 1, CV_64F, distorsionParams);
    aruco::CameraParameters
        cp(cameraMatrix, distorsionMatrix, cv::Size(ColorWidth, ColorHeight));

    // Prepare Vicon
    pViconTracker = Tracking::ObjectTrackingFactory::create();
    pViconTracker->Initialize("Vicon");
    std::cout << pViconTracker->GetInfo().c_str() << std::endl;
    do {
        pViconTracker->ConnectServer();
        std::cout << pViconTracker->GetInfo().c_str() << std::endl;
    } while (!pViconTracker->IsConnected());

    // Read marker size if indicated
    TheMarkerMapConfig.readFromFile("config.yml");
    markerDetector.setDictionary(TheMarkerMapConfig.getDictionary());

#if MARKER_MAP
    // Transform the markersetconfig to meter if is in pixels and the
    // markersize indicated
    if (TheMarkerMapConfig.isExpressedInPixels() && markerSize > 0) {
        TheMarkerMapConfig = TheMarkerMapConfig.convertToMeters(markerSize);
    }
    // Tracks the pose of the marker map
    aruco::MarkerMapPoseTracker MSPoseTracker;
    MSPoseTracker.setParams(cp, TheMarkerMapConfig);

    std::vector<int> markers_from_set;
#endif

    cv::Mat vBoardPos_cam = cv::Mat(3, 1, CV_64F);
    cv::Mat mBoardOri_cam = cv::Mat(3, 3, CV_64F);
    cv::Mat mBoardInvOri_cam = cv::Mat(3, 3, CV_64F);

    cv::Mat vSensorPos_world = cv::Mat(3, 1, CV_64F);
    cv::Mat mSensorOri_world = cv::Mat(3, 3, CV_64F);
    cv::Mat mSensorInvOri_world = cv::Mat(3, 3, CV_64F);

    cv::Mat vCamPos_world = cv::Mat(3, 1, CV_64F);
    cv::Mat mCamOri_world = cv::Mat(3, 3, CV_64F);
    cv::Mat mCamInvOri_world;

    cv::Mat vBoardPos_world = cv::Mat(3, 1, CV_64F);
    cv::Mat mBoardInvOri_world;
    cv::Mat mBoardOri_world = cv::Mat(3, 3, CV_64F);

    cv::Mat vBoardPos_sensor = cv::Mat(3, 1, CV_64F);
    cv::Mat mBoardOri_sensor = cv::Mat(3, 3, CV_64F);
    cv::Mat mBoardInvOri_sensor;

    cv::Mat mCam2Sensor = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat mSensor2Cam = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat vCam2Sensor_world = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat vCam2Sensor_cam = cv::Mat::zeros(3, 1, CV_64F);

    while ('q' != (userInput = cv::waitKey(1))) {
#if USE_KINECT
        pKinect2->GetNewFrames(frames[IRGBDStreamer::kColor],
            frames[IRGBDStreamer::kDepth], frames[IRGBDStreamer::kInfrared]);

        if (frames[IRGBDStreamer::kColor].pData == nullptr) {
            continue;
        }
        inputColMat = cv::Mat(ColorHeight, ColorWidth, CV_8UC4,
            frames[IRGBDStreamer::kColor].pData);
#else
        inputColMat = cv::Mat(1080, 1920, CV_8UC4, cv::Scalar(1, 1, 1, 1));
#endif

        if (pViconTracker->IsConnected()) {
            pViconTracker->Update();
            getPosistion(camName, vCamPos_world);
            getRotationMat(camName, mCamOri_world);
            getPosistion(boardName, vBoardPos_world);
            getRotationMat(boardName, mBoardOri_world);
        }

        cv::cvtColor(inputColMat, tmp, CV_BGRA2RGBA);
        // Kinect2 sensor already did a flip, so to keep pattern recognizable
        // we need to flip it back;
        cv::flip(tmp, outputColMat, 1);
        cv::cvtColor(outputColMat, colDetectMat, CV_BGRA2GRAY);


        cv::Mat zeroT = cv::Mat::zeros(3, 1, CV_64F);
        zeroT.at<double>(0, 2) = 1.5f;

        vMarkers.clear();
        vMarkers = markerDetector.detect(colDetectMat, cp, markerSize);
#if MARKER_MAP
        markers_from_set.clear();
        markers_from_set = TheMarkerMapConfig.getIndices(vMarkers);
        /*for (auto idx : markers_from_set) {
            vMarkers[idx].draw(outputColMat, cv::Scalar(0, 0, 255), 2);
        }*/

        // Detect the 3d camera location wrt the markerset (if possible)
        if (cp.isValid() && MSPoseTracker.estimatePose(vMarkers)) {
            // If pose correctly computed, print the reference system
            aruco::CvDrawingUtils::draw3dAxis(outputColMat, cp,
                //MSPoseTracker.getRvec(), zeroT,
                MSPoseTracker.getRvec(), MSPoseTracker.getTvec(),
                TheMarkerMapConfig[0].getMarkerSize() * 2);
            vBoardPos_sensor = MSPoseTracker.getTvec().clone();
            vBoardPos_sensor.convertTo(vBoardPos_sensor, CV_64F);
            transpose(vBoardPos_sensor, vBoardPos_sensor);
            mBoardOri_sensor = MSPoseTracker.getRTMatrix()(cv::Rect(0,0,3,3));
            mBoardOri_sensor.convertTo(mBoardOri_sensor, CV_64F);
            transpose(mBoardOri_sensor, mBoardInvOri_sensor);
        }
#else
        for (unsigned int i = 0; i < vMarkers.size(); i++) {
                //cout << vMarkers[i] << endl;
                vMarkers[i].draw(outputColMat, cv::Scalar(0, 0, 255), 1);
            }
        for (unsigned int i = 0; i < vMarkers.size(); i++) {
            //aruco::CvDrawingUtils::draw3dCube(colMat, vMarkers[i], cp);
            aruco::CvDrawingUtils::draw3dAxis(outputColMat, vMarkers[i], cp);
        }
#endif
        cv::transpose(mCamOri_world, mCamInvOri_world);
        vBoardPos_cam = mCamInvOri_world * (vBoardPos_world - vCamPos_world);

        cv::transpose(mBoardOri_world, mBoardInvOri_world);
        mBoardOri_cam = mCamInvOri_world * mBoardOri_world;

        if (userInput == 'c') {
            mSensorOri_world =
                mCamOri_world * mBoardOri_cam * mBoardInvOri_sensor;
            transpose(mSensorOri_world, mSensorInvOri_world);
            vSensorPos_world = mCamOri_world * vBoardPos_cam +
                vCamPos_world - mSensorOri_world * vBoardPos_sensor;
            vCam2Sensor_cam = mCamInvOri_world * (vSensorPos_world - vCamPos_world);

            mCam2Sensor =
                mBoardOri_sensor * mBoardInvOri_world * mCamOri_world;
            transpose(mCam2Sensor, mSensor2Cam);
            std::cout << "Calibration Done" << std::endl;
        }

        cv::Mat trans;
        transpose( mCam2Sensor * (vBoardPos_cam - vCam2Sensor_cam), trans);
        std::cout << trans << std::endl;
        aruco::CvDrawingUtils::draw3dAxis(outputColMat, cp, mCam2Sensor * mBoardOri_cam,
            trans, 0.15);
        cv::imshow("Sensor0", outputColMat);
        // WaitKey is essential for imshow to work properly in loop
        cv::waitKey(10);
    }
}