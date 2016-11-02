#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "RGBDStreamer/IRGBDStreamer.h"

#pragma warning(disable:4290)
#include <aruco.h>
#include <cvdrawingutils.h>

#define MARKER_MAP 0

float cameraParams[] = {
    1.0500229488335692e+03f, 0.f, 9.7136112531037020e+02f,
    0.f, 1.0497918256490759e+03f, 5.4918051628416595e+02f,
    0.f, 0.f, 1.f };
float distorsionParams[] = {
    2.9149065825195806e-02f, -2.9654300241936486e-02f,
    -8.0282793629378685e-04f, -1.0993038057764051e-03f };

int main()
{
    IRGBDStreamer* pKinect2 =
        StreamFactory::createFromKinect2(true, true, true);
    pKinect2->StartStream();
    FrameData frames[IRGBDStreamer::kNumBufferTypes];
    uint16_t ColorWidth, ColorHeight;
    pKinect2->GetColorReso(ColorWidth, ColorHeight);

    cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32F, cameraParams);
    cv::Mat distorsionMatrix = cv::Mat(4, 1, CV_32F, distorsionParams);
    aruco::CameraParameters
        cp(cameraMatrix, distorsionMatrix, cv::Size(ColorWidth, ColorHeight));

    cv::Mat colMat, colDetectMat, colDisplayMat, tmp;

    // Read marker size if indicated
    float markerSize = 0.37f;
    // Read marker map
    aruco::MarkerMap TheMarkerMapConfig;
    TheMarkerMapConfig.readFromFile("config.yml");
    aruco::MarkerDetector markerDetector;
    markerDetector.setDictionary(TheMarkerMapConfig.getDictionary());
    std::vector<aruco::Marker> vMarkers;

#if MARKER_MAP
    // Transform the markersetconfig to meter if is in pixels and the
    // markersize indicated
    if (TheMarkerMapConfig.isExpressedInPixels() && markerSize > 0)
        TheMarkerMapConfig = TheMarkerMapConfig.convertToMeters(markerSize);
    // Tracks the pose of the marker map
    aruco::MarkerMapPoseTracker MSPoseTracker;
    MSPoseTracker.setParams(cp, TheMarkerMapConfig);

    std::vector<int> markers_from_set;
#endif

    bool newData = false;
    while (true)
    {
        do {
            newData = pKinect2->GetNewFrames(
                frames[IRGBDStreamer::kColor],
                frames[IRGBDStreamer::kDepth],
                frames[IRGBDStreamer::kInfrared]);
        } while (!newData);

        if (frames[IRGBDStreamer::kColor].pData == nullptr) {
            continue;
        }
        colMat = cv::Mat(ColorHeight, ColorWidth, CV_8UC4,
            frames[IRGBDStreamer::kColor].pData);

        cv::cvtColor(colMat, tmp, CV_BGRA2RGBA);
        // Kinect2 sensor already did a flip, so to keep pattern recognizable
        // we need to flip it back;
        cv::flip(tmp, colMat, 1);
        cv::cvtColor(colMat, colDetectMat, CV_BGRA2GRAY);

        vMarkers.clear();
        vMarkers = markerDetector.detect(colDetectMat, cp, markerSize);

#if MARKER_MAP
        markers_from_set.clear();
        markers_from_set = TheMarkerMapConfig.getIndices(vMarkers);
        for (auto idx : markers_from_set) {
            vMarkers[idx].draw(colMat, cv::Scalar(0, 0, 255), 2);
        }

        // Detect the 3d camera location wrt the markerset (if possible)
        if (cp.isValid() && MSPoseTracker.estimatePose(vMarkers)) {
            // If pose correctly computed, print the reference system
            aruco::CvDrawingUtils::draw3dAxis(colMat, cp,
                MSPoseTracker.getRvec(), MSPoseTracker.getTvec(),
                TheMarkerMapConfig[0].getMarkerSize() * 2);
        }
#else
        for (unsigned int i = 0; i < vMarkers.size(); i++) {
                //cout << vMarkers[i] << endl;
                vMarkers[i].draw(colMat, cv::Scalar(0, 0, 255), 1);
            }
        for (unsigned int i = 0; i < vMarkers.size(); i++) {
            //aruco::CvDrawingUtils::draw3dCube(colMat, vMarkers[i], cp);
            aruco::CvDrawingUtils::draw3dAxis(colMat, vMarkers[i], cp);
        }
#endif
        cv::imshow("Sensor0", colMat);
        // WaitKey is essential for imshow to work properly in loop
        cv::waitKey(10);
    }
}