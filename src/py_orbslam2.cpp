#include <opencv2/core/core.hpp>
#include <ORB_SLAM2/KeyFrame.h>
#include <ORB_SLAM2/Converter.h>
#include <ORB_SLAM2/Tracking.h>
#include <ORB_SLAM2/MapPoint.h>
#include "py_orbslam2.h"

namespace py = pybind11;

ORBSLAM2Python::ORBSLAM2Python(std::string vocabFile, std::string settingsFile, ORB_SLAM2::System::eSensor sensorMode)
    : vocabluaryFile(vocabFile),
      settingsFile(settingsFile),
      sensorMode(sensorMode),
      system(nullptr),
      bUseViewer(false),
      bUseRGB(true)
{
}

ORBSLAM2Python::~ORBSLAM2Python()
{
}

bool ORBSLAM2Python::initialize()
{
    system = std::make_shared<ORB_SLAM2::System>(vocabluaryFile, settingsFile, sensorMode, bUseViewer);
    return true;
}

bool ORBSLAM2Python::isRunning()
{
    return system != nullptr;
}

void ORBSLAM2Python::reset()
{
    if (system)
    {
        system->Reset();
    }
}

bool ORBSLAM2Python::loadAndProcessMono(std::string imageFile, double timestamp)
{
    if (!system)
    {
        return false;
    }
    cv::Mat im = cv::imread(imageFile, cv::IMREAD_COLOR);
    if (bUseRGB)
    {
        cv::cvtColor(im, im, cv::COLOR_BGR2RGB);
    }
    return this->processMono(im, timestamp);
}

bool ORBSLAM2Python::processMono(cv::Mat image, double timestamp)
{
    if (!system)
    {
        return false;
    }
    if (image.data)
    {
        cv::Mat pose = system->TrackMonocular(image, timestamp);
        return !pose.empty();
    }
    else
    {
        return false;
    }
}

bool ORBSLAM2Python::loadAndProcessStereo(std::string leftImageFile, std::string rightImageFile, double timestamp)
{
    if (!system)
    {
        return false;
    }
    cv::Mat leftImage = cv::imread(leftImageFile, cv::IMREAD_COLOR);
    cv::Mat rightImage = cv::imread(rightImageFile, cv::IMREAD_COLOR);
    if (bUseRGB)
    {
        cv::cvtColor(leftImage, leftImage, cv::COLOR_BGR2RGB);
        cv::cvtColor(rightImage, rightImage, cv::COLOR_BGR2RGB);
    }
    return this->processStereo(leftImage, rightImage, timestamp);
}

bool ORBSLAM2Python::processStereo(cv::Mat leftImage, cv::Mat rightImage, double timestamp)
{
    if (!system)
    {
        return false;
    }
    if (leftImage.data && rightImage.data)
    {
        cv::Mat pose = system->TrackStereo(leftImage, rightImage, timestamp);
        return !pose.empty();
    }
    else
    {
        return false;
    }
}

bool ORBSLAM2Python::loadAndProcessRGBD(std::string imageFile, std::string depthImageFile, double timestamp)
{
    if (!system)
    {
        return false;
    }
    cv::Mat im = cv::imread(imageFile, cv::IMREAD_COLOR);
    if (bUseRGB)
    {
        cv::cvtColor(im, im, cv::COLOR_BGR2RGB);
    }
    cv::Mat imDepth = cv::imread(depthImageFile, cv::IMREAD_UNCHANGED);
    return this->processRGBD(im, imDepth, timestamp);
}

bool ORBSLAM2Python::processRGBD(cv::Mat image, cv::Mat depthImage, double timestamp)
{
    if (!system)
    {
        return false;
    }
    if (image.data && depthImage.data)
    {
        cv::Mat pose = system->TrackRGBD(image, depthImage, timestamp);
        return !pose.empty();
    }
    else
    {
        return false;
    }
}

void ORBSLAM2Python::shutdown()
{
    if (system)
    {
        system->Shutdown();
        system.reset();
    }
}

void ORBSLAM2Python::setUseViewer(bool useViewer)
{
    bUseViewer = useViewer;
}

std::vector<Eigen::Matrix4f> ORBSLAM2Python::getTrajectory() const
{
    if (!system)
    {
        return std::vector<Eigen::Matrix4f>();
    }

    // This is copied from the ORB_SLAM2 System.SaveTrajectoryKITTI function, with some changes to output a python tuple.
    std::vector<ORB_SLAM2::KeyFrame *> vpKFs = system->GetKeyFrames();
    std::sort(vpKFs.begin(), vpKFs.end(), ORB_SLAM2::KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    // Of course, if we have no keyframes, then just use the identity matrix.
    cv::Mat Two = cv::Mat::eye(4, 4, CV_32F);
    if (vpKFs.size() > 0)
    {
        cv::Mat Two = vpKFs[0]->GetPoseInverse();
    }

    std::vector<Eigen::Matrix4f> trajectory;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    std::list<ORB_SLAM2::KeyFrame *>::iterator lRit = system->GetTracker()->mlpReferences.begin();
    std::list<double>::iterator lT = system->GetTracker()->mlFrameTimes.begin();
    for (std::list<cv::Mat>::iterator lit = system->GetTracker()->mlRelativeFramePoses.begin(), lend = system->GetTracker()->mlRelativeFramePoses.end(); lit != lend; lit++, lRit++, lT++)
    {
        ORB_SLAM2::KeyFrame *pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

        while (pKF != NULL && pKF->isBad())
        {
            ORB_SLAM2::KeyFrame *pKFParent;

            // std::cout << "bad parent" << std::endl;
            Trw = Trw * pKF->mTcp;
            pKFParent = pKF->GetParent();
            if (pKFParent == pKF)
            {
                // We've found a frame that is it's own parent, presumably a root or something. Break out
                break;
            }
            else
            {
                pKF = pKFParent;
            }
        }
        if (pKF != NULL && !pKF->isBad())
        {
            Trw = Trw * pKF->GetPose() * Two;

            cv::Mat Tcw = (*lit) * Trw;
            Eigen::Matrix4f eigTcw;

            eigTcw << Tcw.at<float>(0, 0), Tcw.at<float>(0, 1), Tcw.at<float>(0, 2), Tcw.at<float>(0, 3),
                Tcw.at<float>(1, 0), Tcw.at<float>(1, 1), Tcw.at<float>(1, 2), Tcw.at<float>(1, 3),
                Tcw.at<float>(2, 0), Tcw.at<float>(2, 1), Tcw.at<float>(2, 2), Tcw.at<float>(2, 3),
                Tcw.at<float>(3, 0), Tcw.at<float>(3, 1), Tcw.at<float>(3, 2), Tcw.at<float>(3, 3);
            trajectory.push_back(eigTcw.inverse());
        }
    }

    return trajectory;
}

PYBIND11_MODULE(orbslam2, m)
{
    py::enum_<ORB_SLAM2::Tracking::eTrackingState>(m, "TrackingState")
        .value("SYSTEM_NOT_READY", ORB_SLAM2::Tracking::eTrackingState::SYSTEM_NOT_READY)
        .value("NO_IMAGES_YET", ORB_SLAM2::Tracking::eTrackingState::NO_IMAGES_YET)
        .value("NOT_INITIALIZED", ORB_SLAM2::Tracking::eTrackingState::NOT_INITIALIZED)
        .value("OK", ORB_SLAM2::Tracking::eTrackingState::OK)
        .value("LOST", ORB_SLAM2::Tracking::eTrackingState::LOST);

    py::enum_<ORB_SLAM2::System::eSensor>(m, "Sensor")
        .value("MONOCULAR", ORB_SLAM2::System::eSensor::MONOCULAR)
        .value("STEREO", ORB_SLAM2::System::eSensor::STEREO)
        .value("RGBD", ORB_SLAM2::System::eSensor::RGBD);

    py::class_<ORBSLAM2Python>(m, "system")
        .def(py::init<std::string, std::string, ORB_SLAM2::System::eSensor>())
        .def("initialize", &ORBSLAM2Python::initialize)
        .def("load_and_process_mono", &ORBSLAM2Python::loadAndProcessMono)
        .def("process_image_mono", &ORBSLAM2Python::processMono)
        .def("load_and_process_stereo", &ORBSLAM2Python::loadAndProcessStereo)
        .def("process_image_stereo", &ORBSLAM2Python::processStereo)
        .def("load_and_process_rgbd", &ORBSLAM2Python::loadAndProcessRGBD)
        .def("process_image_rgbd", &ORBSLAM2Python::processRGBD)
        .def("shutdown", &ORBSLAM2Python::shutdown)
        .def("is_running", &ORBSLAM2Python::isRunning)
        .def("reset", &ORBSLAM2Python::reset)
        .def("set_use_viewer", &ORBSLAM2Python::setUseViewer)
        .def("get_trajectory", &ORBSLAM2Python::getTrajectory);
}