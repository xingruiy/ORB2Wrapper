#ifndef ORB_SLAM2_PYTHON_H
#define ORB_SLAM2_PYTHON_H

#include <memory>
#include <ORB_SLAM2/System.h>
#include <ORB_SLAM2/Tracking.h>

class ORBSLAM2Python
{
public:
    ORBSLAM2Python(std::string vocabFile, std::string settingsFile,
                   ORB_SLAM2::System::eSensor sensorMode = ORB_SLAM2::System::eSensor::RGBD);
    ~ORBSLAM2Python();

    bool initialize();
    bool processMono(cv::Mat image, double timestamp);
    bool loadAndProcessMono(std::string imageFile, double timestamp);
    bool processStereo(cv::Mat leftImage, cv::Mat rightImage, double timestamp);
    bool loadAndProcessStereo(std::string leftImageFile, std::string rightImageFile, double timestamp);
    bool processRGBD(cv::Mat image, cv::Mat depthImage, double timestamp);
    bool loadAndProcessRGBD(std::string imageFile, std::string depthImageFile, double timestamp);
    void reset();
    void shutdown();
    bool isRunning();
    void setUseViewer(bool useViewer);
    std::vector<Eigen::Matrix4f> getTrajectory() const;

private:
    std::string vocabluaryFile;
    std::string settingsFile;
    ORB_SLAM2::System::eSensor sensorMode;
    std::shared_ptr<ORB_SLAM2::System> system;
    bool bUseViewer;
    bool bUseRGB;
};

#endif // ORB_SLAM2_PYTHON_H