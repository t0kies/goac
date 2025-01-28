#include <opencv2/opencv.hpp>       // OpenCV main library
#include <opencv2/aruco.hpp>        // ArUco marker detection module
#include <iostream>                 // Standard input/output stream library

// Smoothing speed for marker transitions (0 for instant, higher for smoother movement)
#define SMOOTHING_SPEED 0.1

// Interpolation function to linearly blend between two 3D points
cv::Point3f interpolatePoints3D(const cv::Point3f& start, const cv::Point3f& end, float factor) {
    return start + factor * (end - start);  // Linear interpolation
}

int main() {
    // Open the default camera
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Error: Unable to open the camera." << std::endl;
        return -1;
    }

    // Load a predefined ArUco dictionary for marker detection
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::DetectorParameters detectorParams;
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);

    // Load the GIF file
    cv::VideoCapture gifCapture("image.gif");
    if (!gifCapture.isOpened()) {
        std::cerr << "Error: Unable to open the GIF file." << std::endl;
        return -1;
    }

    // Extract GIF frames
    std::vector<cv::Mat> gifFrames;
    cv::Mat gifFrame;
    while (gifCapture.read(gifFrame)) {
        gifFrames.push_back(gifFrame.clone());
    }
    gifCapture.release();

    if (gifFrames.empty()) {
        std::cerr << "Error: No frames loaded from the GIF file." << std::endl;
        return -1;
    }

    size_t gifIndex = 0;  // Index for cycling through GIF frames

    // Camera intrinsic parameters (assumed values)
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 800, 0, 320, 0, 800, 240, 0, 0, 1);
    cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);  // Zero distortion coefficients

    // 3D coordinates of the marker corners (100x100 mm square)
    std::vector<cv::Point3f> markerCorners3D = {
        { -0.05f, -0.05f, 0 },  // Bottom-left
        {  0.05f, -0.05f, 0 },  // Bottom-right
        {  0.05f,  0.05f, 0 },  // Top-right
        { -0.05f,  0.05f, 0 }   // Top-left
    };

    cv::Vec3d rvec, tvec;  // Rotation and translation vectors for the marker
    bool markerDetected = false;  // Detection flag

    // Main processing loop
    while (true) {
        cv::Mat frame;
        cap >> frame;  // Capture a new frame
        if (frame.empty()) {
            std::cerr << "Error: Blank frame captured." << std::endl;
            break;
        }

        // Detect ArUco markers
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners;
        detector.detectMarkers(frame, markerCorners, markerIds);

        if (!markerIds.empty()) {
            // Draw detected markers on the frame
            cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);

            // Check for a specific marker ID (e.g., ID 1)
            for (size_t i = 0; i < markerIds.size(); i++) {
                if (markerIds[i] == 1) {  // Target marker ID
                    cv::solvePnP(markerCorners3D, markerCorners[i], cameraMatrix, distCoeffs, rvec, tvec);
                    markerDetected = true;
                }
            }
        }

        if (markerDetected) {
            // Define 3D coordinates of the virtual GIF plane
            std::vector<cv::Point3f> gif3D = {
                { -0.05f, 0.0f,  0.05f },  // Bottom-left
                {  0.05f, 0.0f,  0.05f },  // Bottom-right
                {  0.05f, 0.0f, -0.05f },  // Top-right
                { -0.05f, 0.0f, -0.05f }   // Top-left
            };

            // Project 3D points of the GIF plane into the 2D image plane
            std::vector<cv::Point2f> projectedGifCorners;
            cv::projectPoints(gif3D, rvec, tvec, cameraMatrix, distCoeffs, projectedGifCorners);

            // Resize the GIF frame to match the projected plane dimensions
            cv::Mat resizedGif;
            cv::resize(gifFrames[gifIndex], resizedGif, cv::Size(cv::norm(projectedGifCorners[1] - projectedGifCorners[0]),
                                                                 cv::norm(projectedGifCorners[3] - projectedGifCorners[0])));

            // Define the 2D quadrilateral corresponding to the resized GIF frame
            std::vector<cv::Point2f> gifQuad = {
                { 0.0f, static_cast<float>(resizedGif.rows) },  // Bottom-left
                { static_cast<float>(resizedGif.cols), static_cast<float>(resizedGif.rows) },  // Bottom-right
                { static_cast<float>(resizedGif.cols), 0.0f },  // Top-right
                { 0.0f, 0.0f }  // Top-left
            };

            // Compute the perspective transform and warp the GIF onto the detected plane
            cv::Mat perspectiveMatrix = cv::getPerspectiveTransform(gifQuad, projectedGifCorners);
            cv::warpPerspective(resizedGif, frame, perspectiveMatrix, frame.size(), cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);

            // Cycle through GIF frames
            gifIndex = (gifIndex + 1) % gifFrames.size();
        }

        // Display the frame
        cv::imshow("ArUco Marker Detection", frame);

        // Break loop on 'ESC' key press
        if (cv::waitKey(30) == 27) break;
    }

    // Release resources
    cap.release();
    cv::destroyAllWindows();
    return 0;
}
