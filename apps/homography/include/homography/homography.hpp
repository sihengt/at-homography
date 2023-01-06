#include "apriltags/TagDetector.h"
#include "apriltags/Tag16h5.h"
#include "apriltags/Tag25h7.h"
#include "apriltags/Tag25h9.h"
#include "apriltags/Tag36h9.h"
#include "apriltags/Tag36h11.h"

const bool DEBUG = false;

const char *windowName = "apriltag_detections";
const std::string SQUARES_X = "squares_x";
const std::string SQUARES_Y = "squares_x";
const std::string SQUARE_LENGTH_IN_METERS = "square_length_in_meters";
const std::string APRILTAGS = "aprilTags";
const std::string SPACE_BETWEEN_SQUARES = "space_between_squares"; // % of square edge length

struct Board
{
    /**
     * @brief Struct to hold configuration parameters read from YAML file.
     *
     * squares_x / squares_y dictate number of squares in each direction.
     * square_length_in_meters is the length of each square, in meters.
     * space_between_squares is a FRACTION of the length of each square.
     *
     * | *  | AT | *  | AT |
     * | AT | *  | AT | *  |
     * | *  | AT | *  | AT |
     * | AT | *  | AT | *  |
     */
    int squares_x;
    int squares_y;
    float space_between_squares;
    float square_length_in_meters;
};

class Homography
{
public:
    Homography() : m_tagDetector(NULL),
                   m_tagCodes(AprilTags::tagCodes36h11),
                   m_draw(true)
    {
    }

    void setup()
    /**
     * @brief Setup tag detector and create a named window for OpenCV to draw with.
     * 
     */
    {
        m_tagDetector = new AprilTags::TagDetector(m_tagCodes);
        if (m_draw)
        {
            cv::namedWindow(windowName, 1);
        }
    }

    void setCurrentImage(cv::Mat &image)
    /**
     * @brief Sets current image Homography class is working with at the moment.
     */
    {
        current_image = image;
    }
    void detectApriltags();
    void processDetections(std::vector<std::pair<int, std::vector<cv::Point2f>>> &at_detections);
    void generateAprilTagCoords(std::vector<cv::Point2f> &output);
    void generateStarCoords(std::vector<cv::Point2f> &output);
    void processAprilTagDetections(std::vector<std::pair<int, std::vector<cv::Point2f>>> &at_detections, std::vector<cv::Point2f> &at_detections_aligned);
    void drawStars(std::vector<cv::Point2f> &stars, cv::Mat &H);
    bool readBoardConfig(std::string board_yaml_fp);

private:
    AprilTags::TagCodes m_tagCodes;
    AprilTags::TagDetector *m_tagDetector;
    Board m_boardConfig;

    // Raw AprilTag Detections from running AT Detector
    std::vector<AprilTags::TagDetection> detections;

    // Map AprilTag code to corner coordinates.
    // Corner coordinates are counterclockwise from the same corner of tag.
    std::map<int, cv::Point2f> board_detections;
    std::map<int, cv::Point2f> image_detections;

    std::list<std::string> m_imgNames;
    cv::Mat current_image, current_image_gray;
    bool m_draw;


};