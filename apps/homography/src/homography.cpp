#include <iostream>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

#include <homography.hpp>

// April tags detector and various families that can be selected by command line option
#include "apriltags/TagDetector.h"
#include "apriltags/Tag16h5.h"
#include "apriltags/Tag25h7.h"
#include "apriltags/Tag25h9.h"
#include "apriltags/Tag36h9.h"
#include "apriltags/Tag36h11.h"

std::string type2str(int type)
{
    // Helper function to return the type of matrix in string form.
    std::string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch (depth)
    {
    case CV_8U:
        r = "8U";
        break;
    case CV_8S:
        r = "8S";
        break;
    case CV_16U:
        r = "16U";
        break;
    case CV_16S:
        r = "16S";
        break;
    case CV_32S:
        r = "32S";
        break;
    case CV_32F:
        r = "32F";
        break;
    case CV_64F:
        r = "64F";
        break;
    default:
        r = "User";
        break;
    }

    r += "C";
    r += (chans + '0');

    return r;
}

/**
 * @brief Detects AprilTags using fork of Kaess AT3.
 *
 * Images have to be in grayscale for AT detection to work.
 */
void Homography::detectApriltags()
{
    // Converts image to grayscale
    cv::cvtColor(current_image, current_image_gray, cv::COLOR_BGR2GRAY);
    detections = m_tagDetector->extractTags(current_image_gray);
    // std::cout << detections.size() << " tags detected." << std::endl;

    if (m_draw)
    {
        for (int i = 0; i < detections.size(); i++)
        {
            detections[i].draw(current_image);
        }
        cv::imshow(windowName, current_image); // OpenCV call
    }
}

/**
 * @brief Converts AprilTag detections from customized format into a vector of Point2f.
 *
 * @param at_detections vector containing pairs of [apriltag_id, apriltag points].
 */
void Homography::processDetections(std::vector<std::pair<int, std::vector<cv::Point2f>>> &at_detections)
{
    std::cout << "processDetections" << std::endl;
    // Going through raw AprilTag detections from Kaess AT3 library and appending them to 
    for (AprilTags::TagDetection detection : detections)
    {
        int current_id = detection.id;
        std::vector<cv::Point2f> temp_points;

        // Each detection always has four detections (one on each corner). Pushes these detections
        // into a vector.
        for (int i = 0; i < 4; i++)
            temp_points.push_back(cv::Point2f(detection.p[i].first, detection.p[i].second));
        at_detections.push_back(std::pair<int, std::vector<cv::Point2f>>(current_id, temp_points));
    }

    // For debugging your detections
    if (DEBUG)
    {
        for (auto detect : at_detections)
        {
            std::cout << "apriltag_id=" << detect.first << std::endl;
            for (int i = 0; i < 4; i++)
                std::cout << "\t(x=" << detect.second[i].x << "\ty=" << detect.second[i].y << ")" << std::endl;
        }
    }
}

/**
 * @brief Generates a vector of points containing where the stars would be on the board using geometry.
 *
 * @param board_config
 * @param output
 */
void Homography::generateStarCoords(std::vector<cv::Point2f> &output)
{
    // We'll create a board using milimeters as the coordinate system.
    float square_length_mm = m_boardConfig.square_length_in_meters * 1000.f;
    float space_between_squares_mm = m_boardConfig.space_between_squares * square_length_mm;
    float current_x = 0.f;
    // Stars start at the second square of the board
    float current_y = (float)m_boardConfig.squares_y * square_length_mm + (m_boardConfig.squares_y - 1) * space_between_squares_mm - (0.5 * square_length_mm);

    std::cout << "debugging\t square_length_mm:" << square_length_mm << "\tspace_between_squares_mm" << space_between_squares_mm << "\tcurrent_x: " << current_x << "\tcurrent_y:" << current_y << std::endl;

    // We're multiplying squares_x and y by two because there are two corners per square in each Apriltag column/row.
    for (int i_row = 0; i_row < m_boardConfig.squares_x; i_row++)
    {
        if (i_row % 2 != 0)
            current_x = 0.5 * square_length_mm;
        else
            current_x = 0;
        for (int i_col = 0; i_col < m_boardConfig.squares_y; i_col++)
        {
            if (i_row % 2 == 0)
            {
                if (i_col % 2 == 0)
                    current_x += square_length_mm + space_between_squares_mm + 0.5 * square_length_mm;
                else
                {
                    output.push_back(cv::Point2f(current_x, current_y));
                    current_x += 0.5 * square_length_mm + space_between_squares_mm;
                }
            }
            else
            {
                if (i_col % 2 != 0)
                    current_x += square_length_mm + space_between_squares_mm + 0.5 * square_length_mm;
                else
                {
                    output.push_back(cv::Point2f(current_x, current_y));
                    current_x += 0.5 * square_length_mm + space_between_squares_mm;
                }
            }
        }
        current_y -= square_length_mm + space_between_squares_mm;
    }
}

/**
 * @brief Generates coordinates of all corners of AprilTags on the board.
 * 
 * @param output 
 */
void Homography::generateAprilTagCoords(std::vector<cv::Point2f> &output)
{
    // We'll create a board using milimeters as the coordinate system.
    float square_length_mm = m_boardConfig.square_length_in_meters * 1000;
    float space_between_squares_mm = m_boardConfig.space_between_squares * square_length_mm;
    float current_x = 0;
    // Apriltags start at the bottom left of the board. This brings us to our first corner at the bottom left of the board.
    float current_y = m_boardConfig.squares_y * square_length_mm + (m_boardConfig.squares_y - 1) * space_between_squares_mm;

    std::cout << "debugging\t square_length_mm:" << square_length_mm << "\tspace_between_squares_mm" << space_between_squares_mm << "\tcurrent_x: " << current_x << "\tcurrent_y:" << current_y << std::endl;

    // We're multiplying squares_x and y by two because there are two corners per square in each Apriltag column/row.
    for (int i_row = 0; i_row < m_boardConfig.squares_x * 2; i_row++)
    {
        if ((i_row / 2 % 2) == 0)
            current_x = 0;
        else
            current_x = square_length_mm + space_between_squares_mm;

        for (int i_col = 0; i_col < m_boardConfig.squares_y; i_col++)
        {
            if (i_col == 0)
            {
            } // do nothing
            else if (i_col % 2 != 0)
                current_x += square_length_mm;
            else
                current_x += 2 * space_between_squares_mm + square_length_mm;
            output.push_back(cv::Point2f(current_x, current_y));
        }

        if (i_row % 2 == 0)
            current_y -= square_length_mm;
        else
            current_y -= space_between_squares_mm;
    }
}


/**
 * @brief 
 * 
 * Apriltag detections work CCW, with first index at the bottom left corner:
 * 
 * [3]     [2]
 *   -------
 *   |     |
 *   |     |
 *   -------
 * [0]     [1]
 * 
 * @param at_detections 
 * @param at_detections_aligned 
 */
void Homography::processAprilTagDetections(std::vector<std::pair<int, std::vector<cv::Point2f>>> &at_detections, std::vector<cv::Point2f> &at_detections_aligned)
{
    // This is way too messy. TODO: fix if we need this implementation.
    // Essentially I'm hoping to iterate through all rows and columns, and append the bottom two corners of all tags first, then the top two corners of all tags.
    // After appending bottom/top corners, then we go to the next row, noting the pattern change (star first then AT.)
    // current_apriltag_id will always be board_config.squares_x * i_row + i_col
    for (int i_row = 0; i_row < m_boardConfig.squares_y; i_row++)
    {
        for (int i_col = 0; i_col < m_boardConfig.squares_x; i_col++)
        {
            int current_apriltag_id = m_boardConfig.squares_x * i_row + i_col;
            std::cout << "current_apriltag_id:" << current_apriltag_id << std::endl;

            // Case 1: On even rows (starting from 0), apriltag will always be on first and alternating columns.
            if (i_row % 2 == 0)
            {
                if (i_col % 2 == 0)
                {
                    for (auto detect : at_detections)
                    {
                        std::cout << "current_apriltag_detected:" << detect.first << std::endl;
                        if (detect.first == current_apriltag_id)
                        {
                            at_detections_aligned.push_back(detect.second[0]);
                            at_detections_aligned.push_back(detect.second[1]);
                        }
                    }
                }
                // Skip tags not part of this column
                else
                {
                    continue;
                }
            }
            else
            {
                // for odd rows (beginning from index of 0), take only apriltags on odd cols.
                if (i_col % 2 != 0)
                {
                    for (auto detect : at_detections)
                    {
                        if (detect.first == current_apriltag_id)
                        {
                            at_detections_aligned.push_back(detect.second[0]);
                            at_detections_aligned.push_back(detect.second[1]);
                        }
                    }
                }
                else
                {
                    continue;
                }
            }
        }

        for (int i_col = 0; i_col < m_boardConfig.squares_x; i_col++)
        {
            int current_apriltag_id = m_boardConfig.squares_x * i_row + i_col;
            if (i_row % 2 == 0)
            {
                // process tag for upper corners (i.e. 0, 1)
                if (i_col % 2 == 0)
                {
                    for (auto detect : at_detections)
                    {
                        if (detect.first == current_apriltag_id)
                        {
                            at_detections_aligned.push_back(detect.second[3]);
                            at_detections_aligned.push_back(detect.second[2]);
                        }
                    }
                }
                // skip tags that are not an apriltag within this row.
                else
                {
                    continue;
                }
            }
            else
            {
                // for odd rows (beginning from index of 0), take only apriltags on odd cols.
                if (i_col % 2 != 0)
                {
                    for (auto detect : at_detections)
                    {
                        if (detect.first == current_apriltag_id)
                        {
                            at_detections_aligned.push_back(detect.second[3]);
                            at_detections_aligned.push_back(detect.second[2]);
                        }
                    }
                }
                else
                {
                    continue;
                }
            }
        }
    }

    for (auto detect : at_detections_aligned)
    {
        std::cout << "(" << detect.x << "," << detect.y << ")" << std::endl;
    }
}

/**
 * @brief Reads yaml generated by "createTargetFinal.py" script and stores it into configuration struct.
 * 
 * @param board_yaml_fp filepath to the board's yaml file describing. 
 * @return true if board configuration is read successfully. false otherwise.
 */
bool Homography::readBoardConfig(std::string board_yaml_fp)
{
    YAML::Node board_yaml_node = YAML::LoadFile(board_yaml_fp);
    // Printing out input read it from board YAML.
    if (board_yaml_node[SQUARES_X] &&
        board_yaml_node[SQUARES_Y] &&
        board_yaml_node[APRILTAGS][SQUARE_LENGTH_IN_METERS] &&
        board_yaml_node[SPACE_BETWEEN_SQUARES])
    {
        m_boardConfig.squares_x = board_yaml_node[SQUARES_X].as<int>();
        m_boardConfig.squares_y = board_yaml_node[SQUARES_Y].as<int>();
        m_boardConfig.square_length_in_meters = board_yaml_node[APRILTAGS][SQUARE_LENGTH_IN_METERS].as<float>();
        m_boardConfig.space_between_squares = board_yaml_node[SPACE_BETWEEN_SQUARES].as<float>();

        if (DEBUG)
        {
            std::cout << "squares_x: " << board_yaml_node[SQUARES_X].as<int>() << "\n";
            std::cout << "squares_y: " << board_yaml_node[SQUARES_Y].as<int>() << "\n";
            std::cout << "square_length_in_meters: " << board_yaml_node[APRILTAGS][SQUARE_LENGTH_IN_METERS].as<float>() << "\n";
            std::cout << "space_between_squares: " << board_yaml_node[SPACE_BETWEEN_SQUARES].as<float>() << "\n";
        }
    }
    else
    {
        std::cout << "YAML file does not contain necessary parameters (squares_x/y, square_length_in_meters, space_between_squares)" << std::endl;
        return false;
    }
}

void Homography::drawStars(std::vector<cv::Point2f> &stars, cv::Mat &H)
{
    std::vector<cv::Point3d> homo_stars;
    std::vector<cv::Mat> transformed_stars;

    // Initializing an array to hold data of all star coordinates.

    for (auto star : stars)
    {
        homo_stars.push_back(cv::Point3d(star.x, star.y, 1));
        // double star_data[3][1] = {star.x, star.y, 1};

        // cv::Mat to_push(3, 1, CV_64FC1, star_data);
        // std::cout << "to_push:" << to_push << std::endl;
        // homo_stars.push_back(cv::Mat(3, 1, CV_64FC1, star_data));
    }

    // Converting the matrix from a vector of point3Ds to a matrix of size 3xn
    cv::Mat homo_stars_matrix = cv::Mat(homo_stars).reshape(1).t();
    // std::cout << "homo_stars_mat" << homo_stars_matrix << std::endl;
    // std::cout << "homo_stars_mat.rows=" << homo_stars_matrix.rows << "\tcols:" << homo_stars_matrix.cols << std::endl;
    std::cout << "H*homo_stars_mat" << H * homo_stars_matrix << std::endl;
    cv::Mat result = H * homo_stars_matrix;

    // Convert result into point2f for plotting.
    for (int n = 0; n < result.cols; n++)
    {
        cv::Mat c = result.col(n);
        // some form of standardization lol.
        c.row(0) = c.row(0) / c.row(2);
        c.row(1) = c.row(1) / c.row(2);
        c.row(2) = c.row(2) / c.row(2);

        cv::circle(current_image, cv::Point2f(c.at<double>(0, 0), c.at<double>(1, 0)), 8, cv::Scalar(255, 0, 0, 0), 2);
    }

    cv::imshow(windowName, current_image);
    cv::waitKey(0);
}

int main()
{
    Homography h;
    std::vector<std::pair<int, std::vector<cv::Point2f>>> at_detections;

    h.setup();

    // Read image. Temporary; to substitute with reading multiple images.
    cv::Mat image = cv::imread("/home/siheng/dha/at_homography/data/images/IMG_5574.jpg", 1);
    cv::Mat imageS;
    cv::resize(image, imageS, cv::Size(cv::Point2i(0, 0)), 0.4, 0.4);
    h.setCurrentImage(imageS);

    h.detectApriltags();
    h.processDetections(at_detections);
    cv::waitKey(0);

    h.readBoardConfig("/home/siheng/dha/at_homography/data/board/experiment_target.yaml");
    YAML::Node board_yaml_node = YAML::LoadFile("/home/siheng/dha/at_homography/data/board/experiment_target.yaml");
    Board board_config;
    std::vector<cv::Point2f> board_points, star_points, at_points;

    h.generateAprilTagCoords(board_points);
    h.generateStarCoords(star_points);
    h.processAprilTagDetections(at_detections, at_points);

    cv::Mat homography;
    homography = cv::findHomography(board_points, at_points);
    std::cout << "M = " << std::endl
              << " " << homography << std::endl
              << std::endl;

    std::cout << "H type:" << type2str(homography.type()) << std::endl;
    h.drawStars(star_points, homography);

    std::vector<int> apriltag_ids;
    h.findRelevantAprilTags(apriltag_ids);

    return 0;
}