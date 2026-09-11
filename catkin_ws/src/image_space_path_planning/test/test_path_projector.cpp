#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "image_space_path_planning/path_projector.hpp"

class PathProjectorTest : public ::testing::Test {
protected:
    PathProjector projector;
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    int image_width;
    int image_height;

    void SetUp() override {
        // Camera matrix with typical values
        camera_matrix = (cv::Mat_<double>(3, 3) <<
            525.0, 0.0, 319.5,
            0.0, 525.0, 239.5,
            0.0, 0.0, 1.0);
        
        // No distortion for simplicity
        dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);

        image_width = 640;
        image_height = 480;
    }
    
    void TearDown() override {
        // Clean up if needed
    }
};

// Test basic 2D projection functionality
TEST_F(PathProjectorTest, ProjectTo2D_Valid3DPoints_ProjectsCorrectly) {
    // Create 3D points: points as rows [x, y, z] format (N×3)
    Eigen::MatrixXf path_3d(3, 3);
    path_3d << 0.0f, 0.0f, 1.0f,   // Point 1: (0, 0, 1)
               1.0f, 1.0f, 2.0f,   // Point 2: (1, 1, 2)
              -1.0f, -1.0f, 2.0f;  // Point 3: (-1, -1, 2)
    
    Eigen::MatrixXf projected = projector.projectTo2D(path_3d, camera_matrix, image_width, image_height, dist_coeffs);
    
    // Verify output dimensions (should be N×2)
    ASSERT_EQ(projected.rows(), 3) << "Should have 3 projected points";
    ASSERT_EQ(projected.cols(), 2) << "Each point should have 2D coordinates";
    
    // Test center point (0,0,1) should project to principal point
    EXPECT_NEAR(projected(0, 0), 319.5, 1e-1) << "Center point x projection incorrect";
    EXPECT_NEAR(projected(0, 1), 239.5, 1e-1) << "Center point y projection incorrect";
    
    // Test point (1,1,2) should project to the right and down
    EXPECT_GT(projected(1, 0), 319.5) << "Point (1,1,2) should project right of center";
    EXPECT_GT(projected(1, 1), 239.5) << "Point (1,1,2) should project below center";
    
    // Test point (-1,-1,2) should project to the left and up
    EXPECT_LT(projected(2, 0), 319.5) << "Point (-1,-1,2) should project left of center";
    EXPECT_LT(projected(2, 1), 239.5) << "Point (-1,-1,2) should project above center";
}

// Test 3D reconstruction from 2D points and depth
TEST_F(PathProjectorTest, ProjectTo3D_Valid2DPoints_ReconstructsDepth) {
    // Create synthetic depth map with uniform depth
    cv::Mat depth_map = cv::Mat::ones(480, 640, CV_32FC1) * 2.0f;
    
    // 2D points to reconstruct
    Eigen::MatrixXf path_2d(3, 2);
    path_2d << 319.5f, 239.5f,  // Principal point
               400.0f, 300.0f,  // Off-center point
               100.0f, 100.0f;  // Corner point
    
    Eigen::MatrixXf path_3d = projector.projectTo3D(path_2d, depth_map, camera_matrix, dist_coeffs);
    
    // Verify output dimensions
    ASSERT_EQ(path_3d.rows(), 3) << "Should have 3 reconstructed points";
    ASSERT_EQ(path_3d.cols(), 3) << "Each point should have 3D coordinates";
    
    // All points should have the same depth as the depth map
    for (int i = 0; i < path_3d.rows(); ++i) {
        EXPECT_NEAR(path_3d(i, 2), 2.0f, 1e-3) << "Point " << i << " depth should match depth map";
    }
    
    // Principal point should reconstruct to approximately (0,0,2)
    EXPECT_NEAR(path_3d(0, 0), 0.0f, 1e-2) << "Principal point X coordinate incorrect";
    EXPECT_NEAR(path_3d(0, 1), 0.0f, 1e-2) << "Principal point Y coordinate incorrect";
}

// Test handling of invalid Z coordinates
TEST_F(PathProjectorTest, ProjectTo2D_InvalidZ_HandlesGracefully) {
    // Create points with invalid Z coordinates (points as rows, N×3)
    Eigen::MatrixXf path_3d(2, 3);
    path_3d << 1.0f, 2.0f, 0.0f,   // Point 1: (1, 2, 0) - invalid Z
               1.0f, 2.0f, -1.0f;  // Point 2: (1, 2, -1) - invalid Z
    
    Eigen::MatrixXf projected = projector.projectTo2D(path_3d, camera_matrix, image_width, image_height, dist_coeffs);
    
    // Should handle invalid points gracefully
    // Your implementation might return empty result or filter out invalid points
    EXPECT_EQ(projected.rows(), 0) << "Should ignore all invalid points";
    EXPECT_EQ(projected.cols(), 2) << "Should maintain 2D structure even with no valid points";
}

// Test edge case: empty input
TEST_F(PathProjectorTest, ProjectTo2D_EmptyInput_ReturnsEmpty) {
    Eigen::MatrixXf empty_path(0, 3);
    
    Eigen::MatrixXf projected = projector.projectTo2D(empty_path, camera_matrix, image_width, image_height, dist_coeffs);
    
    EXPECT_EQ(projected.rows(), 0) << "Empty input should return empty output";
    EXPECT_EQ(projected.cols(), 2) << "Should maintain 2D structure";
}

// Test with mixed valid/invalid points
TEST_F(PathProjectorTest, ProjectTo2D_MixedValidInvalid_FiltersCorrectly) {
    Eigen::MatrixXf path_3d(4, 3);
    path_3d << 0.0f, 0.0f, 1.0f,   // Point 1: (0, 0, 1) - valid
               1.0f, 1.0f, 2.0f,   // Point 2: (1, 1, 2) - valid  
               0.0f, 0.0f, 0.0f,   // Point 3: (0, 0, 0) - invalid Z
               2.0f, 2.0f, 3.0f;   // Point 4: (2, 2, 3) - valid
    
    Eigen::MatrixXf projected = projector.projectTo2D(path_3d, camera_matrix, image_width, image_height, dist_coeffs);
    
    EXPECT_EQ(projected.rows(), 3) << "Should keep only valid points";
    EXPECT_EQ(projected.cols(), 2) << "Should maintain 2D structure";
}

// Test reconstruction with varying depths
TEST_F(PathProjectorTest, ProjectTo3D_VaryingDepths_ReconstructsCorrectly) {
    // Create depth map with varying depths
    cv::Mat depth_map = cv::Mat::ones(480, 640, CV_32FC1);
    depth_map.at<float>(239, 319) = 1.0f;  // Center point
    depth_map.at<float>(300, 400) = 2.0f;  // Off-center point
    depth_map.at<float>(100, 100) = 3.0f;  // Corner point
    
    Eigen::MatrixXf path_2d(3, 2);
    path_2d << 319.0f, 239.0f,  // Principal point
               400.0f, 300.0f,  // Off-center point  
               100.0f, 100.0f;  // Corner point
    
    Eigen::MatrixXf path_3d = projector.projectTo3D(path_2d, depth_map, camera_matrix, dist_coeffs);
    
    ASSERT_EQ(path_3d.rows(), 3);
    ASSERT_EQ(path_3d.cols(), 3);
    
    // Check that each point has the correct depth
    EXPECT_NEAR(path_3d(0, 2), 1.0f, 1e-3) << "First point depth incorrect";
    EXPECT_NEAR(path_3d(1, 2), 2.0f, 1e-3) << "Second point depth incorrect";
    EXPECT_NEAR(path_3d(2, 2), 3.0f, 1e-3) << "Third point depth incorrect";
}

// Test boundary conditions
TEST_F(PathProjectorTest, ProjectTo3D_OutOfBounds_HandlesGracefully) {
    cv::Mat depth_map = cv::Mat::ones(480, 640, CV_32FC1) * 2.0f;
    
    // Points outside image boundaries
    Eigen::MatrixXf path_2d(2, 2);
    path_2d << -10.0f, 650.0f,   // Outside left and right boundaries
               -10.0f, 500.0f;   // Outside top and bottom boundaries
    
    // This test depends on how your implementation handles out-of-bounds
    // You might want to check if it throws an exception or returns empty results
    EXPECT_NO_THROW({
        Eigen::MatrixXf path_3d = projector.projectTo3D(path_2d, depth_map, camera_matrix, dist_coeffs);
    }) << "Should handle out-of-bounds points gracefully";
}