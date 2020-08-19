#include <iostream>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>

using namespace std;
using namespace cv;
using namespace rs2;

const float g_min_dist = 0.50;
const float g_max_dist = 0.85;

Mat g_element = getStructuringElement(MORPH_RECT, Size(7, 7));

int main() try
{
	rs2::colorizer color_map;
	rs2::context ctx;
	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);
	cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
	rs2::align align_to(RS2_STREAM_COLOR);
	rs2::pipeline pipe;
	pipe.start(cfg);
	for (int i = 0; i < 30; i++)
	{
		//  Wait for all configured streams to produce a frame
		pipe.wait_for_frames();
	}

	namedWindow("color_img", WINDOW_AUTOSIZE);

	while (1) {
		//  realsenseD435 拍摄到的帧
		frameset frames = pipe.wait_for_frames();

		//  获取RGB图
		//  对齐图像
		frameset aligned_set = align_to.process(frames);
		frame color_frames = aligned_set.get_color_frame();
		

		//  帧转化为Mat 尺寸为RGB帧的尺寸
		Mat color_img = Mat(Size(color_frames.as<video_frame>().get_width(),
			color_frames.as<video_frame>().get_height()), CV_8UC3, (void*)color_frames.get_data(), Mat::AUTO_STEP);
		cvtColor(color_img, color_img, COLOR_BGR2RGB);
		imshow("color_img", color_img);

		depth_frame depth_frames = aligned_set.get_depth_frame();
		//// 从着色的深度数据中创建OpenCV大小（w，h）的OpenCV矩阵
		frame depth_frames_show = depth_frames.apply_filter(color_map);
		Mat depth_img = Mat(Size(depth_frames_show.as<video_frame>().get_width(),
			depth_frames_show.as<video_frame>().get_height()), CV_8UC3, (void*)depth_frames_show.get_data(), Mat::AUTO_STEP);
		imshow("depth_img", depth_img);

		Mat bin_img = Mat::zeros(color_img.rows, color_img.cols, CV_8UC1);
		
		for (int i = 0; i < color_img.cols; ++i) {
			for (int j = 0; j < color_img.rows; ++j) {
				float distance = depth_frames.get_distance(i, j);
				if (g_min_dist <= distance & distance <= g_max_dist) {
					bin_img.at<uchar>(j, i) = 255;
				}
				else {
					bin_img.at<uchar>(j, i) = 0;
				}
			}
		}
		erode(bin_img, bin_img, g_element);
		
		dilate(bin_img, bin_img, g_element);
		
		imshow("bin_img", bin_img);

		if (waitKey(1) == 27) {
			break;
		}
	}

	return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception& e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}
  
