#include <chrono>
#include <iostream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void* userdata)
{
	if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4)
	{
		std::cout << "Left button of the mouse is clicked - position (" << x << ", "
			<< y << ")" << '\n';
		control_points.emplace_back(x, y);
	}
}

void naive_bezier(const std::vector<cv::Point2f>& points, cv::Mat& window)
{
	auto& p_0 = points[0];
	auto& p_1 = points[1];
	auto& p_2 = points[2];
	auto& p_3 = points[3];

	for (double t = 0.0; t <= 1.0; t += 0.001)
	{
		auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
			3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

		window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
	}
}

float factorial(int i) {

	if (i == 0) return 1;
	return i * factorial(i - 1);
}

float binomial_coefficient(int n, int i)
{
	return factorial(n) / (factorial(i) * factorial(n - i));
}

cv::Point2f formula_bezier(const std::vector<cv::Point2f>& control_points, float t) {
	int n = control_points.size() - 1;
	cv::Point2f res(0, 0);
	for (int i = 0; i <= n; ++i)
	{
		res += binomial_coefficient(n, i) * pow(t, i) * pow(1 - t, n - i) * control_points[i];
	}
	return res;
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f>& control_points, float t)
{
	// TODO: Implement de Casteljau's algorithm
	if (control_points.size() < 2) {
		throw std::invalid_argument("At least two control points are required");
	}

	std::vector<cv::Point2f> tmp_control_points = control_points;
	// Iterate over levels of the de Casteljau algorithm
	for (int level = tmp_control_points.size() - 1; level > 0; level--) {
		// Iterate over control points at the current level
		for (int i = 0; i < level; i++) {
			// Calculate the new control point at this level
			tmp_control_points[i] = (1 - t) * tmp_control_points[i] + t * tmp_control_points[i + 1];
		}
	}

	return tmp_control_points[0];
}

void bezier(const std::vector<cv::Point2f>& control_points, cv::Mat& window)
{
	// TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
	// recursive Bezier algorithm.
	for (float t = 0; t <= 1.0; t += 0.001) {
		auto point = recursive_bezier(control_points, t);
		window.at<cv::Vec3b>(point.y, point.x)[0] = 255;
		// Anti-Aliasing
		//auto point_1 = cv::Point2f(ceil(point.x), ceil(point.y));
		//auto point_2 = cv::Point2f(floor(point.x), floor(point.y));
		//window.at<cv::Vec3b>(point_1.y, point_1.x)[1] = std::min(255.0, 255 / cv::norm(point - point_1));
		//window.at<cv::Vec3b>(point_2.y, point_2.x)[1] = std::min(255.0, 255 / cv::norm(point - point_2));
	}
}

int main()
{
	cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
	cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
	cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

	cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

	int key = -1;
	while (key != 27)
	{
		for (auto& point : control_points)
		{
			cv::circle(window, point, 3, { 255, 255, 255 }, 3);
		}

		if (control_points.size() == 4)
		{
			//naive_bezier(control_points, window);
			bezier(control_points, window);

			cv::imshow("Bezier Curve", window);
			cv::imwrite("my_bezier_curve.png", window);
			key = cv::waitKey(0);

			return 0;
		}

		cv::imshow("Bezier Curve", window);
		key = cv::waitKey(20);
	}

	return 0;
}
