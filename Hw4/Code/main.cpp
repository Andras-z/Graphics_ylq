#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

// 反走样
void MSAA_4X(float x, float y, cv::Mat &window)
{
    int min_x = std::max(0, static_cast<int>(floor(x)));
    int max_x = std::min(window.cols - 1, static_cast<int>(ceil(x)));
    int min_y = std::max(0, static_cast<int>(floor(y)));
    int max_y = std::min(window.rows - 1, static_cast<int>(ceil(y)));

    window.at<cv::Vec3b>(y, x)[1] = 255;
    double res = 0;
    double sigma = 1.0; // 高斯权重因子
    double weight_total = 0;

    for (int i = min_x; i <= max_x; i++) {
        for (int j = min_y; j <= max_y; j++) {
            // 邻域像素位置到红点的距离
            double dist = sqrt(pow(x - i, 2) + pow(y - j, 2));
            // 距离权重（高斯权重）
            double weight = exp(-dist / sigma);
            int colors = window.at<cv::Vec3b>(j, i)[1];

            res += static_cast<double>(colors) * weight;
            weight_total += weight;
        }
    }
    res /= weight_total;
    window.at<cv::Vec3b>(y, x)[1] = static_cast<int>(res);
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // Implement de Casteljau's algorithm
    std::vector<cv::Point2f> points = control_points;
    // 当序列大于三个点时会继续迭代
    while (points.size() >= 3) {
        std::vector<cv::Point2f> points_temp;
        // 找到每个点按  t : (1 − t)  比例划分的新的顶点
        for (int i = 0; i < points.size() - 1; i++) {
            cv::Point2f first = points[i];
            cv::Point2f second = points[i + 1];
            cv::Point2f mid = (1 - t) * first + t * second;
            points_temp.push_back(mid);
        }
        points = points_temp;
    }
    // 序列小于三个点时，此时序列中只有两个点，连接两点，按  t : (1 − t) 取中点，那么该中点就为结束点
    cv::Point2f start = points[0];
    cv::Point2f end = points[points.size() - 1];
    cv::Point2f mid = (1 - t) * start + t * end;
    return mid;
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for (double t = 0.0; t <= 1.0; t += 0.001) {
        auto point = recursive_bezier(control_points, t);

        // window.at<cv::Vec3b>(point.y, point.x)[1] = 255;

        MSAA_4X(point.x, point.y, window);
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
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            naive_bezier(control_points, window);
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
