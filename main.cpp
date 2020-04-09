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

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    int n = control_points.size();
    if (n == 1) return control_points[0];
    std::vector<cv::Point2f> p;
    for(int i = 0;i < n-1;i++)
    {
        cv::Point2f m = control_points[i];
        cv::Point2f n = control_points[i+1];
        auto pt = (1.0 - t)*m + t*n;
        p.push_back(pt);
    }
    return recursive_bezier(p,t);

}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for (double t = 0.0;t < 1.0;t += 0.001)
    {
        auto pt = recursive_bezier(control_points,t);
        int xmin = std::floor(pt.x);
        int ymin = std::floor(pt.y);
        float color0 = 0;
        float color1 = 0;
        float color2 = 0;
        float color3 = 0;
        int colorValue = 0;
        color0 = 255*(1.0f-std::sqrt((pt.x-xmin-0.25f)*(pt.x-xmin-0.25f))+
                            std::sqrt((pt.y-ymin-0.25f)*(pt.y-ymin-0.25f)))/1.0f;
        color0 = 255*(1.0f-std::sqrt((pt.x-xmin-0.25f)*(pt.x-xmin-0.25f))+
                            std::sqrt((pt.y-ymin-0.75f)*(pt.y-ymin-0.75f)))/1.0f;
        color0 = 255*(1.0f-std::sqrt((pt.x-xmin-0.75f)*(pt.x-xmin-0.75f))+
                            std::sqrt((pt.y-ymin-0.25f)*(pt.y-ymin-0.25f)))/1.0f;
        color0 = 255*(1.0f-std::sqrt((pt.x-xmin-0.75f)*(pt.x-xmin-0.75f))+
                            std::sqrt((pt.y-ymin-0.75f)*(pt.y-ymin-0.75f)))/1.0f;
        colorValue = (color0 + color1+color2+color3 )/4.0f;
       
        window.at<cv::Vec3b>(pt.y, pt.x)[1] = std::min(255,colorValue + window.at<cv::Vec3b>(pt.y, pt.x)[1]);
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
