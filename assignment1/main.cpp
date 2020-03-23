#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    float angle = rotation_angle * MY_PI / 180;
    model(0,0) = cos(angle);
    model(0,1) = -sin(angle);
    model(1,1) = cos(angle);
    model(1,0) = sin(angle);
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    float eye_angle = eye_fov * MY_PI /180;
    float t = zNear * tan (eye_angle / 2);
    float r = t * aspect_ratio;
    float l = -r;
    float b = -t;
    float n = -zNear;
    float f = -zFar;
    Eigen::Matrix4f ortho_tran = Eigen::Matrix4f::Identity();
    ortho_tran(0,3) = -(r+l)/2.0f;
    ortho_tran(1,3) = -(t+b)/2.0f;
    ortho_tran(2.3) = -(n+f)/2.0f;
    Eigen::Matrix4f ortho_scale = Eigen::Matrix4f::Identity();
    ortho_scale(0,0) = 2.0f / (r-l);
    ortho_scale(1,1) = 2.0f / (t-b);
    ortho_scale(2,2) = 2.0f / (n-f);
    Eigen::Matrix4f p2o;
    p2o << n,0,0,0,0,n,0,0,0,0,n+f,-n*f,0,0,1,0;
    projection = ortho_scale * ortho_tran * p2o * projection;
    return projection;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    float angle_x,angle_y,angle_z;
    float length = sqrt(axis.x() * axis.x() + axis.y()*axis.y()+axis.z()*axis.z());
    angle_x = std::acos(axis.x()/length);
    angle_y = std::acos(axis.y()/length);
    angle_z = std::acos(axis.z()/length);
    Eigen::Matrix4f Rx,Ry,Rz  = Eigen::Matrix4f::Identity();
    Rx<<1,0,0,0,
        0,cos(angle_x),-sin(angle_x),0,
        0,sin(angle_x),cos(angle_x),0,
        0,0,0,1;
    Ry<<cos(angle_y),0,sin(angle_y),0,
        0,1,0,0,
        -sin(angle_y),0,cos(angle_y),0,
        0,0,0,1;
    Rz<<cos(angle_z),-sin(angle_z),0,0,
        sin(angle_z),cos(angle_z),0,0,
        0,0,1,0,
        0,0,0,1;

    Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
    rotation =Rz*Ry*Rx*Eigen::Matrix4f::Identity();
    return rotation;
}
int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
