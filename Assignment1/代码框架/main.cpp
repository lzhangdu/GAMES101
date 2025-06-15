#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <Eigen/Eigen>
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
    model << cos(rotation_angle * MY_PI / 180), -sin(rotation_angle * MY_PI / 180), 0, 0,
        sin(rotation_angle * MY_PI / 180), cos(rotation_angle * MY_PI / 180), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the view projection matrix for the given parameters.
    // Then return it.
    // Matrix4f M_persp2ortho = Matrix4f::Identity();
    // M_persp2ortho << 1, 0, 0, 0,
    //                  0, 1, 0, 0,
    //                  0, 0, (zNear + zFar) / zNear, -zFar,
    //                  0, 0, -1 / zNear, 0;

    // Matrix4f M_ortho = Matrix4f::Identity();
    // float yTop= zNear * tan(eye_fov / 2 * MY_PI / 180);
    // float xRight = yTop * aspect_ratio;
    // M_ortho << 1 / xRight, 0, 0, 0,
    //            0, 1 / yTop, 0, 0,
    //            0, 0, 2 / (zFar - zNear), -(zFar + zNear) / (zFar - zNear),
    //            0, 0, 0, 1;
    
    // projection = M_ortho * M_persp2ortho;
    // return projection;

    // 直接使用标准透视投影矩阵
    float fovY_rad = eye_fov * MY_PI / 180.0f;
    float f = 1.0f / tan(fovY_rad / 2.0f);
    
    projection << f / aspect_ratio, 0, 0, 0,
                  0, f, 0, 0,
                  0, 0, -(zFar + zNear) / (zFar - zNear), -2.0f * zFar * zNear / (zFar - zNear),
                  0, 0, -1, 0;
    
    return projection;
}

// 在 main.cpp 中构造一个函数，该函数的作用是得到绕任意
// 过原点的轴的旋转变换矩阵。
Eigen::Matrix4f get_rotation(Vector3f axis, float angle) {
    axis.normalize(); // 确保轴向量是单位向量
    // Rodrigues旋转公式: R = I*cos(θ) + (1-cos(θ))*n*nᵀ + sin(θ)*[n]×

    // 第一项: I*cos(θ) 
    // 恒等变换的缩放，当θ=0时，这一项=I，没有旋转

    // 第二项: (1-cos(θ))*n*nᵀ 
    // 沿旋转轴方向的投影分量，保持不变

    // 第三项: sin(θ)*[n]× 
    // 反对称矩阵，产生垂直于轴的旋转分量
    // [n]× 是轴向量n的反对称矩阵:
    // [n]× = [ 0   -nz   ny ]
    //        [ nz   0   -nx ]
    //        [-ny   nx   0  ]
    float rad = angle * MY_PI / 180.0f;
    float sin_theta = sin(rad);
    float cos_theta = cos(rad);

    // 第一项: I*cos(θ)
    Eigen::Matrix3f term1 = Eigen::Matrix3f::Identity() * cos_theta;

    // 第二项: (1-cos(θ))*n*nᵀ
    Eigen::Matrix3f term2 = (1 - cos_theta) * (axis * axis.transpose());

    // 第三项: sin(θ)*[n]× (反对称矩阵)
    Eigen::Matrix3f K;
    K << 0, -axis.z(), axis.y(),
         axis.z(), 0, -axis.x(),
         -axis.y(), axis.x(), 0;
    Eigen::Matrix3f term3 = sin_theta * K;

    // 组合三项得到3x3旋转矩阵
    Eigen::Matrix3f rotation_matrix = term1 + term2 + term3;

    // 将3x3旋转矩阵转换为4x4齐次矩阵
    Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
    rotation.block<3, 3>(0, 0) = rotation_matrix;

    return rotation;
}

int main(int argc, const char** argv)
{
    std::cout << "Program started." << std::endl;
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

    // Define rotation axis
    // Eigen::Vector3f rotation_axis(0, 0, 1); // 绕Z轴旋转
    Eigen::Vector3f rotation_axis(1, 1, 0);

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // r.set_model(get_model_matrix(angle));
        r.set_model(get_rotation(rotation_axis, angle)); // 使用自定义旋转函数
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
