#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <spline.h>

Eigen::Vector3d computeCircumcenter(const Eigen::Vector3d& A, const Eigen::Vector3d& B, const Eigen::Vector3d& C, double& radius) {
    Eigen::Vector3d AB = B - A;
    Eigen::Vector3d AC = C - A;
    Eigen::Vector3d N = AB.cross(AC); // 法向量

    double ab2 = AB.squaredNorm();
    double ac2 = AC.squaredNorm();
    Eigen::Vector3d circumcenter = A + ((ab2 * AC.cross(N) + ac2 * N.cross(AB)) / (2.0 * N.squaredNorm()));

    radius = (circumcenter - A).norm(); // 计算外接圆半径
    return circumcenter;
}

Eigen::VectorXf pathProcess(Eigen::VectorXd X_does1,
    Eigen::VectorXd X_does2,
    Eigen::VectorXd X_does3,
    Eigen::VectorXf X_real, double t, double linear_velocity)
{    
    std::vector<double > index = {1.f, 2.f, 3.f, 4.f};
    std::vector<double > x = {X_real[0], X_does1[0], X_does2[0], X_does3[0]};
    std::vector<double > y = {X_real[1], X_does1[1], X_does2[1], X_does3[1]};
    std::vector<double > z = {X_real[2], X_does1[2], X_does2[2], X_does3[2]};

    tk::spline s_x, s_y, s_z;

    s_x.set_boundary(tk::spline::first_deriv, X_real(3), tk::spline::first_deriv, 0.f);
    s_y.set_boundary(tk::spline::first_deriv, X_real(4), tk::spline::first_deriv, 0.f);
    s_z.set_boundary(tk::spline::first_deriv, X_real(5), tk::spline::first_deriv, 0.f);

    s_x.set_points(index, x);
    s_y.set_points(index, y);
    s_z.set_points(index, z);

    // 弧度制 
    // TODO：减去当前yaw pitch
    // const static double delta_index = 0.2;
    Eigen::Vector3d next_point(s_x(1.f+t), s_y(1.f+t), s_z(1.f+t));
    Eigen::Vector3d nnext_point(s_x(1.f+2*t), s_y(1.f+2*t), s_z(1.f+2*t));
    double delta_yaw = std::atan((next_point[1] - X_real[1]) / (next_point[0] - X_real[0]));
    delta_yaw -= X_real(8);
    double delta_pitch = std::atan((next_point[2] - X_real[2]) / (next_point[0] - X_real[0]));
    delta_pitch -= X_real(7);

    Eigen::Vector3d vel(s_x.deriv(1, 1.f + t), 
            s_y.deriv(1, 1.f + t), 
            s_z.deriv(1, 1.f + t));
    vel.normalize();
    vel *= linear_velocity;

    double r;
    Eigen::Vector3d cur {X_real[0], X_real[1], X_real[2]};
    computeCircumcenter(cur, next_point, nnext_point, r);

    Eigen::VectorXf ret(12);
    ret << next_point[0], next_point[1], next_point[2],
    vel(0), vel(1), vel(2), 0, delta_pitch, delta_yaw,0,0,0;

    return ret;
}

int main()
{
    std::vector<double > pose_1 = {6.79967,0.185416,-7.03571};
    std::vector<double > pose_2 = {16.8206,0.230965,-7.2121};
    std::vector<double > pose_3 = {26.8263,1.11458,-7.08114};
    std::vector<double > pose_4 = {36.8293,0.762045,-7.60068};


    std::vector<double > index = {1.f, 2.f, 3.f, 4.f};
    std::vector<double > x = {pose_1[0], pose_2[0], pose_3[0], pose_4[0]};
    std::vector<double > y = {pose_1[1], pose_2[1], pose_3[1], pose_4[1]};
    std::vector<double > z = {pose_1[2], pose_2[2], pose_3[2], pose_4[2]};

    tk::spline s_x, s_y, s_z;
    s_x.set_points(index, x);
    s_y.set_points(index, y);
    s_z.set_points(index, z);

    // for (double i = 1; i < 5; i += 0.1)
    // {
    //     std::cout << "x: " << s_x(i) << " y: " << s_y(i) << " z: " << s_z(i) << std::endl;
    // }
    // return 0;
    
    // 弧度制
    double delta_yaw = std::atan((x[1] - x[0]) / (y[1] - y[0]));
    double delta_pitch = std::atan((z[1] - z[0]) / (x[1] - x[0]));

    double r;
    computeCircumcenter(Eigen::Vector3d(x[0], y[0], z[0]), Eigen::Vector3d(x[1], y[1], z[1]), Eigen::Vector3d(x[2], y[2], z[2]), r);


}


