#include "basic_dev.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "basic_dev"); // ��ʼ��ros �ڵ㣬����Ϊ basic
    ros::NodeHandle n; // ����node���ƾ��
    BasicDev go(&n);
    return 0;
}
