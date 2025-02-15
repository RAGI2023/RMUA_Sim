# --net 使用主机网络
# --name 指定名称
# --rm 结束后自动释放资源

# docker run -it --net host --name basic_dev --rm  basic_dev
docker run --rm \
  -e ROS_MASTER_URI=http://$(hostname -I | awk '{print $1}'):11311 \
  -e ROS_IP=$(hostname -I | awk '{print $1}') \
  --network host \
  rmua
