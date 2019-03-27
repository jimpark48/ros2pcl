# ros2pcl
rosrun ros2pcl flat 으로 실행됩니다.
flat 노드는 depth camera로 받은 pointcloud를 사용하여 
pointcloud의 normal vector를 구하고 marker arror로 표시합니다.
각각의 normal vector를 구할 때는 pointcloud의 세 점을 사용하여 구합니다.

40*30은 depth camera의 해상도 입니다.

normal vector가 flat과 가까울 경우, flat_surface 포인트클라우드에 저장됩니다.
