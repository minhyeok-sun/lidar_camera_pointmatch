# lidar_camera_pointmatch
livox lidar 와 zed2 camera를 활용하여 주변 장애물을 인식하는 프로그램
본 프로젝트는 실시간 운전자의 시선이 어느위치 어느 물체를 보고 있는지 확인하는 프로그램중 외부 환경을 인식하는 세부 부분이다.
livox lidar는 기존 velodyne lidar와 다르게 fov 90 로 좁지만 그 영역만큼은 64channel의 라이더보다 해상도가 뛰어나다. 따라서 5대의 livox lidar을 calibration을 통해 360도의 dense한 pointcloud 수집이 가능하다.
pointcloud는 pcl-ros library을 활용하여 처리하였다.

part1 : lidar point_cloud data 수집 => pcl_ros을 통해 pointcloud plane_segmentation, filtering, k-mean clustering 수행 => clustering bounding box의 중심점을 카메라 image에 projection하여 pixel의 배열 수집

part2 : zed camera data 수집 => darknet_ros(yoloV3) 을 통해 전방의 차량에 대한 object detection 수행 => object detection된 bounding box의 중심점 pixel 배열 수집

part3 : lidar data을 통해 수집한 pixel 배열과 camera lmage를 통해 수집한 pixel 배열의 data 비교 분석하여 해당 pointcloud cluster가 어떤 물체인지 classification 수행

결과 : 카메라를 통해 확인된 물체 (차, 트럭, 사람 등)와 어느위치(X,Y,Z)에 위치하는지 정보 출력

위 방법으로 3d object detection을 하였을 경우 장점 : pointcloud을 통해 object detection 을 수행할 경우 20m 이상 거리에서의 물체에 대한 충분한 dense 한 pointcloud data 수집이 어렵다. 또한 360도의 전방향에 대한 수많은 물체를 탐지할경우 실시간으로 데이터를 처리하는데 문제가 발생한다. 위 방법처럼 카메라를 통해 물체 인식을하고 pointcloud을 통해 위치정보를 파악후 fusion 할경우 최소 10fps이상 결과가 출력되었음을 확인하였다.



