# lidar_camera_pointmatch
livox lidar 와 zed2 camera를 활용하여 주변 장애물을 인식하는 프로그램
본 프로젝트는 실시간 운전자의 시선이 어느위치 어느 물체를 보고 있는지 확인하는 프로그램중 외부 환경을 인식하는 세부 부분이다.
livox lidar는 기존 velodyne lidar와 다르게 fov 90 로 좁지만 그 영역만큼은 64channel의 라이더보다 해상도가 뛰어나다. 따라서 5대의 livox lidar을 calibration을 통해 360도의 dense한 pointcloud 수집이 가능하다.
pointcloud는 pcl-ros library을 활용하여 처리하였다.


