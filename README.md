# wego_limo
LIMO Application by WeGo-Robotics

### Outline
* LIMO에 사용 가능한 Application
* Ubuntu 18.04, Python 2.7, OpenCV 4.1.1 사용
* /cmd_vel, /scan Topic 사용
* 기본 센서 데이터 Subscriber 및 LIMO 제어를 위한 데이터 Publisher 포함
* 실시간 파라미터 변경을 위한 Dynamic Reconfigure 포함
* 동작을 위한 launch파일 포함

### How to use
* 기본 동작을 위한 LIMO_Driver, LiDAR Driver, Camera Driver 구동 후 아래 내용 실행
```bash
$ cd catkin_ws/src
$ git clone https://github.com/WeGo-Robotics/limo_examples.git
$ cd .. && catkin_make
$ source devel/setup.bash
$ roslaunch limo_examples ....launch or rosrun limo_examples ....py 
```

