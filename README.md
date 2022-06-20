# limo_application
LIMO Application by WeGo-Robotics

### Outline
* LIMO에 사용 가능한 Application
* Ubuntu 18.04, Python 2.7, OpenCV 4.1.1 사용
* /cmd_vel, /scan Topic 사용
* 기본 센서 데이터 Subscriber 및 LIMO 제어를 위한 데이터 Publisher 포함
* 실시간 파라미터 변경을 위한 Dynamic Reconfigure 포함
* 함수 기반 ROS 제어 모듈 제공 ([상세 문서](https://docs.google.com/presentation/d/10CO-hCtMiG7AYF_dBP_7NSkoYQYM-HxvswAqN1BDSGE/edit?usp=sharing))
* 함수 기반 동작에 대한 문서 제공 ([상세 문서](https://docs.google.com/presentation/d/1NEUGhZ-5K-Ka9FAoMjRNX7j1X3KtOGSTw6Oytlp_P_I/edit?usp=sharing))
* ROS 기반 예제에 대한 사용 방법에 대한 문서 제공 ([상세 문서](https://docs.google.com/presentation/d/1nWVl21-Lr6hjMwL_O3UKQOr4gGpMvT6tEdaYsvO0WqQ/edit?usp=sharing))

### How to use
* 기본 동작을 위한 LIMO_Driver, LiDAR Driver, Camera Driver 구동 후 아래 내용 실행
```bash
$ cd catkin_ws/src
$ git clone https://github.com/WeGo-Robotics/limo_examples.git
$ cd .. && catkin_make
$ source devel/setup.bash
$ roslaunch limo_examples ....launch or rosrun limo_examples ....py 
```

