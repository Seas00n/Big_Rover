# 使用calibration
```
cd calibration
mkdir build
cd build 
cmake ..
make 
```
复制build/devel/lib/calibration/camera_calibration到src文件夹下


配置default-chessboard.cfg文件
```
cd ../calibration/src
rm camera.xml
./camera_calibration default-chessboard.cfg
```
运行将camera.xml拷贝到april_tag_pose的src文件夹下