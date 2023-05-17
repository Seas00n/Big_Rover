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
./camera_calibration default-chessboard.cfg
```