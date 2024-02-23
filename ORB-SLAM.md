ORB-SLAM-ros kurulumu için indirilmesi gerekenler:

pangolin:
```
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build && cd build
cmake ..
make
sudo make install
```

Ardından sonra ORB-Slam3 indirilmesi gerekiyor:

```
# Clone the repo:
git clone https://github.com/thien94/ORB_SLAM3.git ORB_SLAM3

# Build
cd ORB_SLAM3
chmod +x build.sh
./build.sh
```

