# GSLAM-DSO

## 1. Introduction

This code is the [DSO](https://github.com/JakobEngel/dso) plugin implementation base on [GSLAM](https://github.com/zdzhaoyong/GSLAM).

![GSLAM-DSO](./data/images/gslam_dso.gif)

## 2. Build and Install
### 2.1. Build and Install GSLAM

git clone https://github.com/zdzhaoyong/GSLAM

### 2.2. Build and Install GSLAM-DSO

```
mkdir build;
cd build;
cmake .. -DCMAKE_BUILD_TYPE=Release;
make;
sudo make install
```

## 3. Run DSO with gslam

```
gslam qviz dso play -dataset /Dataset/TUM/RGBD/rgbd_dataset_freiburg3_structure_texture_far/.tumrgbd -autostart
```
