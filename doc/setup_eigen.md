# How to Set Up Eigen Library

Namely, you can set up `Eigen` in three ways:

- download `eigen` using a package manager (for Mac and Ubuntu)
- download the repository (and configure if you want to optimize eigen)
- clone the repository (and configure if you want to optimize eigen)

Below, we discuss these options in detail

----



## Install using a Package Manager

for Mac, install `eigen` using the package manager `brew` as

```bash
$ brew install eigen
```
For ubuntu, install `eigen` using `apt-get` as

```bash
$ sudo apt-get install libeigen3-dev
```
Unfortunately, for windows, there is not a easyway to install `eigen` with commands.

---



## Download & Configure

Download the repository from here

 http://eigen.tuxfamily.org/index.php?title=Main_Page#Download

1. Download the compressed file (e.g.,` eigen-3.*.*.zip`)  and extract it. This result in a directory `eigen-3.*.*`
2. Put the extracted file under the `3rd_party` directory as `pba-<username>/3rd_party/eigen-3.*.*` .

Basically, you can stop here, but the `eigen` is not optimized for your environment.   
To configure the `eigen` library, type the commands below

```bash
cd pba-<username>/3rd_party/eigen-3.*.* # move to the directory
mkdir build # make a new directory for "out-of-source build"
cd build    # move to the new directory
cmake ..    # configure (this may take one or two minutes)
cmake --install . --prefix ../../Eigen_Lib  # install eigen into the "Eigen_Lib" folder
```

Make sure you have a header file `Dense` at

```
pba-<username>/3rd_party/Eigen_Lib/include/eigen3/Eigen/Dense
```



## Clone & Configure

Basically it is similar to the "Download & Configure" approach. 
The only difference is that instead of download we use submodule to clone the `Eigen` repository. 

```bash
$ git submodule update --init 3rd_party/eigen # clone the "eigen" repository (this may take one or two mins)
```

You can stop here, but the `eigen` is not optimized for your environment.
To configure the `eigen` library, type the commands below
```bash
$ cd 3rd_party/eigen # move to the eigen repository
$ mkdir build # make a directory for "out-of-source build"
$ cd build    # move to the new directory
$ cmake ..    # configure (this may take one or two mins)
$ cmake --install . --prefix ../../Eigen_Lib # install eigen into the "Eigen_Lib" folder
```

Make sure you have a header file `Dense` at

```
pba-<username>/3rd_party/Eigen_Lib/include/eigen3/Eigen/Dense
```







 



