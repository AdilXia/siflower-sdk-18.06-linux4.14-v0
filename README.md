#sdk for siflower router products,include openwrt/linux/uboot
SDK参考文档链接：https://developers.siflower.cn/
快速入门参考https://developers.siflower.cn/#/documentDisplay?id=5d2c32516fdbe80001cb3100&docId=167)
简单编译步骤如下：
1、操作系统使用ubuntu14.04以上
2、安装gcc4.8.4或以上版本
3、安装交叉编译环境
    sudo apt-get update
    sudo apt-get install git-core build-essential libssl-dev libncurses5-dev unzip gawk zlib1g-dev
    sudo apt-get install subversion mercurial
4、编译uboot(如果板子中存在uboot，则无须烧录)
   cd uboot
   不同的版型命令不同，常见的两种版型如下
   xc1200M：
     ./make.sh p10h fullmask
     生成镜像在uboot目录下，如 uboot_master_p10h_fullmask_.bin
   evb开发板：
     ./make.sh evb_v5 fullmask
     生成镜像在uboot目录下，如 uboot_master_evb_v5_fullmask_.bin
5、编译openwrt
   cd openwrt-18.06
   不同的版型命令不同，常见的两种版型如下
   xc1200M：
     ./make.sh p10h fullmask
     生成镜像在uboot目录下，如 openwrt_master_p10h_fullmask_rel_.bin
   evb开发板：
     ./make.sh evb_v5 fullmask
     生成镜像在uboot目录下，如 openwrt_master_evb_v5_fullmask_rel_.bin

简单的烧录步骤：
1、uboot烧录
   参考 https://developers.siflower.cn/#/documentDisplay?docId=174 的3.4.2章节
2、openwrt烧录
   参考 https://developers.siflower.cn/#/documentDisplay?docId=174 的3.2章节
