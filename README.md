#简述
>该sdk包含了openwrt、linux、uboot这3个部分的源码<br>
>目前适配了xc1200M、EVB开发板这两种版型<br>
#SDK参考链接
>https://developers.siflower.cn/
#快速入门
>https://developers.siflower.cn/#/documentDisplay?id=5d2c32516fdbe80001cb3100&docId=167)
#简单编译步骤如下：
>1、操作系统使用ubuntu14.04以上<br>
>2、安装gcc4.8.4或以上版本<br>
>3、安装交叉编译环境<br>
>    sudo apt-get update<br>
>    sudo apt-get install git-core build-essential libssl-dev libncurses5-dev unzip gawk zlib1g-dev<br>
>    sudo apt-get install subversion mercurial<br>
>4、编译uboot(如果板子中存在uboot，则无须烧录)<br>
>   cd uboot<br>
>   不同的版型命令不同，常见的两种版型如下<br>
>   xc1200M：<br>
>     ./make.sh p10h fullmask<br>
>     生成镜像在uboot目录下，如 uboot_master_p10h_fullmask_.bin<br>
>   evb开发板：<br>
>     ./make.sh evb_v5 fullmask<br>
>     生成镜像在uboot目录下，如 uboot_master_evb_v5_fullmask_.bin<br>
>5、编译openwrt
>   cd openwrt-18.06<br>
>   不同的版型命令不同，常见的两种版型如下<br>
>   xc1200M：<br>
>     ./make.sh p10h fullmask<br>
>     生成镜像在uboot目录下，如 openwrt_master_p10h_fullmask_rel_.bin<br>
>   evb开发板：<br>
>     ./make.sh evb_v5 fullmask<br>
>     生成镜像在uboot目录下，如 openwrt_master_evb_v5_fullmask_rel_.bin<br>
>
#简单的烧录步骤：
>1、uboot烧录<br>
>   参考 https://developers.siflower.cn/#/documentDisplay?docId=174 的3.4.2章节<br>
>2、openwrt烧录<br>
>   参考 https://developers.siflower.cn/#/documentDisplay?docId=174 的3.2章节<br>
