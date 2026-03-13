首先调整bashrc，需要补充下面几条定义：

# 注册GMP工程的位置
export GMP_PRO_LOCATION='/home/javnson/git_repo/gmp_pro'

# 指定MATLAB的安装位置
export MATLAB_ROOT='/usr/local/MATLAB/R2024b'

# 指定MATLAB Simulink模块支持库的位置
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6

编译过程：

mkdir build
cp build_target.sh build/build_target.sh
cd build
chmod u+x build_target.sh
./build_target.sh
cmake --build .

需要借助build_target.sh指明需要使用vcpkg托管依赖库。

vcpkg的安装过程如下：

git clone https://github.com/microsoft/vcpkg.git
./vcpkg/bootstrap-vcpkg.sh

注意有必要的话需要修改build_target.sh中指定的vcpkg的路径。

目前slib的安装程序无法执行copy指令，说是权限受限，所以需要手动复制文件，并添加path。
matlab的path是添加在一个m文件中，如果addpath报错，需要手动添加slib的路径到文件中。