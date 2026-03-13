#cmake .. -DCMAKE_TOOLCHAIN_FILE=~/git_repo/vcpkg/scripts/buildsystems/vcpkg.cmake

# 运行配置，显式开启清单模式并指定工具链
cmake .. \
  -DCMAKE_TOOLCHAIN_FILE=/home/javnson/git_repo/vcpkg/scripts/buildsystems/vcpkg.cmake \
  -DVCPKG_MANIFEST_MODE=ON \
  -DVCPKG_TARGET_TRIPLET=x64-linux