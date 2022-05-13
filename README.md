# ICP算法功能使用说明

# 下载
git clone https://github.com/o0cto/ICP_TEST.git

# 运行
提供两种运行方式，二选一即可。

## 运行方式一：一键脚本
./setup.sh

## 运行方式二：在terminal命令顺序行执行以下指令
cd ICP_test
mkdir build
cd build
cmake -DCMAKE_CXX_FLAGS=-fPIC ..
make -j6

./testICP

=====================================
          ICP_TEST Results           
=====================================
# 运行结果输出如下：

start ICP, please wait ...
start: 0;
end:0; 0.033213; 40097;
start: 1;
end:1; 0.012761; 39405;
start: 2;
end:2; 0.005947; 38474;
start: 3;
end:3; 0.003536; 37048;
...
...
start: 39;
end:39; 0.000296; 30006;
start: 40;
end:40; 0.000285; 29894;
start: 41;
end:41; 0.000274; 29778;
start: 42;
end:42; 0.000265; 29658;
ICP result = 1
Register took 127.873 seconds.
ICP success.

# 修改说明
1. 添加了包含最耗时函数computeCloud2MeshDistances()的源文件：tests/DistanceComputationTools.cpp
2. computeCloud2MeshDistances函数中调用的最耗时函数为：ComputeNeighborhood2MeshDistancesWithOctree()