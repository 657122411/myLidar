# myLidar
重构了Lidar项目的代码，将过程函数封装成类和对象，方便解耦和并在下一步数据处理时方便留下接口。

visual studio 需要配置levmar库，可以用Clion版的cmakelist编译减少麻烦，见https://github.com/657122411/LidarBathymetry


### 配置流程
1.若直接下载该工程，首先删除工程中已包含的Axb.c compiler.h levmar.h lm.c lm.h misc.c misc.h

2.将下载的levmar2.6压缩文件解压，获得一个名字为levmar2.6的文件夹，将这个问价夹包含在工程目录下，同时在属性设置中添加到包含目录 
 
3.然后将levmar中的 Axb.c  compiler.h  levmar.h levmar.c  lm.c  lm.h  misc.c  misc.h 添加到工程中，头文件文件夹右键添加已有项。 
注意！有一些后缀是core的C文件（*_core.c），这些文件是不能直接编译的，即不需要添加到Visual c++ 的工程里。 

4.注释掉 levmar.h第31行中的#define HAVE_LAPACK ，Levmar依赖于一个叫LAPACK的库，这个库是Fortran语言编写的，可用于解多元线性方程式、计算特征向量、计算矩阵的QR分解，奇异值分解等等，但是dlevmar_der()，dlevmar_dif()，slevmar_der()，slevmar_dif()，dlevmar_bc_der()，dlevmar_bc_dif()，slevmar_bc_der()，slevmar_bc_dif()是不需要依赖这个库的。

 
参考网站：https://blog.csdn.net/shajun0153/article/details/75073137
