1. 必须要在CCS中添加此文件夹作为Product才可以正确使用GMP相关的服务。
2. 这个工程依赖于Sysconfig，C2000ware，建议在CCS18.2即老版本下运行，新版本下CCS20+版本运行后这些工程将不再能够被CCS18.2打开。

相关工具的路径：

https://www.ti.com/tool/SYSCONFIG

https://www.ti.com/tool/zh-cn/download/CCSTUDIO/12.8.1

https://www.ti.com.cn/tool/zh-cn/download/C2000WARE/5.04.00.00

GMP提供的系列工程并没有在C2000ware的更高版本上细致调试过，可能会存在问题。