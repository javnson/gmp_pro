# CANopen EDS 编译器

`canopen_eds_cc.py` 使用 Python 标准库，把 CiA 306 风格 EDS 的对象段生成 GMP
红黑树对象字典 `.h/.c`。支持 `[1234]`、`[1234sub5]`，以及 `ParameterName`、
`DataType`、`AccessType`、默认值、`PDOMapping` 和字符串/Domain 的 `DataLength`。

命令行的 `--storage pointer|value` 选择默认存储模型，单个对象可用
`GMPStorage` 覆盖；`--node-id` 负责展开 `$NODEID+常量`。指针模型生成的变量具有
外部链接，适合连接应用数据并在 CCS 中观察。实际值模型最多保存 8 个 CAN 字节。

工具会拒绝重复索引、不支持的数据类型或访问属性、非法表达式和越界默认值。
它有意不实现完整的 CiA 306 表达式语言、compact sub-object 和厂商私有类型。
运行 `python -m unittest discover -s tests -v` 可执行生成器单元测试。
