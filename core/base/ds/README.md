# Data Structures

- `list.h`：侵入式单向链表，Facility 模块 ID 为 `core|base|ds|list`。
- `ring_buf.h` 与 `src/ring_buffer.c`：静态环形缓冲区，Facility 模块 ID 为
  `core|base|ds|ring_buffer`。

这些实现不分配动态内存，调用者负责提供对象和存储区。
