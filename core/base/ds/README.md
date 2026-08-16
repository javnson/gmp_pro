# Data Structures

- `list.h`：无动态分配的侵入式双向循环链表，采用与 Windows NT
  `LIST_ENTRY` 相同的哨兵头模型。插入、删除、首尾弹出、owner 恢复、有限步数
  完整性校验均为头文件内联 API；Facility 模块 ID 为 `core|base|ds|list`。
- `ring_buf.h` 与 `src/ring_buffer.c`：静态环形缓冲区，Facility 模块 ID 为
  `core|base|ds|ring_buffer`。
- `rbtree.h`：无动态分配的侵入式红黑树，Facility 模块 ID 为
  `core|base|ds|rbtree`。节点显式保存 `parent/left/right/color/linked`，不把颜色位
  复用到指针中，因此不依赖目标平台的指针对齐。它提供插入、查找、删除、有序
  前后遍历，以及带节点数上限的结构、排序、红黑性质校验。

这些实现不分配动态内存，调用者负责提供对象和存储区。
链表头和未挂接节点都必须先调用 `gmp_list_init()`。节点成功删除后会重新进入
自链接状态，因此重复删除和重复插入会被拒绝。遍历不可信链表前应先调用
`gmp_list_validate()` 并提供明确的最大节点数，避免损坏链表导致无限循环。
红黑树也遵循相同的所有权规则：根节点先调用 `gmp_rb_root_init()`，每个节点先调用
`gmp_rb_node_init()`；重复插入和重复删除会被拒绝。若输入可能被破坏，应调用
`gmp_rb_validate()` 并给出可信的最大节点数，再开始遍历。
