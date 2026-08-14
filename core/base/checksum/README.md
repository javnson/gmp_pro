# Checksum

CRC16-CCITT 实现在 `src/crc16.c`，公开接口为 `crc16.h`，Facility 模块 ID
为 `core|base|checksum|crc16`。长度使用 `size_gt`，数据单元使用 `byte_gt`，
以保持 8 位和 16 位可寻址平台的一致性。
