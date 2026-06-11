# probe-rs 常用命令手册（STM32F1）

> 基于 probe-rs CLI，针对 STM32F103C8T6（蓝丸板）整理，其他型号替换 `--chip` 参数即可。

---

## 1. 烧录固件

### 烧录 HEX 文件（基本）

```powershell
probe-rs.exe download --chip STM32F103C8 --binary-format hex firmware.hex
```

### 烧录 HEX 文件 + 校验

```powershell
probe-rs.exe download --chip STM32F103C8 --binary-format hex --verify firmware.hex
```

### 烧录 HEX 文件 + 校验 + 复位（推荐）

```powershell
probe-rs.exe download --chip STM32F103C8 --binary-format hex --verify firmware.hex && probe-rs.exe reset --chip STM32F103C8
```

### 整片擦除后烧录

```powershell
probe-rs.exe download --chip STM32F103C8 --binary-format hex --chip-erase firmware.hex
```

### 烧录前检查，内容相同则跳过

```powershell
probe-rs.exe download --chip STM32F103C8 --binary-format hex --preverify firmware.hex
```

### 烧录 BIN 文件（需指定基地址）

```powershell
probe-rs.exe download --chip STM32F103C8 --binary-format bin --base-address 0x08000000 firmware.bin
```

---

## 2. 复位

### 复位目标芯片

```powershell
probe-rs.exe reset --chip STM32F103C8
```

---

## 3. 擦除

### 擦除整片闪存

```powershell
probe-rs.exe erase --chip STM32F103C8
```

---

## 4. 验证

### 回读校验（不烧录，仅对比）

```powershell
probe-rs.exe verify --chip STM32F103C8 --binary-format hex firmware.hex
```

---

## 5. 探针与芯片信息

### 列出已连接的调试器

```powershell
probe-rs.exe list
```

### 查看探针和目标芯片信息

```powershell
probe-rs.exe info --chip STM32F103C8
```

### 查看支持的芯片列表

```powershell
# 查看全部
probe-rs.exe chip list

# 只看 STM32F1 系列
probe-rs.exe chip list | findstr "STM32F1"

# 只看 STM32F103 系列
probe-rs.exe chip list | findstr "STM32F103"
```

---

## 6. 烧录后运行（仅 ELF）

```powershell
probe-rs.exe run --chip STM32F103C8 firmware.elf
```

> ⚠️ `run` 命令仅支持 ELF 格式，烧录后自动复位运行，不支持 hex/bin。

---

## 7. 常用可选参数

| 参数 | 说明 |
|------|------|
| `--chip <型号>` | 指定目标芯片型号（必须） |
| `--binary-format hex\|bin\|elf` | 固件格式，默认 ELF，hex 须显式指定 |
| `--base-address <地址>` | BIN 文件烧录基地址（仅 bin 格式） |
| `--verify` | 烧录后回读校验 |
| `--preverify` | 烧录前检查，相同则跳过 |
| `--chip-erase` | 整片擦除后烧录 |
| `--connect-under-reset` | 复位状态下连接（调试口被禁时使用） |
| `--probe VID:PID` | 指定调试器（多探针时使用） |
| `--protocol swd\|jtag` | 连接协议，STM32 默认 SWD |
| `--speed <kHz>` | 协议速度（kHz） |
| `--disable-double-buffering` | 禁用双缓冲（超时失败时尝试） |

---

## 8. 支持的 STM32F1 常见型号

| 子系列 | 型号示例 |
|--------|----------|
| STM32F100 | C4, C6, C8, CB, R4, R6, R8, RB, RC, RD, RE, V8, VB, VC, VD, VE |
| STM32F101 | C4, C6, C8, CB, R4, R6, R8, RB, RC, RD, RE |
| STM32F102 | C4, C6, C8, CB, R4, R6, R8, RB |
| STM32F103 | C4, C6, **C8**, CB, R4, R6, R8, RB, RC, RD, RE, RF, RG, T4, T6, T8, TB, V8, VB, VC |
| STM32F105 | R8, RB, RC, RD, RE, V8, VB, VC, VD, VE |
| STM32F107 | RB, RC, RD, RE, VB, VC, VD, VE |

> 用 `probe-rs.exe chip list | findstr "STM32F1"` 查看完整列表。
