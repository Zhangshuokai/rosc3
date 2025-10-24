# Menuconfig 中文化架构设计文档

## 文档信息

| 项目 | 内容 |
|------|------|
| **文档标题** | Menuconfig中文化架构设计 |
| **文档版本** | v1.0.0 |
| **编制日期** | 2025-10-24 |
| **设计方案** | 静态Kconfig文件替换方案 |
| **适用平台** | ESP-IDF + PlatformIO |
| **目标芯片** | ESP32-C3 |

---

## 目录

- [1. 设计概述](#1-设计概述)
- [2. 文件组织结构](#2-文件组织结构)
- [3. 语言切换脚本设计](#3-语言切换脚本设计)
- [4. 翻译规范和原则](#4-翻译规范和原则)
- [5. 使用流程设计](#5-使用流程设计)
- [6. 维护策略](#6-维护策略)
- [7. 集成方案](#7-集成方案)
- [8. 附录](#8-附录)

---

## 1. 设计概述

### 1.1 技术方案选择

经过技术调研，本项目采用**静态Kconfig文件替换方案**实现menuconfig的中文化。

**方案原理**：
```
┌─────────────────────────────────────────────────┐
│          语言切换流程                            │
├─────────────────────────────────────────────────┤
│                                                 │
│  ┌──────────────┐      ┌──────────────┐        │
│  │ 用户执行命令  │      │ 选择语言     │        │
│  │ lang en/zh   │ ───> │ en / zh      │        │
│  └──────────────┘      └──────┬───────┘        │
│                              │                 │
│                              ▼                 │
│          ┌─────────────────────────────┐       │
│          │  menuconfig_lang.py 脚本    │       │
│          │  - 备份当前Kconfig          │       │
│          │  - 复制对应语言版本         │       │
│          │  - 更新语言标记             │       │
│          └────────────┬────────────────┘       │
│                       │                        │
│                       ▼                        │
│          ┌─────────────────────────────┐       │
│          │  Kconfig.projbuild 更新     │       │
│          │  (英文或中文版本)            │       │
│          └────────────┬────────────────┘       │
│                       │                        │
│                       ▼                        │
│          ┌─────────────────────────────┐       │
│          │  pio run -t menuconfig      │       │
│          │  (显示对应语言界面)          │       │
│          └─────────────────────────────┘       │
│                                                 │
└─────────────────────────────────────────────────┘
```

### 1.2 方案优势

| 优势 | 说明 |
|------|------|
| **简单可靠** | 无需修改ESP-IDF或menuconfig工具 |
| **完全兼容** | 与现有构建系统完美集成 |
| **易于维护** | 两个独立文件，互不干扰 |
| **灵活切换** | 随时可在中英文间切换 |
| **版本控制友好** | 所有版本文件都纳入Git管理 |

### 1.3 技术架构图

```
项目根目录 (rosc3/)
│
├── config/                          # 配置文件目录
│   ├── Kconfig.projbuild.en        # 英文版配置（主）
│   ├── Kconfig.projbuild.zh        # 中文版配置
│   ├── .current_lang               # 当前语言标记文件
│   └── README.md                   # 配置说明文档
│
├── tools/                           # 工具脚本目录
│   ├── menuconfig_lang.py          # 语言切换脚本
│   └── validate_kconfig.py         # 配置验证脚本（可选）
│
├── Kconfig.projbuild               # 工作配置文件（符号链接或副本）
│
└── platformio.ini                   # PlatformIO配置（包含快捷命令）
```

---

## 2. 文件组织结构

### 2.1 目录结构详细说明

#### 2.1.1 config/ 目录

**用途**：存储所有语言版本的Kconfig配置文件

```
config/
├── Kconfig.projbuild.en           # 英文版本（124行）
├── Kconfig.projbuild.zh           # 中文版本（124行）
├── .current_lang                  # 当前语言标记
└── README.md                      # 使用说明
```

**文件说明**：

| 文件 | 描述 | 版本控制 |
|------|------|----------|
| `Kconfig.projbuild.en` | 英文版配置，作为参考基准 | ✅ Git跟踪 |
| `Kconfig.projbuild.zh` | 中文版配置，翻译自英文版 | ✅ Git跟踪 |
| `.current_lang` | 存储当前使用的语言（en/zh） | ⚠️ Git忽略 |
| `README.md` | 配置文件使用说明 | ✅ Git跟踪 |

#### 2.1.2 tools/ 目录

**用途**：存储语言切换和验证工具

```
tools/
├── menuconfig_lang.py             # 语言切换主脚本
└── validate_kconfig.py            # 配置验证脚本（可选）
```

#### 2.1.3 根目录文件

**Kconfig.projbuild**
- 这是menuconfig实际读取的配置文件
- 由`menuconfig_lang.py`脚本动态生成
- 内容与`config/Kconfig.projbuild.{en|zh}`保持一致
- **不应直接编辑此文件**

### 2.2 文件内容示例

#### 2.2.1 config/README.md

```markdown
# Kconfig配置文件说明

本目录包含menuconfig的多语言配置文件。

## 文件说明

- `Kconfig.projbuild.en` - 英文版配置（主版本）
- `Kconfig.projbuild.zh` - 中文版配置

## 使用方法

切换到中文：
```bash
python tools/menuconfig_lang.py zh
pio run -t menuconfig
```

切换到英文：
```bash
python tools/menuconfig_lang.py en
pio run -t menuconfig
```

## 维护规则

1. **英文版为主**：所有新增配置项首先在英文版中添加
2. **同步翻译**：英文版更新后，立即更新中文版
3. **保持结构一致**：两个版本的结构、顺序、配置项名称必须完全一致
4. **只翻译用户可见内容**：menu、prompt、help等，配置项名称保持英文

## 注意事项

⚠️ 不要直接编辑根目录的 `Kconfig.projbuild` 文件！
✅ 始终编辑 `config/` 目录中的源文件
```

#### 2.2.2 config/.current_lang

```
zh
```

简单的文本文件，只包含当前语言代码（`en` 或 `zh`）。

---

## 3. 语言切换脚本设计

### 3.1 menuconfig_lang.py 功能设计

#### 3.1.1 脚本架构

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Menuconfig Language Switcher
用于在英文和中文menuconfig之间切换

用法:
    python tools/menuconfig_lang.py [en|zh]
    python tools/menuconfig_lang.py --status
"""

import os
import sys
import shutil
from pathlib import Path
from typing import Optional

# 配置常量
PROJECT_ROOT = Path(__file__).parent.parent
CONFIG_DIR = PROJECT_ROOT / "config"
KCONFIG_TARGET = PROJECT_ROOT / "Kconfig.projbuild"
CURRENT_LANG_FILE = CONFIG_DIR / ".current_lang"

SUPPORTED_LANGUAGES = {
    'en': {
        'name': 'English',
        'source': CONFIG_DIR / "Kconfig.projbuild.en",
        'description': '英文版配置'
    },
    'zh': {
        'name': '简体中文',
        'source': CONFIG_DIR / "Kconfig.projbuild.zh",
        'description': '中文版配置'
    }
}

DEFAULT_LANGUAGE = 'en'
```

#### 3.1.2 核心功能函数

```python
def get_current_language() -> str:
    """获取当前设置的语言"""
    if CURRENT_LANG_FILE.exists():
        return CURRENT_LANG_FILE.read_text().strip()
    return DEFAULT_LANGUAGE


def set_current_language(lang: str) -> None:
    """保存当前语言设置"""
    CURRENT_LANG_FILE.write_text(lang)


def validate_language(lang: str) -> bool:
    """验证语言代码是否有效"""
    return lang in SUPPORTED_LANGUAGES


def backup_current_kconfig() -> Optional[Path]:
    """备份当前Kconfig文件"""
    if KCONFIG_TARGET.exists():
        backup_path = KCONFIG_TARGET.with_suffix('.bak')
        shutil.copy2(KCONFIG_TARGET, backup_path)
        return backup_path
    return None


def switch_language(target_lang: str) -> bool:
    """切换到指定语言"""
    try:
        # 1. 验证语言
        if not validate_language(target_lang):
            print(f"❌ 错误: 不支持的语言 '{target_lang}'")
            print(f"   支持的语言: {', '.join(SUPPORTED_LANGUAGES.keys())}")
            return False
        
        # 2. 检查源文件
        source_file = SUPPORTED_LANGUAGES[target_lang]['source']
        if not source_file.exists():
            print(f"❌ 错误: 源文件不存在: {source_file}")
            return False
        
        # 3. 备份当前文件
        current_lang = get_current_language()
        if KCONFIG_TARGET.exists():
            backup_path = backup_current_kconfig()
            print(f"📦 已备份当前配置: {backup_path}")
        
        # 4. 复制新文件
        shutil.copy2(source_file, KCONFIG_TARGET)
        print(f"✅ 已切换到 {SUPPORTED_LANGUAGES[target_lang]['name']}")
        print(f"   源文件: {source_file}")
        print(f"   目标文件: {KCONFIG_TARGET}")
        
        # 5. 保存语言设置
        set_current_language(target_lang)
        
        # 6. 提示用户运行menuconfig
        print(f"\n💡 现在可以运行 menuconfig:")
        print(f"   pio run -t menuconfig")
        
        return True
        
    except Exception as e:
        print(f"❌ 切换失败: {e}")
        return False


def show_status() -> None:
    """显示当前状态"""
    current = get_current_language()
    print("=" * 50)
    print("Menuconfig 语言状态")
    print("=" * 50)
    print(f"当前语言: {SUPPORTED_LANGUAGES[current]['name']} ({current})")
    print(f"配置文件: {KCONFIG_TARGET}")
    
    if KCONFIG_TARGET.exists():
        size = KCONFIG_TARGET.stat().st_size
        print(f"文件大小: {size} 字节")
    else:
        print("⚠️  配置文件不存在")
    
    print("\n可用语言:")
    for code, info in SUPPORTED_LANGUAGES.items():
        marker = "✓" if code == current else " "
        status = "存在" if info['source'].exists() else "缺失"
        print(f"  [{marker}] {code:4s} - {info['name']:10s} ({status})")
    print("=" * 50)
```

#### 3.1.3 命令行接口

```python
def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(
        description='Menuconfig 语言切换工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  切换到中文:  python tools/menuconfig_lang.py zh
  切换到英文:  python tools/menuconfig_lang.py en
  查看状态:    python tools/menuconfig_lang.py --status
        """
    )
    
    parser.add_argument(
        'language',
        nargs='?',
        choices=['en', 'zh'],
        help='目标语言 (en=英文, zh=中文)'
    )
    
    parser.add_argument(
        '--status', '-s',
        action='store_true',
        help='显示当前语言状态'
    )
    
    args = parser.parse_args()
    
    # 显示状态
    if args.status or args.language is None:
        show_status()
        return
    
    # 切换语言
    success = switch_language(args.language)
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
```

### 3.2 错误处理机制

| 错误类型 | 检测方式 | 处理策略 |
|---------|---------|---------|
| 源文件不存在 | 文件存在性检查 | 显示错误，终止操作 |
| 目标文件无法写入 | 权限检查 | 提示权限问题，终止操作 |
| 无效的语言代码 | 语言验证 | 显示支持的语言列表 |
| 备份失败 | 异常捕获 | 警告但继续执行 |

### 3.3 安全保护措施

```python
# 1. 文件完整性验证
def verify_kconfig_file(file_path: Path) -> bool:
    """验证Kconfig文件基本格式"""
    try:
        content = file_path.read_text(encoding='utf-8')
        # 检查必须存在的关键字
        required_keywords = ['menu', 'config', 'endmenu']
        return all(keyword in content for keyword in required_keywords)
    except Exception:
        return False

# 2. 原子性操作
def atomic_copy(src: Path, dst: Path) -> bool:
    """原子性文件复制"""
    temp_file = dst.with_suffix('.tmp')
    try:
        shutil.copy2(src, temp_file)
        temp_file.replace(dst)  # 原子性替换
        return True
    except Exception as e:
        if temp_file.exists():
            temp_file.unlink()
        raise e
```

---

## 4. 翻译规范和原则

### 4.1 翻译范围

#### 4.1.1 需要翻译的内容

| 元素 | 英文示例 | 中文示例 | 说明 |
|------|---------|---------|------|
| `menu` | `menu "WiFi Configuration"` | `menu "WiFi 配置"` | 菜单标题 |
| `prompt` | `string "WiFi SSID"` | `string "WiFi 名称"` | 配置项提示 |
| `help` | `WiFi network SSID...` | `WiFi 网络名称...` | 帮助文本 |
| `choice` | `prompt "WiFi Auth Mode"` | `prompt "WiFi 认证模式"` | 选择项提示 |
| `bool` | `bool "WPA2-PSK"` | `bool "WPA2-PSK"` | 选项名称 |

#### 4.1.2 不应翻译的内容

| 元素 | 示例 | 原因 |
|------|------|------|
| 配置项名称 | `config WIFI_SSID` | C代码中使用，必须保持英文 |
| 默认值 | `default "YourWiFi"` | 技术配置，保持原样 |
| 范围定义 | `range 1024 65535` | 数值定义，无需翻译 |
| 依赖条件 | `depends on WIFI_ENABLE` | 逻辑表达式，保持英文 |

### 4.2 术语翻译规范

#### 4.2.1 专业术语处理原则

**保留英文的术语**（附带中文说明）：
```kconfig
# ✅ 推荐
config WIFI_SSID
    string "WiFi SSID（网络名称）"
    
config ROS_NODE_NAME
    string "ROS 节点名称"
    
config MICRO_ROS_AGENT_IP
    string "Micro-ROS Agent IP地址"
```

**完全中文化的术语**：
```kconfig
# ✅ 常见术语可完全翻译
config WIFI_MAX_RETRY
    int "WiFi 最大重试次数"
    
config LOG_LEVEL
    prompt "日志级别"
```

#### 4.2.2 标准术语对照表

| 英文 | 中文翻译 | 使用场景 |
|------|---------|---------|
| WiFi | WiFi | 保持英文 |
| ROS | ROS | 保持英文 |
| IP Address | IP地址 | 混用 |
| Port | 端口 | 完全翻译 |
| Node | 节点 | 完全翻译 |
| Agent | Agent | 保持英文 |
| Domain ID | Domain ID（域标识） | 混用 |
| Namespace | 命名空间 | 完全翻译 |
| Sensor | 传感器 | 完全翻译 |
| Sampling Rate | 采样频率 | 完全翻译 |
| Log Level | 日志级别 | 完全翻译 |
| Timeout | 超时时间 | 完全翻译 |
| Retry | 重试 | 完全翻译 |
| Password | 密码 | 完全翻译 |
| Authentication | 认证 | 完全翻译 |

### 4.3 文本长度建议

#### 4.3.1 菜单标题

```kconfig
# ✅ 简洁明了（推荐8-12个汉字）
menu "WiFi 配置"
menu "系统配置"
menu "传感器配置"

# ⚠️ 避免过长
menu "WiFi无线网络连接配置选项"  # 太长
```

#### 4.3.2 配置项提示

```kconfig
# ✅ 清晰准确（推荐4-15个汉字）
string "WiFi 网络名称"
int "最大重试次数"
bool "启用调试日志"

# ⚠️ 避免过长
string "请输入您要连接的无线网络的SSID名称"  # 太长，应放在help中
```

#### 4.3.3 帮助文本

```kconfig
# ✅ 详细说明，支持多行
help
    WiFi 网络名称（SSID），用于连接无线路由器。
    
    配置说明：
    - 长度范围：1-32个字符
    - 支持中英文和特殊字符
    - 可通过NVS存储在运行时修改
    
    注意：特殊字符需要正确转义。
```

### 4.4 翻译风格指南

#### 4.4.1 语气

- **使用陈述语气**：`WiFi 网络名称` ✅（而非 `请输入WiFi网络名称` ❌）
- **避免主观描述**：`最大重试次数` ✅（而非 `合理的重试次数` ❌）
- **保持技术准确**：遵循行业标准术语

#### 4.4.2 格式规范

```kconfig
# ✅ 标准格式
menu "WiFi 配置"
    config WIFI_SSID
        string "WiFi 网络名称（SSID）"
        default "YourWiFi"
        help
            WiFi 网络标识符，用于连接到无线接入点。
            
            此配置可在运行时通过NVS存储覆盖。

    config WIFI_PASSWORD
        string "WiFi 密码"
        default "YourPassword"
        help
            WPA2/WPA3加密网络的密码。
            
            安全警告：密码以明文形式存储在 sdkconfig 文件中。
            生产环境建议使用NVS加密存储。
endmenu
```

### 4.5 翻译示例对比

#### 示例1：WiFi配置菜单

**英文版** (`config/Kconfig.projbuild.en`):
```kconfig
menu "WiFi Configuration"
    config WIFI_SSID
        string "WiFi SSID"
        default "YourWiFi"
        help
            WiFi network SSID to connect to.
            This can be overridden by NVS storage at runtime.

    config WIFI_PASSWORD
        string "WiFi Password"
        default "YourPassword"
        help
            WiFi network password.
            This can be overridden by NVS storage at runtime.
            
            WARNING: Password is stored in plain text in sdkconfig.
            For production, use NVS encrypted storage.
endmenu
```

**中文版** (`config/Kconfig.projbuild.zh`):
```kconfig
menu "WiFi 配置"
    config WIFI_SSID
        string "WiFi 网络名称（SSID）"
        default "YourWiFi"
        help
            要连接的 WiFi 网络名称。
            此配置可在运行时通过 NVS 存储覆盖。

    config WIFI_PASSWORD
        string "WiFi 密码"
        default "YourPassword"
        help
            WiFi 网络密码。
            此配置可在运行时通过 NVS 存储覆盖。
            
            警告：密码以明文形式存储在 sdkconfig 文件中。
            生产环境请使用 NVS 加密存储。
endmenu
```

---

## 5. 使用流程设计

### 5.1 开发者工作流

#### 5.1.1 首次使用

```bash
# 1. 克隆项目
git clone <project-url>
cd rosc3

# 2. 查看当前语言状态
python tools/menuconfig_lang.py --status

# 3. 切换到中文（如需要）
python tools/menuconfig_lang.py zh

# 4. 打开 menuconfig
pio run -t menuconfig

# 5. 配置完成后构建
pio run
```

#### 5.1.2 日常使用

```bash
# 快速切换语言（使用platformio.ini别名）
pio run -t lang-zh      # 切换到中文
pio run -t lang-en      # 切换到英文
pio run -t lang-status  # 查看状态

# 或直接使用脚本
python tools/menuconfig_lang.py zh
python tools/menuconfig_lang.py en
```

### 5.2 PlatformIO集成

#### 5.2.1 platformio.ini 配置

```ini
[env:esp32-c3]
platform = espressif32
board = adafruit_qtpy_esp32c3
framework = espidf

# 自定义目标：语言切换
extra_scripts = 
    pre:tools/pio_lang_targets.py

# 说明：
# - pio run -t lang-zh      切换到中文
# - pio run -t lang-en      切换到英文  
# - pio run -t lang-status  查看语言状态
```

#### 5.2.2 tools/pio_lang_targets.py

```python
"""PlatformIO自定义目标：语言切换"""
Import("env")
import subprocess
import sys
from pathlib import Path

project_dir = Path(env.get("PROJECT_DIR"))
lang_script = project_dir / "tools" / "menuconfig_lang.py"

def switch_to_chinese(target, source, env):
    """切换到中文"""
    subprocess.run([sys.executable, str(lang_script), "zh"])

def switch_to_english(target, source, env):
    """切换到英文"""
    subprocess.run([sys.executable, str(lang_script), "en"])

def show_lang_status(target, source, env):
    """显示语言状态"""
    subprocess.run([sys.executable, str(lang_script), "--status"])

# 注册自定义目标
env.AlwaysBuild(env.Alias("lang-zh", None, switch_to_chinese))
env.AlwaysBuild(env.Alias("lang-en", None, switch_to_english))
env.AlwaysBuild(env.Alias("lang-status", None, show_lang_status))
```

### 5.3 团队协作策略

#### 5.3.1 默认语言设置

**方案A：统一默认为英文**
```python
# tools/menuconfig_lang.py
DEFAULT_LANGUAGE = 'en'  # 团队统一使用英文
```

**优点**：
- 国际化标准
- 便于与上游ESP-IDF文档对照
- 技术术语更准确

**方案B：允许个人偏好**
```bash
# 开发者个人设置（不提交到Git）
echo "zh" > config/.current_lang

# 在 .gitignore 中忽略
config/.current_lang
```

#### 5.3.2 Git 配置

**.gitignore**:
```gitignore
# Menuconfig语言设置（个人偏好，不提交）
config/.current_lang

# Menuconfig备份文件
Kconfig.projbuild.bak

# SDK配置（本地生成）
sdkconfig
sdkconfig.old
```

**必须提交到Git的文件**:
```
✅ config/Kconfig.projbuild.en     # 英文版本
✅ config/Kconfig.projbuild.zh     # 中文版本
✅ config/README.md                # 使用说明
✅ tools/menuconfig_lang.py        # 语言切换脚本
✅ tools/pio_lang_targets.py       # PlatformIO集成
```

### 5.4 使用场景示例

#### 场景1：新手开发者（偏好中文）

```bash
# 步骤1：切换到中文
pio run -t lang-zh

# 步骤2：打开menuconfig配置WiFi
pio run -t menuconfig
# 在界面中看到：
# "ROSC3 配置"
#   ├── "WiFi 配置"
#   ├── "ROS 配置"
#   └── "系统配置"

# 步骤3：保存退出，构建项目
pio run
```

#### 场景2：高级开发者（偏好英文，查阅官方文档）

```bash
# 切换到英文（与ESP-IDF官方文档对照）
pio run -t lang-en
pio run -t menuconfig

# 配置完成
pio run
```

#### 场景3：文档撰写（需要双语对照）

```bash
# 1. 英文界面截图
pio run -t lang-en
pio run -t menuconfig
# 截图...

# 2. 中文界面截图
pio run -t lang-zh
pio run -t menuconfig
# 截图...
```

---

## 6. 维护策略

### 6.1 同步维护流程

#### 6.1.1 标准工作流

```
┌─────────────────────────────────────────────────┐
│          配置项修改流程                          │
├─────────────────────────────────────────────────┤
│                                                 │
│  1️⃣  编辑英文版本                               │
│      vim config/Kconfig.projbuild.en           │
│      ├── 添加新配置项                           │
│      ├── 修改existing配置                       │
│      └── 测试验证                               │
│                                                 │
│  2️⃣  同步更新中文版本                           │
│      vim config/Kconfig.projbuild.zh           │
│      ├── 找到对应位置                           │
│      ├── 翻译新内容                             │
│      ├── 保持结构一致                           │
│      └── 验证配置项名称一致                     │
│                                                 │
│  3️⃣  验证一致性                                 │
│      python tools/validate_kconfig.py          │
│      └── 检查配置项数量、名称、顺序             │
│                                                 │
│  4️⃣  测试两种语言                               │
│      ├── pio run -t lang-en && pio run -t menuconfig │
│      └── pio run -t lang-zh && pio run -t menuconfig │
│                                                 │
│  5️⃣  提交Git                                    │
│      git add config/Kconfig.projbuild.*        │
│      git commit -m "menuconfig: 添加XXX配置项"  │
│                                                 │
└─────────────────────────────────────────────────┘
```

#### 6.1.2 新增配置项示例

**步骤1：在英文版中添加**
```kconfig
# config/Kconfig.projbuild.en
menu "ROS Configuration"
    # ... existing configs ...
    
    # 新增配置项
    config ROS_QOS_RELIABILITY
        prompt "ROS QoS Reliability"
        default ROS_QOS_RELIABLE
        help
            Quality of Service reliability policy.
        
        config ROS_QOS_RELIABLE
            bool "Reliable"
        config ROS_QOS_BEST_EFFORT
            bool "Best Effort"
    endchoice
endmenu
```

**步骤2：在中文版中同步**
```kconfig
# config/Kconfig.projbuild.zh
menu "ROS 配置"
    # ... 现有配置 ...
    
    # 新增配置项（位置与英文版完全对应）
    config ROS_QOS_RELIABILITY
        prompt "ROS 服务质量可靠性策略"
        default ROS_QOS_RELIABLE
        help
            服务质量（QoS）可靠性策略配置。
        
        config ROS_QOS_RELIABLE
            bool "可靠模式"
        config ROS_QOS_BEST_EFFORT
            bool "尽力而为模式"
    endchoice
endmenu
```

**步骤3：验证一致性**
```bash
python tools/validate_kconfig.py
```

### 6.2 验证工具设计

#### 6.2.1 tools/validate_kconfig.py

```python
#!/usr/bin/env python3
"""
Kconfig文件一致性验证工具
验证英文版和中文版的配置项名称、数量、顺序是否一致
"""

import re
from pathlib import Path
from typing import List, Tuple

CONFIG_DIR = Path(__file__).parent.parent / "config"
EN_FILE = CONFIG_DIR / "Kconfig.projbuild.en"
ZH_FILE = CONFIG_DIR / "Kconfig.projbuild.zh"


def extract_config_items(file_path: Path) -> List[str]:
    """提取所有config配置项名称"""
    content = file_path.read_text(encoding='utf-8')
    # 匹配 "config CONFIG_NAME" 模式
    pattern = r'^\s*config\s+(\w+)'
    matches = re.findall(pattern, content, re.MULTILINE)
    return matches


def extract_menu_structure(file_path: Path) -> List[Tuple[str, int]]:
    """提取菜单结构（menu/endmenu的层级）"""
    content = file_path.read_text(encoding='utf-8')
    structure = []
    indent_level = 0
    
    for line in content.split('\n'):
        stripped = line.strip()
        if stripped.startswith('menu '):
            structure.append(('menu', indent_level))
            indent_level += 1
        elif stripped == 'endmenu':
            indent_level -= 1
            structure.append(('endmenu', indent_level))
    
    return structure


def validate_kconfig() -> bool:
    """验证Kconfig文件一致性"""
    print("=" * 60)
    print("Kconfig 文件一致性验证")
    print("=" * 60)
    
    # 1. 检查文件存在
    if not EN_FILE.exists():
        print(f"❌ 错误: 英文版文件不存在: {EN_FILE}")
        return False
    
    if not ZH_FILE.exists():
        print(f"❌ 错误: 中文版文件不存在: {ZH_FILE}")
        return False
    
    print(f"✅ 文件存在检查通过")
    
    # 2. 提取配置项
    en_configs = extract_config_items(EN_FILE)
    zh_configs = extract_config_items(ZH_FILE)
    
    print(f"\n📊 配置项统计:")
    print(f"   英文版: {len(en_configs)} 个配置项")
    print(f"   中文版: {len(zh_configs)} 个配置项")
    
    # 3. 验证数量
    if len(en_configs) != len(zh_configs):
        print(f"❌ 错误: 配置项数量不一致！")
        return False
    
    print(f"✅ 配置项数量一致")
    
    # 4. 验证名称和顺序
    all_match = True
    for i, (en_name, zh_name) in enumerate(zip(en_configs, zh_configs)):
        if en_name != zh_name:
            print(f"❌ 错误: 第 {i+1} 个配置项不匹配:")
            print(f"   英文版: {en_name}")
            print(f"   中文版: {zh_name}")
            all_match = False
    
    if all_match:
        print(f"✅ 所有配置项名称和顺序一致")
    else:
        return False
    
    # 5. 验证菜单结构
    en_structure = extract_menu_structure(EN_FILE)
    zh_structure = extract_menu_structure(ZH_FILE)
    
    if en_structure == zh_structure:
        print(f"✅ 菜单结构一致")
    else:
        print(f"❌ 警告: 菜单结构可能不一致")
        print(f"   英文版: {len([x for x in en_structure if x[0] == 'menu'])} 个菜单")
        print(f"   中文版: {len([x for x in zh_structure if x[0] == 'menu'])} 个菜单")
    
    print("\n" + "=" * 60)
    print("✅ 验证通过：两个版本保持一致")
    print("=" * 60)
    return True


if __name__ == '__main__':
    import sys
    success = validate_kconfig()
    sys.exit(0 if success else 1)
```

### 6.3 版本管理规范

#### 6.3.1 提交规范

```bash
# ✅ 良好的提交示例
git commit -m "menuconfig: 添加ROS QoS可靠性配置项

- 在英文版添加 ROS_QOS_RELIABILITY 配置
- 同步更新中文版翻译
- 验证配置项一致性通过
"

# ❌ 不推荐的提交
git commit -m "update kconfig"  # 信息不明确
```

#### 6.3.2 提交检查清单

在提交前检查：

- [ ] 英文版和中文版都已更新
- [ ] 配置项名称（`config XXX`）完全一致
- [ ] 运行 `validate_kconfig.py` 验证通过
- [ ] 在两种语言下测试过 menuconfig
- [ ] 提交信息清晰描述了更改内容

### 6.4 文档维护

#### 6.4.1 需要同步更新的文档

当修改Kconfig时，同时更新：

1. **需求分析/Menuconfig配置项说明.md**
   - 添加新配置项的详细说明
   - 更新配置项列表

2. **README.md**（如果影响用户使用）
   - 更新配置说明
   - 添加新功能介绍

3. **config/README.md**
   - 记录重要的配置变更

#### 6.4.2 变更日志

在 `config/README.md` 中维护变更日志：

```markdown
## 变更历史

### v1.1.0 (2025-10-25)
- ➕ 新增: ROS QoS可靠性配置
- ➕ 新增: WiFi静态IP配置选项
- 🔧 修复: WiFi密码长度限制说明

### v1.0.0 (2025-10-24)
- 🎉 初始版本：支持WiFi、ROS、系统基础配置
```

---

## 7. 集成方案

### 7.1 与构建系统集成

#### 7.1.1 PlatformIO工作流

```
┌─────────────────────────────────────────────┐
│     PlatformIO 构建流程                      │
├─────────────────────────────────────────────┤
│                                             │
│  pio run -t menuconfig                     │
│       │                                     │
│       ▼                                     │
│  读取 Kconfig.projbuild                    │
│  (由 menuconfig_lang.py 维护)             │
│       │                                     │
│       ▼                                     │
│  显示配置界面(en/zh)                        │
│       │                                     │
│       ▼                                     │
│  生成 sdkconfig                            │
│       │                                     │
│       ▼                                     │
│  pio run                                   │
│  (使用 sdkconfig 构建)                     │
│                                             │
└─────────────────────────────────────────────┘
```

#### 7.1.2 不影响构建过程

**重要**：语言切换**只影响menuconfig界面显示**，不影响：
- ✅ 生成的 `sdkconfig` 内容（配置项名称始终是英文）
- ✅ C代码中的宏定义（如 `CONFIG_WIFI_SSID`）
- ✅ 编译和链接过程
- ✅ 最终固件功能

### 7.2 CI/CD集成

#### 7.2.1 GitHub Actions示例

```yaml
name: Build and Test

on: [push, pull_request]

jobs:
  validate-kconfig:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      
      - name: Setup Python
        uses: actions/setup-python@v4
        with:
          python-version: '3.x'
      
      - name: Validate Kconfig Consistency
        run: |
          python tools/validate_kconfig.py
      
      - name: Test Language Switch (EN)
        run: |
          python tools/menuconfig_lang.py en
          test -f Kconfig.projbuild
      
      - name: Test Language Switch (ZH)
        run: |
          python tools/menuconfig_lang.py zh
          test -f Kconfig.projbuild
  
  build:
    needs: validate-kconfig
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      
      - name: Setup PlatformIO
        uses: platformio/platformio-action@v1
      
      # 使用默认语言（英文）构建
      - name: Build Project
        run: |
          python tools/menuconfig_lang.py en
          pio run
```

### 7.3 IDE集成

#### 7.3.1 VSCode任务配置

`.vscode/tasks.json`:
```json
{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "Menuconfig: 切换到中文",
            "type": "shell",
            "command": "python",
            "args": ["tools/menuconfig_lang.py", "zh"],
            "problemMatcher": []
        },
        {
            "label": "Menuconfig: 切换到英文",
            "type": "shell",
            "command": "python",
            "args": ["tools/menuconfig_lang.py", "en"],
            "problemMatcher": []
        },
        {
            "label": "Menuconfig: 查看状态",
            "type": "shell",
            "command": "python",
            "args": ["tools/menuconfig_lang.py", "--status"],
            "problemMatcher": []
        },
        {
            "label": "Menuconfig: 打开配置（中文）",
            "type": "shell",
            "command": "pio",
            "args": ["run", "-t", "menuconfig"],
            "dependsOn": "Menuconfig: 切换到中文",
            "problemMatcher": []
        }
    ]
}
```

---

## 8. 附录

### 8.1 完整的Kconfig翻译示例

见项目中的：
- `config/Kconfig.projbuild.en` - 英文完整版
- `config/Kconfig.projbuild.zh` - 中文完整版

### 8.2 常见问题解答（FAQ）

#### Q1: 切换语言后需要重新配置吗？

**A**: 不需要。`sdkconfig` 文件中的配置项名称始终是英文，切换语言只改变menuconfig的显示界面，已有的配置值保持不变。

#### Q2: 可以在构建时自动使用某种语言吗？

**A**: 可以。在 `platformio.ini` 中添加：
```ini
extra_scripts = 
    pre:tools/auto_lang.py
```

`tools/auto_lang.py`:
```python
Import("env")
import subprocess
import sys

# 自动切换到中文
subprocess.run([sys.executable, "tools/menuconfig_lang.py", "zh"])
```

#### Q3: 如何添加更多语言支持？

**A**: 
1. 在 `config/` 目录创建新语言文件（如 `Kconfig.projbuild.ja` 日文）
2. 在 `menuconfig_lang.py` 的 `SUPPORTED_LANGUAGES` 中添加配置
3. 翻译全部内容
4. 运行验证工具

#### Q4: 团队成员偏好不同语言怎么办？

**A**: 每个开发者可以独立设置，`config/.current_lang` 已在 `.gitignore` 中排除，不会相互影响。

#### Q5: 英文版和中文版不一致怎么办？

**A**: 运行验证工具：
```bash
python tools/validate_kconfig.py
```
会自动检测并报告不一致的地方。

### 8.3 性能影响评估

| 方面 | 影响 | 说明 |
|------|------|------|
| **构建时间** | 无影响 | 仅文件复制，耗时<1秒 |
| **运行时性能** | 无影响 | 语言切换不影响固件 |
| **存储空间** | +8KB | 两份Kconfig文件 |
| **内存占用** | 无影响 | 不涉及运行时内存 |

### 8.4 备选方案对比

| 方案 | 优点 | 缺点 | 评分 |
|------|------|------|------|
| **静态文件替换**（本方案） | 简单、可靠、易维护 | 需要维护两份文件 | ⭐⭐⭐⭐⭐ |
| **动态翻译** | 只需一份配置文件 | 需要修改ESP-IDF | ⭐⭐ |
| **gettext集成** | 标准i18n方案 | menuconfig不支持 | ⭐ |
| **补丁方案** | 可扩展性好 | 维护复杂，易出错 | ⭐⭐⭐ |

### 8.5 参考资料

- [ESP-IDF Kconfig文档](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32/api-reference/kconfig.html)
- [Linux Kconfig语法](https://www.kernel.org/doc/html/latest/kbuild/kconfig-language.html)
- [PlatformIO自定义目标](https://docs.platformio.org/en/latest/scripting/custom_targets.html)

---

## 结语

本设计文档提供了完整的menuconfig中文化解决方案，采用静态文件替换策略，具有以下特点：

✅ **简单可靠**：无需修改ESP-IDF核心代码  
✅ **易于维护**：清晰的维护流程和验证工具  
✅ **灵活切换**：支持随时在中英文间切换  
✅ **团队友好**：支持不同成员的语言偏好  
✅ **文档完善**：详细的使用说明和翻译规范  

**下一步行动**：

1. **实施阶段**：根据本设计文档创建实际的配置文件和脚本
2. **测试阶段**：在实际项目中测试语言切换功能
3. **优化阶段**：根据使用反馈优化工作流程

---

**文档版本**: v1.0.0  
**编写人**: 架构师  
**审核人**: 技术负责人  
**最后更新**: 2025-10-24