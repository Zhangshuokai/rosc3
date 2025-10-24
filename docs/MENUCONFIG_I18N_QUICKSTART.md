# Menuconfig 中文化功能快速入门指南

## 功能简介

本项目的 menuconfig 配置界面支持**中英文双语切换**，让开发者可以使用熟悉的语言进行系统配置。

### 主要特性

✨ **一键切换** - 简单命令即可在中英文间切换  
🔄 **实时生效** - 切换后立即可用，无需重新编译  
📦 **自动备份** - 每次切换都会自动备份当前配置  
🎯 **零影响构建** - 只改变界面显示，不影响代码编译  
👥 **团队友好** - 支持不同开发者使用不同语言偏好

### 适用场景

- 🇨🇳 **中文开发者** - 使用中文界面快速理解配置项含义
- 🇬🇧 **英文开发者** - 保持英文界面与官方文档对照
- 📚 **文档撰写** - 需要双语界面截图时快速切换
- 👨‍🏫 **教学培训** - 根据学员语言背景选择合适界面

---

## 快速开始

### 三步上手

```bash
# 步骤 1: 查看当前语言状态
python tools/menuconfig_lang.py --status

# 步骤 2: 切换到中文（或英文）
python tools/menuconfig_lang.py zh    # 中文
python tools/menuconfig_lang.py en    # 英文

# 步骤 3: 打开 menuconfig 配置界面
pio run -t menuconfig
```

就这么简单！🎉

### 详细步骤说明

#### 1️⃣ 查看当前状态

```bash
python tools/menuconfig_lang.py --status
```

**输出示例**：
```
============================================================
Menuconfig 语言状态
============================================================
当前语言: English (en)
配置文件: Kconfig.projbuild
文件大小: 3760 字节
修改时间: 2025-10-23 22:11:24

可用语言:
  [✓] en   - English    (存在)
  [ ] zh   - 简体中文       (存在)
       切换命令: python tools/menuconfig_lang.py zh
============================================================
```

#### 2️⃣ 切换语言

**切换到中文**：
```bash
python tools/menuconfig_lang.py zh
```

**切换到英文**：
```bash
python tools/menuconfig_lang.py en
```

**成功输出示例**：
```
📦 已备份当前配置: config\.backup\Kconfig.projbuild.20251024_101526.bak
✅ 已切换到 简体中文
   源文件: config\Kconfig.projbuild.zh
   目标文件: Kconfig.projbuild

💡 现在可以运行 menuconfig:
   pio run -t menuconfig
```

#### 3️⃣ 打开配置界面

```bash
pio run -t menuconfig
```

现在你会看到对应语言的配置界面！

---

## 常用命令

### 基本命令

| 命令 | 说明 |
|------|------|
| `python tools/menuconfig_lang.py --status` | 查看当前语言状态 |
| `python tools/menuconfig_lang.py zh` | 切换到中文 |
| `python tools/menuconfig_lang.py en` | 切换到英文 |
| `python tools/menuconfig_lang.py --help` | 显示帮助信息 |

### 配合 menuconfig 使用

```bash
# 完整工作流
python tools/menuconfig_lang.py zh        # 切换语言
pio run -t menuconfig                     # 配置项目
pio run                                   # 构建项目
pio run --target upload --target monitor  # 上传并监视
```

### 快捷方式（可选）

你可以在 [`platformio.ini`](../platformio.ini:1) 中添加自定义目标：

```ini
[env:esp32-c3]
extra_scripts = pre:tools/pio_lang_targets.py

# 使用方式:
# pio run -t lang-zh      # 切换到中文
# pio run -t lang-en      # 切换到英文
# pio run -t lang-status  # 查看状态
```

---

## 使用示例

### 示例 1: 新手开发者配置 WiFi

**场景**：首次配置 WiFi 连接，偏好使用中文界面

```bash
# 1. 切换到中文
python tools/menuconfig_lang.py zh

# 2. 打开配置界面
pio run -t menuconfig

# 3. 在界面中操作：
#    → 进入 "ROSC3 配置"
#    → 进入 "WiFi 配置"
#    → 修改 "WiFi 网络名称（SSID）"
#    → 修改 "WiFi 密码"
#    → 保存并退出（按 S 然后 Q）

# 4. 构建并上传
pio run --target upload
```

**中文界面示例**：
```
┌─────────────── ROSC3 配置 ───────────────┐
│  → WiFi 配置                             │
│    ROS 配置                              │
│    系统配置                              │
│                                          │
│  WiFi 配置:                              │
│    WiFi 网络名称（SSID）: [YourWiFi]    │
│    WiFi 密码: [****]                     │
│    认证模式: WPA2-PSK                    │
└──────────────────────────────────────────┘
```

### 示例 2: 高级开发者对照文档

**场景**：需要查看 ESP-IDF 官方文档，使用英文界面对照

```bash
# 切换到英文，方便与官方文档术语对照
python tools/menuconfig_lang.py en
pio run -t menuconfig
```

**英文界面示例**：
```
┌─────────────── ROSC3 Configuration ──────────────┐
│  → WiFi Configuration                            │
│    ROS Configuration                             │
│    System Configuration                          │
│                                                  │
│  WiFi Configuration:                             │
│    WiFi SSID: [YourWiFi]                        │
│    WiFi Password: [****]                         │
│    Authentication Mode: WPA2-PSK                 │
└──────────────────────────────────────────────────┘
```

### 示例 3: 团队协作

**场景**：团队成员语言偏好不同

```bash
# 开发者 A（偏好中文）
git clone <project-url>
cd rosc3
python tools/menuconfig_lang.py zh
# config/.current_lang 不会提交到 Git

# 开发者 B（偏好英文）
git clone <project-url>
cd rosc3
python tools/menuconfig_lang.py en
# 各自设置互不影响
```

---

## 故障排查

### 问题 1: 切换语言后界面仍是英文/中文

**可能原因**：
- menuconfig 已经在运行中
- 终端缓存问题

**解决方案**：
```bash
# 1. 关闭所有 menuconfig 窗口
# 2. 重新切换语言
python tools/menuconfig_lang.py zh

# 3. 重新打开 menuconfig
pio run -t menuconfig
```

### 问题 2: 提示源文件不存在

**错误信息**：
```
❌ 源文件不存在: config\Kconfig.projbuild.zh
```

**解决方案**：
```bash
# 检查文件是否存在
ls config/

# 应该看到：
# Kconfig.projbuild.en
# Kconfig.projbuild.zh
# .current_lang
# README.md

# 如果文件缺失，从 Git 恢复
git checkout config/Kconfig.projbuild.zh
```

### 问题 3: 切换后配置项丢失

**说明**：这是**正常现象**！

- 语言切换**只影响界面显示**
- 所有配置值保存在 [`sdkconfig`](../sdkconfig.esp32-c3:1) 文件中
- 配置项名称始终是英文（如 `CONFIG_WIFI_SSID`）
- 切换语言不会丢失任何配置

**验证方法**：
```bash
# 查看配置文件，配置项名称始终是英文
cat sdkconfig | grep WIFI_SSID
# 输出: CONFIG_WIFI_SSID="YourWiFi"
```

### 问题 4: 权限错误（Windows）

**错误信息**：
```
Permission denied: Kconfig.projbuild
```

**解决方案**：
```bash
# 以管理员身份运行命令提示符或 PowerShell
# 或者关闭占用文件的程序（如文本编辑器）
```

### 问题 5: 中英文版本不一致

**现象**：配置项数量或顺序不同

**解决方案**：
```bash
# 运行验证工具
python tools/validate_kconfig.py

# 如果报错，查看详细输出并修复
# 详见维护者指南
```

---

## 参考资源

### 项目文档

- 📖 [**架构设计文档**](MENUCONFIG_I18N_DESIGN.md) - 完整的技术架构和实现细节
- 🧪 [**测试报告**](MENUCONFIG_I18N_TEST_REPORT.md) - 功能验证和测试结果
- 🛠️ [**维护者指南**](MENUCONFIG_I18N_MAINTENANCE.md) - 如何添加/修改配置项（保持双语同步）
- 📋 [**配置文件说明**](../config/README.md) - config 目录的详细说明

### 相关工具

- [`tools/menuconfig_lang.py`](../tools/menuconfig_lang.py:1) - 语言切换脚本（核心工具）
- [`tools/validate_kconfig.py`](../tools/validate_kconfig.py:1) - 配置验证工具

### 配置文件

- [`config/Kconfig.projbuild.en`](../config/Kconfig.projbuild.en:1) - 英文版配置
- [`config/Kconfig.projbuild.zh`](../config/Kconfig.projbuild.zh:1) - 中文版配置

### 外部资源

- [ESP-IDF Kconfig 文档](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32/api-reference/kconfig.html)
- [PlatformIO 文档](https://docs.platformio.org/)
- [Linux Kconfig 语法](https://www.kernel.org/doc/html/latest/kbuild/kconfig-language.html)

---

## 常见问题 (FAQ)

### Q1: 切换语言需要重新编译项目吗？

**A**: ❌ **不需要**。语言切换只影响 menuconfig 界面显示，不影响编译结果。

### Q2: 切换语言会影响已有的配置吗？

**A**: ❌ **不会**。所有配置值保存在 `sdkconfig` 文件中，配置项名称始终是英文，切换语言不会丢失任何配置。

### Q3: 团队成员可以使用不同的语言吗？

**A**: ✅ **可以**。`config/.current_lang` 文件已在 `.gitignore` 中排除，每个开发者可以独立设置语言偏好。

### Q4: 如何确认当前使用的语言？

**A**: 运行 `python tools/menuconfig_lang.py --status` 查看当前语言状态。

### Q5: 支持其他语言（如日语、韩语）吗？

**A**: 目前仅支持中英文。如需添加其他语言，请参考[维护者指南](MENUCONFIG_I18N_MAINTENANCE.md)中的"添加新语言"章节。

### Q6: 语言切换后为什么界面还是英文？

**A**: 可能是 menuconfig 已经在运行。关闭 menuconfig，重新切换语言，然后再打开。

### Q7: 备份文件会一直累积吗？

**A**: 是的。备份文件保存在 `config/.backup/` 目录下，可以定期手动清理旧备份（建议保留最近的几个）。

---

## 下一步

现在你已经掌握了 menuconfig 中文化功能的基本使用，可以：

1. ✅ 开始配置你的项目（WiFi、ROS、系统参数等）
2. 📖 阅读[架构设计文档](MENUCONFIG_I18N_DESIGN.md)了解技术细节
3. 🛠️ 如需修改配置项，参考[维护者指南](MENUCONFIG_I18N_MAINTENANCE.md)

---

**文档版本**: v1.0.0  
**最后更新**: 2025-10-24  
**适用项目**: ROSC3 ESP32-C3