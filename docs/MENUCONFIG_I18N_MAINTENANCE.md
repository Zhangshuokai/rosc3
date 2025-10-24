# Menuconfig 中文化维护者指南

## 文档目标

本指南面向需要**添加或修改** menuconfig 配置项的开发者，帮助你正确维护中英文双语配置文件的同步。

**适用场景**：
- ✏️ 添加新的配置项（WiFi、ROS、传感器等）
- 🔧 修改现有配置项的描述或默认值
- 🌏 保持中英文版本一致性
- 📝 翻译新的配置项

---

## 核心原则

### ⚠️ 关键规则

1. **英文版为主** - 所有修改首先在英文版进行
2. **立即同步** - 修改英文版后立即更新中文版
3. **结构一致** - 两个版本的结构、顺序必须完全相同
4. **符号不变** - 配置项名称（`config XXX`）保持英文
5. **验证通过** - 提交前必须运行验证工具

### 📁 文件位置

| 文件 | 路径 | 说明 |
|------|------|------|
| **英文版** | [`config/Kconfig.projbuild.en`](../config/Kconfig.projbuild.en:1) | 主版本，优先编辑 |
| **中文版** | [`config/Kconfig.projbuild.zh`](../config/Kconfig.projbuild.zh:1) | 翻译版本，同步更新 |
| **验证工具** | [`tools/validate_kconfig.py`](../tools/validate_kconfig.py:1) | 一致性检查 |

### ⛔ 禁止操作

- ❌ **不要直接编辑** 根目录的 `Kconfig.projbuild` 文件
- ❌ **不要修改** 配置项符号名称（如 `WIFI_SSID`）
- ❌ **不要改变** 配置项的顺序
- ❌ **不要只更新一个版本**而忘记另一个

---

## 添加新配置项

### 标准流程

```
1️⃣ 编辑英文版 → 2️⃣ 同步中文版 → 3️⃣ 验证一致性 → 4️⃣ 测试两种语言 → 5️⃣ 提交 Git
```

### 详细步骤

#### 步骤 1: 在英文版中添加

编辑 [`config/Kconfig.projbuild.en`](../config/Kconfig.projbuild.en:1)：

```kconfig
menu "WiFi Configuration"
    # ... 现有配置 ...
    
    # 添加新配置项
    config WIFI_STATIC_IP
        bool "Enable Static IP"
        default n
        help
            Enable static IP configuration instead of DHCP.
            
            When enabled, you need to configure:
            - Static IP address
            - Subnet mask
            - Gateway address
            - DNS server
    
    config WIFI_STATIC_IP_ADDR
        string "Static IP Address"
        depends on WIFI_STATIC_IP
        default "192.168.1.100"
        help
            Static IP address for WiFi interface.
            Format: xxx.xxx.xxx.xxx
endmenu
```

**注意事项**：
- 配置项名称使用全大写英文
- 使用有意义的前缀（如 `WIFI_`、`ROS_`）
- 提供清晰的 help 文本
- 合理设置默认值

#### 步骤 2: 在中文版中同步

编辑 [`config/Kconfig.projbuild.zh`](../config/Kconfig.projbuild.zh:1)：

```kconfig
menu "WiFi 配置"
    # ... 现有配置 ...
    
    # 添加新配置项（位置与英文版完全对应）
    config WIFI_STATIC_IP
        bool "启用静态 IP"
        default n
        help
            启用静态 IP 配置，不使用 DHCP。
            
            启用后需要配置：
            - 静态 IP 地址
            - 子网掩码
            - 网关地址
            - DNS 服务器
    
    config WIFI_STATIC_IP_ADDR
        string "静态 IP 地址"
        depends on WIFI_STATIC_IP
        default "192.168.1.100"
        help
            WiFi 接口的静态 IP 地址。
            格式：xxx.xxx.xxx.xxx
endmenu
```

**翻译要点**：
- ✅ 配置项名称保持不变：`config WIFI_STATIC_IP`
- ✅ 翻译菜单标题：`"WiFi Configuration"` → `"WiFi 配置"`
- ✅ 翻译提示文本：`"Enable Static IP"` → `"启用静态 IP"`
- ✅ 翻译帮助文本，保持含义准确
- ✅ 默认值保持一致：`default "192.168.1.100"`

#### 步骤 3: 验证一致性

```bash
python tools/validate_kconfig.py
```

**期望输出**：
```
============================================================
Kconfig 文件一致性验证
============================================================
✅ 文件存在检查通过
📊 配置项统计:
   英文版: 21 个配置项
   中文版: 21 个配置项
✅ 配置项数量一致
✅ 所有配置项名称和顺序一致
✅ 菜单结构一致
============================================================
✅ 验证通过：两个版本保持一致
============================================================
```

**如果验证失败**：
```bash
❌ 错误: 第 20 个配置项不匹配:
   英文版: WIFI_STATIC_IP
   中文版: WIFI_STATIC_IP_ADDR
```
→ 检查配置项顺序，确保中英文版本完全对应

#### 步骤 4: 测试两种语言

**测试英文版**：
```bash
python tools/menuconfig_lang.py en
pio run -t menuconfig

# 检查：
# 1. 新配置项是否显示
# 2. 默认值是否正确
# 3. 帮助文本是否清晰
# 4. 依赖关系是否正常
```

**测试中文版**：
```bash
python tools/menuconfig_lang.py zh
pio run -t menuconfig

# 检查：
# 1. 中文显示是否正确
# 2. 翻译是否准确
# 3. 功能是否正常
```

#### 步骤 5: 提交到 Git

```bash
# 添加修改的文件
git add config/Kconfig.projbuild.en
git add config/Kconfig.projbuild.zh

# 提交（使用清晰的提交信息）
git commit -m "menuconfig: 添加 WiFi 静态 IP 配置项

- 新增 WIFI_STATIC_IP 开关
- 新增 WIFI_STATIC_IP_ADDR 地址配置
- 同步更新中英文版本
- 验证一致性通过
"

# 推送
git push
```

---

## 修改现有配置项

### 场景 1: 修改默认值

假设需要修改 WiFi SSID 的默认值：

**英文版** ([`config/Kconfig.projbuild.en`](../config/Kconfig.projbuild.en:1))：
```kconfig
config WIFI_SSID
    string "WiFi SSID"
    default "MyESP32"        # 修改此处
    help
        WiFi network SSID to connect to.
```

**中文版** ([`config/Kconfig.projbuild.zh`](../config/Kconfig.projbuild.zh:1))：
```kconfig
config WIFI_SSID
    string "WiFi 网络名称（SSID）"
    default "MyESP32"        # 同步修改
    help
        要连接的 WiFi 网络名称。
```

### 场景 2: 修改帮助文本

**英文版**：
```kconfig
config WIFI_PASSWORD
    string "WiFi Password"
    default "YourPassword"
    help
        WiFi network password.
        
        Security Note: Password is stored in plain text.
        For production, use secure storage methods.
```

**中文版**：
```kconfig
config WIFI_PASSWORD
    string "WiFi 密码"
    default "YourPassword"
    help
        WiFi 网络密码。
        
        安全提示：密码以明文形式存储。
        生产环境请使用安全存储方式。
```

### 场景 3: 添加新选项到现有 choice

假设在 WiFi 认证模式中添加 WPA3 支持：

**英文版**：
```kconfig
choice WIFI_AUTH_MODE
    prompt "WiFi Authentication Mode"
    default WIFI_AUTH_WPA2_PSK
    help
        WiFi authentication mode.

    config WIFI_AUTH_OPEN
        bool "Open (No Security)"
    config WIFI_AUTH_WPA2_PSK
        bool "WPA2-PSK"
    config WIFI_AUTH_WPA3_PSK
        bool "WPA3-PSK"              # 新增
endchoice
```

**中文版**：
```kconfig
choice WIFI_AUTH_MODE
    prompt "WiFi 认证模式"
    default WIFI_AUTH_WPA2_PSK
    help
        WiFi 认证模式选择。

    config WIFI_AUTH_OPEN
        bool "开放模式（无加密）"
    config WIFI_AUTH_WPA2_PSK
        bool "WPA2-PSK"
    config WIFI_AUTH_WPA3_PSK
        bool "WPA3-PSK"              # 新增（位置对应）
endchoice
```

---

## 翻译规范快速参考

### 需要翻译的元素

| 元素 | 英文示例 | 中文示例 |
|------|---------|---------|
| `menu` | `menu "WiFi Configuration"` | `menu "WiFi 配置"` |
| `prompt` | `string "WiFi SSID"` | `string "WiFi 网络名称（SSID）"` |
| `bool` | `bool "Enable Debug"` | `bool "启用调试"` |
| `int` | `int "Timeout (ms)"` | `int "超时时间（毫秒）"` |
| `help` | `help\n    Help text...` | `help\n    帮助文本...` |

### 不翻译的元素

| 元素 | 说明 | 示例 |
|------|------|------|
| `config XXX` | 配置项名称 | `config WIFI_SSID` |
| `default` | 默认值 | `default "192.168.1.1"` |
| `range` | 数值范围 | `range 1 100` |
| `depends on` | 依赖条件 | `depends on WIFI_ENABLE` |

### 术语对照表

| 英文 | 中文 | 说明 |
|------|------|------|
| WiFi | WiFi | 保持英文 |
| SSID | SSID（网络名称） | 混用，括号补充 |
| IP Address | IP 地址 | 混用 |
| Port | 端口 | 完全翻译 |
| Timeout | 超时时间 | 完全翻译 |
| ROS | ROS | 保持英文 |
| Domain ID | Domain ID（域标识） | 混用 |
| QoS | QoS（服务质量） | 混用 |
| Agent | Agent | 保持英文 |
| Node | 节点 | 完全翻译 |

### 文本长度建议

- 菜单标题：8-12 个汉字
- 配置项提示：4-15 个汉字
- 帮助文本：详细说明，无长度限制

---

## 提交前检查清单

在提交代码前，请确认以下项目：

### 必做检查 ✅

- [ ] 英文版和中文版都已更新
- [ ] 配置项名称（`config XXX`）完全一致
- [ ] 配置项顺序完全对应
- [ ] 默认值保持一致
- [ ] 运行 `python tools/validate_kconfig.py` 通过
- [ ] 在英文模式下测试过 menuconfig
- [ ] 在中文模式下测试过 menuconfig
- [ ] Git 提交信息清晰描述了更改内容

### 可选检查 💡

- [ ] 更新相关文档（如 [`需求分析/Menuconfig配置项说明.md`](../需求分析/Menuconfig配置项说明.md:1)）
- [ ] 在 [`config/README.md`](../config/README.md:1) 中记录变更
- [ ] 检查是否影响现有代码（如新增配置项需要在代码中使用）

---

## 常见问题处理

### 问题 1: 验证工具报错配置项不匹配

**错误信息**：
```
❌ 错误: 第 15 个配置项不匹配:
   英文版: WIFI_RETRY_MAX
   中文版: WIFI_TIMEOUT
```

**原因**：配置项顺序不一致

**解决方案**：
1. 检查第 15 个配置项附近的代码
2. 确保中英文版本的配置项顺序完全一致
3. 逐一对比，找出遗漏或多余的配置项

### 问题 2: 菜单层级不匹配

**错误信息**：
```
❌ 警告: 菜单结构可能不一致
   英文版: 4 个菜单
   中文版: 3 个菜单
```

**原因**：`menu` 和 `endmenu` 数量不匹配

**解决方案**：
1. 检查是否有遗漏的 `endmenu`
2. 确保 `menu` 和 `endmenu` 成对出现
3. 使用文本编辑器的括号匹配功能辅助检查

### 问题 3: 翻译不准确

**场景**：技术术语翻译不统一

**解决方案**：
1. 参考本文档的"术语对照表"
2. 查看 [`docs/MENUCONFIG_I18N_DESIGN.md`](MENUCONFIG_I18N_DESIGN.md:1) 中的翻译规范
3. 保持与现有翻译风格一致
4. 必要时咨询团队成员

### 问题 4: Git 冲突

**场景**：多人同时修改配置文件导致冲突

**解决方案**：
```bash
# 1. 拉取最新代码
git pull

# 2. 解决冲突（使用文本编辑器）
# 确保合并后中英文版本保持一致

# 3. 验证一致性
python tools/validate_kconfig.py

# 4. 测试两种语言
python tools/menuconfig_lang.py en && pio run -t menuconfig
python tools/menuconfig_lang.py zh && pio run -t menuconfig

# 5. 提交
git add config/Kconfig.projbuild.*
git commit -m "resolve: 解决配置文件冲突"
```

---

## 高级技巧

### 技巧 1: 使用 diff 工具对比

```bash
# 快速对比中英文版本的结构
diff -u config/Kconfig.projbuild.en config/Kconfig.projbuild.zh
```

### 技巧 2: 批量翻译辅助

对于大量配置项，可以：
1. 先完成英文版全部修改
2. 使用文本编辑器的"查找替换"功能翻译固定模式
3. 手动调整特殊翻译
4. 运行验证工具确认

### 技巧 3: 使用编辑器插件

推荐 VS Code 插件：
- **Kconfig Language Support** - 语法高亮
- **Todo Tree** - 标记待翻译项
- **Bracket Pair Colorizer** - 括号匹配

### 技巧 4: 自动化验证

在 Git pre-commit hook 中添加验证：

`.git/hooks/pre-commit`:
```bash
#!/bin/bash
# 自动验证 Kconfig 一致性

if git diff --cached --name-only | grep -q "config/Kconfig.projbuild"; then
    echo "检测到 Kconfig 文件修改，运行验证..."
    python tools/validate_kconfig.py
    if [ $? -ne 0 ]; then
        echo "❌ Kconfig 验证失败，请修复后再提交"
        exit 1
    fi
fi
```

---

## 示例：完整的配置项添加过程

### 场景：添加 ROS QoS 可靠性配置

**需求**：添加 ROS 服务质量（QoS）可靠性策略配置

#### 步骤 1: 编辑英文版

```kconfig
menu "ROS Configuration"
    # ... 现有配置 ...
    
    choice ROS_QOS_RELIABILITY
        prompt "ROS QoS Reliability"
        default ROS_QOS_RELIABLE
        help
            Quality of Service reliability policy for ROS topics.
            
            - Reliable: Ensures all messages are delivered
            - Best Effort: May drop messages under load
        
        config ROS_QOS_RELIABLE
            bool "Reliable"
        config ROS_QOS_BEST_EFFORT
            bool "Best Effort"
    endchoice
endmenu
```

#### 步骤 2: 编辑中文版

```kconfig
menu "ROS 配置"
    # ... 现有配置 ...
    
    choice ROS_QOS_RELIABILITY
        prompt "ROS 服务质量可靠性策略"
        default ROS_QOS_RELIABLE
        help
            ROS 话题的服务质量（QoS）可靠性策略。
            
            - 可靠模式：确保所有消息都被传递
            - 尽力而为：负载高时可能丢弃消息
        
        config ROS_QOS_RELIABLE
            bool "可靠模式"
        config ROS_QOS_BEST_EFFORT
            bool "尽力而为模式"
    endchoice
endmenu
```

#### 步骤 3: 验证

```bash
$ python tools/validate_kconfig.py
============================================================
✅ 验证通过：两个版本保持一致
============================================================
```

#### 步骤 4: 测试

```bash
# 英文测试
$ python tools/menuconfig_lang.py en
$ pio run -t menuconfig
# 界面显示：ROS QoS Reliability [Reliable/Best Effort]

# 中文测试  
$ python tools/menuconfig_lang.py zh
$ pio run -t menuconfig
# 界面显示：ROS 服务质量可靠性策略 [可靠模式/尽力而为模式]
```

#### 步骤 5: 提交

```bash
$ git add config/Kconfig.projbuild.*
$ git commit -m "menuconfig: 添加 ROS QoS 可靠性配置

- 新增 ROS_QOS_RELIABILITY 选择项
- 提供可靠模式和尽力而为模式两种选项
- 同步更新中英文版本
- 验证通过
"
$ git push
```

---

## 参考资源

### 项目文档
- 📖 [架构设计文档](MENUCONFIG_I18N_DESIGN.md) - 完整的技术设计
- 🚀 [快速入门指南](MENUCONFIG_I18N_QUICKSTART.md) - 用户使用指南
- 🧪 [测试报告](MENUCONFIG_I18N_TEST_REPORT.md) - 功能测试结果

### 工具脚本
- [`tools/menuconfig_lang.py`](../tools/menuconfig_lang.py:1) - 语言切换脚本
- [`tools/validate_kconfig.py`](../tools/validate_kconfig.py:1) - 验证工具

### 外部参考
- [Kconfig 语法文档](https://www.kernel.org/doc/html/latest/kbuild/kconfig-language.html)
- [ESP-IDF Kconfig 指南](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32/api-reference/kconfig.html)

---

## 获取帮助

遇到问题时：

1. 📖 查阅本文档的"常见问题处理"章节
2. 🔍 查看 [架构设计文档](MENUCONFIG_I18N_DESIGN.md) 了解技术细节
3. 💬 在项目 Issue 中提问
4. 👥 咨询团队成员

---

**文档版本**: v1.0.0  
**最后更新**: 2025-10-24  
**维护者**: 技术团队