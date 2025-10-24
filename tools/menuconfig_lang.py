#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Menuconfig Language Switcher
用于在英文和中文menuconfig之间切换

用法:
    python tools/menuconfig_lang.py [en|zh]
    python tools/menuconfig_lang.py --status
    python tools/menuconfig_lang.py --help
"""

import os
import sys
import shutil
import argparse
from pathlib import Path
from typing import Optional
from datetime import datetime


# ============================================================================
# 配置常量
# ============================================================================

PROJECT_ROOT = Path(__file__).parent.parent
CONFIG_DIR = PROJECT_ROOT / "config"
BACKUP_DIR = CONFIG_DIR / ".backup"
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

# 颜色输出（ANSI转义码）
class Colors:
    """终端颜色定义"""
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    BOLD = '\033[1m'
    END = '\033[0m'


# ============================================================================
# 核心功能函数
# ============================================================================

def print_success(message: str) -> None:
    """打印成功消息（绿色）"""
    print(f"{Colors.GREEN}✅ {message}{Colors.END}")


def print_error(message: str) -> None:
    """打印错误消息（红色）"""
    print(f"{Colors.RED}❌ {message}{Colors.END}")


def print_warning(message: str) -> None:
    """打印警告消息（黄色）"""
    print(f"{Colors.YELLOW}⚠️  {message}{Colors.END}")


def print_info(message: str) -> None:
    """打印信息消息（蓝色）"""
    print(f"{Colors.BLUE}💡 {message}{Colors.END}")


def get_current_language() -> str:
    """获取当前设置的语言"""
    if CURRENT_LANG_FILE.exists():
        try:
            lang = CURRENT_LANG_FILE.read_text().strip()
            if lang in SUPPORTED_LANGUAGES:
                return lang
        except Exception:
            pass
    return DEFAULT_LANGUAGE


def set_current_language(lang: str) -> None:
    """保存当前语言设置"""
    CONFIG_DIR.mkdir(exist_ok=True)
    CURRENT_LANG_FILE.write_text(lang)


def validate_language(lang: str) -> bool:
    """验证语言代码是否有效"""
    return lang in SUPPORTED_LANGUAGES


def verify_kconfig_file(file_path: Path) -> bool:
    """验证Kconfig文件基本格式"""
    try:
        content = file_path.read_text(encoding='utf-8')
        # 检查必须存在的关键字
        required_keywords = ['menu', 'config', 'endmenu']
        return all(keyword in content for keyword in required_keywords)
    except Exception:
        return False


def create_backup(file_path: Path) -> Optional[Path]:
    """创建带时间戳的备份文件"""
    if not file_path.exists():
        return None
    
    try:
        # 创建备份目录
        BACKUP_DIR.mkdir(parents=True, exist_ok=True)
        
        # 生成带时间戳的备份文件名
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup_name = f"{file_path.name}.{timestamp}.bak"
        backup_path = BACKUP_DIR / backup_name
        
        # 复制文件
        shutil.copy2(file_path, backup_path)
        return backup_path
    except Exception as e:
        print_warning(f"备份创建失败: {e}")
        return None


def atomic_copy(src: Path, dst: Path) -> bool:
    """原子性文件复制（使用临时文件+替换）"""
    temp_file = dst.with_suffix('.tmp')
    try:
        # 复制到临时文件
        shutil.copy2(src, temp_file)
        
        # 验证临时文件
        if not verify_kconfig_file(temp_file):
            print_error("源文件格式验证失败")
            temp_file.unlink()
            return False
        
        # 原子性替换
        temp_file.replace(dst)
        return True
    except Exception as e:
        print_error(f"文件复制失败: {e}")
        if temp_file.exists():
            temp_file.unlink()
        return False


def switch_language(target_lang: str) -> bool:
    """切换到指定语言"""
    try:
        # 1. 验证语言
        if not validate_language(target_lang):
            print_error(f"不支持的语言 '{target_lang}'")
            print(f"   支持的语言: {', '.join(SUPPORTED_LANGUAGES.keys())}")
            return False
        
        # 2. 检查源文件
        source_file = SUPPORTED_LANGUAGES[target_lang]['source']
        if not source_file.exists():
            print_error(f"源文件不存在: {source_file}")
            print_info(f"请确保 {source_file.name} 存在于 config/ 目录中")
            return False
        
        # 3. 验证源文件格式
        if not verify_kconfig_file(source_file):
            print_error(f"源文件格式无效: {source_file}")
            return False
        
        # 4. 检查是否已经是目标语言
        current_lang = get_current_language()
        if current_lang == target_lang:
            print_info(f"当前已经是 {SUPPORTED_LANGUAGES[target_lang]['name']} 模式")
            return True
        
        # 5. 备份当前文件
        if KCONFIG_TARGET.exists():
            backup_path = create_backup(KCONFIG_TARGET)
            if backup_path:
                print(f"📦 已备份当前配置: {backup_path.relative_to(PROJECT_ROOT)}")
        
        # 6. 原子性复制新文件
        if not atomic_copy(source_file, KCONFIG_TARGET):
            return False
        
        print_success(f"已切换到 {SUPPORTED_LANGUAGES[target_lang]['name']}")
        print(f"   源文件: {source_file.relative_to(PROJECT_ROOT)}")
        print(f"   目标文件: {KCONFIG_TARGET.relative_to(PROJECT_ROOT)}")
        
        # 7. 保存语言设置
        set_current_language(target_lang)
        
        # 8. 提示用户运行menuconfig
        print()
        print_info("现在可以运行 menuconfig:")
        print(f"   {Colors.CYAN}pio run -t menuconfig{Colors.END}")
        
        return True
        
    except Exception as e:
        print_error(f"切换失败: {e}")
        return False


def show_status() -> None:
    """显示当前状态"""
    current = get_current_language()
    
    print("=" * 60)
    print(f"{Colors.BOLD}Menuconfig 语言状态{Colors.END}")
    print("=" * 60)
    
    # 当前语言
    current_info = SUPPORTED_LANGUAGES[current]
    print(f"当前语言: {Colors.GREEN}{current_info['name']} ({current}){Colors.END}")
    print(f"配置文件: {KCONFIG_TARGET.relative_to(PROJECT_ROOT)}")
    
    # 文件状态
    if KCONFIG_TARGET.exists():
        size = KCONFIG_TARGET.stat().st_size
        mtime = datetime.fromtimestamp(KCONFIG_TARGET.stat().st_mtime)
        print(f"文件大小: {size} 字节")
        print(f"修改时间: {mtime.strftime('%Y-%m-%d %H:%M:%S')}")
    else:
        print_warning("配置文件不存在")
    
    print()
    print("可用语言:")
    for code, info in SUPPORTED_LANGUAGES.items():
        marker = "✓" if code == current else " "
        exists = info['source'].exists()
        status = f"{Colors.GREEN}存在{Colors.END}" if exists else f"{Colors.RED}缺失{Colors.END}"
        
        print(f"  [{marker}] {code:4s} - {info['name']:10s} ({status})")
        if exists and code != current:
            print(f"       切换命令: {Colors.CYAN}python tools/menuconfig_lang.py {code}{Colors.END}")
    
    print("=" * 60)


# ============================================================================
# 命令行接口
# ============================================================================

def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description='Menuconfig 语言切换工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  切换到中文:  python tools/menuconfig_lang.py zh
  切换到英文:  python tools/menuconfig_lang.py en
  查看状态:    python tools/menuconfig_lang.py --status
  查看帮助:    python tools/menuconfig_lang.py --help
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
    
    # 显示状态（无参数或--status）
    if args.status or args.language is None:
        show_status()
        return
    
    # 切换语言
    success = switch_language(args.language)
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()