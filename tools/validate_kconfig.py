#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Kconfig文件一致性验证工具
验证英文版和中文版的配置项名称、数量、顺序是否一致

用法:
    python tools/validate_kconfig.py              # 验证所有语言
    python tools/validate_kconfig.py --lang zh    # 只验证中文版
    python tools/validate_kconfig.py --verbose    # 详细输出
"""

import re
import sys
import argparse
from pathlib import Path
from typing import List, Tuple, Dict, Optional


# ============================================================================
# 配置常量
# ============================================================================

PROJECT_ROOT = Path(__file__).parent.parent
CONFIG_DIR = PROJECT_ROOT / "config"
EN_FILE = CONFIG_DIR / "Kconfig.projbuild.en"
ZH_FILE = CONFIG_DIR / "Kconfig.projbuild.zh"


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
# 解析函数
# ============================================================================

def extract_config_items(file_path: Path) -> List[Tuple[int, str]]:
    """
    提取所有config配置项名称及其行号
    
    返回: [(行号, 配置项名称), ...]
    """
    content = file_path.read_text(encoding='utf-8')
    items = []
    
    # 匹配 "config CONFIG_NAME" 模式
    pattern = r'^\s*config\s+(\w+)'
    
    for line_num, line in enumerate(content.split('\n'), start=1):
        match = re.match(pattern, line)
        if match:
            config_name = match.group(1)
            items.append((line_num, config_name))
    
    return items


def extract_menu_structure(file_path: Path) -> List[Tuple[int, str, int]]:
    """
    提取菜单结构（menu/endmenu的层级）
    
    返回: [(行号, 类型, 缩进级别), ...]
    """
    content = file_path.read_text(encoding='utf-8')
    structure = []
    indent_level = 0
    
    for line_num, line in enumerate(content.split('\n'), start=1):
        stripped = line.strip()
        
        if stripped.startswith('menu '):
            structure.append((line_num, 'menu', indent_level))
            indent_level += 1
        elif stripped == 'endmenu':
            indent_level -= 1
            structure.append((line_num, 'endmenu', indent_level))
        elif stripped.startswith('choice'):
            structure.append((line_num, 'choice', indent_level))
            indent_level += 1
        elif stripped == 'endchoice':
            indent_level -= 1
            structure.append((line_num, 'endchoice', indent_level))
    
    return structure


def extract_menu_titles(file_path: Path) -> List[Tuple[int, str]]:
    """
    提取所有菜单标题
    
    返回: [(行号, 菜单标题), ...]
    """
    content = file_path.read_text(encoding='utf-8')
    titles = []
    
    # 匹配 menu "标题" 模式
    pattern = r'^\s*menu\s+"([^"]+)"'
    
    for line_num, line in enumerate(content.split('\n'), start=1):
        match = re.match(pattern, line)
        if match:
            title = match.group(1)
            titles.append((line_num, title))
    
    return titles


def check_syntax_errors(file_path: Path) -> List[str]:
    """
    检查基础Kconfig语法错误
    
    返回: 错误列表
    """
    errors = []
    content = file_path.read_text(encoding='utf-8')
    lines = content.split('\n')
    
    menu_stack = []
    choice_stack = []
    
    for line_num, line in enumerate(lines, start=1):
        stripped = line.strip()
        
        # 检查menu/endmenu配对
        if stripped.startswith('menu '):
            menu_stack.append(line_num)
        elif stripped == 'endmenu':
            if not menu_stack:
                errors.append(f"行 {line_num}: 'endmenu' 没有对应的 'menu'")
            else:
                menu_stack.pop()
        
        # 检查choice/endchoice配对
        if stripped.startswith('choice'):
            choice_stack.append(line_num)
        elif stripped == 'endchoice':
            if not choice_stack:
                errors.append(f"行 {line_num}: 'endchoice' 没有对应的 'choice'")
            else:
                choice_stack.pop()
    
    # 检查未闭合的结构
    if menu_stack:
        errors.append(f"未闭合的 'menu'，起始行: {', '.join(map(str, menu_stack))}")
    if choice_stack:
        errors.append(f"未闭合的 'choice'，起始行: {', '.join(map(str, choice_stack))}")
    
    return errors


# ============================================================================
# 验证函数
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
    print(f"{Colors.BLUE}ℹ️  {message}{Colors.END}")


def validate_single_file(file_path: Path, lang_name: str, verbose: bool = False) -> bool:
    """验证单个文件的语法"""
    print(f"\n{Colors.BOLD}验证 {lang_name} 版本{Colors.END}")
    print(f"文件: {file_path.relative_to(PROJECT_ROOT)}")
    print("-" * 60)
    
    # 检查文件存在
    if not file_path.exists():
        print_error(f"文件不存在: {file_path}")
        return False
    
    # 检查语法错误
    errors = check_syntax_errors(file_path)
    if errors:
        print_error(f"发现 {len(errors)} 个语法错误:")
        for error in errors:
            print(f"  {error}")
        return False
    
    print_success("语法检查通过")
    
    # 统计信息
    config_items = extract_config_items(file_path)
    menu_structure = extract_menu_structure(file_path)
    menu_count = len([x for x in menu_structure if x[1] == 'menu'])
    choice_count = len([x for x in menu_structure if x[1] == 'choice'])
    
    print(f"📊 统计信息:")
    print(f"   配置项数量: {len(config_items)}")
    print(f"   菜单数量: {menu_count}")
    print(f"   选择项数量: {choice_count}")
    
    if verbose:
        print(f"\n📋 配置项列表:")
        for line_num, name in config_items[:10]:  # 只显示前10个
            print(f"   行 {line_num:4d}: {name}")
        if len(config_items) > 10:
            print(f"   ... 还有 {len(config_items) - 10} 个配置项")
    
    return True


def validate_consistency(verbose: bool = False) -> bool:
    """验证中英文版本的一致性"""
    print("=" * 60)
    print(f"{Colors.BOLD}Kconfig 文件一致性验证{Colors.END}")
    print("=" * 60)
    
    # 1. 检查文件存在
    if not EN_FILE.exists():
        print_error(f"英文版文件不存在: {EN_FILE}")
        return False
    
    if not ZH_FILE.exists():
        print_error(f"中文版文件不存在: {ZH_FILE}")
        return False
    
    print_success("文件存在检查通过")
    
    # 2. 提取配置项
    en_configs = extract_config_items(EN_FILE)
    zh_configs = extract_config_items(ZH_FILE)
    
    print(f"\n📊 配置项统计:")
    print(f"   英文版: {len(en_configs)} 个配置项")
    print(f"   中文版: {len(zh_configs)} 个配置项")
    
    # 3. 验证数量
    if len(en_configs) != len(zh_configs):
        print_error(f"配置项数量不一致！")
        print(f"   差异: {abs(len(en_configs) - len(zh_configs))} 个配置项")
        
        # 找出缺失或多余的配置项
        en_names = set(name for _, name in en_configs)
        zh_names = set(name for _, name in zh_configs)
        
        only_en = en_names - zh_names
        only_zh = zh_names - en_names
        
        if only_en:
            print(f"\n仅在英文版中存在的配置项:")
            for name in sorted(only_en):
                print(f"  - {name}")
        
        if only_zh:
            print(f"\n仅在中文版中存在的配置项:")
            for name in sorted(only_zh):
                print(f"  - {name}")
        
        return False
    
    print_success("配置项数量一致")
    
    # 4. 验证名称和顺序
    all_match = True
    mismatches = []
    
    for i, ((en_line, en_name), (zh_line, zh_name)) in enumerate(zip(en_configs, zh_configs)):
        if en_name != zh_name:
            all_match = False
            mismatches.append({
                'index': i + 1,
                'en_line': en_line,
                'en_name': en_name,
                'zh_line': zh_line,
                'zh_name': zh_name
            })
    
    if all_match:
        print_success("所有配置项名称和顺序一致")
    else:
        print_error(f"发现 {len(mismatches)} 处配置项不匹配:")
        for mismatch in mismatches[:5]:  # 只显示前5个
            print(f"\n  第 {mismatch['index']} 个配置项:")
            print(f"    英文版 (行 {mismatch['en_line']:4d}): {mismatch['en_name']}")
            print(f"    中文版 (行 {mismatch['zh_line']:4d}): {mismatch['zh_name']}")
        
        if len(mismatches) > 5:
            print(f"\n  ... 还有 {len(mismatches) - 5} 处不匹配")
        
        return False
    
    # 5. 验证菜单结构
    en_structure = extract_menu_structure(EN_FILE)
    zh_structure = extract_menu_structure(ZH_FILE)
    
    if len(en_structure) != len(zh_structure):
        print_warning("菜单结构数量不一致")
        print(f"   英文版: {len(en_structure)} 个结构元素")
        print(f"   中文版: {len(zh_structure)} 个结构元素")
    else:
        # 检查结构类型是否一致
        structure_match = True
        for (en_line, en_type, en_level), (zh_line, zh_type, zh_level) in zip(en_structure, zh_structure):
            if en_type != zh_type or en_level != zh_level:
                structure_match = False
                break
        
        if structure_match:
            print_success("菜单结构一致")
        else:
            print_warning("菜单结构存在差异")
    
    # 6. 详细信息（verbose模式）
    if verbose:
        print(f"\n{Colors.BOLD}详细信息:{Colors.END}")
        
        en_menus = extract_menu_titles(EN_FILE)
        zh_menus = extract_menu_titles(ZH_FILE)
        
        print(f"\n菜单标题对比:")
        for (en_line, en_title), (zh_line, zh_title) in zip(en_menus, zh_menus):
            print(f"  EN (行 {en_line:4d}): {en_title}")
            print(f"  ZH (行 {zh_line:4d}): {zh_title}")
            print()
    
    # 最终结果
    print("\n" + "=" * 60)
    if all_match:
        print_success("验证通过：两个版本保持一致")
        print("=" * 60)
        return True
    else:
        print_error("验证失败：发现不一致")
        print("=" * 60)
        return False


# ============================================================================
# 命令行接口
# ============================================================================

def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description='Kconfig 文件一致性验证工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  验证所有语言:        python tools/validate_kconfig.py
  只验证中文版:        python tools/validate_kconfig.py --lang zh
  详细输出:            python tools/validate_kconfig.py --verbose
  只检查语法:          python tools/validate_kconfig.py --syntax-only
        """
    )
    
    parser.add_argument(
        '--lang',
        choices=['en', 'zh'],
        help='只验证指定语言版本'
    )
    
    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='显示详细输出'
    )
    
    parser.add_argument(
        '--syntax-only',
        action='store_true',
        help='只检查语法，不验证一致性'
    )
    
    args = parser.parse_args()
    
    success = True
    
    # 只验证单个文件
    if args.lang:
        if args.lang == 'en':
            success = validate_single_file(EN_FILE, '英文', args.verbose)
        else:
            success = validate_single_file(ZH_FILE, '中文', args.verbose)
    
    # 只检查语法
    elif args.syntax_only:
        print("=" * 60)
        print(f"{Colors.BOLD}语法检查{Colors.END}")
        print("=" * 60)
        
        en_ok = validate_single_file(EN_FILE, '英文', args.verbose)
        zh_ok = validate_single_file(ZH_FILE, '中文', args.verbose)
        
        success = en_ok and zh_ok
    
    # 完整验证
    else:
        success = validate_consistency(args.verbose)
    
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()