"""
CSV数据可视化辅助脚本

功能：读取CSV文件，将物理量数据生成可视化图表
- 支持组合图表（XYZ三轴合一）
- 支持独立图表（各轴单独显示）
- 输出SVG格式，使用Times New Roman字体
- 按物理量分类存储到子文件夹

作者：Assistant
日期：2026-02-10
"""

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib
from matplotlib import rcParams
from pathlib import Path
import re
from typing import Dict, List, Tuple, Optional
import argparse
import sys


# =============================================================================
# 配置模块
# =============================================================================

# 物理量分组配置
# 定义各物理量对应的CSV列名模式、输出文件夹、单位等
PHYSICAL_QUANTITIES_CONFIG = {
    'position': {
        'patterns': [r'pos_[xyz]', r'position_[xyz]', r'p[xyz]', r'x_pos', r'y_pos', r'z_pos'],
        'folder': 'position',
        'unit': 'm',
        'labels': ['X', 'Y', 'Z'],
        'full_name': 'Position',
        'color': ['#1f77b4', '#ff7f0e', '#2ca02c']  # 蓝、橙、绿
    },
    'velocity': {
        'patterns': [r'vel_[xyz]', r'velocity_[xyz]', r'v[xyz]', r'vx', r'vy', r'vz'],
        'folder': 'velocity',
        'unit': 'm/s',
        'labels': ['Vx', 'Vy', 'Vz'],
        'full_name': 'Velocity',
        'color': ['#d62728', '#9467bd', '#8c564b']  # 红、紫、棕
    },
    'acceleration': {
        'patterns': [r'^ax$', r'^ay$', r'^az$', r'acc_[xyz]', r'acceleration_[xyz]'],
        'folder': 'acceleration',
        'unit': 'm/s²',
        'labels': ['Ax', 'Ay', 'Az'],
        'full_name': 'Acceleration',
        'color': ['#e377c2', '#7f7f7f', '#bcbd22']  # 粉、灰、黄绿
    },
    'force': {
        'patterns': [r'force_[xyz]', r'f[xyz]', r'fx', r'fy', r'fz'],
        'folder': 'force',
        'unit': 'N',
        'labels': ['Fx', 'Fy', 'Fz'],
        'full_name': 'Force',
        'color': ['#17becf', '#aec7e8', '#ffbb78']
    },
    'torque': {
        'patterns': [r'torque_[xyz]', r'tau_[xyz]', r't[xyz]', r'tx', r'ty', r'tz'],
        'folder': 'torque',
        'unit': 'N·m',
        'labels': ['Tx', 'Ty', 'Tz'],
        'full_name': 'Torque',
        'color': ['#98df8a', '#ff9896', '#c5b0d5']
    },
    'attitude': {
        'patterns': [r'att_[xyz]', r'attitude_[xyz]', r'roll', r'pitch', r'yaw', 
                     r'phi', r'theta', r'psi'],
        'folder': 'attitude',
        'unit': 'rad',
        'labels': ['Roll', 'Pitch', 'Yaw'],
        'full_name': 'Attitude',
        'color': ['#c49c94', '#f7b6d3', '#c7c7c7']
    },
    'quaternion': {
        'patterns': [r'q[xyzw]', r'quat_[xyzw]', r'quaternion_[xyzw]'],
        'folder': 'quaternion',
        'unit': '',
        'labels': ['qx', 'qy', 'qz', 'qw'],
        'full_name': 'Quaternion',
        'color': ['#c49c94', '#f7b6d3', '#c7c7c7', '#bcbd22']
    },
    'angular_velocity': {
        'patterns': [r'omega_[xyz]', r'ang_vel_[xyz]', r'w[xyz]', r'wx', r'wy', r'wz'],
        'folder': 'angular_velocity',
        'unit': 'rad/s',
        'labels': ['ωx', 'ωy', 'ωz'],
        'full_name': 'Angular Velocity',
        'color': ['#dbdb8d', '#9edae5', '#393b79']
    },
    'angular_acceleration': {
        'patterns': [r'alpha_[xyz]', r'ang_acc_[xyz]', r'alphax', r'alphay', r'alphaz'],
        'folder': 'angular_acceleration',
        'unit': 'rad/s²',
        'labels': ['αx', 'αy', 'αz'],
        'full_name': 'Angular Acceleration',
        'color': ['#8c564b', '#e377c2', '#7f7f7f']
    }
}

# 时间列的识别模式
TIME_PATTERNS = [
    r'^time$', r'^t$', r'^timestamp$', r'^step$', 
    r'^iteration$', r'^sim_time$', r'^elapsed_time$'
]

# 图表样式配置
PLOT_STYLE_CONFIG = {
    'figure_size': (10, 6),
    'dpi': 300,
    'line_width': 1.5,
    'font_size': 12,
    'title_size': 14,
    'label_size': 12,
    'legend_size': 10,
    'tick_size': 10,
    'grid_alpha': 0.3,
    'grid_linestyle': '--'
}


def setup_fonts():
    """
    配置全局字体为Times New Roman
    
    设置matplotlib的所有文本元素使用Times字体
    """
    rcParams['font.family'] = 'serif'
    rcParams['font.serif'] = ['Times New Roman']
    rcParams['axes.unicode_minus'] = False  # 解决负号显示问题
    rcParams['font.size'] = PLOT_STYLE_CONFIG['font_size']
    rcParams['axes.titlesize'] = PLOT_STYLE_CONFIG['title_size']
    rcParams['axes.labelsize'] = PLOT_STYLE_CONFIG['label_size']
    rcParams['xtick.labelsize'] = PLOT_STYLE_CONFIG['tick_size']
    rcParams['ytick.labelsize'] = PLOT_STYLE_CONFIG['tick_size']
    rcParams['legend.fontsize'] = PLOT_STYLE_CONFIG['legend_size']


# =============================================================================
# CSV解析模块
# =============================================================================

class CSVParser:
    """
    CSV文件解析器
    
    负责读取CSV文件、识别时间列、按物理量分组数据列
    支持多物体数据（按body_id分组）
    """
    
    def __init__(self, file_path: str):
        """
        初始化解析器
        
        Args:
            file_path: CSV文件路径
        """
        self.file_path = Path(file_path)
        self.df = None
        self.time_col = None
        self.grouped_columns = {}
        self.body_id_col = None
        self.body_name_col = None
        self.unique_bodies = []
        
    def read_csv(self) -> bool:
        """
        读取CSV文件
        
        Returns:
            bool: 读取成功返回True，否则返回False
        """
        try:
            self.df = pd.read_csv(self.file_path)
            print(f"成功读取CSV文件: {self.file_path}")
            print(f"数据形状: {self.df.shape}")
            print(f"列名: {list(self.df.columns)}")
            return True
        except Exception as e:
            print(f"读取CSV文件失败: {e}")
            return False
    
    def identify_time_column(self) -> Optional[str]:
        """
        识别时间列
        
        根据预定义的时间列模式匹配列名
        
        Returns:
            str: 时间列名，未找到返回None
        """
        for col in self.df.columns:
            col_lower = col.lower().strip()
            for pattern in TIME_PATTERNS:
                if re.match(pattern, col_lower, re.IGNORECASE):
                    self.time_col = col
                    print(f"识别到时间列: {col}")
                    return col
        
        # 如果没有匹配到，使用第一列作为时间列（假设）
        if len(self.df.columns) > 0:
            self.time_col = self.df.columns[0]
            print(f"未识别到时间列，使用第一列作为时间列: {self.time_col}")
            return self.time_col
        
        return None
    
    def identify_body_columns(self) -> bool:
        """
        识别物体标识列（body_id和body_name）
        
        Returns:
            bool: 是否找到物体标识列
        """
        # 查找body_id列
        for col in self.df.columns:
            if col.lower() in ['body_id', 'bodyid', 'id']:
                self.body_id_col = col
                break
        
        # 查找body_name列
        for col in self.df.columns:
            if col.lower() in ['body_name', 'bodyname', 'name']:
                self.body_name_col = col
                break
        
        # 如果找到body_id，获取所有唯一的物体
        if self.body_id_col:
            self.unique_bodies = self.df[self.body_id_col].unique()
            print(f"识别到 {len(self.unique_bodies)} 个物体: {self.unique_bodies}")
            return True
        
        return False
    
    def get_body_data(self, body_id) -> pd.DataFrame:
        """
        获取指定物体的数据
        
        Args:
            body_id: 物体ID
            
        Returns:
            DataFrame: 该物体的数据
        """
        if self.body_id_col:
            return self.df[self.df[self.body_id_col] == body_id].copy()
        return self.df.copy()
    
    def get_body_name(self, body_id) -> str:
        """
        获取物体名称
        
        Args:
            body_id: 物体ID
            
        Returns:
            str: 物体名称或ID的字符串形式
        """
        if self.body_name_col:
            body_data = self.df[self.df[self.body_id_col] == body_id]
            if not body_data.empty:
                name = body_data[self.body_name_col].iloc[0]
                if pd.notna(name) and str(name).strip():
                    return str(name)
        return f"body_{body_id}"
    
    def group_columns_by_quantity(self) -> Dict[str, List[str]]:
        """
        按物理量分组数据列
        
        根据配置中的正则模式将列名分组到对应物理量
        
        Returns:
            Dict: 物理量名称到列名列表的映射
        """
        self.grouped_columns = {}
        remaining_columns = set(self.df.columns) - {self.time_col}
        
        for quantity_name, config in PHYSICAL_QUANTITIES_CONFIG.items():
            matched_cols = []
            patterns = config['patterns']
            
            for col in list(remaining_columns):
                col_lower = col.lower().strip()
                for pattern in patterns:
                    if re.search(pattern, col_lower, re.IGNORECASE):
                        matched_cols.append(col)
                        remaining_columns.discard(col)
                        break
            
            if matched_cols:
                # 按X, Y, Z顺序排序
                matched_cols = self._sort_xyz_columns(matched_cols)
                self.grouped_columns[quantity_name] = matched_cols
                print(f"物理量 '{quantity_name}' 匹配到列: {matched_cols}")
        
        # 处理未匹配的列
        if remaining_columns:
            print(f"未匹配的列: {remaining_columns}")
            # 将未匹配的列作为独立物理量处理
            for col in remaining_columns:
                self.grouped_columns[f'other_{col}'] = [col]
        
        return self.grouped_columns
    
    def _sort_xyz_columns(self, columns: List[str]) -> List[str]:
        """
        将XYZ三轴列按X, Y, Z顺序排序
        
        Args:
            columns: 列名列表
            
        Returns:
            List: 排序后的列名列表
        """
        def get_axis_order(col_name: str) -> int:
            """获取轴向排序权重"""
            col_lower = col_name.lower()
            if any(x in col_lower for x in ['_x', 'x_', 'x']):
                # 优先匹配明确的x标记
                if '_x' in col_lower or 'x_' in col_lower or col_lower.endswith('x'):
                    return 0
            if any(y in col_lower for y in ['_y', 'y_', 'y']):
                if '_y' in col_lower or 'y_' in col_lower or col_lower.endswith('y'):
                    return 1
            if any(z in col_lower for z in ['_z', 'z_', 'z']):
                if '_z' in col_lower or 'z_' in col_lower or col_lower.endswith('z'):
                    return 2
            # 对于roll/pitch/yaw或phi/theta/psi
            if any(r in col_lower for r in ['roll', 'phi']):
                return 0
            if any(p in col_lower for p in ['pitch', 'theta']):
                return 1
            if any(y in col_lower for y in ['yaw', 'psi']):
                return 2
            return 3
        
        return sorted(columns, key=get_axis_order)
    
    def get_data(self) -> pd.DataFrame:
        """获取数据DataFrame"""
        return self.df
    
    def get_time_data(self) -> pd.Series:
        """获取时间数据"""
        if self.time_col and self.time_col in self.df.columns:
            return self.df[self.time_col]
        return pd.Series(range(len(self.df)))


# =============================================================================
# 图表生成模块
# =============================================================================

class PlotGenerator:
    """
    图表生成器
    
    负责生成组合图表和独立图表
    支持按物体分目录存储
    """
    
    def __init__(self, output_base_dir: str, body_name: str = None):
        """
        初始化图表生成器
        
        Args:
            output_base_dir: 输出根目录路径
            body_name: 物体名称（用于创建子目录）
        """
        self.output_base_dir = Path(output_base_dir)
        self.body_name = body_name
        self._setup_output_dirs()
        
    def _setup_output_dirs(self):
        """创建输出目录结构"""
        # 如果有物体名称，创建物体子目录
        if self.body_name:
            self.output_base_dir = self.output_base_dir / self.body_name
        
        # 创建figures根目录
        self.output_base_dir.mkdir(parents=True, exist_ok=True)
        
        # 为每个物理量创建子文件夹
        for quantity_name, config in PHYSICAL_QUANTITIES_CONFIG.items():
            folder_path = self.output_base_dir / config['folder']
            folder_path.mkdir(exist_ok=True)
        
        # 创建其他物理量的文件夹
        others_path = self.output_base_dir / 'others'
        others_path.mkdir(exist_ok=True)
        
        print(f"输出目录结构已创建: {self.output_base_dir}")
    
    def _get_output_folder(self, quantity_name: str) -> Path:
        """
        获取物理量对应的输出文件夹
        
        Args:
            quantity_name: 物理量名称
            
        Returns:
            Path: 输出文件夹路径
        """
        if quantity_name in PHYSICAL_QUANTITIES_CONFIG:
            folder_name = PHYSICAL_QUANTITIES_CONFIG[quantity_name]['folder']
        elif quantity_name.startswith('other_'):
            folder_name = 'others'
        else:
            folder_name = 'others'
        
        return self.output_base_dir / folder_name
    
    def create_combined_plot(self, time_data: pd.Series, data_dict: Dict[str, pd.Series],
                            quantity_name: str, csv_filename: str) -> str:
        """
        创建组合图表（XYZ三轴合一）
        
        Args:
            time_data: 时间序列数据
            data_dict: 各轴数据字典 {列名: 数据序列}
            quantity_name: 物理量名称
            csv_filename: 原始CSV文件名（用于生成输出文件名）
            
        Returns:
            str: 生成的文件路径
        """
        config = PHYSICAL_QUANTITIES_CONFIG.get(quantity_name, {
            'unit': '',
            'labels': list(data_dict.keys()),
            'full_name': quantity_name,
            'color': ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd']
        })
        
        # 创建图形
        fig, ax = plt.subplots(figsize=PLOT_STYLE_CONFIG['figure_size'])
        
        # 绘制各轴数据
        columns = list(data_dict.keys())
        labels = config['labels'][:len(columns)] if len(config['labels']) >= len(columns) else columns
        colors = config['color'][:len(columns)] if len(config['color']) >= len(columns) else config['color']
        
        for i, (col, label) in enumerate(zip(columns, labels)):
            color = colors[i % len(colors)]
            ax.plot(time_data, data_dict[col], label=label, 
                   linewidth=PLOT_STYLE_CONFIG['line_width'], color=color)
        
        # 设置图表属性
        ax.set_xlabel('Time (s)', fontsize=PLOT_STYLE_CONFIG['label_size'])
        ax.set_ylabel(f"{config['full_name']} ({config['unit']})", 
                     fontsize=PLOT_STYLE_CONFIG['label_size'])
        ax.set_title(f"{config['full_name']} Components", 
                    fontsize=PLOT_STYLE_CONFIG['title_size'])
        ax.legend(loc='best', fontsize=PLOT_STYLE_CONFIG['legend_size'])
        ax.grid(True, alpha=PLOT_STYLE_CONFIG['grid_alpha'], 
               linestyle=PLOT_STYLE_CONFIG['grid_linestyle'])
        
        # 调整布局
        plt.tight_layout()
        
        # 保存图表
        output_folder = self._get_output_folder(quantity_name)
        base_name = Path(csv_filename).stem
        output_file = output_folder / f"{base_name}_{quantity_name}_combined.svg"
        
        plt.savefig(output_file, format='svg', dpi=PLOT_STYLE_CONFIG['dpi'],
                   bbox_inches='tight')
        plt.close(fig)
        
        print(f"生成组合图表: {output_file}")
        return str(output_file)
    
    def create_individual_plot(self, time_data: pd.Series, data: pd.Series,
                               column_name: str, quantity_name: str, 
                               axis_index: int, csv_filename: str) -> str:
        """
        创建独立图表（单轴）
        
        Args:
            time_data: 时间序列数据
            data: 单轴数据序列
            column_name: 列名
            quantity_name: 物理量名称
            axis_index: 轴向索引（0=X, 1=Y, 2=Z）
            csv_filename: 原始CSV文件名
            
        Returns:
            str: 生成的文件路径
        """
        config = PHYSICAL_QUANTITIES_CONFIG.get(quantity_name, {
            'unit': '',
            'labels': ['X', 'Y', 'Z'],
            'full_name': quantity_name,
            'color': ['#1f77b4', '#ff7f0e', '#2ca02c']
        })
        
        # 创建图形
        fig, ax = plt.subplots(figsize=PLOT_STYLE_CONFIG['figure_size'])
        
        # 获取颜色和标签
        color = config['color'][axis_index % len(config['color'])]
        label = config['labels'][axis_index] if axis_index < len(config['labels']) else column_name
        
        # 绘制数据
        ax.plot(time_data, data, linewidth=PLOT_STYLE_CONFIG['line_width'], 
               color=color, label=label)
        
        # 设置图表属性
        ax.set_xlabel('Time (s)', fontsize=PLOT_STYLE_CONFIG['label_size'])
        ax.set_ylabel(f"{config['full_name']} ({config['unit']})", 
                     fontsize=PLOT_STYLE_CONFIG['label_size'])
        ax.set_title(f"{config['full_name']} - {label} Component", 
                    fontsize=PLOT_STYLE_CONFIG['title_size'])
        ax.legend(loc='best', fontsize=PLOT_STYLE_CONFIG['legend_size'])
        ax.grid(True, alpha=PLOT_STYLE_CONFIG['grid_alpha'], 
               linestyle=PLOT_STYLE_CONFIG['grid_linestyle'])
        
        # 调整布局
        plt.tight_layout()
        
        # 保存图表
        output_folder = self._get_output_folder(quantity_name)
        base_name = Path(csv_filename).stem
        axis_suffix = ['x', 'y', 'z'][axis_index] if axis_index < 3 else str(axis_index)
        output_file = output_folder / f"{base_name}_{quantity_name}_{axis_suffix}.svg"
        
        plt.savefig(output_file, format='svg', dpi=PLOT_STYLE_CONFIG['dpi'],
                   bbox_inches='tight')
        plt.close(fig)
        
        print(f"生成独立图表: {output_file}")
        return str(output_file)
    
    def generate_all_plots(self, parser: CSVParser, csv_filename: str) -> List[str]:
        """
        生成所有图表
        
        Args:
            parser: CSV解析器实例
            csv_filename: CSV文件名
            
        Returns:
            List: 生成的所有文件路径列表
        """
        generated_files = []
        time_data = parser.get_time_data()
        df = parser.get_data()
        
        for quantity_name, columns in parser.grouped_columns.items():
            # 准备数据字典
            data_dict = {col: df[col] for col in columns}
            
            # 生成组合图表
            try:
                combined_file = self.create_combined_plot(
                    time_data, data_dict, quantity_name, csv_filename
                )
                generated_files.append(combined_file)
            except Exception as e:
                print(f"生成组合图表失败 ({quantity_name}): {e}")
            
            # 生成独立图表
            for i, col in enumerate(columns):
                try:
                    individual_file = self.create_individual_plot(
                        time_data, df[col], col, quantity_name, i, csv_filename
                    )
                    generated_files.append(individual_file)
                except Exception as e:
                    print(f"生成独立图表失败 ({col}): {e}")
        
        return generated_files


# =============================================================================
# 主函数和命令行接口
# =============================================================================

def process_csv_file(csv_file_path: str, output_dir: str = None) -> bool:
    """
    处理单个CSV文件
    
    支持多物体数据，所有图表保存在以CSV文件名命名的子文件夹中
    
    Args:
        csv_file_path: CSV文件路径
        output_dir: 输出目录，默认为figures目录
        
    Returns:
        bool: 处理成功返回True
    """
    csv_path = Path(csv_file_path)
    
    if not csv_path.exists():
        print(f"错误: 文件不存在 {csv_file_path}")
        return False
    
    # 设置输出目录：figures/CSV文件名（不含扩展名）
    csv_name = csv_path.stem  # 获取文件名（不含扩展名）
    if output_dir is None:
        # 默认输出到项目根目录的figures/CSV文件名/
        output_dir = csv_path.parent.parent / 'figures' / csv_name
    else:
        output_dir = Path(output_dir) / csv_name
    
    print(f"\n{'='*60}")
    print(f"处理文件: {csv_path.name}")
    print(f"输出目录: {output_dir}")
    print(f"{'='*60}\n")
    
    # 步骤1: 解析CSV
    parser = CSVParser(csv_file_path)
    if not parser.read_csv():
        return False
    
    # 步骤2: 识别时间列和物体列
    parser.identify_time_column()
    has_multiple_bodies = parser.identify_body_columns()
    
    # 步骤3: 分组列
    parser.group_columns_by_quantity()
    
    if not parser.grouped_columns:
        print("警告: 未识别到任何物理量数据列")
        return False
    
    # 步骤4: 生成图表
    total_files = 0
    
    if has_multiple_bodies and len(parser.unique_bodies) > 1:
        # 多物体情况：为每个物体单独生成图表
        print(f"\n检测到多物体数据，为每个物体单独生成图表...\n")
        
        for body_id in parser.unique_bodies:
            body_name = parser.get_body_name(body_id)
            print(f"\n处理物体: {body_name} (ID: {body_id})")
            print("-" * 40)
            
            # 获取该物体的数据
            body_data = parser.get_body_data(body_id)
            
            # 创建临时解析器用于该物体
            body_parser = CSVParser(csv_file_path)
            body_parser.df = body_data
            body_parser.time_col = parser.time_col
            body_parser.grouped_columns = parser.grouped_columns
            
            # 生成图表，按物体分目录（在CSV子文件夹下）
            generator = PlotGenerator(output_dir, body_name)
            generated_files = generator.generate_all_plots(body_parser, csv_path.name)
            total_files += len(generated_files)
    else:
        # 单物体情况：直接生成图表
        generator = PlotGenerator(output_dir)
        generated_files = generator.generate_all_plots(parser, csv_path.name)
        total_files = len(generated_files)
    
    print(f"\n{'='*60}")
    print(f"处理完成! 共生成 {total_files} 个图表文件")
    print(f"{'='*60}\n")
    
    return True


def main():
    """
    主函数 - 命令行入口
    """
    # 设置字体
    setup_fonts()
    
    # 解析命令行参数
    parser = argparse.ArgumentParser(
        description='CSV数据可视化工具 - 生成物理量曲线图表',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
示例:
  python csv_visualizer.py data.csv
  python csv_visualizer.py data.csv -o output_folder
  python csv_visualizer.py *.csv
        '''
    )
    
    parser.add_argument('csv_files', nargs='+', 
                       help='CSV文件路径（支持多个文件和通配符）')
    parser.add_argument('-o', '--output', type=str, default=None,
                       help='输出目录路径（默认为figures目录）')
    parser.add_argument('-l', '--list', action='store_true',
                       help='列出支持的物理量类型')
    
    args = parser.parse_args()
    
    # 列出支持的物理量
    if args.list:
        print("\n支持的物理量类型:")
        print("-" * 40)
        for name, config in PHYSICAL_QUANTITIES_CONFIG.items():
            print(f"  {name:20s} - {config['full_name']} ({config['unit']})")
            print(f"    匹配模式: {', '.join(config['patterns'][:3])}...")
        print()
        return
    
    # 处理CSV文件
    import glob
    
    success_count = 0
    fail_count = 0
    
    for pattern in args.csv_files:
        # 展开通配符
        matching_files = glob.glob(pattern)
        
        if not matching_files:
            # 如果没有匹配到，直接使用原路径
            matching_files = [pattern]
        
        for file_path in matching_files:
            if process_csv_file(file_path, args.output):
                success_count += 1
            else:
                fail_count += 1
    
    # 输出总结
    print(f"\n{'='*60}")
    print(f"处理总结: 成功 {success_count} 个文件, 失败 {fail_count} 个文件")
    print(f"{'='*60}")
    
    return 0 if fail_count == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
