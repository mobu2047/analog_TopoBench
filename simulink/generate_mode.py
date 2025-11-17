"""Simulink模型代码和配置自动生成器

功能说明：
1. 调用MATLAB引擎执行Simulink模型导出脚本
2. 自动生成模型参数配置文件（auto_params.yaml）
3. 可选：生成default.yaml配置文件

使用方法：
    # 使用默认模型名称
    python generate_mode.py
    
    # 指定模型名称
    python generate_mode.py --model Sub2_model_20210419
    
    # 指定模型名称并生成default.yaml
    python generate_mode.py --model Sub2_model_20210419 --gen-default
"""

import argparse
import sys
import subprocess
from pathlib import Path
import matlab.engine


def generate_model_and_config(
    model_name: str = 'Sub2_model_20210419',
    gen_default: bool = False,
    stop_time: float = 0.1
):
    """生成模型代码并自动生成配置文件
    
    参数：
        model_name: Simulink模型名称（不含.slx后缀）
        gen_default: 是否生成default.yaml配置文件
        stop_time: 仿真停止时间（用于default.yaml）
    
    工作流程：
        1. 启动MATLAB引擎
        2. 调用generate_model_code_2019a.m导出模型参数到JSON
        3. 使用config_mapper.py生成auto_params.yaml
        4. 可选：生成default.yaml配置文件
    """
    print(f"=== 开始生成模型配置 ===")
    print(f"模型名称: {model_name}")
    
    # 步骤1: 启动MATLAB引擎并导出模型参数
    print("\n[1/3] 启动MATLAB引擎并导出模型参数...")
    try:
        eng = matlab.engine.start_matlab()
        print("MATLAB引擎已启动")
        
        # 获取simulink文件夹的绝对路径并切换工作目录
        script_dir = Path(__file__).parent.resolve()
        matlab_work_dir = str(script_dir)
        print(f"切换MATLAB工作目录到: {matlab_work_dir}")
        eng.cd(matlab_work_dir, nargout=0)
        
        # 调用修改后的MATLAB函数，传入模型名称
        eng.generate_model_code_2019a(model_name, nargout=0)
        print(f"模型 {model_name} 参数导出完成")
        
        # 关闭MATLAB引擎
        eng.quit()
        print("MATLAB引擎已关闭")
    except Exception as e:
        print(f"错误：MATLAB执行失败 - {e}")
        return False
    
    # 步骤2: 使用config_mapper生成auto_params.yaml
    print("\n[2/3] 生成auto_params.yaml配置文件...")
    try:
        # 获取项目根目录
        script_dir = Path(__file__).parent
        project_root = script_dir.parent
        
        # 定义路径
        json_path = project_root / "simulink" / "export_model_graph" / "model_params.json"
        auto_params_path = project_root / "config" / "auto_params.yaml"
        
        # 检查JSON文件是否存在
        if not json_path.exists():
            print(f"错误：找不到导出的JSON文件: {json_path}")
            return False
        
        # 调用config_mapper生成参数配置
        cmd = [
            sys.executable, "-m", 
            "inverter_ai_control.utils.config_mapper",
            "--json", str(json_path),
            "--out", str(auto_params_path)
        ]
        
        result = subprocess.run(cmd, capture_output=True, text=True)
        
        if result.returncode == 0:
            print(f"auto_params.yaml已生成: {auto_params_path}")
        else:
            print(f"警告：config_mapper执行返回非零状态码")
            if result.stderr:
                print(f"错误输出: {result.stderr}")
    except Exception as e:
        print(f"错误：生成auto_params.yaml失败 - {e}")
        return False
    
    # 步骤3: 可选生成default.yaml
    if gen_default:
        print("\n[3/3] 生成default.yaml配置文件...")
        try:
            default_config_path = project_root / "config" / "default.yaml"
            model_slx_path = f"simulink/{model_name}.slx"
            
            cmd = [
                sys.executable, "-m",
                "inverter_ai_control.utils.config_mapper",
                "--gen-default",
                "--model-path", model_slx_path,
                "--stop-time", str(stop_time),
                "--auto", str(auto_params_path),
                "--config", str(default_config_path)
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True)
            
            if result.returncode == 0:
                print(f"default.yaml已生成: {default_config_path}")
            else:
                print(f"警告：生成default.yaml时出现问题")
                if result.stderr:
                    print(f"错误输出: {result.stderr}")
        except Exception as e:
            print(f"错误：生成default.yaml失败 - {e}")
            return False
    else:
        print("\n[3/3] 跳过default.yaml生成（使用--gen-default启用）")
    
    print("\n=== 配置生成完成 ===")
    return True


def main():
    """命令行入口"""
    parser = argparse.ArgumentParser(
        description="自动生成Simulink模型代码和配置文件",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例：
  # 使用默认模型
  python generate_mode.py
  
  # 指定模型名称
  python generate_mode.py --model Sub2_model_20210419
  
  # 生成完整配置（包括default.yaml）
  python generate_mode.py --model Sub2_model_20210419 --gen-default --stop-time 0.1
        """
    )
    
    parser.add_argument(
        '--model',
        type=str,
        default='Sub2_model_20210419',
        help='Simulink模型名称（不含.slx后缀），默认: Sub2_model_20210419'
    )
    
    parser.add_argument(
        '--gen-default',
        action='store_true',
        help='是否生成default.yaml配置文件'
    )
    
    parser.add_argument(
        '--stop-time',
        type=float,
        default=0.1,
        help='仿真停止时间（秒），默认: 0.1'
    )
    
    args = parser.parse_args()
    
    # 执行生成流程
    success = generate_model_and_config(
        model_name=args.model,
        gen_default=args.gen_default,
        stop_time=args.stop_time
    )
    
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
