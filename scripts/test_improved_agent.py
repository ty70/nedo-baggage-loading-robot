#!/usr/bin/env python3
import sys
import os

# Add the src directory and the simulator root to the path so we can import the evaluation modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from ground_handling.app import EvaluationApp
from argparse import ArgumentParser

def parse_args():
    parser = ArgumentParser()
    parser.add_argument('--config-path', default='configs/sample_config.json', help='config path', type=str)
    parser.add_argument('--module-path', default='agents/improved_grid/', help='agent module path', type=str)
    parser.add_argument('--result-dir', default = 'results/', help = 'result dir', type=str)
    parser.add_argument('--result-fname', default = 'test_improved_results.json', help = 'result fname', type=str)
    parser.add_argument('--render-mode', default = None, help = 'render mode(human or None)', type=str)
    parser.add_argument('--verbose', default = False, help = 'verbose', type=bool)

    return parser.parse_args()

def main():
    args = parse_args()
    module_path = args.module_path # 相対パス
    # Convert module path like 'agents/improved_grid/' to module name like 'agents.improved_grid.agent'
    agent_module_path = '.'.join(module_path.split('/')) + 'agent'
    config_path = args.config_path
    result_dir = args.result_dir
    result_fname = args.result_fname

    print(f"Module path: {module_path}")
    print(f"Agent module path: {agent_module_path}")
    print(f"Config path: {config_path}")
    print(f"Result dir: {result_dir}")
    print(f"Result fname: {result_fname}")

    runner = EvaluationApp(config_path = config_path,
                           module_path = module_path,
                           agent_module = agent_module_path,
                           agent_class = 'Agent',
                           result_dir = result_dir,
                           result_fname = result_fname)
    runner.run(render_mode=args.render_mode, verbose=args.verbose)

if __name__ == '__main__':
    main()