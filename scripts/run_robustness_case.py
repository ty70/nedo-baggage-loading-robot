"""One isolated baseline evaluation; instrument outputs, not agent decisions."""
import json
import os
from pathlib import Path
import sys
import time

PROJECT = Path(__file__).resolve().parents[1]
STATE = Path(os.environ.get('NEDO_CLOUD_STATE','/workspace/nedo-cloud'))
sys.path.insert(0,str(PROJECT))
sys.path.insert(0,str(STATE/'checker'))
import cpp_aabb_checker
if Path(cpp_aabb_checker.__file__).resolve().parent != (STATE/'checker').resolve():
    raise RuntimeError('Run cloud_setup.sh before this benchmark')
# This top-level instrumentation also runs in the simulator's spawn worker.
from agents.search_cand_v14_order_2716_diagnostic.agent import Agent as Baseline
_original_dump = Baseline._dump_bpp_layout


def capture_plan(self, containers, stage='update'):
    _original_dump(self,containers,stage)
    output=os.environ.get('NEDO_BENCH_CASE_OUTPUT')
    if output:
        record={'stage':stage,'safe_ids':sorted(map(int,self.preplanned)),
                'geometric_ids':sorted(map(int,self.geometric_preplanned))}
        with (Path(output)/'plan_history.jsonl').open('a') as f:
            f.write(json.dumps(record)+'\n')


Baseline._dump_bpp_layout = capture_plan


def main():
    from scripts.run_test import main as evaluate, parse_args
    args=parse_args()
    out=Path(args.result_dir).resolve()
    out.mkdir(parents=True,exist_ok=True)
    os.environ['NEDO_BENCH_CASE_OUTPUT']=str(out)
    os.environ['EP_NEED_3D_BPP']=str(out/'bpp_layout.json')
    expected=set(json.loads(Path(args.config_path).read_text()))
    from src.ground_handling.env import GroundHandlingEnv
    original=GroundHandlingEnv.evaluate

    def capture_actual(self):
        result=original(self)
        packed={str(c.index):[int(i.index) for i in c.packed_items]
                for c in self.container_manager.containers}
        (out/'actual_counts.json').write_text(json.dumps(packed,indent=2)+'\n')
        return result

    GroundHandlingEnv.evaluate=capture_actual
    start=time.perf_counter()
    evaluate()
    seconds=time.perf_counter()-start
    results=json.loads((out/args.result_fname).read_text())
    errors=[k for k,v in results.items() if v.get('status')!='success' or v.get('evaluation') is None]
    missing=sorted(expected-set(results))
    code=1 if errors or missing else 0
    (out/'runtime.json').write_text(json.dumps({'wall_seconds':seconds,'exit_code':code,
        'error_tasks':errors,'missing_tasks':missing,'checker':cpp_aabb_checker.__file__},indent=2)+'\n')
    raise SystemExit(code)


if __name__ == '__main__':
    main()
