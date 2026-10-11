"""Run synthetic cases sequentially and keep errors separate from low packing."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import signal
import statistics
import subprocess
import sys
import tempfile
import time

PROJECT=Path(__file__).resolve().parents[1]
PROFILE={'PYTHONDONTWRITEBYTECODE':'1','OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1',
         'EP_CPP_MODE':'cpp','SC_SEARCH_TRIALS':'16','SC_SEARCH_MAX_SECONDS':'60',
         'SC_FEASIBILITY_BEAM':'12','SC_FEASIBILITY_BRANCH':'8'}


def classify(log, result):
    states=result.get('place_states',{})
    if result.get('status')!='success' or result.get('evaluation') is None:
        return 'execution_error'
    if 'SAFE-ABORT' in log:
        return 'no_plan_safe_abort'
    if '[TimedAgentRunner]' in log and re.search(r'\[TimedAgentRunner\].*timeout:',log):
        return 'agent_timeout'
    if states.get('is_included') is False:
        return 'inclusion_rejected'
    if states.get('is_valid') is False:
        return 'transport_rejected'
    if states.get('is_placed_safe') is False:
        return 'settling_rejected'
    return 'all_items_placed' if result['evaluation']['num_placed_items']==1 else 'other_stop'


def summarize(rows):
    groups={}
    for field in ('split','group'):
        groups[field]={}
        for value in sorted({r[field] for r in rows}):
            rs=[r for r in rows if r[field]==value]
            valid=[r for r in rs if r['evaluation_ok']]
            groups[field][value]={
                'cases':len(rs),'evaluation_ok':len(valid),'errors':len(rs)-len(valid),
                'mean_fill_score':statistics.mean(r['fill_score'] for r in valid) if valid else None,
                'mean_placed_ratio':statistics.mean(r['placed_ratio'] for r in valid) if valid else None,
                'minimum_placed_ratio':min((r['placed_ratio'] for r in valid),default=None),
                'complete_cases':sum(r['placed_ratio']==1 for r in valid),
                'stop_reasons':{reason:sum(r['stop_reason']==reason for r in rs) for reason in sorted({r['stop_reason'] for r in rs})}}
    return groups


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--cases-dir',required=True)
    p.add_argument('--output-root',required=True)
    p.add_argument('--split',choices=['all','development','holdout'],default='development')
    p.add_argument('--case-ids',nargs='*')
    p.add_argument('--module-path',default='agents/search_cand_v15_shelf_clearance/')
    p.add_argument('--case-timeout',type=float,default=420)
    a=p.parse_args()
    cases=Path(a.cases_dir).resolve()
    manifest=json.loads((cases/'manifest.json').read_text())
    entries=[e for e in manifest['cases'] if (a.split=='all' or e['split']==a.split) and (not a.case_ids or e['id'] in a.case_ids)]
    if not entries: p.error('No cases selected')
    root=Path(a.output_root).resolve()
    if root==PROJECT/'results' or (PROJECT/'results') in root.parents:
        p.error('Use an external output directory to preserve existing results')
    root.mkdir(parents=True,exist_ok=True)
    output=Path(tempfile.mkdtemp(prefix='v15-robustness-',dir=root))
    env=os.environ.copy();env.update(PROFILE)
    rows=[]
    provenance={'base_commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=PROJECT,text=True).strip(),
                'python':sys.version,'platform':platform.platform(),'module_path':a.module_path,
                'profile':PROFILE,'manifest_sha256':hashlib.sha256((cases/'manifest.json').read_bytes()).hexdigest(),
                'limitations':'Local fill_score and packed ratio only; no hidden contest score. Synthetic holdout is observed once for frozen baseline.',
                'source_hashes':{str(f.relative_to(PROJECT)):hashlib.sha256(f.read_bytes()).hexdigest() for f in sorted((PROJECT/'scripts').glob('*robustness*.py'))}}
    print('output:',output,flush=True)
    for entry in entries:
        cid=entry['id'];out=output/cid;out.mkdir()
        config=json.loads((cases/(cid+'.json')).read_text())[cid]
        actual_hash=hashlib.sha256(json.dumps(config,sort_keys=True).encode()).hexdigest()
        if actual_hash!=entry['sha256']:raise RuntimeError(f'Case hash mismatch: {cid}')
        cmd=[sys.executable,str(PROJECT/'scripts/run_robustness_case.py'),
             '--config-path',str(cases/(cid+'.json')),'--module-path',a.module_path,
             '--result-dir',str(out),'--result-fname','evaluation.json','--verbose','True']
        start=time.perf_counter();timed_out=False
        with (out/'run.log').open('w') as log:
            proc=subprocess.Popen(cmd,cwd=PROJECT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:code=proc.wait(timeout=a.case_timeout)
            except subprocess.TimeoutExpired:
                timed_out=True;os.killpg(proc.pid,signal.SIGKILL);proc.wait();code=-9
        wall=time.perf_counter()-start
        log=(out/'run.log').read_text(errors='replace')
        result={}
        if (out/'evaluation.json').exists():
            result=json.loads((out/'evaluation.json').read_text()).get(cid,{})
        evaluation=result.get('evaluation') or {}
        history=[json.loads(line) for line in (out/'plan_history.jsonl').read_text().splitlines()] if (out/'plan_history.jsonl').exists() else []
        safe_union=set().union(*(set(h['safe_ids']) for h in history)) if history else set()
        initial=next((h for h in history if h['stage']=='optimize'),None)
        counts=json.loads((out/'actual_counts.json').read_text()) if (out/'actual_counts.json').exists() else {}
        packed=sum(map(len,counts.values())) if counts else None
        ok=code==0 and result.get('status')=='success' and bool(evaluation)
        row={**entry,'evaluation_ok':ok,'exit_code':code,'wall_seconds':wall,
             'fill_score':evaluation.get('fill_score'),'placed_ratio':evaluation.get('num_placed_items'),
             'actual_packed_count':packed,'initial_offline_plan_count':len(initial['safe_ids']) if initial else None,
             'planned_union_count':len(safe_union),'stop_reason':'case_timeout' if timed_out else classify(log,result),
             'optimization_seconds':result.get('time_results',{}).get('optimization'),
             'policy_seconds':result.get('time_results',{}).get('policy'),
             'timeout_messages':[line for line in log.splitlines() if '[TimedAgentRunner]' in line and 'timeout:' in line],
             'safe_abort_messages':[line for line in log.splitlines() if 'SAFE-ABORT' in line],
             'collision_messages':[line for line in log.splitlines() if 'collision:' in line],
             'placed_but_unplanned_ids':sorted(set().union(*(set(v) for v in counts.values()))-safe_union) if counts else []}
        rows.append(row)
        report={'provenance':provenance,'summary':summarize(rows),'cases':rows}
        (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        print(f'{cid} {entry["group"]} {entry["split"]}: packed={packed}/{entry["num_items"]} fill={row["fill_score"]} stop={row["stop_reason"]} ({wall:.1f}s)',flush=True)
    fields=['id','group','split','optimize','num_items','num_containers','nominal_volume_load','evaluation_ok','fill_score','placed_ratio','actual_packed_count','initial_offline_plan_count','planned_union_count','stop_reason','wall_seconds','optimization_seconds','policy_seconds']
    with (output/'cases.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');writer.writeheader();writer.writerows(rows)
    print('report:',output/'report.json',flush=True)
    raise SystemExit(0 if all(r['evaluation_ok'] for r in rows) else 1)


if __name__=='__main__':
    main()
