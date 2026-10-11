"""Deterministic synthetic cases; no edits to the original sample configuration."""
import argparse
import copy
import hashlib
import itertools
import json
from pathlib import Path
import random

GROUPS = ('order', 'dimensions', 'load', 'containers', 'shelf_opening', 'attributes')
SEEDS = (271601, 271602, 271603, 971604, 971605)


def fits_some_container(item, containers):
    # Necessary individual-fit condition only, not a packing/transport proof.
    dims = [item[k] for k in ('length', 'width', 'height')]
    for c in containers:
        space = (c['length']-2*c['thickness']-.04,
                 c['width']-2*c['thickness']-.04,
                 c['height']-2*c['thickness']-.04)
        if any(all(d < s for d,s in zip(p,space)) for p in itertools.permutations(dims)):
            return True
    return False


def validate(case):
    cs = case['containers']['container_list']
    items = case['item_stream']['item_list']
    assert [c['index'] for c in cs] == list(range(len(cs)))
    assert len({i['index'] for i in items}) == len(items)
    assert items and case['item_stream']['max_space'] <= case['item_stream']['look_ahead']
    for c in cs:
        assert 0 < c['thickness'] < min(c['length'],c['width'],c['height'])/4
        assert 0 <= c['cut_x'] < c['length']-2*c['thickness']
        assert 0 <= c['cut_y'] < c['height']-2*c['thickness']
        assert not c['packed_items']
    assert all(fits_some_container(i,cs) for i in items)


def generate(source):
    pool = source['000']['item_stream']['item_list'] + source['001']['item_stream']['item_list']
    cases, entries = {}, []
    for gi, group in enumerate(GROUPS):
        for si, seed in enumerate(SEEDS):
            rng = random.Random(seed+gi*10000)
            optimize = si % 2 == 0
            template = '000' if optimize else '001'
            c = copy.deepcopy(source[template])
            cs = c['containers']['container_list']
            items = c['item_stream']['item_list']
            if group == 'order':
                rng.shuffle(items)
            elif group == 'dimensions':
                # New sizes include small, flat, tall and elongated bags.
                for item in items:
                    dims = [rng.uniform(.18,.80),rng.uniform(.16,.55),rng.uniform(.10,.48)]
                    if rng.random() < .3:
                        dims[2] = rng.uniform(.08,.16)
                    for key,value in zip(('length','width','height'),dims):
                        item[key] = round(value,4)
            elif group == 'load':
                count = (20,32,48,60,72)[si]
                items[:] = [copy.deepcopy(rng.choice(pool)) for _ in range(count)]
                rng.shuffle(items)
            elif group == 'containers':
                count = (1,2,3,1,2)[si]
                base = copy.deepcopy(cs[0])
                cs[:] = []
                for idx in range(count):
                    box = copy.deepcopy(base)
                    box.update(index=idx,length=round(rng.uniform(1.5,2.5),3),
                               width=round(rng.uniform(1.2,1.9),3),
                               height=round(rng.uniform(1.25,1.95),3))
                    cs.append(box)
                c['containers']['spacing'] = max(b['length'] for b in cs)+.5
                c['camera']['num_containers'] = count
            elif group == 'shelf_opening':
                box = cs[0]
                box['require_shelf'] = si % 2 == 0
                box['cut_x'] = round(box['length']*(.10,.18,.25,.32,.38)[si],3)
                box['cut_y'] = round(box['height']*(.12,.20,.28,.35,.40)[si],3)
                box['thickness'] = (.025,.035,.045,.055,.065)[si]
            elif group == 'attributes':
                for item in items:
                    item['mass'] = round(rng.uniform(3,25),2)
                    item['is_prioritized'] = rng.random() < (.1,.25,.4,.55,.7)[si]
                    item['is_soft'] = rng.random() < (.1,.3,.5,.7,.9)[si]
                    if item['is_soft']:
                        item.update(lateralFriction=.8,rollingFriction=.02,spinningFriction=.02,
                                    restitution=0,contactStiffness=rng.uniform(2000,4000),
                                    contactDamping=800,linearDamping=.8)
                    else:
                        for key in ('contactStiffness','contactDamping','linearDamping'):
                            item.pop(key,None)
            for idx,item in enumerate(items):
                item['index'] = idx
            c['agent'].update(optimize=optimize,init_timeout=10.,optimization_timeout=180.,policy_timeout=8.,max_mem=12)
            c['item_stream'].update(look_ahead=1 if optimize else 10,max_space=1,visible_pool=[])
            c['visualizer']['vis'] = False
            validate(c)
            cid = f'{101+gi*5+si:03d}'
            cases[cid] = c
            volume = sum(i['length']*i['width']*i['height'] for i in items)
            inner = sum((b['length']-2*b['thickness'])*(b['width']-2*b['thickness'])*(b['height']-2*b['thickness']) for b in cs)
            entries.append({'id':cid,'group':group,'split':'development' if si<3 else 'holdout',
                            'seed':seed+gi*10000,'source_template':template,'optimize':optimize,
                            'num_items':len(items),'num_containers':len(cs),
                            'nominal_volume_load':volume/inner,
                            'sha256':hashlib.sha256(json.dumps(c,sort_keys=True).encode()).hexdigest()})
    return cases, {'version':1,'seed_scheme':'group offset + fixed seed; 3 development / 2 holdout per group',
                   'limitations':'Synthetic sensitivity cases, not samples from the hidden contest distribution. Individual-fit validation does not prove transportability or all-item feasibility.',
                   'cases':entries}


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--source',default='configs/sample_config.json')
    p.add_argument('--output-dir',required=True)
    a=p.parse_args()
    source=Path(a.source)
    cases,manifest=generate(json.loads(source.read_text()))
    manifest['source_sha256']=hashlib.sha256(source.read_bytes()).hexdigest()
    out=Path(a.output_dir)
    out.mkdir(parents=True,exist_ok=True)
    for cid,c in cases.items():
        (out/(cid+'.json')).write_text(json.dumps({cid:c},indent=2)+'\n')
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(f'Generated {len(cases)} cases: 18 development / 12 holdout in {out}')


if __name__ == '__main__':
    main()
