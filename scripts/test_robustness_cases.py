"""Check suite validity and diagnostic classification independently of physics."""
import copy
import json
from pathlib import Path
import unittest
from scripts.generate_robustness_cases import generate, fits_some_container
from scripts.benchmark_robustness import classify, summarize


class RobustnessTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source=json.loads(Path('configs/sample_config.json').read_text())
        cls.cases,cls.manifest=generate(cls.source)

    def test_reproducibility_and_no_mutation(self):
        before=copy.deepcopy(self.source)
        self.assertEqual((self.cases,self.manifest),generate(self.source))
        self.assertEqual(before,self.source)

    def test_case_boundaries_and_splits(self):
        self.assertEqual(len(self.cases),30)
        self.assertTrue(set(self.cases).isdisjoint(self.source))
        entries=self.manifest['cases']
        self.assertEqual(sum(e['split']=='holdout' for e in entries),12)
        self.assertEqual(len({e['sha256'] for e in entries}),30)
        for c in self.cases.values():
            self.assertEqual(c['validator'],self.source['000']['validator'])
            self.assertEqual(c['agent']['policy_timeout'],8.)
            self.assertTrue(all(fits_some_container(i,c['containers']['container_list']) for i in c['item_stream']['item_list']))

    def test_each_group_changes_its_intended_axis(self):
        for entry in self.manifest['cases']:
            c=self.cases[entry['id']];base=self.source[entry['source_template']]
            group=entry['group']
            if group in ('containers','shelf_opening'):
                self.assertNotEqual(c['containers'],base['containers'])
            elif group=='load':
                self.assertNotEqual(len(c['item_stream']['item_list']),len(base['item_stream']['item_list']))
            else:
                self.assertNotEqual(c['item_stream']['item_list'],base['item_stream']['item_list'])

    def test_error_not_counted_as_packing_stop(self):
        ok={'status':'success','evaluation':{'num_placed_items':.5},'place_states':{'is_included':True,'is_valid':False,'is_placed_safe':False}}
        self.assertEqual(classify('',ok),'transport_rejected')
        self.assertEqual(classify('SAFE-ABORT',ok),'no_plan_safe_abort')
        self.assertEqual(classify('SAFE-ABORT',{'status':'format_error'}),'execution_error')
        rows=[dict(group='order',split='development',evaluation_ok=True,fill_score=10.,placed_ratio=.5,stop_reason='transport_rejected'),
              dict(group='order',split='development',evaluation_ok=False,fill_score=None,placed_ratio=None,stop_reason='execution_error')]
        report=summarize(rows)['group']['order']
        self.assertEqual(report['errors'],1)
        self.assertEqual(report['mean_fill_score'],10.)


if __name__=='__main__':
    unittest.main()
