"""Publication race regression; these synthetic inputs prove no physical result."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

source = Path(__file__).resolve().parents[1] / 'rosclaw/evidence_io.py'
spec = importlib.util.spec_from_file_location('evidence_io', source)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class AtomicEvidenceTests(unittest.TestCase):
    def test_reader_sees_only_complete_old_or_new_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            target=Path(directory)/'visit.json'
            old={'status':'OLD'}
            new={'status':'PASS','trajectory':list(range(30000))}
            module.write_json_atomic(target,old)
            actual_replace=module.os.replace
            def before_replace(temporary, final):
                self.assertEqual(json.loads(target.read_text()),old)
                self.assertEqual(json.loads(Path(temporary).read_text()),new)
                self.assertEqual(list(Path(directory).glob('*.json')),[target])
                actual_replace(temporary,final)
            with patch.object(module.os,'replace',side_effect=before_replace):
                module.write_json_atomic(target,new)
            self.assertEqual(json.loads(target.read_text()),new)
            self.assertEqual(list(Path(directory).iterdir()),[target])

    def test_failed_replace_preserves_prior_evidence_and_cleans_temp(self):
        with tempfile.TemporaryDirectory() as directory:
            target=Path(directory)/'visit.json'
            module.write_json_atomic(target,{'status':'OLD'})
            with patch.object(module.os,'replace',side_effect=OSError('disk failure')):
                with self.assertRaises(OSError):
                    module.write_json_atomic(target,{'status':'PASS'})
            self.assertEqual(json.loads(target.read_text()),{'status':'OLD'})
            self.assertEqual(list(Path(directory).iterdir()),[target])

    def test_nonfinite_json_cannot_publish_or_replace_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            target=Path(directory)/'visit.json'
            with self.assertRaises(ValueError):
                module.write_json_atomic(target,{'speed':float('nan')})
            self.assertFalse(target.exists())
            self.assertEqual(list(Path(directory).iterdir()),[])


if __name__=='__main__':
    unittest.main()
