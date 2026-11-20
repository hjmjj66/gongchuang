from pathlib import Path
import tempfile
import unittest

from agv_perception.material_config import parse_material_config,read_material_config
from agv_perception.material_model import MaterialModelDetection,MaterialModelError,load_material_model


class ConfigModelTests(unittest.TestCase):
    def test_yaml_and_python_defaults_match(self):
        path=Path(__file__).resolve().parents[1]/'config/material.yaml'
        self.assertEqual(read_material_config(path),parse_material_config({}))

    def test_selectors_are_independent_and_unselected_config_ignored(self):
        for params in ({'material_backend':1,'top.model_path':'','whole.class_labels':[],
                        'confidence_threshold':'unused'},
                       {'material_model':2,'top_circle.hough_dp':'unused','top.model_path':''},
                       {'material_model':1,'whole.model_path':''}):
            parse_material_config(params)
        self.assertIsNone(load_material_model(parse_material_config({'material_backend':1})))

    def test_invalid_config_at_boundary(self):
        for params in ({'material_backend':True},{'material_model':3},{'target_color_id':7},
                       {'top_circle.blur_kernel':4},{'top_circle.hough_param2':float('nan')},
                       {'top.class_labels':['red_top']},{'search_roi':[0,0,1]},
                       {'hsv.blue_ranges':[180,0,0,180,255,255],'material_backend':1},
                       {'camera_fps':0},{'unknown':1}):
            with self.subTest(params=params),self.assertRaises(ValueError):
                parse_material_config(params)

    def test_model_missing_fails_explicitly(self):
        for path,status in (('', 'MODEL_NOT_CONFIGURED'),('/tmp/no_material_model_123.bin','MODEL_LOAD_FAILED')):
            with self.assertRaises(MaterialModelError) as ctx:
                load_material_model(parse_material_config({'top.model_path':path}))
            self.assertEqual(ctx.exception.status,status)

    def test_label_file_mismatch_before_runtime_import(self):
        with tempfile.TemporaryDirectory() as tmp:
            model=Path(tmp)/'model.bin'
            classes=Path(tmp)/'classes.txt'
            model.write_bytes(b'not a real model')
            classes.write_text('red_top\nblue_top\n')
            with self.assertRaisesRegex(MaterialModelError,'顺序不一致'):
                load_material_model(parse_material_config({'top.model_path':str(model),'top.classes_path':str(classes)}))

    def test_external_detection_validation(self):
        for args in ((0,'red',.9,(0,0,20,20)),(1,'red',float('nan'),(0,0,20,20)),
                     (1,'red',.9,(20,20,0,0)),(1,'red',.9,(0,0,float('inf'),20))):
            with self.assertRaises(ValueError):
                MaterialModelDetection(*args)


if __name__ == '__main__':
    unittest.main()
