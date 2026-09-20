#!/usr/bin/env python3
"""Native C++ TensorRT vs ONNX Runtime CPU. Synthetic normalized inputs, no ROS."""
import argparse
import subprocess
import tempfile
from pathlib import Path
import numpy as np
import onnxruntime as ort
p=argparse.ArgumentParser()
p.add_argument('--probe',required=True);p.add_argument('--engine',required=True);p.add_argument('--onnx',required=True)
a=p.parse_args()
options=ort.SessionOptions();options.intra_op_num_threads=1;options.inter_op_num_threads=1
session=ort.InferenceSession(a.onnx,sess_options=options,providers=['CPUExecutionProvider'])
rng=np.random.default_rng(1928)
with tempfile.TemporaryDirectory(prefix='minco_engine_') as temp:
    for i in range(3):
        depth=rng.uniform(0.05,1,(1,1,96,160)).astype('float32');obs=rng.uniform(-.5,.5,(1,9,3,5)).astype('float32')
        if i==0:depth.fill(1);obs.fill(0)
        input_file=Path(temp)/'input.bin';output_file=Path(temp)/'output.bin'
        np.r_[depth.ravel(),obs.ravel()].astype('<f4').tofile(input_file)
        subprocess.run([a.probe,a.engine,str(input_file),str(output_file)],check=True)
        outputs=np.fromfile(output_file,dtype='<f4');assert outputs.size==525
        expected=session.run(['endstate','score','radius'],{'depth':depth,'obs':obs})
        offset=0
        for name,ref in zip(['endstate','score','radius'],expected):
            actual=outputs[offset:offset+ref.size].reshape(ref.shape);offset+=ref.size
            np.testing.assert_allclose(actual,ref,rtol=1e-4,atol=1e-5)
            print(f'case={i} output={name} max_abs_error={np.max(np.abs(actual-ref)):.8g}')
