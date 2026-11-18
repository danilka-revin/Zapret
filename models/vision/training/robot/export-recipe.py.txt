#!/usr/bin/env python3
"""Export a frozen bench profile to dynamic-shape FP32 ONNX and verify parity.

The graph is the raw eight-class detector, NOT HSV fusion/body checking/NMS.
Full-pipeline parity is additionally checked through the existing YoloDetector
on every VAL image. No TEST data is used; production weights are never changed.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys
import tempfile
from datetime import datetime, timezone

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from robot_control.vision.detector import Detection, iou


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def compare_detections(pt: list[Detection], onnx: list[Detection]) -> dict:
    """One-to-one parity, including colour/unknown; confidence is NOT accuracy."""
    remaining = set(range(len(onnx)))
    issues, overlaps, confidence_errors = [], [], []
    for det in pt:
        candidates = [i for i in remaining if onnx[i].cls == det.cls and onnx[i].color == det.color]
        best = max(candidates, key=lambda i: iou(det, onnx[i])) if candidates else None
        overlap = iou(det, onnx[best]) if best is not None else 0.
        if best is None or overlap < .999:
            issues.append({'reason':'missing or changed class/signal/box','pt':det.to_dict()})
            continue
        remaining.remove(best)
        overlaps.append(overlap)
        error = abs(det.confidence-onnx[best].confidence)
        confidence_errors.append(error)
        if error > 1e-4:
            issues.append({'reason':'confidence drift','absoluteError':error})
    for index in sorted(remaining):
        issues.append({'reason':'extra ONNX detection','onnx':onnx[index].to_dict()})
    return {'passed':not issues,'issues':issues,'matched':len(overlaps),
            'minimumIoU':min(overlaps,default=None),
            'maximumConfidenceError':max(confidence_errors,default=0.)}


def export(profile_file: Path, output: Path, report_file: Path, data: Path) -> dict:
    import cv2
    import numpy as np
    import onnx
    import onnxruntime as ort
    import torch
    from ultralytics import YOLO
    from ultralytics.data.augment import LetterBox
    from robot_control.vision.classes import TRAINING_CLASS_NAMES
    from robot_control.vision.detector import YoloDetector
    from scripts.train_vision import inspect_dataset
    torch.set_num_threads(2); cv2.setNumThreads(1)
    profile = json.loads(profile_file.read_text())
    weights = Path(profile['weights'])
    if not weights.is_absolute():
        weights = ROOT/weights
    if sha256(weights) != profile['sha256']:
        raise ValueError('frozen profile/weights SHA mismatch')
    if output.suffix != '.onnx' or output.resolve() == weights.resolve():
        raise ValueError('output must be a separate .onnx file')
    config,audit=inspect_dataset(data.resolve())
    if audit['fingerprint'] != profile['datasetFingerprint']:
        raise ValueError('profile/evaluation dataset changed')
    if profile.get('heuristicEnabled') is not False:
        raise ValueError('parity requires the frozen no-heuristic profile')
    images=sorted((Path(config['path'])/config['val']).glob('*.jpg'))
    if not images:
        raise ValueError('no VAL parity images')
    cache=ROOT/'.cache/vision/robot';cache.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='export-',dir=cache) as temporary:
        copied=Path(temporary)/'model.pt';shutil.copy2(weights,copied)
        model=YOLO(str(copied))
        if tuple(model.names.values()) != TRAINING_CLASS_NAMES:
            raise ValueError('expected the eight learned colour classes')
        graph=Path(model.export(format='onnx',imgsz=profile['imgsz'],batch=1,
                               dynamic=True,simplify=False,opset=17,half=False,
                               device='cpu',nms=False))
        onnx.checker.check_model(str(graph))
        options=ort.SessionOptions();options.intra_op_num_threads=2;options.inter_op_num_threads=1
        session=ort.InferenceSession(str(graph),sess_options=options,providers=['CPUExecutionProvider'])
        input_meta=session.get_inputs()[0]
        if not isinstance(input_meta.shape[2],str) or not isinstance(input_meta.shape[3],str):
            raise RuntimeError('export is not dynamic in height/width')
        # Fused PT eval and ONNX must agree BEFORE NMS at several shapes.
        reference=YOLO(str(copied)).model.cpu().float().eval().fuse(verbose=False)
        raw=[];inputs=[]
        rng=np.random.default_rng(17)
        side=profile['imgsz'];short=max(160,(side*5//8)//32*32)
        for height,width in ((side,side),(short,side),(side,short)):
            inputs.append((f'synthetic-{height}x{width}',rng.random((1,3,height,width),dtype=np.float32)))
        for path in images[::max(1,len(images)//8)][:8]:
            frame=cv2.imread(str(path))
            image=LetterBox(profile['imgsz'],auto=True,stride=32)(image=frame)
            image=np.ascontiguousarray(image[:,:,::-1].transpose(2,0,1)[None],dtype=np.float32)/255.
            inputs.append((path.name,image))
        output_metadata=[{'name':o.name,'shape':o.shape,'dtype':o.type} for o in session.get_outputs()]
        for name,image in inputs:
            with torch.inference_mode():
                expected=reference(torch.from_numpy(image))[0].cpu().numpy()
            actual=session.run(None,{input_meta.name:image})[0]
            passed=bool(np.allclose(expected,actual,rtol=1e-4,atol=2e-3))
            raw.append({'image':name,'inputShape':list(image.shape),'outputShape':list(actual.shape),
                        'maximumAbsoluteError':float(np.abs(expected-actual).max()),'passed':passed})
        del session,reference,model
        # Verify the actual robot adapter, not just a raw model tensor.
        kwargs=dict(confidence=profile['confidence'],imgsz=profile['imgsz'],device='cpu',
                    prefer_red=profile['preferRed'],traffic_mode=profile['trafficMode'])
        pt_detector=YoloDetector(str(weights),**kwargs)
        onnx_detector=YoloDetector(str(graph),**kwargs)
        rows=[]
        for path in images:
            frame=cv2.imread(str(path))
            result=compare_detections(pt_detector.detect(frame),onnx_detector.detect(frame))
            rows.append({'image':path.name,**result})
        passed=all(r['passed'] for r in raw+rows)
        report={'createdAt':datetime.now(timezone.utc).isoformat(),'weights':str(weights.relative_to(ROOT)),
                'weightsSha256':sha256(weights),'profileSha256':sha256(profile_file),
                'onnx':str(output),'onnxSha256':sha256(graph),'onnxBytes':graph.stat().st_size,
                'format':'ONNX FP32, opset 17, dynamic batch/height/width, no embedded NMS',
                'input':{'name':input_meta.name,'layout':'NCHW','dtype':'float32',
                         'preprocessing':'BGR -> RGB, stride-32 LetterBox, scale /255, batch dimension'},
                'outputs':output_metadata,
                'classes':list(TRAINING_CLASS_NAMES),'onnxVersion':onnx.__version__,'onnxRuntime':ort.__version__,
                'datasetFingerprint':audit['fingerprint'],'split':'val','images':len(rows),
                'rawTensorChecks':raw,'runtimeChecks':rows,'passed':passed,
                'runtimeProfile':{k:profile[k] for k in ('confidence','imgsz','preferRed','trafficMode','heuristicEnabled')},
                'warning':'Numerical/adapter parity, NOT independent accuracy or safety validation. Raw ONNX does not include colour fusion, body guard, NMS or temporal logic. Robot hardware not benchmarked. No TEST-based tuning.'}
        report_file.parent.mkdir(parents=True,exist_ok=True)
        report_file.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        if not passed:
            raise RuntimeError(f'ONNX parity failed; export not published. See {report_file}')
        output.parent.mkdir(parents=True,exist_ok=True)
        stage=output.with_suffix('.onnx.tmp');shutil.copy2(graph,stage);stage.replace(output)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile',type=Path,default=ROOT/'models/vision/yolo-stop-robot-profile.json')
    parser.add_argument('--output',type=Path,default=ROOT/'models/vision/yolo-stop-robot.onnx')
    parser.add_argument('--report',type=Path,default=ROOT/'models/vision/training/robot/onnx-parity.json')
    parser.add_argument('--data',type=Path,default=ROOT/'datasets/yolo-stop/data.yaml')
    args=parser.parse_args()
    result=export(args.profile,args.output,args.report,args.data)
    print(json.dumps({k:v for k,v in result.items() if k not in ('rawTensorChecks','runtimeChecks')},ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
