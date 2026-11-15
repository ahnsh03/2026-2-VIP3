"""Thin adapter for the external gcn-parking-slot inference model."""
from pathlib import Path
import sys

import cv2
import numpy as np


def load_model(repository, config_file, checkpoint, device="auto"):
    """Load GCN lazily so importing vip3_perception never imports torch."""
    repository = Path(repository).resolve()
    if not (repository / "psdet" / "models" / "builder.py").is_file():
        raise FileNotFoundError("GCN repository is missing: {}".format(repository))
    if not checkpoint or not Path(checkpoint).is_file():
        raise FileNotFoundError("Set ~checkpoint to a downloaded GCN checkpoint")
    if str(repository) not in sys.path:
        sys.path.insert(0, str(repository))

    import torch
    from psdet.utils.config import cfg_from_file
    from psdet.models.builder import build_model

    config_path = Path(config_file)
    if not config_path.is_absolute():
        config_path = repository / "config" / config_path
    cfg = cfg_from_file(str(config_path))
    if device == "auto":
        device = "cuda" if torch.cuda.is_available() else "cpu"
    target = torch.device(device)
    model = build_model(cfg.model)
    model.load_params_from_file(str(checkpoint), to_cpu=(target.type == "cpu"))
    model.to(target).eval()
    return model, torch, target


def infer(model, torch, device, image, input_size=512):
    """Run one BGR BEV image and return the upstream slot predictions."""
    if image is None or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("expected a decoded three-channel BEV image")
    resized = cv2.resize(image, (input_size, input_size), interpolation=cv2.INTER_LINEAR)
    tensor = torch.from_numpy(resized.astype(np.float32) / 255.0)
    tensor = tensor.permute(2, 0, 1).unsqueeze(0).to(device)
    with torch.no_grad():
        predictions, _ = model({"image": tensor})
    return resized, predictions["slots_pred"][0]


def slot_endpoints(slots, input_size=512):
    """Convert GCN's normalized [x0,y0,x1,y1] endpoints to input pixels."""
    result = []
    for slot in slots:
        values = slot[1].detach().cpu().numpy() if hasattr(slot[1], "detach") else np.asarray(slot[1])
        if values.size != 4 or not np.isfinite(values).all():
            continue
        x0, y0, x1, y1 = (float(v) * input_size - 0.5 for v in values)
        result.append([[x0, y0], [x1, y1]])
    return result


def draw_slots(image, endpoints):
    """Draw predicted slot separators on a copy of the resized input."""
    out = image.copy()
    for (x0, y0), (x1, y1) in endpoints:
        dx, dy = x1 - x0, y1 - y0
        norm = float(np.hypot(dx, dy))
        if norm < 1e-6:
            continue
        length = 0.199519231
        # GCN's own demo uses the long separator for its normalized vertical-slot interval.
        dist = ((x0-x1)/image.shape[1])**2 + ((y0-y1)/image.shape[0])**2
        if 0.044771278151623496 <= dist <= 0.1099427457599304:
            length = 0.46875
        vx, vy = dx / norm, dy / norm
        p0, p1 = (int(round(x0)), int(round(y0))), (int(round(x1)), int(round(y1)))
        p2 = (int(round(x0 + image.shape[0] * length * vy)),
              int(round(y0 - image.shape[1] * length * vx)))
        p3 = (int(round(x1 + image.shape[0] * length * vy)),
              int(round(y1 - image.shape[1] * length * vx)))
        for a, b in ((p0, p1), (p0, p2), (p1, p3)):
            cv2.line(out, a, b, (255, 0, 0), 2)
        cv2.circle(out, p0, 3, (0, 0, 255), 4)
        cv2.circle(out, p1, 3, (0, 0, 255), 4)
    return out
