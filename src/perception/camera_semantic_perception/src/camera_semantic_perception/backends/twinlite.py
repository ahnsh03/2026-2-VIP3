"""TwinLiteNet+ implementation of the model-independent backend contract."""

from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Mapping, Optional, Tuple, Union

import numpy as np

from ..backend import BackendInfo, CategoricalOutput, SemanticBackend


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


class TwinLiteBackend(SemanticBackend):
    def __init__(
        self,
        checkpoint: str,
        model_config: str = "medium",
        twinlite_root: Optional[str] = None,
        device: str = "cuda",
        weights: str = "auto",
        amp: bool = True,
        channels_last: bool = False,
        input_hw: Tuple[int, int] = (384, 640),
    ) -> None:
        import torch

        from twinlite_morai import (
            adapt_twinlite_outputs,
            configure_twinlite_task,
            load_external_twinlite,
            task_from_checkpoint,
        )

        if weights not in ("auto", "ema", "model"):
            raise ValueError("weights must be auto, ema, or model")
        self._torch = torch
        self._adapt_outputs = adapt_twinlite_outputs
        self._device = torch.device(device)
        if self._device.type == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("CUDA was requested but is unavailable")

        checkpoint_path = Path(checkpoint).expanduser().resolve()
        if not checkpoint_path.is_file():
            raise RuntimeError("checkpoint not found: {}".format(checkpoint_path))
        state = torch.load(str(checkpoint_path), map_location="cpu")
        supported_schemas = {
            "twinlite-morai-checkpoint-1.0.0",
            "twinlite-morai-checkpoint-1.1.0",
        }
        if state.get("schema_version") not in supported_schemas:
            raise RuntimeError(
                "unsupported checkpoint schema: {}".format(
                    state.get("schema_version")
                )
            )
        use_ema = weights == "ema" or (
            weights == "auto" and state.get("ema_state_dict") is not None
        )
        state_key = "ema_state_dict" if use_ema else "model_state_dict"
        if state.get(state_key) is None:
            raise RuntimeError("checkpoint has no {}".format(state_key))

        model, upstream_commit = load_external_twinlite(
            config=model_config, root=twinlite_root
        )
        task_spec = task_from_checkpoint(state)
        configure_twinlite_task(model, task_spec)
        if state.get("upstream_commit") != upstream_commit:
            raise RuntimeError("checkpoint and TwinLite source commits differ")
        checkpoint_config = (state.get("args") or {}).get("config")
        if checkpoint_config and checkpoint_config != model_config:
            raise RuntimeError(
                "checkpoint model config is {}, requested {}".format(
                    checkpoint_config, model_config
                )
            )
        model.load_state_dict(state[state_key])
        self._model = model.to(self._device).eval()
        self._channels_last = bool(channels_last)
        if self._channels_last:
            self._model = self._model.to(memory_format=torch.channels_last)
        self._amp = bool(amp) and self._device.type == "cuda"
        self._secondary_head = task_spec.secondary_head
        categorical_classes = {}
        if self._secondary_head == "road_marking":
            categorical_classes[self._secondary_head] = tuple(
                task_spec.class_names[self._secondary_head]
            )
        self._info = BackendInfo(
            name="twinlite_plus",
            model_config=str(model_config),
            heads=task_spec.heads,
            input_hw=(int(input_hw[0]), int(input_hw[1])),
            device=str(self._device),
            checkpoint_path=str(checkpoint_path),
            checkpoint_sha256=_sha256(checkpoint_path),
            checkpoint_epoch=int(state.get("epoch", -1)),
            weights="ema" if use_ema else "model",
            upstream_commit=str(upstream_commit),
            categorical_classes=categorical_classes,
        )

    @property
    def info(self) -> BackendInfo:
        return self._info

    def infer(
        self, batch: np.ndarray
    ) -> Mapping[str, Union[np.ndarray, CategoricalOutput]]:
        torch = self._torch
        if batch.ndim != 4 or batch.shape[1:] != (
            3,
            self.info.input_hw[0],
            self.info.input_hw[1],
        ):
            raise ValueError("invalid TwinLite input shape: {}".format(batch.shape))
        tensor = torch.from_numpy(np.ascontiguousarray(batch)).to(
            self._device, non_blocking=True
        )
        if self._channels_last:
            tensor = tensor.contiguous(memory_format=torch.channels_last)
        with torch.no_grad():
            with torch.cuda.amp.autocast(enabled=self._amp):
                logits_by_head = self._adapt_outputs(
                    self._model(tensor), self._secondary_head
                )
            probabilities = {}
            for head, logits in logits_by_head.items():
                if head in self.info.categorical_classes:
                    softmax = torch.softmax(logits.float(), dim=1)
                    confidence, class_ids = torch.max(softmax, dim=1)
                    probabilities[head] = CategoricalOutput(
                        class_ids=class_ids.to(torch.uint8).detach().cpu().numpy(),
                        confidence=confidence.detach().cpu().numpy().astype(
                            np.float32, copy=False
                        ),
                    )
                    continue
                elif logits.shape[1] == 2:
                    value = torch.softmax(logits.float(), dim=1)[:, 1]
                else:
                    value = torch.sigmoid(logits[:, 0].float())
                probabilities[head] = (
                    value.detach().cpu().numpy().astype(np.float32, copy=False)
                )
        return probabilities
