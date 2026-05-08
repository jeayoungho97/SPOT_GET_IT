import os
import math
from typing import List, Optional

import numpy as np


class PolicyRunner:
    """
    RL policy inference wrapper.

    backend:
      - dummy: action 0 반환
      - onnx: onnxruntime으로 policy.onnx 실행

    이 파일의 목적:
      rl_locomotion_node가 ONNX/TensorRT/TorchScript 세부 구현을 몰라도 되게 한다.
    """

    def __init__(
        self,
        backend: str,
        model_path: str,
        obs_dim: int,
        action_dim: int,
        obs_clip: float = 100.0,
        require_model: bool = False,
    ):
        self.backend = backend.lower().strip()
        self.model_path = model_path
        self.obs_dim = obs_dim
        self.action_dim = action_dim
        self.obs_clip = obs_clip
        self.require_model = require_model

        self.session = None
        self.input_name: Optional[str] = None
        self.output_name: Optional[str] = None

        if self.backend == "dummy":
            return

        if self.backend == "onnx":
            self._load_onnx()
            return

        raise ValueError(f"Unsupported policy backend: {backend}")

    def _load_onnx(self):
        if not self.model_path:
            if self.require_model:
                raise FileNotFoundError("model_path is empty but require_model=true")
            self.backend = "dummy"
            return

        if not os.path.exists(self.model_path):
            if self.require_model:
                raise FileNotFoundError(f"Policy model not found: {self.model_path}")
            self.backend = "dummy"
            return

        try:
            import onnxruntime as ort
        except ImportError as exc:
            raise ImportError(
                "onnxruntime is not installed. "
                "Install it or set policy_backend=dummy."
            ) from exc

        self.session = ort.InferenceSession(
            self.model_path,
            providers=["CPUExecutionProvider"],
        )

        inputs = self.session.get_inputs()
        outputs = self.session.get_outputs()

        if len(inputs) < 1:
            raise RuntimeError("ONNX model has no inputs")
        if len(outputs) < 1:
            raise RuntimeError("ONNX model has no outputs")

        self.input_name = inputs[0].name
        self.output_name = outputs[0].name

    def infer(self, obs: List[float]) -> List[float]:
        if len(obs) != self.obs_dim:
            raise ValueError(f"obs length mismatch: {len(obs)} != {self.obs_dim}")

        obs_np = np.asarray(obs, dtype=np.float32)

        if not np.all(np.isfinite(obs_np)):
            raise ValueError("obs contains NaN or Inf")

        obs_np = np.clip(obs_np, -self.obs_clip, self.obs_clip)

        if self.backend == "dummy":
            return [0.0] * self.action_dim

        if self.backend == "onnx":
            return self._infer_onnx(obs_np)

        raise RuntimeError(f"Invalid policy backend state: {self.backend}")

    def _infer_onnx(self, obs_np: np.ndarray) -> List[float]:
        if self.session is None:
            raise RuntimeError("ONNX session is not loaded")

        input_tensor = obs_np.reshape(1, self.obs_dim)

        outputs = self.session.run(
            [self.output_name],
            {self.input_name: input_tensor},
        )

        action = np.asarray(outputs[0], dtype=np.float32).reshape(-1)

        if action.shape[0] != self.action_dim:
            raise RuntimeError(
                f"action dim mismatch: {action.shape[0]} != {self.action_dim}"
            )

        if not np.all(np.isfinite(action)):
            raise ValueError("policy output contains NaN or Inf")

        return action.tolist()

    def backend_name(self) -> str:
        return self.backend
