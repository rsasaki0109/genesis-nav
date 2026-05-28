"""Humanoid state fields used by safety envelopes."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class HumanoidNavState:
    pelvis_frame: str
    base_frame: str
    left_foot_frame: str
    right_foot_frame: str
    fall_detected: bool = False
    balance_margin: float = 0.0
