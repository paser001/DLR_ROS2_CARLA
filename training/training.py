#!/usr/bin/env python3

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Tuple

import torch
import torch.nn as nn
import torch.nn.functional as F
import timm
import csv
import json




DATA_PATH = '/data/carla_datasets'

"""
state vector:
    [
        speed,
        applied_steer,
        goal_rel_x,
        goal_rel_y,
        goal_yaw_error,
        laser1_distance,
        laser2_distance,
        laser3_distance,
        laser4_distance,
    ]
"""




@dataclass
class ModelConfig:
    image_backbone: str = "resnet34"
    image_pretrained: bool = True

    image_feat_dim: int = 256
    lidar_feat_dim: int = 128
    state_feat_dim: int = 64

    state_dim: int = 9

    fusion_hidden_dim: int = 256
    head_hidden_dim: int = 128

    lidar_bev_channels: int = 64

    output_dim: int = 2



class ResNetImageEncoder(nn.Module):
    def __init__(self, model_name: str, pretrained: bool, out_dim: int) -> None:
        super().__init__()

        self.backbone = timm.create_model(
            model_name,
            pretrained=pretrained,
            num_classes=0,
            global_pool="avg",
        )
        in_dim = self.backbone.num_features  
        # print("resnet_dim ", in_dim) # 512 for resnet34

        self.proj = nn.Sequential(
            nn.Linear(in_dim, out_dim),
            nn.ReLU(inplace=True),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        x: (B, 3, H, W)
        returns: (B, out_dim)
        """
        feat = self.backbone(x)
        feat = self.proj(feat)
        return feat


class StateEncoder(nn.Module):
    def __init__(self, input_dim: int, out_dim: int) -> None:
        super().__init__()

        self.net = nn.Sequential(
            nn.Linear(input_dim, 64),
            nn.ReLU(inplace=True),
            nn.Linear(64, out_dim),
            nn.ReLU(inplace=True),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)


class PointPillarsBackbonePlaceholder(nn.Module):
    """
    Placeholder for a PointPillars-style LiDAR encoder.

    Expected real behavior:
        raw point cloud -> pillarization -> pillar feature net ->
        scatter to BEV pseudo-image -> BEV backbone

    For now, this module expects a precomputed BEV tensor:
        lidar_bev: (B, C, H, W)
    """
    def __init__(self, in_channels: int, out_channels: int) -> None:
        super().__init__()

        # TODO: Replace with real PointPillars implementation
        self.backbone = nn.Sequential(
            nn.Conv2d(in_channels, 64, kernel_size=3, stride=1, padding=1),
            nn.ReLU(inplace=True),

            nn.Conv2d(64, 64, kernel_size=3, stride=2, padding=1),
            nn.ReLU(inplace=True),

            nn.Conv2d(64, out_channels, kernel_size=3, stride=2, padding=1),
            nn.ReLU(inplace=True),
        )

    def forward(self, lidar_bev: torch.Tensor) -> torch.Tensor:
        return self.backbone(lidar_bev)



class LidarEncoder(nn.Module):
    """
    LiDAR branch:
        lidar_bev -> PointPillars  -> small CNN  -> feature vector
    """
    def __init__(self, in_channels: int, pp_bev_channels: int, out_dim: int) -> None:
        super().__init__()

        self.pointpillars = PointPillarsBackbonePlaceholder(
            in_channels=in_channels,
            out_channels=pp_bev_channels,
        )

        self.bev_head = nn.Sequential(
            nn.Conv2d(pp_bev_channels, 128, kernel_size=3, stride=2, padding=1),
            nn.ReLU(inplace=True),

            nn.Conv2d(128, 128, kernel_size=3, stride=2, padding=1),
            nn.ReLU(inplace=True),

            nn.AdaptiveAvgPool2d((1, 1)),
        )

        self.fc = nn.Sequential(
            nn.Flatten(),
            nn.Linear(128, out_dim),
            nn.ReLU(inplace=True),
        )

    def forward(self, lidar_bev: torch.Tensor) -> torch.Tensor:
        x = self.pointpillars(lidar_bev)
        x = self.bev_head(x)
        x = self.fc(x)
        return x



class FusionHead(nn.Module):
    def __init__(
        self,
        input_dim: int,
        fusion_hidden_dim: int,
        head_hidden_dim: int,
        output_dim: int,
    ) -> None:
        super().__init__()

        self.net = nn.Sequential(
            nn.Linear(input_dim, fusion_hidden_dim),
            nn.ReLU(inplace=True),

            nn.Linear(fusion_hidden_dim, head_hidden_dim),
            nn.ReLU(inplace=True),

            nn.Linear(head_hidden_dim, output_dim),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)



class DockingNet(nn.Module):
    """
    Full multimodal model:
        3 x camera ResNet-34
        1 x LiDAR PointPillars-style branch + small CNN head
        1 x scalar/state MLP
        concat all features
        fusion MLP
        outputs:
            steering
            speed
    """
    def __init__(self, cfg: ModelConfig) -> None:
        super().__init__()
        self.cfg = cfg

        self.left_cam = ResNetImageEncoder(
            model_name=cfg.image_backbone,
            pretrained=cfg.image_pretrained,
            out_dim=cfg.image_feat_dim,
        )
        self.center_cam = ResNetImageEncoder(
            model_name=cfg.image_backbone,
            pretrained=cfg.image_pretrained,
            out_dim=cfg.image_feat_dim,
        )
        self.right_cam = ResNetImageEncoder(
            model_name=cfg.image_backbone,
            pretrained=cfg.image_pretrained,
            out_dim=cfg.image_feat_dim,
        )

        self.lidar = LidarEncoder(
            in_channels=1,
            pp_bev_channels=cfg.lidar_bev_channels,
            out_dim=cfg.lidar_feat_dim,
        )

        self.state = StateEncoder(
            input_dim=cfg.state_dim,
            out_dim=cfg.state_feat_dim,
        )

        fused_dim = (
            cfg.image_feat_dim * 3
            + cfg.lidar_feat_dim
            + cfg.state_feat_dim
        )

        self.head = FusionHead(
            input_dim=fused_dim,
            fusion_hidden_dim=cfg.fusion_hidden_dim,
            head_hidden_dim=cfg.head_hidden_dim,
            output_dim=cfg.output_dim,
        )

    def forward(
        self,
        img_left: torch.Tensor,
        img_center: torch.Tensor,
        img_right: torch.Tensor,
        lidar_bev: torch.Tensor,
        state_vec: torch.Tensor,
    ) -> Dict[str, torch.Tensor]:
        feat_left = self.left_cam(img_left)
        feat_center = self.center_cam(img_center)
        feat_right = self.right_cam(img_right)

        feat_lidar = self.lidar(lidar_bev)
        feat_state = self.state(state_vec)

        fused = torch.cat(
            [feat_left, feat_center, feat_right, feat_lidar, feat_state],
            dim=1,
        )

        raw = self.head(fused)

        steering = torch.tanh(raw[:, 0:1])
        speed = torch.tanh(raw[:, 1:2])

        return {
            "steering": steering,
            "speed": speed,
            "raw_output": raw,
            "fused_feat": fused,
        }



def control_loss(
    pred: Dict[str, torch.Tensor],
    target_steering: torch.Tensor,
    target_speed: torch.Tensor,
) -> torch.Tensor:
    loss_steer = F.l1_loss(pred["steering"], target_steering)
    loss_speed = F.l1_loss(pred["speed"], target_speed)
    return loss_steer + loss_speed


# ============================================================
#  Batch Builder
# ============================================================

def make_dummy_batch(
    batch_size: int = 2,
    image_size: Tuple[int, int] = (224, 224),
    bev_size: Tuple[int, int] = (128, 128),
    state_dim: int = 9,
    device: str = "cpu",
) -> Dict[str, torch.Tensor]:
    """
    State vector order:
        [speed, applied_steer, goal_rel_x, goal_rel_y, goal_yaw_error,
         laser1, laser2, laser3, laser4]
    """
    h, w = image_size
    bh, bw = bev_size

    batch = {
        "img_left": torch.randn(batch_size, 3, h, w, device=device),
        "img_center": torch.randn(batch_size, 3, h, w, device=device),
        "img_right": torch.randn(batch_size, 3, h, w, device=device),
        "lidar_bev": torch.randn(batch_size, 1, bh, bw, device=device),
        "state_vec": torch.randn(batch_size, state_dim, device=device),
        "target_steering": torch.randn(batch_size, 1, device=device).clamp(-1.0, 1.0),
        "target_speed": torch.randn(batch_size, 1, device=device).clamp(-1.0, 1.0),
    }
    return batch



def train_step(
    model: DockingNet,
    optimizer: torch.optim.Optimizer,
    batch: Dict[str, torch.Tensor],
) -> float:
    model.train()
    optimizer.zero_grad()

    pred = model(
        img_left=batch["img_left"],
        img_center=batch["img_center"],
        img_right=batch["img_right"],
        lidar_bev=batch["lidar_bev"],
        state_vec=batch["state_vec"],
    )

    loss = control_loss(
        pred=pred,
        target_steering=batch["target_steering"],
        target_speed=batch["target_speed"],
    )

    loss.backward()
    optimizer.step()
    return float(loss.item())



def main() -> None:
    device = "cuda" if torch.cuda.is_available() else "cpu"

    cfg = ModelConfig(
        image_backbone="resnet34",
        image_pretrained=True,
        image_feat_dim=256,
        lidar_feat_dim=128,
        state_feat_dim=64,
        state_dim=9,
        fusion_hidden_dim=256,
        head_hidden_dim=128,
        lidar_bev_channels=64,
        output_dim=2,
    )

    model = DockingNet(cfg).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-4)

    batch = make_dummy_batch(
        batch_size=4,
        image_size=(224, 224),
        bev_size=(128, 128),
        state_dim=cfg.state_dim,
        device=device,
    )

    loss_value = train_step(model, optimizer, batch)
    print(f"train loss: {loss_value:.4f}")

    model.eval()
    with torch.no_grad():
        pred = model(
            img_left=batch["img_left"],
            img_center=batch["img_center"],
            img_right=batch["img_right"],
            lidar_bev=batch["lidar_bev"],
            state_vec=batch["state_vec"],
        )

    print("steering shape:", pred["steering"].shape)
    print("speed shape:", pred["speed"].shape)
    print("fused feature shape:", pred["fused_feat"].shape)


if __name__ == "__main__":
    main()

