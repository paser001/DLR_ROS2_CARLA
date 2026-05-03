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

from pathlib import Path
import cv2
import numpy as np
import pandas as pd
from torch.utils.data import Dataset, DataLoader, random_split




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




# ============================================================
# Dataset
# ============================================================

class CarlaDockingDataset(Dataset):
    def __init__(
        self,
        run_dir: str,
        image_size: Tuple[int, int] = (224, 224),
        lidar_bev_size: Tuple[int, int] = (128, 128),
    ):
        self.run_dir = Path(run_dir)
        self.csv_path = self.run_dir / "records" / "samples.csv"

        self.df = pd.read_csv(self.csv_path)
        self.df = self.df.reset_index(drop=True)

        self.image_size = image_size
        self.lidar_bev_size = lidar_bev_size

    def __len__(self):
        return len(self.df)

    def load_image(self, rel_path: str) -> torch.Tensor:
        img_path = self.run_dir / rel_path

        img = cv2.imread(str(img_path), cv2.IMREAD_COLOR)
        if img is None:
            raise FileNotFoundError(f"Could not read image: {img_path}")

        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, self.image_size)

        img = img.astype(np.float32) / 255.0

        # ImageNet normalization for pretrained ResNet
        mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
        img = (img - mean) / std

        # HWC -> CHW
        img = np.transpose(img, (2, 0, 1))

        return torch.tensor(img, dtype=torch.float32)

    def load_lidar_bev(self, rel_path: str) -> torch.Tensor:
        """
        Temporary simple BEV occupancy map.

        Later replace this with real PointPillars preprocessing.
        """
        lidar_path = self.run_dir / rel_path
        points = np.load(lidar_path)  # expected shape: (N, 3)

        H, W = self.lidar_bev_size
        bev = np.zeros((H, W), dtype=np.float32)

        if points.shape[0] > 0:
            x = points[:, 0]
            y = points[:, 1]

            # TODO: tune these to your actual LiDAR coordinate frame
            x_min, x_max = -10.0, 10.0
            y_min, y_max = -10.0, 10.0

            valid = (x >= x_min) & (x <= x_max) & (y >= y_min) & (y <= y_max)
            x = x[valid]
            y = y[valid]

            if x.shape[0] > 0:
                ix = ((x - x_min) / (x_max - x_min) * (W - 1)).astype(np.int32)
                iy = ((y - y_min) / (y_max - y_min) * (H - 1)).astype(np.int32)

                ix = np.clip(ix, 0, W - 1)
                iy = np.clip(iy, 0, H - 1)

                bev[iy, ix] = 1.0

        # add channel dimension: (1, H, W)
        bev = bev[None, :, :]

        return torch.tensor(bev, dtype=torch.float32)

    def __getitem__(self, idx: int) -> Dict[str, torch.Tensor]:
        row = self.df.iloc[idx]

        img_center = self.load_image(row["dalsa2_rgb_path"])
        img_left = self.load_image(row["leopard4_rgb_path"])
        img_right = self.load_image(row["leopard5_rgb_path"])

        lidar_bev = self.load_lidar_bev(row["seyond6_lidar_path"])

        state_vec = torch.tensor([
            row["speed"],
            row["applied_steer"],
            row["goal_rel_x"],
            row["goal_rel_y"],
            row["goal_yaw_error"],
            row["laser1_distance"],
            row["laser2_distance"],
            row["laser3_distance"],
            row["laser4_distance"],
        ], dtype=torch.float32)

        target_steering = torch.tensor([row["applied_steer"]], dtype=torch.float32)
        target_speed = torch.tensor([row["speed"]], dtype=torch.float32)

        return {
            "img_left": img_left,
            "img_center": img_center,
            "img_right": img_right,
            "lidar_bev": lidar_bev,
            "state_vec": state_vec,
            "target_steering": target_steering,
            "target_speed": target_speed,
        }



def control_loss(
    pred: Dict[str, torch.Tensor],
    target_steering: torch.Tensor,
    target_speed: torch.Tensor,
) -> torch.Tensor:
    loss_steer = F.l1_loss(pred["steering"], target_steering)
    loss_speed = F.l1_loss(pred["speed"], target_speed)
    return loss_steer + loss_speed



def move_batch_to_device(
    batch: Dict[str, torch.Tensor],
    device: torch.device,
) -> Dict[str, torch.Tensor]:
    return {k: v.to(device) for k, v in batch.items()}


def train_one_epoch(
    model: DockingNet,
    optimizer: torch.optim.Optimizer,
    dataloader: DataLoader,
    device: torch.device,
) -> float:
    model.train()
    total_loss = 0.0

    for batch in dataloader:
        batch = move_batch_to_device(batch, device)

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

        total_loss += loss.item()

    return total_loss / len(dataloader)


@torch.no_grad()
def validate(
    model: DockingNet,
    dataloader: DataLoader,
    device: torch.device,
) -> float:
    model.eval()
    total_loss = 0.0

    for batch in dataloader:
        batch = move_batch_to_device(batch, device)

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

        total_loss += loss.item()

    return total_loss / len(dataloader)


def save_checkpoint(
    path: str,
    model: DockingNet,
    optimizer: torch.optim.Optimizer,
    cfg: ModelConfig,
    epoch: int,
    train_loss: float,
    val_loss: float,
):
    torch.save({
        "epoch": epoch,
        "model_state_dict": model.state_dict(),
        "optimizer_state_dict": optimizer.state_dict(),
        "config": cfg,
        "train_loss": train_loss,
        "val_loss": val_loss,
    }, path)



def main() -> None:
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    run_dir = "/data/carla_datasets/dataset_run_04_27_15_09"

    batch_size = 8
    num_epochs = 30
    learning_rate = 1e-4
    val_split = 0.2

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

    dataset = CarlaDockingDataset(
        run_dir=run_dir,
        image_size=(224, 224),
        lidar_bev_size=(128, 128),
    )

    num_val = int(len(dataset) * val_split)
    num_train = len(dataset) - num_val

    train_dataset, val_dataset = random_split(
        dataset,
        [num_train, num_val],
        generator=torch.Generator().manual_seed(42),
    )

    train_loader = DataLoader(
        train_dataset,
        batch_size=batch_size,
        shuffle=True,
        num_workers=4,
        pin_memory=True,
    )

    val_loader = DataLoader(
        val_dataset,
        batch_size=batch_size,
        shuffle=False,
        num_workers=4,
        pin_memory=True,
    )

    model = DockingNet(cfg).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)

    best_val_loss = float("inf")

    checkpoint_dir = Path("checkpoints")
    checkpoint_dir.mkdir(parents=True, exist_ok=True)

    for epoch in range(num_epochs):
        train_loss = train_one_epoch(
            model=model,
            optimizer=optimizer,
            dataloader=train_loader,
            device=device,
        )

        val_loss = validate(
            model=model,
            dataloader=val_loader,
            device=device,
        )

        print(
            f"Epoch [{epoch + 1}/{num_epochs}] "
            f"Train Loss: {train_loss:.6f} | "
            f"Val Loss: {val_loss:.6f}"
        )

        save_checkpoint(
            path=str(checkpoint_dir / "latest.pt"),
            model=model,
            optimizer=optimizer,
            cfg=cfg,
            epoch=epoch,
            train_loss=train_loss,
            val_loss=val_loss,
        )


        if val_loss < best_val_loss:
            best_val_loss = val_loss
            save_checkpoint(
                path=str(checkpoint_dir / "best.pt"),
                model=model,
                optimizer=optimizer,
                cfg=cfg,
                epoch=epoch,
                train_loss=train_loss,
                val_loss=val_loss,
            )
            print(f"Saved new best model with val loss {best_val_loss:.6f}")

    print("Training finished.")
    print(f"Best validation loss: {best_val_loss:.6f}")


if __name__ == "__main__":
    main()