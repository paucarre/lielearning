import torch
import pypose as pp
import genesis as gs
import argparse

def main():
    parser = argparse.ArgumentParser(description="Static weighted averaging of 4 SE(3) drone poses (PyPose Lie groups) + Genesis visualization")
    parser.add_argument("--optimization_iterations", type=int, default=2, help="Number of iterations in the optimization process")
    args = parser.parse_args()

    # 4 pose proposals in (SE(3)), note this is translation and quaternion!
    poses_data = torch.tensor([
        [ 0.8,  0.6, -0.4,   0.25,  0.18, -0.32,  0.905],   # Pose 1: strong roll left + some yaw
        [-0.7, -1.1,  0.9,  -0.15,  0.38,  0.28,  0.870],   # Pose 2: strong pitch down + yaw
        [ 1.2, -0.3, -1.1,   0.32, -0.22,  0.15,  0.895],   # Pose 3: strong roll right + moderate pitch
        [-0.4,  1.0,  0.5,  -0.28,  0.25, -0.35,  0.855],   # Pose 4: dynamic combination (highest variability)
    ], dtype=torch.float32)
    original_poses = pp.SE3(poses_data)
    
    poses = original_poses.clone()
    # Information
    information = torch.tensor([0.35, 0.30, 0.20, 0.15], dtype=torch.float32)
    print(f"Using information: {information.tolist()} (sum = {information.sum().item():.3f})")
    pose_optimization = []
    for _ in range(args.optimization_iterations):
        # weighted averaging on lie algebra
        weighted_log = ( ( information.unsqueeze(1) * poses.Log().tensor()) / information.sum() ).sum(dim=0)
        avg_se3 = pp.se3(weighted_log).Exp()
        pose_optimization.append(avg_se3)
        poses = avg_se3.Inv() * poses


    print(f"✅ Computed weighted average pose: {avg_se3}")

    # Genesis Visualization
    gs.init(backend=gs.cpu)

    scene = gs.Scene(
        viewer_options=gs.options.ViewerOptions(
            camera_pos=(0, -5, 3),
            camera_lookat=(0.3, 0.1, 1.0),
            camera_fov=40,
            res=(1280, 720),
            max_FPS=60,
        ),
        vis_options=gs.options.VisOptions(
            show_world_frame=False,
            world_frame_size=0.5,
            show_link_frame=True,
            link_frame_size=0.18,
            background_color=(0.60, 0.60, 0.60)
        ),
        show_viewer=True,
        show_FPS=True,
    )

    # semi-transparent drones at the input poses (proposals)
    print("Placing 4 semi-transparent drones at the input poses...")
    alpha = 0.8
    for pose in original_poses:
        data = pose.tensor().cpu()
        pos  = data[:3].numpy()
        quat = data[3:].numpy()

        scene.add_entity(
            gs.morphs.URDF(
                file="urdf/drones/cf2x.urdf",
                scale=1.0,
                pos=pos,
                quat=quat,
            ),
            surface=gs.surfaces.Default(color=(1.0, 0.4, 0.1, alpha)),
        )

    # Solid drone at the weighted average pose
    print("Placing solid drone at the weighted average pose...")
    integration = pp.identity_SE3()
    for idx, avg_se3 in enumerate(pose_optimization):
        integration = integration * avg_se3
        avg_data = integration.tensor().cpu()
        avg_pos  = avg_data[:3].numpy()
        avg_quat = avg_data[3:].numpy()

        color_intensity = idx / len(pose_optimization)
        scene.add_entity(
            gs.morphs.URDF(
                file="urdf/drones/cf2x.urdf",
                scale=1.0,
                pos=avg_pos,
                quat=avg_quat,
            ),
            surface=gs.surfaces.Default(color=(1.0 * color_intensity, 0.6 * color_intensity, 1. * color_intensity, 1.)),
        )
        

    scene.build()

    # Keep viewer open
    print("\n=== Static Weighted Averaging Visualization Ready ===")
    print("   • 4 semi-transparent drones  = the 4 input proposals")
    print("   • 1 solid drone              = weighted average result")
    print("   • RGB axes (X=red, Y=green, Z=blue) permanently shown at the 4 input poses")
    print("   • Everything is completely static — you can rotate/zoom freely")
    print("Close the viewer window or press ESC when done")

    input("Press Enter to close the viewer...")


if __name__ == "__main__":
    main()