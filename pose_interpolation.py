import torch
import pypose as pp
import genesis as gs
import time
import argparse

def main():
    parser = argparse.ArgumentParser(description="Drone interpolation + PERMANENT static RGB axes trail (all poses always visible)")
    parser.add_argument("--num_poses", type=int, default=40, help="Number of interpolated poses")
    parser.add_argument("--duration", type=float, default=8.0, help="Animation duration in seconds")
    parser.add_argument("--loops", type=int, default=10, help="Visualization loops")
    
    args = parser.parse_args()

    # initial and final poses in SE(3), note this is translation and quaternion!
    Ti_data = torch.tensor([0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
    Tf_data = torch.tensor([2.0, 0.0, 1.45,  0.13, 0.39, 0.16, 0.895])

    Ti = pp.SE3(Ti_data)
    Tf = pp.SE3(Tf_data)

    print("Initial pose Ti:", Ti)
    print("Final pose Tf:  ", Tf)

    # geodesic interpolation using lie algebra
    rel_se3 = Ti.Inv() @ Tf
    xi      = rel_se3.Log()
    interpolated_poses = []
    for k in range(args.num_poses):
        t = k / (args.num_poses - 1) if args.num_poses > 1 else 0.0
        scaled_xi = pp.se3(t * xi.tensor())   # scale relative twist
        delta_T   = scaled_xi.Exp()
        interp_pose = Ti @ delta_T            # geodesic on SE(3)
        interpolated_poses.append(interp_pose)
        
    print(f"✅ Generated {len(interpolated_poses)} interpolated SE(3) poses")

    # Genesis Visualization
    gs.init(backend=gs.gpu if torch.cuda.is_available() else gs.cpu)

    scene = gs.Scene(
        viewer_options=gs.options.ViewerOptions(
            camera_pos=(0, -5, 3),
            camera_lookat=(1, 0, 1),
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

    print("Creating permanent RGB axes for ALL poses...")
    for pose in interpolated_poses:
        data = pose.tensor().cpu()
        pos  = data[:3].numpy()
        quat = data[3:].numpy()

        scene.add_entity(
            morph=gs.morphs.Sphere(
                radius=0.004,
                pos=pos,
                quat=quat,
                fixed=True,
            ),
            surface=gs.surfaces.Default(color=(1.0, 1.0, 1.0, 0.0)),
        )

    # Drone
    first_data = interpolated_poses[0].tensor().cpu()
    first_pos  = first_data[:3].numpy()
    first_quat = first_data[3:].numpy()

    drone = scene.add_entity(
        gs.morphs.URDF(
            file="urdf/drones/cf2x.urdf",
            scale=1.0,
            pos=first_pos,
            quat=first_quat,
        ),
        surface=gs.surfaces.Default(color=(1.0, 0.6, 0.1, 1.0)),
    )

    scene.build()

    # Animate
    dt = args.duration / max(args.num_poses - 1, 1)

    print(f"Animating drone through {args.num_poses} static pose axes over {args.duration}s ...")
    for _ in range(args.loops):
        for pose in interpolated_poses:
            data = pose.tensor()
            pos  = data[:3].cpu().numpy()
            quat = data[3:].cpu().numpy()

            drone.set_pos(pos)
            drone.set_quat(quat)

            scene.step()
            time.sleep(dt)

    print("Animation finished!")
    input("Press Enter to close the viewer...")


if __name__ == "__main__":
    main()