import pickle
import numpy as np
import open3d as o3d

# Path to pickle
INPUT_FILE = "lidar_capture.pkl"

# If you want to export a PLY point cloud, set a filename; else set to None
EXPORT_PLY = "cloud.ply"

# If you want to dump raw JSON, set a filename; else set to None
EXPORT_JSON = None


def load_records(fname):
    with open(fname, 'rb') as f:
        return pickle.load(f)


def summary(records):
    total_frames = len(records)
    total_points = sum(len(r['points']) for r in records)
    print(f"Loaded {total_frames} frames, {total_points} total points")


def to_ply(records, out_ply):
    all_pts = []
    all_cols = []
    for r in records:
        for x, y, z, i in r['points']:
            all_pts.append([x, y, z])
            all_cols.append([i, i, i])
    pts_np = np.array(all_pts, dtype=np.float32)
    cols_np = np.array(all_cols, dtype=np.float32)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts_np)
    pcd.colors = o3d.utility.Vector3dVector(cols_np)
    o3d.io.write_point_cloud(out_ply, pcd)
    print(f"Wrote point cloud to {out_ply}")


def main():
    recs = load_records(INPUT_FILE)
    summary(recs)
    if EXPORT_PLY:
        to_ply(recs, EXPORT_PLY)
    if EXPORT_JSON:
        import json
        with open(EXPORT_JSON, 'w') as f:
            json.dump(recs, f, default=lambda o: float(o), indent=2)
        print(f"Dumped raw JSON to {EXPORT_JSON}")

if __name__ == "__main__":
    main()
