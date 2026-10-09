"""Make a separate, smaller Baby Jerry preview package.

Run with Python 3, numpy, matplotlib, and fast-simplification installed.
All coordinates stay in meters; this script never recenters or rescales a mesh.
The three large meshes are simplified. Every other mesh is copied unchanged.
"""

import argparse
import json
from pathlib import Path
import shutil
import struct
import xml.etree.ElementTree as ET

import numpy as np
import fast_simplification


STL_RECORD = np.dtype([
    ("normal", "<f4", (3,)),
    ("vertices", "<f4", (3, 3)),
    ("attribute", "<u2"),
])
TARGETS = {
    "_2_75__Anti_Static_Omni_Directional_Wheel__220mm_Travel___276_8106_.stl": "Middle omni-wheel",
    "V5_Robot_Brain__276_4810_.stl": "Brain",
    "V5_Robot_Battery__276_4811_.stl": "Battery",
}


def read_stl(path):
    """Validate binary STL, then weld only exactly identical vertices."""
    with path.open("rb") as stream:
        header = stream.read(84)
        if len(header) != 84:
            raise ValueError(f"Incomplete STL: {path}")
        count = struct.unpack_from("<I", header, 80)[0]
        if path.stat().st_size != 84 + 50 * count:
            raise ValueError(f"Expected binary STL: {path}")
        records = np.fromfile(stream, dtype=STL_RECORD, count=count)
    coordinates = records["vertices"].reshape(-1, 3)
    if not np.isfinite(coordinates).all():
        raise ValueError(f"Nonfinite coordinates: {path}")
    points, indices = np.unique(coordinates, axis=0, return_inverse=True)
    return points.astype(np.float64), indices.reshape(-1, 3).astype(np.int32)


def write_stl(path, points, faces):
    """Recompute normals and write a compact binary STL."""
    triangles = points[faces].astype(np.float32)
    normals = np.cross(triangles[:, 1] - triangles[:, 0],
                       triangles[:, 2] - triangles[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    if not np.isfinite(triangles).all() or np.any(lengths == 0):
        raise ValueError("Simplification produced invalid or degenerate triangles")
    records = np.zeros(len(faces), dtype=STL_RECORD)
    records["vertices"] = triangles
    records["normal"] = normals / lengths[:, None]
    with path.open("wb") as stream:
        stream.write(b"Baby Jerry simplified visualization mesh".ljust(80, b" "))
        stream.write(struct.pack("<I", len(faces)))
        records.tofile(stream)


def comparison(path, label, original, reduced):
    """Render the same orthographic view for original and reduced surfaces."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection

    # A fixed viewing basis gives both columns identical orientation and scale.
    direction = np.array([1.0, -1.5, 0.85])
    direction /= np.linalg.norm(direction)
    right = np.cross(direction, [0.0, 0.0, 1.0])
    right /= np.linalg.norm(right)
    up = np.cross(right, direction)
    basis = np.column_stack((right, up, direction))
    projected = original[0] @ basis
    lower, upper = projected[:, :2].min(0), projected[:, :2].max(0)
    padding = (upper - lower).max() * 0.07
    fig, axes = plt.subplots(1, 2, figsize=(10, 5))
    for ax, title, (points, faces) in zip(axes, ["Original", "Reduced"], [original, reduced]):
        triangles = points[faces]
        normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
        normals /= np.maximum(np.linalg.norm(normals, axis=1)[:, None], 1e-30)
        shade = 0.35 + 0.6 * np.abs(normals @ direction)
        colors = np.column_stack((shade * 0.72, shade * 0.85, shade, np.ones(len(faces))))
        vertices = triangles @ basis
        order = np.argsort(vertices[:, :, 2].mean(1))
        ax.add_collection(PolyCollection(vertices[order, :, :2],
                                        facecolors=colors[order], edgecolors="none",
                                        linewidths=0, rasterized=True))
        ax.set_xlim(lower[0] - padding, upper[0] + padding)
        ax.set_ylim(lower[1] - padding, upper[1] + padding)
        ax.set_aspect("equal")
        ax.axis("off")
        ax.set_title(f"{title}: {len(faces):,} triangles")
    fig.suptitle(label)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="Original package folder containing meshes/ and urdf/")
    parser.add_argument("destination", type=Path, help="New package folder; must not already exist")
    parser.add_argument("--keep", type=float, default=0.2, help="Target triangle fraction to retain")
    args = parser.parse_args()
    source, destination = args.source.resolve(), args.destination.resolve()
    if not 0 < args.keep <= 1:
        parser.error("--keep must be between 0 and 1")
    if destination.exists() or source == destination or source in destination.parents:
        parser.error("Choose a new destination outside the source package")
    for name in TARGETS:
        if not (source / "meshes" / name).is_file():
            parser.error(f"Missing source mesh: {name}")
    shutil.copytree(source / "meshes", destination / "meshes")
    (destination / "urdf").mkdir()
    report = {"target_fraction": args.keep, "meshes": []}
    for name, label in TARGETS.items():
        print(f"Simplifying {label}...", flush=True)
        original_file = source / "meshes" / name
        points, faces = read_stl(original_file)
        bounds = np.stack((points.min(0), points.max(0)))
        # If the requested reduction changes the outer size too far, back off.
        # A full-detail fallback ensures we never have to rescale the wheel.
        fractions = sorted(set([args.keep] + [v for v in [0.35, 0.5, 0.75, 1.0] if v > args.keep]))
        for fraction in fractions:
            if fraction == 1.0:
                new_points, new_faces = points.copy(), faces.copy()
            else:
                new_points, new_faces = fast_simplification.simplify(
                    points, faces, target_count=max(1, int(len(faces) * fraction)),
                    agg=5.0, preserve_border=True,
                )
            new_faces = np.asarray(new_faces, dtype=np.int32)
            if not len(new_faces) or not np.isfinite(new_points).all():
                raise ValueError(f"Invalid simplified mesh: {name}")
            new_bounds = np.stack((new_points.min(0), new_points.max(0)))
            change = float(np.abs(bounds - new_bounds).max())
            if change <= 0.001:
                break
            print(f"{label}: {fraction:.0%} retained shifts bounds {change * 1000:.3f} mm; retrying", flush=True)
        target_file = destination / "meshes" / name
        write_stl(target_file, new_points, new_faces)
        # Check the actual written float32 STL, rather than only in-memory geometry.
        written_points, written_faces = read_stl(target_file)
        if len(written_faces) != len(new_faces):
            raise ValueError("Written triangle count differs")
        comparison(destination / f"{label.lower().replace(' ', '_')}_comparison.png",
                   label, (points, faces), (written_points, written_faces))
        result = {
            "name": name, "label": label,
            "original_triangles": len(faces), "reduced_triangles": len(new_faces),
            "accepted_target_fraction": fraction,
            "original_bytes": original_file.stat().st_size,
            "reduced_bytes": target_file.stat().st_size,
            "original_bounds_m": bounds.tolist(), "reduced_bounds_m": new_bounds.tolist(),
            "maximum_bounding_edge_change_mm": change * 1000,
        }
        report["meshes"].append(result)
        print(f"{label}: {len(faces):,} -> {len(new_faces):,} triangles; "
              f"maximum bounding edge change {change * 1000:.4f} mm", flush=True)

    # Only mesh filenames change. Link origins, joint axes and limits stay intact.
    urdf_files = list((source / "urdf").glob("*.urdf"))
    if len(urdf_files) != 1:
        raise ValueError("Source must contain exactly one URDF")
    tree = ET.parse(urdf_files[0])
    for mesh in tree.findall(".//mesh"):
        filename = Path(mesh.get("filename")).name
        target = destination / "meshes" / filename
        if not target.is_file():
            raise ValueError(f"Missing referenced mesh: {target}")
        mesh.set("filename", target.as_uri())
    tree.write(destination / "urdf" / "babyjerry_preview.urdf", encoding="utf-8", xml_declaration=True)
    # This existing launcher reads the URDF next to itself, so it works in the copy.
    if (source / "preview.launch.py").is_file():
        shutil.copy2(source / "preview.launch.py", destination / "preview.launch.py")
    report["original_package_mesh_bytes"] = sum(f.stat().st_size for f in (source / "meshes").glob("*.stl"))
    report["reduced_package_mesh_bytes"] = sum(f.stat().st_size for f in (destination / "meshes").glob("*.stl"))
    report["validation_note"] = "Bounds and comparative renders checked; visual simplification is lossy. Validate joint motion and appearance in Foxglove."
    (destination / "simplification_report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"Saved reduced package: {destination}", flush=True)


if __name__ == "__main__":
    main()
