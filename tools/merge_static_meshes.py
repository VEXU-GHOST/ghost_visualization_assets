"""Merge a URDF's many small visual meshes into a few large STLs.

Run with Python 3 and numpy only. Coordinates stay in meters.

Links are split into rigid groups:
  * the static group: every link that only connects to the root through fixed
    joints. Its meshes are merged into one STL in the root link's frame.
  * one moving group per non-fixed joint: the joint's child link plus all
    links below it that connect through fixed joints. Its meshes are merged
    into one STL in the child link's frame, so the joint still rotates the
    whole group (e.g. one STL per wheel).

Merged links stay in the URDF as empty frames: joints, TF frames and inertials
are unchanged, only <visual> elements move. The input files are never modified.
"""

import argparse
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np


STL_RECORD = np.dtype([
    ("normal", "<f4", (3,)),
    ("vertices", "<f4", (3, 3)),
    ("attribute", "<u2"),
])
DEFAULT_COLOR = (0.8, 0.8, 0.8, 1.0)


def read_stl_triangles(path):
    """Return an (N, 3, 3) float64 array from a binary STL."""
    with path.open("rb") as stream:
        header = stream.read(84)
        if len(header) != 84:
            raise ValueError(f"Incomplete STL: {path}")
        count = struct.unpack_from("<I", header, 80)[0]
        if path.stat().st_size != 84 + 50 * count:
            raise ValueError(f"Expected binary STL (is it a Git LFS pointer?): {path}")
        records = np.fromfile(stream, dtype=STL_RECORD, count=count)
    triangles = records["vertices"].astype(np.float64)
    if not np.isfinite(triangles).all():
        raise ValueError(f"Nonfinite coordinates: {path}")
    return triangles


def write_stl(path, triangles, label):
    """Write (N, 3, 3) triangles as a binary STL with recomputed normals."""
    triangles = triangles.astype(np.float32)
    normals = np.cross(triangles[:, 1] - triangles[:, 0],
                       triangles[:, 2] - triangles[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    # Degenerate triangles have no defined normal; keep them but avoid NaN.
    normals = normals / np.where(lengths == 0, 1.0, lengths)[:, None]
    records = np.zeros(len(triangles), dtype=STL_RECORD)
    records["vertices"] = triangles
    records["normal"] = normals
    with path.open("wb") as stream:
        stream.write(label.encode("ascii")[:80].ljust(80, b" "))
        stream.write(struct.pack("<I", len(triangles)))
        records.tofile(stream)


def pose(origin):
    """4x4 transform for a URDF <origin> element (None means identity)."""
    matrix = np.eye(4)
    if origin is None:
        return matrix
    x, y, z = (float(v) for v in origin.get("xyz", "0 0 0").split())
    roll, pitch, yaw = (float(v) for v in origin.get("rpy", "0 0 0").split())
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    # URDF convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll).
    matrix[:3, :3] = (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
                      @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
                      @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))
    matrix[:3, 3] = (x, y, z)
    return matrix


def mesh_path(filename, package_name, package_root):
    prefix = f"package://{package_name}/"
    if not filename.startswith(prefix):
        raise ValueError(f"Mesh must be a package://{package_name}/ URL: {filename}")
    return package_root / filename[len(prefix):]


def build_groups(robot, root_name, static_name="chassis"):
    """Return ({link: pose in root frame at zero joint angles}, {anchor: (label, [links])}).

    The anchor of a group is the link whose frame the merged mesh lives in:
    the root link for the static group, a moving joint's child otherwise.
    """
    children = {}
    for joint in robot.findall("joint"):
        children.setdefault(joint.find("parent").get("link"), []).append(joint)
    poses = {root_name: np.eye(4)}
    groups = {root_name: (static_name, [root_name])}
    stack = [(root_name, root_name)]  # (link, anchor of its group)
    while stack:
        parent, anchor = stack.pop()
        for joint in children.get(parent, []):
            child = joint.find("child").get("link")
            poses[child] = poses[parent] @ pose(joint.find("origin"))
            if joint.get("type") == "fixed":
                groups[anchor][1].append(child)
                stack.append((child, anchor))
            else:
                groups[child] = (joint.get("name"), [child])
                stack.append((child, child))
    return poses, groups


def visual_color(visual):
    color = visual.find("material/color")
    if color is None:
        return DEFAULT_COLOR
    return tuple(float(v) for v in color.get("rgba").split())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("urdf", type=Path, help="Source URDF (left unmodified)")
    parser.add_argument("--package-name", default="ghost_visualization_assets",
                        help="ROS package name used in package:// mesh URLs")
    parser.add_argument("--package-root", type=Path,
                        help="Directory package:// URLs resolve against (default: repo root)")
    parser.add_argument("--root", default="root",
                        help="Link that owns the merged static mesh; must be an ancestor of every link to merge")
    parser.add_argument("--static-name", default="chassis",
                        help="Name for the static group's STL (<name>_merged.stl)")
    parser.add_argument("--out-urdf", type=Path,
                        help="Output URDF (default: <name>_merged.urdf next to the source)")
    parser.add_argument("--out-mesh-dir", type=Path,
                        help="Where merged STLs go (default: the meshes/ directory next to the source URDF's urdf/ directory)")
    parser.add_argument("--by-color", action="store_true",
                        help="Keep colors: write one STL per material color within each group "
                             "(STL has no color of its own, so otherwise each group gets its dominant color)")
    args = parser.parse_args()

    urdf = args.urdf.resolve()
    package_root = (args.package_root or Path(__file__).resolve().parent.parent).resolve()
    out_urdf = (args.out_urdf or urdf.with_name(urdf.stem + "_merged.urdf")).resolve()
    out_dir = (args.out_mesh_dir or urdf.parent.parent / "meshes").resolve()
    if out_urdf == urdf:
        parser.error("Refusing to overwrite the source URDF")

    tree = ET.parse(urdf, parser=ET.XMLParser(target=ET.TreeBuilder(insert_comments=True)))
    robot = tree.getroot()
    links = {link.get("name"): link for link in robot.findall("link")}
    if args.root not in links:
        parser.error(f"No link named {args.root!r}")
    poses, groups = build_groups(robot, args.root, args.static_name)
    outside = [name for name in links if name not in poses]
    if outside:
        print(f"Outside the {args.root!r} subtree, untouched: {', '.join(outside)}")

    out_dir.mkdir(parents=True, exist_ok=True)
    cache = {}
    results = []  # (anchor, [(path, color, triangle count)], visuals merged)
    for anchor, (label, members) in groups.items():
        anchor_inverse = np.linalg.inv(poses[anchor])
        by_color = {}  # color -> list of (N, 3, 3)
        merged_visuals = 0
        for name in members:
            for visual in links[name].findall("visual"):
                mesh = visual.find("geometry/mesh")
                if mesh is None:
                    raise ValueError(f"Link {name!r} has a non-mesh visual; cannot merge it")
                path = mesh_path(mesh.get("filename"), args.package_name, package_root)
                if path not in cache:
                    cache[path] = read_stl_triangles(path)
                scale = np.array([float(v) for v in mesh.get("scale", "1 1 1").split()])
                # Pose of this visual in the anchor link's frame.
                transform = anchor_inverse @ poses[name] @ pose(visual.find("origin"))
                points = cache[path].reshape(-1, 3) * scale
                points = points @ transform[:3, :3].T + transform[:3, 3]
                triangles = points.reshape(-1, 3, 3)
                # A mirroring transform flips triangle winding; undo it.
                if np.linalg.det(transform[:3, :3]) * np.prod(np.sign(scale)) < 0:
                    triangles = triangles[:, ::-1]
                by_color.setdefault(visual_color(visual), []).append(triangles)
                merged_visuals += 1
        if not by_color:
            continue
        merged = {color: np.concatenate(parts) for color, parts in by_color.items()}
        if not args.by_color:
            dominant = max(merged, key=lambda color: len(merged[color]))
            merged = {dominant: np.concatenate(list(merged.values()))}
        outputs = []
        for color, triangles in merged.items():
            suffix = ""
            if args.by_color:
                suffix = "_" + "".join(f"{round(c * 255):02x}" for c in color[:3])
            path = out_dir / f"{label}_merged{suffix}.stl"
            write_stl(path, triangles, f"Merged {label} visualization mesh, meters")
            outputs.append((path, color, len(triangles)))
        results.append((anchor, outputs, merged_visuals))

        # Move the visuals: strip them from every member, then give the anchor the merged mesh(es).
        for name in members:
            for visual in links[name].findall("visual"):
                links[name].remove(visual)
        for path, color, _ in outputs:
            visual = ET.SubElement(links[anchor], "visual")
            ET.SubElement(visual, "origin", xyz="0 0 0", rpy="0 0 0")
            geometry = ET.SubElement(visual, "geometry")
            relative = path.relative_to(package_root).as_posix()
            ET.SubElement(geometry, "mesh", filename=f"package://{args.package_name}/{relative}", scale="1 1 1")
            material = ET.SubElement(visual, "material", name=f"{path.stem}_material")
            ET.SubElement(material, "color", rgba=" ".join(f"{c:g}" for c in color))

    ET.indent(tree, space="    ")
    tree.write(out_urdf, encoding="utf-8", xml_declaration=True)
    for anchor, outputs, merged_visuals in results:
        print(f"{anchor}: merged {merged_visuals} visuals")
        for path, _, count in outputs:
            print(f"    {path.relative_to(package_root)}: {count:,} triangles, {path.stat().st_size / 1e6:.1f} MB")
    print(f"Wrote {out_urdf}")


if __name__ == "__main__":
    main()
