#!/usr/bin/env python3
"""
Fetch + patch the AWS RoboMaker Small House World furniture/wall models.

The ~106 MB of Collada meshes + textures are intentionally NOT committed to this
repo (see .gitignore). On a fresh checkout, run this once to populate
assets/models/aws_robomaker_residential_* so `robot_world:=small_house` works.

Source : https://github.com/aws-robotics/aws-robomaker-small-house-world (branch ros1)
License: Apache-2.0  (c) Amazon.com, Inc. or its affiliates.

Harmonic (gz-sim 8) patches applied here (Gazebo Classic tolerated these; Harmonic does not):
  * Mark every model <static>1</static> (static house scenery; Harmonic rejects
    non-static links whose inertia is imperfect).
  * Fix aws_robomaker_residential_ShoeRack_01: its <inertia> had a duplicate <ixx>
    and no <izz>, so izz defaulted to 1.0 and violated the moment-of-inertia
    triangle inequality -> add <izz>0.05</izz>.
"""
import os, re, shutil, subprocess, tempfile

HERE = os.path.dirname(os.path.realpath(__file__))
PKG = os.path.normpath(os.path.join(HERE, ".."))
MODELS_DST = os.path.join(PKG, "assets", "models")
WORLD_DST = os.path.join(PKG, "assets", "worlds", "small_house.sdf")
REPO = "https://github.com/aws-robotics/aws-robomaker-small-house-world.git"
BRANCH = "ros1"

STATIC_RE = re.compile(r"<static>\s*(1|true)\s*</static>", re.I)
MODEL_RE = re.compile(r"(<model\b[^>]*>)")

def patch(models_dir):
    made = 0
    for name in os.listdir(models_dir):
        if not name.startswith("aws_robomaker_residential_"):
            continue
        f = os.path.join(models_dir, name, "model.sdf")
        if not os.path.isfile(f):
            continue
        s = open(f).read()
        if not STATIC_RE.search(s):
            m = MODEL_RE.search(s)
            if m:
                s = s[:m.start()] + m.group(1) + "\n    <static>1</static>" + s[m.end():]
                made += 1
        s = s.replace("<iyz>0</iyz>\n            <ixx>0.02</ixx>",
                      "<iyz>0</iyz>\n            <izz>0.05</izz>")
        open(f, "w").write(s)
    print("patched: made %d models static; ShoeRack izz fixed" % made)

def main():
    tmp = tempfile.mkdtemp(prefix="aws_house_")
    try:
        subprocess.check_call(["git", "clone", "--depth", "1", "--branch", BRANCH, REPO, tmp])
        src = os.path.join(tmp, "models")
        n = 0
        for name in sorted(os.listdir(src)):
            if not name.startswith("aws_robomaker_residential_"):
                continue
            d = os.path.join(MODELS_DST, name)
            if os.path.isdir(d):
                shutil.rmtree(d)
            shutil.copytree(os.path.join(src, name), d)
            n += 1
        print("copied %d models -> %s" % (n, MODELS_DST))
        if not os.path.exists(WORLD_DST):
            shutil.copy(os.path.join(tmp, "worlds", "small_house.world"), WORLD_DST)
            print("copied world -> %s" % WORLD_DST)
        patch(MODELS_DST)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print("done. rebuild: colcon build --symlink-install --packages-select articubot_one")

if __name__ == "__main__":
    main()
