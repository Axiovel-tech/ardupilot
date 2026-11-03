"""Compile each drone of a .skyc into the .skyb that Axio Server would upload.

Mirrors flockwave.server.ext.mavlink.driver.MAVLinkUAV.upload_show for the
blocks a plain show has (trajectory, lights); no show_pro extension.
Usage: skyc_to_skyb.py show.skyc outdir
"""
import json, sys, zipfile
from pathlib import Path
import trio
from flockwave.server.show.formats import SkybrushBinaryShowFile
from flockwave.server.show.specification import get_trajectory_from_show_specification
from flockwave.server.show.lights import get_light_program_from_show_specification

async def main(src, out):
    z = zipfile.ZipFile(src)
    show = json.loads(z.read("show.json"))
    out = Path(out); out.mkdir(parents=True, exist_ok=True)
    for i, d in enumerate(show["swarm"]["drones"]):
        spec = d["settings"]
        async with SkybrushBinaryShowFile.create_in_memory() as f:
            await f.add_trajectory(get_trajectory_from_show_specification(spec))
            await f.add_encoded_light_program(get_light_program_from_show_specification(spec))
            await f.finalize()
            data = f.get_contents()
        name = spec.get("name") or f"d{i+1:03d}"
        (out / f"{i:03d}.skyb").write_bytes(data)
        print(i, name, len(data), spec.get("home"))

trio.run(main, sys.argv[1], sys.argv[2])
