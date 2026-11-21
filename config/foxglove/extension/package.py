#!/usr/bin/env python3
"""Zip this extension into ../autocode-bot-panels.foxe (a .foxe is a zip of package.json, dist/, README)."""
import os
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "autocode-bot-panels.foxe")
with zipfile.ZipFile(OUT, "w", zipfile.ZIP_DEFLATED) as z:
    for rel in ("package.json", "README.md", "dist/extension.js"):
        z.write(os.path.join(HERE, rel), rel)
print("wrote", os.path.normpath(OUT))
