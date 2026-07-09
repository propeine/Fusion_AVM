#!/usr/bin/env bash
# Assemble installable Fusion script folders (bundles avm_physics where needed)
set -e
cd "$(dirname "$0")"
rm -rf release && mkdir -p release
for s in AVM_Probe AVM_Prune; do
  cp -r "fusion/$s" release/
done
cp -r fusion/AVM_LevelClone release/
cp -r avm_physics release/AVM_LevelClone/
find release -name __pycache__ -type d -exec rm -rf {} + 2>/dev/null || true
(cd release && for d in */; do zip -qr "${d%/}.zip" "$d"; done)
echo "release/ ready:"; ls release/*.zip
