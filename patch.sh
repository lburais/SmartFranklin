#!/usr/bin/env bash
set -e

echo "🔧 Patching SmartFranklin to use M5Unit-WeightI2C instead of M5Unit-MiniScale..."

# 1. Replace includes
grep -rl "M5UnitMiniScale.h" src include 2>/dev/null | while read f; do
    echo "  → Updating include in $f"
    gsed -i.bak 's|#include <M5UnitMiniScale.h>|#include <M5UnitWeightI2C.h>|g' "$f"
done

# 2. Replace class names
grep -rl "M5UnitMiniScale" src include 2>/dev/null | while read f; do
    echo "  → Updating class name in $f"
    gsed -i.bak 's|M5UnitMiniScale|M5UnitWeightI2C|g' "$f"
done

# 3. Replace method names (MiniScale → WeightI2C equivalents)
# Adjust these depending on your actual usage
grep -rl "getWeight" src include 2>/dev/null | while read f; do
    echo "  → Updating getWeight() in $f"
    gsed -i.bak 's|getWeight|getValue|g' "$f"
done

# 4. Update PlatformIO dependencies
if ! grep -q "M5Unit-WeightI2C" platformio.ini; then
    echo "  → Adding M5Unit-WeightI2C to platformio.ini"
    cat <<EOF >> platformio.ini

; Added automatically by patch_weighti2c.sh
lib_deps =
    m5stack/M5Unit-WeightI2C
EOF
fi

echo "✅ Patch complete."

