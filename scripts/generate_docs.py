"""SmartFranklin docs generation hook.

SPDX-License-Identifier: MIT
"""

import os
Import("env")

def generate_docs(source, target, env):
    os.system("doxygen Doxyfile")

env.AddPostAction("buildprog", generate_docs)
