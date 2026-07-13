import os

Import("env")

# Silence compiler warnings for everything that is not our own code
# (lib_deps, the Arduino core and other framework/platform sources).
# Files under src/ keep the warning flags from build_flags.

src_dir = os.path.normcase(os.path.realpath(env.subst("$PROJECT_SRC_DIR")))


def is_own_code(path):
    path = os.path.normcase(os.path.realpath(path))
    return path == src_dir or path.startswith(src_dir + os.sep)


def suppress_warnings(target_env, node):
    if is_own_code(node.srcnode().get_abspath()):
        return node
    # Rebuilding the object node resets the construction variables, so the
    # include paths and defines of this specific library have to be carried
    # over explicitly, otherwise its own headers stop resolving.
    return target_env.Object(
        node,
        CFLAGS=target_env["CFLAGS"] + ["-w"],
        CXXFLAGS=target_env["CXXFLAGS"] + ["-w"],
        CCFLAGS=target_env["CCFLAGS"],
        CPPPATH=target_env["CPPPATH"],
        CPPDEFINES=target_env["CPPDEFINES"],
        ASFLAGS=target_env["ASFLAGS"],
        ASPPFLAGS=target_env["ASPPFLAGS"],
    )


env.AddBuildMiddleware(suppress_warnings, "*")
