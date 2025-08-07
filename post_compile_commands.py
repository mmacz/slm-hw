import os
import shutil

def after_build(source, target, env):
    build_dir = env.subst("$BUILD_DIR")
    compile_commands_path = os.path.join(build_dir, "compile_commands.json")
    dest_path = os.path.join(env["PROJECT_DIR"], "compile_commands.json")

    if os.path.exists(compile_commands_path):
        try:
            if os.path.exists(dest_path) or os.path.islink(dest_path):
                os.remove(dest_path)
            os.symlink(compile_commands_path, dest_path)
            print(f"[postbuild] Symlink created: {dest_path} -> {compile_commands_path}")
        except Exception as e:
            print(f"[postbuild] Failed to symlink compile_commands.json: {e}")
    else:
        print("[postbuild] compile_commands.json not found")

# Rejestrujemy
Import("env")
env.AddPostAction("buildprog", after_build)

