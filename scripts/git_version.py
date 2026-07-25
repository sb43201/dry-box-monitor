import subprocess

Import("env")


def git_output(*args):
    try:
        return subprocess.check_output(
            ["git", *args],
            cwd=env.subst("$PROJECT_DIR"),
            stderr=subprocess.DEVNULL,
            text=True,
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


commit = git_output("rev-parse", "--short=7", "HEAD")
dirty = git_output("status", "--porcelain", "--untracked-files=no")
if dirty not in ("", "unknown"):
    commit += "-dirty"

env.Append(
    CPPDEFINES=[
        ("DRYBOX_FIRMWARE_VERSION", env.StringifyMacro("1.0.0")),
        ("DRYBOX_GIT_COMMIT", env.StringifyMacro(commit)),
    ]
)
