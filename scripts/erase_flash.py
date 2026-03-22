Import("env")


def erase_flash(source, target, env_):
    upload_port = env_.subst("$UPLOAD_PORT")
    if not upload_port or upload_port == "$UPLOAD_PORT":
        print("Set UPLOAD_PORT or pass --upload-port before running erase_flash.")
        return 1

    uploader = env_.subst("$UPLOADER")
    pythonexe = env_.subst("$PYTHONEXE")
    cmd = [
        pythonexe,
        uploader,
        "--chip",
        "esp32",
        "--port",
        upload_port,
        "erase_flash",
    ]
    return env_.Execute(" ".join(cmd))


env.AddCustomTarget(
    name="erase_flash",
    dependencies=None,
    actions=[erase_flash],
    title="Erase Flash",
    description="Erase the full ESP32 flash before uploading firmware",
)
