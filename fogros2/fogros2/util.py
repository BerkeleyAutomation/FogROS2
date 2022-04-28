import os
import errno
import shutil

_work_dir_cache = None
_instance_dir_cache = None

def _mkdir(path, mode=0o700):
    try:
        os.mkdir(path, mode=mode)
    except OSError as e:
        if e.errno != errno.EEXIST:
            raise

def work_dir():
    global _work_dir_cache
    if _work_dir_cache is None:
        home = os.path.expanduser("~")
        path = os.path.join(home, ".fogros2")
        _mkdir(path)
        _work_dir_cache = path
    return _work_dir_cache

def instance_dir():
    global _instance_dir_cache
    if _instance_dir_cache is None:
        path = os.path.join(work_dir(), "instances")
        _mkdir(path)
        _instance_dir_cache = path
    return _instance_dir_cache

def make_zip_file(dir_name, target_path):
    root_dir, workspace_name = os.path.split(dir_name)
    print(root_dir, workspace_name)
    return shutil.make_archive(base_dir=workspace_name, root_dir=root_dir, format="zip", base_name=target_path)
