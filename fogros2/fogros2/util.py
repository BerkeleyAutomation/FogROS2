import os
import shutil


def make_zip_file(dir_name, target_path):
    root_dir, workspace_name = os.path.split(dir_name)
    print(root_dir, workspace_name)
    return shutil.make_archive(base_dir=workspace_name, root_dir=root_dir, format="zip", base_name=target_path)
