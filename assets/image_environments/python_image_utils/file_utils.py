import sys, string, os

def get_all_files_in_dir_recursively(fp):
    files = []
    for rr, d, f in os.walk(fp):
        for file in f:
            files.append(file)

    return files

def is_file_in_dir(fp, file_name):
    files = get_all_files_in_given_dir(fp)
    return file_name in files

def get_all_directories_in_given_dir(fp):
    directories = os.walk(fp).next()[1]
    return directories

def get_all_files_in_given_dir(fp):
    files = os.walk(fp).next()[2]
    return files



