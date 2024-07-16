import os, sys

def replace_shebang(folder_path):
    print(f'Processing folder {folder_path}...')
    for root, dirs, files in os.walk(folder_path):
        for file in files:
            file_path = os.path.join(root, file)
            if os.path.isfile(file_path) and file_path.endswith('.py'):
                with open(file_path, 'r', encoding='utf-8') as f:
                    lines = f.readlines()
                    if lines and lines[0].startswith('#!'):
                        lines[0] = f'#!{sys.executable}\n'  # Replace with the desired shebang
                        with open(file_path, 'w') as f:
                            f.writelines(lines)
                        print(f'Shebang replaced in {file_path}')

if __name__ == '__main__':
    *args, = sys.argv[1:]
    if not args:
        print('Usage: python fix_shebang.py <folder_path>')
        sys.exit(1)
    replace_shebang(args[0])