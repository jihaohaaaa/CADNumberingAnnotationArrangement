# TODO: 命令行程序

# src/CADAlgo/__main__.py

import argparse
from CADAlgo import generate, check, parser  # 导入你的模块函数


def main():
    parser = argparse.ArgumentParser(description="CAD Numbering & Annotation CLI")
    parser.add_argument("--input", help="Input file")
    parser.add_argument("--output", help="Output file")
    parser.add_argument("--check", action="store_true", help="Run check only")
    args = parser.parse_args()

    if args.check:
        print("Running check only...")
        # Implement check logic here
        pass
    else:
        print("Running full generation process...")
        # Implement full generation logic here
        pass


if __name__ == "__main__":
    main()
