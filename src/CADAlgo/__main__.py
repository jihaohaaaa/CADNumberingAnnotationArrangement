import argparse
import json
from pathlib import Path
from typing import Any

from CADAlgo.parser import parse_schema


def _load_json(path: str) -> dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def main():
    ap = argparse.ArgumentParser(
        description="CAD Numbering & Annotation CLI (generate solution lines)"
    )
    ap.add_argument("--input", required=True, help="Input schema JSON file")
    ap.add_argument(
        "--output",
        required=False,
        help="Output solution JSON file (default: <input>.solution.json)",
    )
    ap.add_argument(
        "--samples-distance",
        type=int,
        default=3,
        help="Sampling step along exterior boundary (default: 3)",
    )
    ap.add_argument(
        "--proximity-threshold",
        type=float,
        default=1.0,
        help="Minimum distance between any two generated lines (default: 1.0)",
    )
    ap.add_argument(
        "--ensure-ascii",
        action="store_true",
        help="Force ASCII in JSON output (default: UTF-8)",
    )
    args = ap.parse_args()

    input_path = Path(args.input)
    if not input_path.is_file():
        ap.error(f"Input file not found: {input_path}")

    data = _load_json(str(input_path))
    schema = parse_schema(data)

    output_path = (
        Path(args.output) if args.output else input_path.with_suffix(".solution.json")
    )

    solution = schema.export_solution(
        str(output_path),
        samples_distance=args.samples_distance,
        proximity_threshold=args.proximity_threshold,
        ensure_ascii=args.ensure_ascii,
    )


if __name__ == "__main__":  # pragma: no cover
    main()
