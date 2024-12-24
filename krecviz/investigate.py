#!/usr/bin/env python3

import sys
import argparse

import rerun as rr


def dump_rrd(rrd_path: str) -> None:
    print(f"\n=== Analyzing RRD: {rrd_path} ===")
    # Load the single recording from the file:
    recording = rr.dataframe.load_recording(rrd_path)

    # Inspect the schema to see which index columns (timelines) exist:
    schema = recording.schema()
    index_cols = schema.index_columns()
    print(f"Index columns found: {index_cols}")

    # As of Rerun 0.19, we must pick exactly one index column to build a view.
    if not index_cols:
        print("No index columns found. Cannot create a view.")
        return

    # We'll pick "log_tick" or something else if you prefer:
    index_col = "log_tick"
    print(f"Using index column: '{index_col}'")

    # Build a view that includes everything in the recording:
    view = recording.view(index=index_col, contents="/**")

    # Convert the view to a pandas DataFrame:
    df = view.select().read_pandas()

    # ----------------------------------------------------------------------
    # Filter out columns we don't want to see (e.g. mesh data).
    # We'll keep only columns that contain at least one of these keywords:
    #   "ViewCoordinates", "TransformMat3x3", "Translation3D", or "Text".
    # Adjust this list as needed for your debugging.
    # ----------------------------------------------------------------------
    KEEP_PATTERNS = ("ViewCoordinates", "TransformMat3x3", "Translation3D", "Text")

    # We'll also keep the index columns themselves, e.g. "log_tick", "log_time"
    # because it's useful to see how many rows we got (though it's likely 0).
    # So let's define a function to check if a column is "good."
    def should_keep(colname: str) -> bool:
        if colname in ("log_tick", "log_time"):
            return True
        return any(pattern in colname for pattern in KEEP_PATTERNS)

    # Filter columns:
    kept_cols = [c for c in df.columns if should_keep(c)]
    df = df[kept_cols]

    # Now save to CSV:
    csv_path = rrd_path.replace(".rrd", ".csv")
    df.to_csv(csv_path, index=True)
    print(f"\nSaved filtered DataFrame to: {csv_path}")

    print("\n--- DataFrame columns (filtered) ---")
    for col in df.columns:
        print(col)
    print("--- end of DataFrame ---\n")


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze RRD files")
    parser.add_argument(
        "--correct",
        default="/home/kuwajerw/repos/viz_stuff/krecviz/krecviz/fully_correct.rrd",
        help="Path to the correct RRD file",
    )
    parser.add_argument(
        "--problem",
        default="/home/kuwajerw/repos/viz_stuff/krecviz_rust/translation_but_notransformation_new.rrd",
        help="Path to the problematic RRD file",
    )

    args = parser.parse_args()

    dump_rrd(args.correct)
    dump_rrd(args.problem)


if __name__ == "__main__":
    main()
