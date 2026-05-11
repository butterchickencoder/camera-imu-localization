import argparse

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["mono", "stereo"], required=True)
    parser.add_argument("--output", default= "results/poses.csv")

    args = parser.parse_args()

    if args.mode == "mono":
        from vision.feature_tracker_mono import run
    else:
        from vision.feature_tracker_stereo import run

    run(args)