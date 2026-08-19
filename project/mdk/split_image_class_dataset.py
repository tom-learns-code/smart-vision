import argparse
import math
import random
import shutil
from pathlib import Path


IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".gif", ".webp"}


def is_image(path):
    return path.is_file() and path.suffix.lower() in IMAGE_EXTS


def collect_classes(src_dir):
    classes = []
    for class_dir in sorted(src_dir.iterdir()):
        if class_dir.is_dir():
            images = [p for p in class_dir.rglob("*") if is_image(p)]
            classes.append((class_dir, sorted(images)))
    return classes


def prepare_output(out_dir, overwrite):
    if out_dir.exists():
        if not overwrite:
            raise SystemExit(
                "Output directory already exists: %s\n"
                "Use --overwrite if you want to recreate it." % out_dir
            )
        shutil.rmtree(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)


def copy_file(src_file, dst_file):
    dst_file.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src_file, dst_file)


def split_per_class(src_dir, out_dir, chunk_size, shuffle, seed):
    classes = collect_classes(src_dir)
    if not classes:
        raise SystemExit("No class folders found in: %s" % src_dir)

    rng = random.Random(seed)
    class_chunks = []
    max_parts = 0

    for class_dir, images in classes:
        if shuffle:
            images = list(images)
            rng.shuffle(images)

        chunks = [
            images[i:i + chunk_size]
            for i in range(0, len(images), chunk_size)
        ]
        class_chunks.append((class_dir, chunks))
        max_parts = max(max_parts, len(chunks))

    for part_index in range(max_parts):
        part_dir = out_dir / ("part_%03d" % (part_index + 1))

        for class_dir, chunks in class_chunks:
            dst_class_dir = part_dir / class_dir.name
            dst_class_dir.mkdir(parents=True, exist_ok=True)

            if part_index >= len(chunks):
                continue

            for src_file in chunks[part_index]:
                rel_path = src_file.relative_to(class_dir)
                copy_file(src_file, dst_class_dir / rel_path)

    return max_parts, classes


def split_global(src_dir, out_dir, chunk_size, shuffle, seed):
    classes = collect_classes(src_dir)
    all_images = []

    for class_dir, images in classes:
        for image in images:
            all_images.append((class_dir, image))

    if not all_images:
        raise SystemExit("No images found in: %s" % src_dir)

    if shuffle:
        rng = random.Random(seed)
        rng.shuffle(all_images)

    max_parts = int(math.ceil(len(all_images) / float(chunk_size)))

    for part_index in range(max_parts):
        part_dir = out_dir / ("part_%03d" % (part_index + 1))

        for class_dir, _ in classes:
            (part_dir / class_dir.name).mkdir(parents=True, exist_ok=True)

        start = part_index * chunk_size
        end = start + chunk_size

        for class_dir, src_file in all_images[start:end]:
            rel_path = src_file.relative_to(class_dir)
            copy_file(src_file, part_dir / class_dir.name / rel_path)

    return max_parts, classes


def write_report(out_dir, src_dir, mode, chunk_size, parts, classes):
    report = out_dir / "split_report.txt"
    total = 0
    lines = [
        "source=%s" % src_dir,
        "mode=%s" % mode,
        "chunk_size=%d" % chunk_size,
        "parts=%d" % parts,
        "",
        "classes:",
    ]

    for class_dir, images in classes:
        total += len(images)
        lines.append("%s: %d images" % (class_dir.name, len(images)))

    lines.insert(4, "total_images=%d" % total)
    report.write_text("\n".join(lines), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(
        description="Split an image classification dataset while keeping folder structure."
    )
    parser.add_argument(
        "--src",
        default=r"E:\BaiduNetdiskDownload\上位机\image_class",
        help="Source dataset folder."
    )
    parser.add_argument(
        "--out",
        default=r"E:\BaiduNetdiskDownload\上位机\image_class_split",
        help="Output folder."
    )
    parser.add_argument(
        "--chunk-size",
        type=int,
        default=15,
        help="Images per class per part in per-class mode, or total images per part in global mode."
    )
    parser.add_argument(
        "--mode",
        choices=("per-class", "global"),
        default="per-class",
        help="per-class keeps about chunk-size images for every class in each part."
    )
    parser.add_argument(
        "--no-shuffle",
        action="store_true",
        help="Do not shuffle images before splitting."
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="Random seed used when shuffling."
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Delete existing output folder before splitting."
    )

    args = parser.parse_args()

    src_dir = Path(args.src)
    out_dir = Path(args.out)

    if args.chunk_size <= 0:
        raise SystemExit("--chunk-size must be greater than 0.")
    if not src_dir.exists():
        raise SystemExit("Source directory does not exist: %s" % src_dir)

    prepare_output(out_dir, args.overwrite)

    if args.mode == "per-class":
        parts, classes = split_per_class(
            src_dir, out_dir, args.chunk_size, not args.no_shuffle, args.seed
        )
    else:
        parts, classes = split_global(
            src_dir, out_dir, args.chunk_size, not args.no_shuffle, args.seed
        )

    write_report(out_dir, src_dir, args.mode, args.chunk_size, parts, classes)

    print("Done.")
    print("Source:", src_dir)
    print("Output:", out_dir)
    print("Mode:", args.mode)
    print("Parts:", parts)
    print("Report:", out_dir / "split_report.txt")


if __name__ == "__main__":
    main()
