#!/usr/bin/env python3

import os
import subprocess
import tempfile
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
BOOTLOADER_DIR = REPO_ROOT / "Tools" / "AP_Bootloader"
ABIN_TEST_DIR = BOOTLOADER_DIR / "tests"
SITL_DIR = REPO_ROOT / "libraries" / "AP_HAL_SITL"
GTEST_DIR = REPO_ROOT / "modules" / "gtest" / "googletest"


ABIN_HEADER_MUTANTS = [
    ("md5_length", "strlen(text) != MD5_TEXT_SIZE", "strlen(text) == MD5_TEXT_SIZE"),
    ("md5_decode", "!decode_hex(text[index * 2U], high) ||", "!decode_hex(text[index * 2U], high) &&"),
    ("header_line_limit", "line_length >= line_capacity - 1U", "line_length > line_capacity - 1U"),
    ("duplicate_md5", "header.has_md5 || !parse_md5", "header.has_md5 && !parse_md5"),
]


ABIN_FILE_MUTANTS = [
    ("body_md5", "sizeof(calculated)) == 0", "sizeof(calculated)) != 0"),
    ("unsigned_descriptor", "        is_signed = false;\n    } else if", "        is_signed = true;\n    } else if"),
    ("descriptor_count", "location.matches == 1", "location.matches >= 1"),
    ("descriptor_size", "descriptor.image_size != header.body_size || ", ""),
    ("descriptor_board", "descriptor.board_id != board_id", "false"),
    ("descriptor_crc1", "crc1 == descriptor.image_crc1 &&", "true &&"),
    ("descriptor_crc2", "crc2 == descriptor.image_crc2", "true"),
    ("image_limit", "padded_size <= maximum_image_size && ", ""),
    ("descriptor_type", "if (valid && location.is_signed)", "if (valid && !location.is_signed)"),
    ("stream_finish", "return success && sink.finish();", "return success;"),
]


SITL_MUTANTS = [
    ("flash_priority", "if (file_exists(FLASH_PATH))", "if (false)"),
    ("verify_resume", "if (file_exists(VERIFY_PATH))", "if (false)"),
    ("input_detection", "if (!file_exists(INPUT_PATH))", "if (file_exists(INPUT_PATH))"),
    ("input_transition", "? VERIFY_PATH : nullptr", "? nullptr : VERIFY_PATH"),
    ("fault_detection", "configured != nullptr &&", "configured == nullptr &&"),
    ("explicit_enable", "value != nullptr &&", "value == nullptr &&"),
    ("no_update", "if (path == nullptr)", "if (path != nullptr)"),
    ("failure_injection", 'if (requested_result("failure"))', 'if (!requested_result("failure"))'),
    ("failure_result", "marked ? Result::FAILED : Result::INTERRUPTED", "marked ? Result::INTERRUPTED : Result::FAILED"),
    ("verify_transition", "strcmp(path, VERIFY_PATH) == 0", "strcmp(path, VERIFY_PATH) != 0"),
    ("interruption", 'if (requested_result("interrupted"))', 'if (!requested_result("interrupted"))'),
    ("success_transition", "if (!move_file(FLASH_PATH, FLASHED_PATH))", "if (move_file(FLASH_PATH, FLASHED_PATH))"),
    ("success_result", "return Result::FLASHED;", "return Result::NO_UPDATE;"),
]


def replaced(source: str, old: str, new: str) -> str:
    if source.count(old) != 1:
        raise ValueError(f"mutation target occurs {source.count(old)} times: {old}")
    return source.replace(old, new, 1)


def run(command: list[str]) -> subprocess.CompletedProcess:
    return subprocess.run(command, capture_output=True, text=True, check=False)


def compile_and_run_abin(source: Path, companion: Path, output: Path) -> bool:
    command = [
        os.environ.get("CXX", "g++"), "-std=gnu++14", "-Wall", "-Wextra", "-Werror",
        f"-I{ABIN_TEST_DIR / 'include'}", f"-I{BOOTLOADER_DIR}",
        str(ABIN_TEST_DIR / "test_abin_file.cpp"), str(source), str(companion),
        str(BOOTLOADER_DIR / "md5.cpp"), "-o", str(output),
    ]
    compiled = run(command)
    return compiled.returncode == 0 and run([str(output)]).returncode == 0


def compile_and_run_sitl(source: Path, output: Path) -> bool:
    command = [
        os.environ.get("CXX", "g++"), "-std=gnu++14", "-Wall", "-Wextra", "-Werror",
        "-pthread", f"-I{REPO_ROOT / 'tests'}", f"-I{REPO_ROOT / 'libraries'}",
        f"-I{SITL_DIR}", f"-I{GTEST_DIR / 'include'}", f"-I{GTEST_DIR}",
        str(SITL_DIR / "tests" / "test_sdcard_ota.cpp"), str(source),
        str(GTEST_DIR / "src" / "gtest-all.cc"), "-o", str(output),
    ]
    compiled = run(command)
    return compiled.returncode == 0 and run([str(output)]).returncode == 0


def exercise(name: str, original: Path, mutants: list[tuple[str, str, str]],
             compiler) -> list[str]:
    source = original.read_text(encoding="utf-8")
    survivors = []
    with tempfile.TemporaryDirectory(prefix=f"{name}-mutants-") as directory_name:
        directory = Path(directory_name)
        baseline = directory / original.name
        baseline.write_text(source, encoding="utf-8")
        if not compiler(baseline, directory / "baseline"):
            raise RuntimeError(f"{name} baseline tests failed")

        for index, (label, old, new) in enumerate(mutants):
            mutant = directory / f"mutant-{index}.cpp"
            mutant.write_text(replaced(source, old, new), encoding="utf-8")
            if compiler(mutant, directory / f"test-{index}"):
                survivors.append(label)
            else:
                print(f"KILLED {name}:{label}")
    return survivors


def main() -> int:
    header = BOOTLOADER_DIR / "abin_header.cpp"
    validator = BOOTLOADER_DIR / "abin_file.cpp"
    survivors = exercise(
        "abin-header", header, ABIN_HEADER_MUTANTS,
        lambda source, output: compile_and_run_abin(source, validator, output))
    survivors += exercise(
        "abin-validator", validator, ABIN_FILE_MUTANTS,
        lambda source, output: compile_and_run_abin(source, header, output))
    survivors += exercise(
        "sitl", SITL_DIR / "SDCardOTA.cpp", SITL_MUTANTS, compile_and_run_sitl)
    tested = len(ABIN_HEADER_MUTANTS) + len(ABIN_FILE_MUTANTS) + len(SITL_MUTANTS)
    print(f"targeted mutants: {tested}; survivors: {len(survivors)}")
    if survivors:
        print("SURVIVED " + ", ".join(survivors))
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
