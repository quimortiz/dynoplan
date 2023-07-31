import unittest
import subprocess

# TODO: Python should give error if c++ gives error!


class TestBenchmark(unittest.TestCase):
    def test_bench(self):
        cmd = [
            "python3",
            "../benchmark/benchmark.py",
            "-m",
            "bench",
            "-bc",
            "../benchmark/config/compare_minimal.yaml",
        ]

        print("running cmd: ", " ".join(cmd))

        out = subprocess.run(cmd)
        self.assertEqual(out.returncode, 0)

    def test_bench_time(self):
        cmd = [
            "python3",
            "../benchmark/benchmark.py",
            "-m",
            "bench_time",
            "-bc",
            "../benchmark/config/bench_time_minimal.yaml",
        ]

        out = subprocess.run(cmd)
        self.assertEqual(out.returncode, 0)

    def test_components(self):
        cmd = [
            "python3",
            "../benchmark/benchmark.py",
            "-m",
            "study",
            "-bc",
            "../benchmark/config/bench_abblation_study_minimal.yaml",
        ]
        out = subprocess.run(cmd)
        self.assertEqual(out.returncode, 0)

    def test_heuristic(self):
        cmd = [
            "python3",
            "../benchmark/benchmark.py",
            "-m",
            "bench_search",
            "-bc",
            "../benchmark/config/bench_search_minimal.yaml",
        ]

        out = subprocess.run(cmd)
        self.assertEqual(out.returncode, 0)


if __name__ == "__main__":
    unittest.main()
