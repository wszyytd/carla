from src.carla_experiments.progress import ProgressReporter


class RecordingStream:
    def __init__(self) -> None:
        self.text = ""
        self.flush_count = 0

    def write(self, value: str) -> None:
        self.text += value

    def flush(self) -> None:
        self.flush_count += 1


def test_progress_reporter_writes_one_line_and_flushes() -> None:
    stream = RecordingStream()

    ProgressReporter(stream)("连接 CARLA")

    assert stream.text == "连接 CARLA\n"
    assert stream.flush_count == 1
