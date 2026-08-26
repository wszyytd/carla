from __future__ import annotations

import sys
from typing import TextIO


class ProgressReporter:
    """Write progress lines immediately for observable remote runs."""

    def __init__(self, stream: TextIO | None = None) -> None:
        self.stream = stream or sys.stdout

    def __call__(self, message: str) -> None:
        print(message, file=self.stream, flush=True)
