"""Logging setup. Loads ``config/logging.yaml`` if PyYAML is available."""

from __future__ import annotations

import json
import logging
import logging.config
from pathlib import Path
from typing import Optional, Union

DEFAULT_FORMAT = "%(asctime)s.%(msecs)03d %(levelname)-7s %(name)s: %(message)s"


class JsonFormatter(logging.Formatter):
    """One JSON object per line — convenient for latency / fault analysis."""

    def format(self, record: logging.LogRecord) -> str:
        obj = {
            "t": round(record.created, 6),
            "level": record.levelname,
            "logger": record.name,
            "msg": record.getMessage(),
        }
        for key in ("request_id", "action", "elapsed_ms", "code"):
            if hasattr(record, key):
                obj[key] = getattr(record, key)
        if record.exc_info:
            obj["exc"] = self.formatException(record.exc_info)
        return json.dumps(obj, ensure_ascii=False)


def setup_logging(level: Union[int, str] = "INFO", config_path: Optional[Union[str, Path]] = None, json_lines: bool = False) -> None:
    if config_path is not None:
        path = Path(config_path)
        if path.is_file():
            try:
                import yaml  # type: ignore

                with path.open("r", encoding="utf-8") as fh:
                    cfg = yaml.safe_load(fh)
                logging.config.dictConfig(cfg)
                return
            except Exception as exc:  # fall back to basic config
                logging.basicConfig(level=level, format=DEFAULT_FORMAT, datefmt="%H:%M:%S")
                logging.getLogger(__name__).warning("failed to load %s: %s", path, exc)
                return
    handler = logging.StreamHandler()
    handler.setFormatter(JsonFormatter() if json_lines else logging.Formatter(DEFAULT_FORMAT, "%H:%M:%S"))
    root = logging.getLogger()
    root.handlers[:] = [handler]
    root.setLevel(level)
